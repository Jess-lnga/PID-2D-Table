#include "imu_data.h"
#include <Wire.h>
#include <math.h>

namespace ImuData
{
namespace
{
constexpr uint32_t SAMPLE_US = 10000;
constexpr unsigned I2C_ATTEMPTS = 2;
constexpr unsigned BUS_RECOVERY_FAILURES = 5;
constexpr unsigned long LOST_FRAME_MS = 500;
constexpr unsigned long CAL_TIMEOUT_MS = 10000;
unsigned failedFrames = 0;
unsigned long calibrationStartedMs = 0;
void configureBus()
{
  Wire.begin(21, 22);
  Wire.setClock(100000);
  Wire.setTimeOut(5); // Bound each transaction instead of blocking control for 20 ms.
}
constexpr uint32_t DEBOUNCE_MS = 30;
constexpr unsigned CALIBRATION_SAMPLES = 200;
constexpr float CAL_GYRO_STD_DPS = 1.5f; // Was 0.5: tolerate modest stationary noise.
constexpr float CAL_ACCEL_STD_G = 0.04f;
constexpr float CAL_ACCEL_STEP_G = 0.20f;
constexpr float CAL_GYRO_STEP_DPS = 8.0f;
constexpr unsigned CAL_MAX_REJECTED = 30; // At most 30 discarded frames per 200 accepted.
constexpr unsigned CAL_MOTION_STREAK = 20; // Stop the manual attempt after ~200 ms sustained outliers.
constexpr float RAD = PI / 180.0f;
State state;
bool switchCandidate = false;
unsigned long switchChangedMs = 0, retryMs = 0, lastFrameMs = 0;
uint32_t sampleUs = 0;
unsigned calibrationCount = 0;
float bias[3] = {}, sums[3] = {}, squares[3] = {};
float accelSums[3] = {}, accelSquares[3] = {};
unsigned rejectedWindow = 0, rejectedStreak = 0;
float previousAccel[3] = {}, previousGyro[3] = {};
bool previousSample = false;
bool previousCalibrationValid = false;
unsigned long diagnosticMs = 0;
uint32_t attitudeUs = 0;

void reportI2cError(uint8_t reg, uint8_t error)
{
  ++state.i2cErrors;
  state.lastRegister = reg;
  state.lastI2cError = error;
  state.diagnostic = "Erreur I2C (voir registre et code)";
  Serial.printf("MPU I2C: adresse=0x%02X registre=0x%02X erreur=%u\n", state.address, reg, error);
}
float q[4] = {1, 0, 0, 0};
bool attitudeInitialized = false;

bool readRegisters(uint8_t reg, uint8_t* data, size_t count)
{
  for (unsigned attempt = 0; attempt < I2C_ATTEMPTS; ++attempt)
  {
    if (attempt) ++state.i2cRetries;
    Wire.beginTransmission(state.address);
    Wire.write(reg);
    const uint8_t error = Wire.endTransmission(false);
    if (error) { reportI2cError(reg, error); continue; }
    const size_t received = Wire.requestFrom(state.address, (uint8_t)count);
    if (received != count)
    {
      while (Wire.available()) Wire.read();
      reportI2cError(reg, 0xFF); // Short read, not a Wire transmission code.
      continue;
    }
    for (size_t i = 0; i < count; ++i) data[i] = Wire.read();
    return true;
  }
  return false;
}
bool writeRegister(uint8_t reg, uint8_t value)
{
  Wire.beginTransmission(state.address);
  Wire.write(reg);
  Wire.write(value);
  const uint8_t error = Wire.endTransmission();
  if (error) reportI2cError(reg, error);
  return error == 0;
}
// Clears the measurement window only; this never starts a calibration attempt.
void resetCalibration()
{
  calibrationCount = 0;
  rejectedWindow = rejectedStreak = 0;
  for (int i = 0; i < 3; ++i) sums[i] = squares[i] = accelSums[i] = accelSquares[i] = 0;
  state.calibrationSamples = 0;
  previousSample = false;
  state.hasData = false;
  attitudeInitialized = false;
}
void cancelCalibration(const char* reason)
{
  if (state.calibrationActive) {
    state.calibrationActive = false;
    state.calibrationFailed = true;
    state.calibrated = previousCalibrationValid;
    Serial.println(reason);
  }
  resetCalibration(); // Clear sampling window only; preserve accepted bias and zero.
}
void failedFrame()
{
  state.hasData = false; // Never command from a failed or partially read frame.
  ++failedFrames;
  state.i2cConsecutiveFailures = failedFrames;
  if (failedFrames == BUS_RECOVERY_FAILURES)
  {
    Wire.end();
    configureBus();
    ++state.i2cRecoveries;
    state.diagnostic = "Bus I2C reinitialise; attente de mesures";
  }
  if (millis() - lastFrameMs > LOST_FRAME_MS)
  {
    state.connected = false;
    cancelCalibration("Calibration interrompue: perte MPU prolongee");
    state.diagnostic = "Perte MPU prolongee: reconnexion sans calibration automatique";
    retryMs = millis();
  }
}
bool connectSensor()
{
  cancelCalibration("Calibration interrompue: reconnexion MPU");
  state.deviceId = 0;
  for (uint8_t address : {uint8_t(0x68), uint8_t(0x69)})
  {
    state.address = address;
    uint8_t id = 0;
    if (!readRegisters(0x75, &id, 1)) continue;
    // MPU-6500, MPU-9250 and MPU-9255 share these accel/gyro registers.
    if (id != 0x70 && id != 0x71 && id != 0x73)
    {
      state.deviceId = id;
      state.diagnostic = "Identification MPU non prise en charge";
      Serial.printf("MPU adresse 0x%02X: WHO_AM_I inattendu 0x%02X\n", address, id);
      continue;
    }
    if (!writeRegister(0x6B, 0x80)) continue;
    delay(100);
    bool ok = writeRegister(0x6B, 0x01) && writeRegister(0x6C, 0x00)
      && writeRegister(0x1A, 0x03) // Gyro DLPF: 41 Hz
      && writeRegister(0x19, 9)    // 1 kHz / 10 = 100 Hz
      && writeRegister(0x1B, 0)    // +/-250 deg/s
      && writeRegister(0x1C, 0)    // +/-2 g
      && writeRegister(0x1D, 0x03) // Accel DLPF: 41 Hz
      && writeRegister(0x37, 0)    // Clear INT_STATUS only when reading INT_STATUS.
      && writeRegister(0x38, 1);   // Enable DATA_RDY status; INT pin may remain unwired.
    if (!ok) continue;
    delay(50);
    // Verify actual configuration instead of treating ACK as proof it was applied.
    const uint8_t regs[] = {0x6B, 0x6C, 0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x37, 0x38};
    const uint8_t expected[] = {1, 0, 9, 3, 0, 0, 3, 0, 1};
    for (unsigned i = 0; i < sizeof(regs); ++i)
    {
      uint8_t value = 0;
      if (!readRegisters(regs[i], &value, 1)) { ok = false; break; }
      Serial.printf("MPU config 0x%02X = 0x%02X (attendu 0x%02X)\n", regs[i], value, expected[i]);
      if (value != expected[i])
      {
        state.diagnostic = "Configuration MPU non appliquee";
        state.lastRegister = regs[i];
        ok = false;
        break;
      }
    }
    if (!ok) continue;
    state.deviceId = id;
    state.diagnostic = state.calibrated ? "Calibration conservee" : "Cliquer sur Calibrate";
    Serial.printf("MPU detecte: adresse 0x%02X, WHO_AM_I 0x%02X. Calibration uniquement manuelle.\n", address, id);
    failedFrames = state.i2cConsecutiveFailures = 0;
    sampleUs = micros();
    lastFrameMs = millis();
    return true;
  }
  state.address = 0;
  return false;
}
int16_t signedWord(const uint8_t* data)
{
  return (int16_t)((uint16_t(data[0]) << 8) | data[1]);
}
void updateAttitude(float ax, float ay, float az, float gx, float gy, float gz, float dt)
{
  const float norm = sqrtf(ax*ax + ay*ay + az*az);
  if (!attitudeInitialized)
  {
    if (norm < 0.1f) return;
    const float roll = atan2f(ay, az);
    const float pitch = atan2f(-ax, sqrtf(ay*ay + az*az));
    const float cr = cosf(roll/2), sr = sinf(roll/2);
    const float cp = cosf(pitch/2), sp = sinf(pitch/2);
    q[0] = cr*cp; q[1] = sr*cp; q[2] = cr*sp; q[3] = -sr*sp;
    attitudeInitialized = true;
  }
  gx *= RAD; gy *= RAD; gz *= RAD;
  // Quaternion complementary filter: gravity corrects gyro drift.
  // Reject strong linear acceleration; yaw is intentionally unreferenced.
  if (norm > 0.85f && norm < 1.15f)
  {
    ax /= norm; ay /= norm; az /= norm;
    const float vx = 2*(q[1]*q[3] - q[0]*q[2]);
    const float vy = 2*(q[0]*q[1] + q[2]*q[3]);
    const float vz = q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3];
    constexpr float correction = 2.0f;
    gx += correction*(ay*vz - az*vy);
    gy += correction*(az*vx - ax*vz);
    gz += correction*(ax*vy - ay*vx);
  }
  const float w=q[0], x=q[1], y=q[2], z=q[3], step=0.5f*dt;
  q[0] += (-x*gx-y*gy-z*gz)*step;
  q[1] += (w*gx+y*gz-z*gy)*step;
  q[2] += (w*gy-x*gz+z*gx)*step;
  q[3] += (w*gz+x*gy-y*gx)*step;
  const float length = sqrtf(q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3]);
  for (float& v : q) v /= length;
  state.rollDeg = atan2f(2*(q[0]*q[1]+q[2]*q[3]), 1-2*(q[1]*q[1]+q[2]*q[2])) / RAD;
  state.pitchDeg = asinf(constrain(2*(q[0]*q[2]-q[3]*q[1]), -1.0f, 1.0f)) / RAD;
  // Report angles relative to the stationary calibration pose.
  state.rollDeg = remainderf(state.rollDeg - state.rollZeroDeg, 360.0f);
  state.pitchDeg -= state.pitchZeroDeg;
  state.hasData = true;
  state.lastUpdateMs = millis();
}
}
void begin()
{
  state = State{};
  // Boot and reconnection must wait for an explicit Calibrate button request.
  state.calibrationActive = false;
  Serial.println("MPU: calibration MANUELLE uniquement (bouton Calibrate).");
  previousCalibrationValid = false;
  resetCalibration();
  for (int i=0;i<3;++i) bias[i]=0;
  // External 10 kOhm pull-down; internal pull-down also supports a bare switch.
  pinMode(MODE_SWITCH_PIN, INPUT_PULLDOWN);
  switchCandidate = state.enabled = digitalRead(MODE_SWITCH_PIN) == HIGH;
  switchChangedMs = millis();
  failedFrames = 0;
  configureBus();
  retryMs = millis() - 1000;
}
void update()
{
  const unsigned long now = millis();
  const bool closed = digitalRead(MODE_SWITCH_PIN) == HIGH;
  if (closed != switchCandidate)
  {
    switchCandidate = closed;
    switchChangedMs = now;
  }
  if (state.enabled != switchCandidate && now - switchChangedMs >= DEBOUNCE_MS)
  {
    state.enabled = switchCandidate;
    state.hasData = false;
    attitudeInitialized = false;
    sampleUs = attitudeUs = micros();
    lastFrameMs = millis();
    if (!state.enabled) cancelCalibration("Calibration interrompue: interrupteur OFF");
    state.diagnostic = state.enabled ? "En attente de mesures" : "Interrupteur OFF";
    Serial.println(state.enabled ? "Mode IMU (pilotage plateforme)" : "Mode automatique");
  }
  if (!state.enabled) return;
  if (state.calibrationActive && now - calibrationStartedMs >= CAL_TIMEOUT_MS)
  {
    cancelCalibration("Calibration interrompue: delai de 10 secondes depasse");
    state.diagnostic = "Calibration trop longue: cliquer Calibrate";
  }
  if (!state.connected)
  {
    if (now - retryMs < 1000) return;
    Wire.end();
    configureBus();
    ++state.i2cRecoveries;
    state.connected = connectSensor();
    retryMs = millis();
    return;
  }
  const uint32_t currentUs = micros(), elapsed = currentUs - sampleUs;
  if (elapsed < SAMPLE_US) return;
  sampleUs = currentUs;
  uint8_t ready = 0, data[14];
  if (!readRegisters(0x3A, &ready, 1))
  {
    failedFrame();
    return;
  }
  state.interruptStatus = ready;
  if (!(ready & 1))
  {
    if (millis() - lastFrameMs > LOST_FRAME_MS)
    {
      state.diagnostic = "Aucune nouvelle mesure: DATA_RDY reste a zero";
      Serial.printf("MPU: aucune mesure depuis 500 ms, INT_STATUS=0x%02X\n", ready);
      state.connected = false;
      cancelCalibration("Calibration interrompue: absence de mesures");
      retryMs = millis();
    }
    return;
  }
  if (!readRegisters(0x3B, data, sizeof(data)))
  {
    failedFrame();
    return;
  }
  failedFrames = state.i2cConsecutiveFailures = 0;
  lastFrameMs = millis();
  const float ax=signedWord(data)/16384.0f, ay=signedWord(data+2)/16384.0f, az=signedWord(data+4)/16384.0f;
  float gyro[3] = {signedWord(data+8)/131.0f, signedWord(data+10)/131.0f, signedWord(data+12)/131.0f};
  state.accelG[0] = ax; state.accelG[1] = ay; state.accelG[2] = az;
  state.gravityG = sqrtf(ax*ax + ay*ay + az*az);
  for (int i = 0; i < 3; ++i) state.gyroDps[i] = gyro[i];
  ++state.samples;
  state.lastUpdateMs = millis();
  const bool validAcceleration = state.gravityG > 0.5f && state.gravityG < 1.5f;
  if (state.calibrationActive)
  {
    bool moving = false;
    if (previousSample)
    {
      for (int i = 0; i < 3; ++i)
        moving |= fabsf(state.accelG[i] - previousAccel[i]) > CAL_ACCEL_STEP_G
               || fabsf(gyro[i] - previousGyro[i]) > CAL_GYRO_STEP_DPS;
    }
    bool excessiveGyro = false;
    for (float value : gyro) excessiveGyro |= fabsf(value) > 20.0f;
    if (!validAcceleration || moving || excessiveGyro)
    {
      ++rejectedWindow;
      ++rejectedStreak;
      state.diagnostic = !validAcceleration ? "Acceleration invalide: mesure ignoree" :
        excessiveGyro ? "Rotation forte: mesure ignoree" : "Pic de mesure ignore (progression conservee)";
      // One spike no longer destroys a nearly complete calibration window.
      if (rejectedStreak >= CAL_MOTION_STREAK || rejectedWindow > CAL_MAX_REJECTED)
      {
        ++state.calibrationRestarts;
        Serial.printf("Calibration interrompue: %u mesures rejetees, %u consecutives\n", rejectedWindow, rejectedStreak);
        cancelCalibration("Calibration echouee: mouvement durable ou mesures invalides; cliquer Calibrate");
        state.diagnostic = "Calibration echouee: cliquer Calibrate";
      }
    }
    else
    {
      rejectedStreak = 0;
      for (int i = 0; i < 3; ++i)
      {
        accelSums[i] += state.accelG[i];
        accelSquares[i] += state.accelG[i]*state.accelG[i];
        sums[i] += gyro[i];
        squares[i] += gyro[i]*gyro[i];
      }
      state.calibrationSamples = ++calibrationCount;
      state.diagnostic = "Calibration: garder immobile";
      if (calibrationCount >= CALIBRATION_SAMPLES)
      {
        bool stable = true;
        for (int i = 0; i < 3; ++i)
        {
          const float mean = sums[i]/calibrationCount;
          const float gyroVariance = fmaxf(0, squares[i]/calibrationCount - mean*mean);
          const float accelMean = accelSums[i]/calibrationCount;
          const float accelVariance = fmaxf(0, accelSquares[i]/calibrationCount - accelMean*accelMean);
          stable &= gyroVariance <= CAL_GYRO_STD_DPS*CAL_GYRO_STD_DPS
                 && accelVariance <= CAL_ACCEL_STD_G*CAL_ACCEL_STD_G;
          if (gyroVariance > CAL_GYRO_STD_DPS*CAL_GYRO_STD_DPS || accelVariance > CAL_ACCEL_STD_G*CAL_ACCEL_STD_G)
            Serial.printf("Calibration axe %d: ecart-type gyro=%.3f deg/s accel=%.4f g\n", i, sqrtf(gyroVariance), sqrtf(accelVariance));
        }
        if (!stable)
        {
          ++state.calibrationRestarts;
          cancelCalibration("Calibration echouee: mesures instables; cliquer Calibrate");
          state.diagnostic = "Calibration echouee: cliquer Calibrate";
        }
        else
        {
          for (int i = 0; i < 3; ++i) state.gyroBiasDps[i] = bias[i] = sums[i]/calibrationCount;
          const float refAx = accelSums[0]/calibrationCount;
          const float refAy = accelSums[1]/calibrationCount;
          const float refAz = accelSums[2]/calibrationCount;
          state.rollZeroDeg = atan2f(refAy, refAz)/RAD;
          state.pitchZeroDeg = atan2f(-refAx, sqrtf(refAy*refAy + refAz*refAz))/RAD;
          state.calibrated = true;
          state.calibrationActive = false;
          state.calibrationFailed = false;
          Serial.printf("Zero manipulation: roll=%.3f pitch=%.3f deg\n", state.rollZeroDeg, state.pitchZeroDeg);
          attitudeInitialized = false;
          // Start from the averaged gravity reference, not the last noisy frame.
          updateAttitude(refAx, refAy, refAz, 0, 0, 0, 0);
          Serial.printf("Calibration terminee: biais gyro=(%.3f, %.3f, %.3f) deg/s\n", bias[0], bias[1], bias[2]);
        }
      }
    }
    if (validAcceleration && !moving && !excessiveGyro)
    {
      for (int i = 0; i < 3; ++i)
      {
        previousAccel[i] = state.accelG[i];
        previousGyro[i] = gyro[i];
      }
      previousSample = true;
    }
    // Show gravity-based preview during calibration, never pass invalid data to the cube.
    state.hasData = validAcceleration;
    if (validAcceleration)
    {
      state.rollDeg = atan2f(ay, az)/RAD;
      state.pitchDeg = atan2f(-ax, sqrtf(ay*ay + az*az))/RAD;
    }
  }
  if (!state.calibrated && !state.calibrationActive)
  {
    state.hasData = validAcceleration;
    if (validAcceleration) {
      state.rollDeg = atan2f(ay, az)/RAD;
      state.pitchDeg = atan2f(-ax, sqrtf(ay*ay+az*az))/RAD;
    }
    if (!state.calibrationFailed) state.diagnostic = "Cliquer sur Calibrate pour activer le pilotage IMU";
  }
  if (state.calibrated && state.gravityG < 0.1f)
  {
    state.hasData = false;
    state.diagnostic = "Acceleration nulle: mesure invalide";
  }
  else if (state.calibrated)
  {
    const uint32_t attitudeElapsed = currentUs - attitudeUs;
    if (attitudeElapsed > 100000) attitudeInitialized = false;
    updateAttitude(ax, ay, az, gyro[0]-bias[0], gyro[1]-bias[1], gyro[2]-bias[2],
      attitudeElapsed > 100000 ? 0.01f : attitudeElapsed*1e-6f);
    state.diagnostic = "Mesures actives (fusion accel + gyro)";
  }
  attitudeUs = currentUs;
  if (millis() - diagnosticMs >= 1000)
  {
    diagnosticMs = millis();
    Serial.printf("MPU #%lu: accel=(%.3f %.3f %.3f) g |a|=%.3f gyro=(%.3f %.3f %.3f) deg/s INT=0x%02X cal=%u/200 resets=%lu: %s\n",
      (unsigned long)state.samples, ax, ay, az, state.gravityG, gyro[0], gyro[1], gyro[2], ready,
      state.calibrationSamples, (unsigned long)state.calibrationRestarts, state.diagnostic);
  }
}
bool calibrate()
{
  if (!state.enabled || !state.connected) return false;
  if (!state.calibrationActive) previousCalibrationValid = state.calibrated;
  resetCalibration();
  calibrationStartedMs = millis();
  state.calibrated = false;
  state.calibrationActive = true; // The only activation point: HTTP /imu/calibrate.
  state.calibrationFailed = false;
  state.calibrationRestarts = 0;
  sampleUs = attitudeUs = micros();
  state.diagnostic = "Calibration demandee: garder immobile";
  Serial.println("Calibration manuelle demandee depuis le navigateur.");
  return true;
}
bool isEnabled() { return state.enabled; }
State getState() { return state; }
}
