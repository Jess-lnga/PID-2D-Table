#include "imu_data.h"
#include <Wire.h>
#include <math.h>

namespace ImuData
{
namespace
{
constexpr uint32_t SAMPLE_US = 10000;
constexpr uint32_t DEBOUNCE_MS = 30;
constexpr unsigned CALIBRATION_SAMPLES = 200;
constexpr float RAD = PI / 180.0f;
State state;
bool switchCandidate = false;
unsigned long switchChangedMs = 0, retryMs = 0, lastFrameMs = 0;
uint32_t sampleUs = 0;
unsigned calibrationCount = 0;
float bias[3] = {}, sums[3] = {}, squares[3] = {};
float accelSums[3] = {};
float previousAccel[3] = {}, previousGyro[3] = {};
bool previousSample = false;
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
  Wire.beginTransmission(state.address);
  Wire.write(reg);
  const uint8_t error = Wire.endTransmission(false);
  if (error != 0) { reportI2cError(reg, error); return false; }
  const size_t received = Wire.requestFrom(state.address, (uint8_t)count);
  if (received != count)
  {
    while (Wire.available()) Wire.read();
    reportI2cError(reg, 0xFF); // Short read, not a Wire transmission code.
    return false;
  }
  for (size_t i = 0; i < count; ++i) data[i] = Wire.read();
  return true;
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
void resetCalibration()
{
  calibrationCount = 0;
  for (int i = 0; i < 3; ++i) sums[i] = squares[i] = accelSums[i] = 0;
  state.calibrationSamples = 0;
  state.pitchZeroDeg = state.rollZeroDeg = 0;
  previousSample = false;
  state.calibrated = false;
  state.hasData = false;
  attitudeInitialized = false;
}
bool connectSensor()
{
  resetCalibration();
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
    state.diagnostic = "Calibration: garder immobile";
    Serial.printf("MPU detecte: adresse 0x%02X, WHO_AM_I 0x%02X. Garder immobile 2 s.\n", address, id);
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
  // External 10 kOhm pull-down; internal pull-down also supports a bare switch.
  pinMode(MODE_SWITCH_PIN, INPUT_PULLDOWN);
  switchCandidate = state.enabled = digitalRead(MODE_SWITCH_PIN) == HIGH;
  switchChangedMs = millis();
  Wire.begin(21, 22);
  Wire.setClock(100000);
  Wire.setTimeOut(20);
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
    if (!state.calibrated) resetCalibration();
    state.diagnostic = state.enabled ? "En attente de mesures" : "Interrupteur OFF";
    Serial.println(state.enabled ? "Mode IMU (affichage uniquement)" : "Mode automatique");
  }
  if (!state.enabled) return;
  if (!state.connected)
  {
    if (now - retryMs < 1000) return;
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
    state.connected = false;
    resetCalibration();
    retryMs = millis();
    return;
  }
  state.interruptStatus = ready;
  if (!(ready & 1))
  {
    if (millis() - lastFrameMs > 500)
    {
      state.diagnostic = "Aucune nouvelle mesure: DATA_RDY reste a zero";
      Serial.printf("MPU: aucune mesure depuis 500 ms, INT_STATUS=0x%02X\n", ready);
      state.connected = false;
      resetCalibration();
      retryMs = millis();
    }
    return;
  }
  if (!readRegisters(0x3B, data, sizeof(data)))
  {
    state.connected = false;
    resetCalibration();
    retryMs = millis();
    return;
  }
  lastFrameMs = millis();
  const float ax=signedWord(data)/16384.0f, ay=signedWord(data+2)/16384.0f, az=signedWord(data+4)/16384.0f;
  float gyro[3] = {signedWord(data+8)/131.0f, signedWord(data+10)/131.0f, signedWord(data+12)/131.0f};
  state.accelG[0] = ax; state.accelG[1] = ay; state.accelG[2] = az;
  state.gravityG = sqrtf(ax*ax + ay*ay + az*az);
  for (int i = 0; i < 3; ++i) state.gyroDps[i] = gyro[i];
  ++state.samples;
  state.lastUpdateMs = millis();
  const bool validAcceleration = state.gravityG > 0.5f && state.gravityG < 1.5f;
  if (!state.calibrated)
  {
    bool moving = false;
    if (previousSample)
    {
      for (int i = 0; i < 3; ++i)
        moving |= fabsf(state.accelG[i] - previousAccel[i]) > 0.05f
               || fabsf(gyro[i] - previousGyro[i]) > 1.5f;
    }
    bool excessiveGyro = false;
    for (float value : gyro) excessiveGyro |= fabsf(value) > 20.0f;
    if (!validAcceleration || moving || excessiveGyro)
    {
      ++state.calibrationRestarts;
      resetCalibration();
      state.diagnostic = !validAcceleration ? "Acceleration invalide: verifier les valeurs brutes" :
        excessiveGyro ? "Rotation trop forte ou biais gyro excessif" : "Mouvement detecte: calibration recommencee";
    }
    else
    {
      for (int i = 0; i < 3; ++i)
      {
        accelSums[i] += state.accelG[i];
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
          stable &= squares[i]/calibrationCount - mean*mean < 0.25f;
        }
        if (!stable)
        {
          ++state.calibrationRestarts;
          resetCalibration();
          state.diagnostic = "Gyro instable: calibration recommencee";
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
          Serial.printf("Zero manipulation: roll=%.3f pitch=%.3f deg\n", state.rollZeroDeg, state.pitchZeroDeg);
          attitudeInitialized = false;
          Serial.printf("Calibration terminee: biais gyro=(%.3f, %.3f, %.3f) deg/s\n", bias[0], bias[1], bias[2]);
        }
      }
    }
    for (int i = 0; i < 3; ++i)
    {
      previousAccel[i] = state.accelG[i];
      previousGyro[i] = gyro[i];
    }
    previousSample = true;
    // Show gravity-based preview during calibration, never pass invalid data to the cube.
    state.hasData = validAcceleration;
    if (validAcceleration)
    {
      state.rollDeg = atan2f(ay, az)/RAD;
      state.pitchDeg = atan2f(-ax, sqrtf(ay*ay + az*az))/RAD;
    }
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
  resetCalibration();
  state.calibrationRestarts = 0;
  for (int i = 0; i < 3; ++i) bias[i] = state.gyroBiasDps[i] = 0;
  sampleUs = attitudeUs = micros();
  state.diagnostic = "Calibration demandee: garder immobile";
  Serial.println("Calibration manuelle demandee depuis le navigateur.");
  return true;
}
bool isEnabled() { return state.enabled; }
State getState() { return state; }
}
