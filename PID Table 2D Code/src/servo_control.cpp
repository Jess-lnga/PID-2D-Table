#include "servo_control.h"
#include "servo_calibration.h"
#include "kinematics.h"
#include "platform_control.h"

#include <math.h>

namespace ServoControl
{
namespace
{
bool demoRunning = false;
bool waveReady = false;
bool waitingCalibration = false;
const char* waveStatus = "Vague suspendue";
DemoPose wavePose = {PlatformControl::heightMm(), 0, 0, false};
unsigned long lastWaveUpdate = 0, waveStart = 0;
uint32_t wavePhaseMs = 0;
int waveStartTicks[3] = {};
int servo1_us = SERVO1_MID_US;
int servo2_us = SERVO2_MID_US;
int servo3_us = SERVO3_MID_US;
int rawTicks[3] = {};

#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
constexpr bool USE_LEDC_PIN_API = true;
#else
constexpr bool USE_LEDC_PIN_API = false;
constexpr int SERVO1_CHANNEL = 0;
constexpr int SERVO2_CHANNEL = 1;
constexpr int SERVO3_CHANNEL = 2;
#endif

uint32_t microsecondsToDuty(int pulse_us)
{
  return (uint32_t)(((uint64_t)pulse_us * 65535ULL) / 20000ULL);
}

int servoPin(int servoNumber)
{
  switch (servoNumber)
  {
    case 1:
      return SERVO1_PIN;
    case 2:
      return SERVO2_PIN;
    case 3:
      return SERVO3_PIN;
    default:
      return -1;
  }
}

int servoChannel(int servoNumber)
{
  if (USE_LEDC_PIN_API)
  {
    return servoPin(servoNumber);
  }

#if !(defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3)
  switch (servoNumber)
  {
    case 1:
      return SERVO1_CHANNEL;
    case 2:
      return SERVO2_CHANNEL;
    case 3:
      return SERVO3_CHANNEL;
    default:
      return -1;
  }
#else
  return -1;
#endif
}

void setServoPulseByNumber(int servoNumber, int pulse_us)
{
  uint32_t duty = microsecondsToDuty(pulse_us);
  rawTicks[servoNumber-1] = duty;
  ledcWrite(servoChannel(servoNumber), duty);
}

} // namespace

void begin()
{
  bool servo1_OK = false;
  bool servo2_OK = false;
  bool servo3_OK = false;

  if (USE_LEDC_PIN_API)
  {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
    servo1_OK = ledcAttach(SERVO1_PIN, SERVO_FREQ, SERVO_RESOLUTION);
    servo2_OK = ledcAttach(SERVO2_PIN, SERVO_FREQ, SERVO_RESOLUTION);
    servo3_OK = ledcAttach(SERVO3_PIN, SERVO_FREQ, SERVO_RESOLUTION);
#endif
  }
  else
  {
#if !(defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3)
    ledcSetup(SERVO1_CHANNEL, SERVO_FREQ, SERVO_RESOLUTION);
    ledcSetup(SERVO2_CHANNEL, SERVO_FREQ, SERVO_RESOLUTION);
    ledcSetup(SERVO3_CHANNEL, SERVO_FREQ, SERVO_RESOLUTION);

    ledcAttachPin(SERVO1_PIN, SERVO1_CHANNEL);
    ledcAttachPin(SERVO2_PIN, SERVO2_CHANNEL);
    ledcAttachPin(SERVO3_PIN, SERVO3_CHANNEL);

    servo1_OK = true;
    servo2_OK = true;
    servo3_OK = true;
#endif
  }

  if (!servo1_OK || !servo2_OK || !servo3_OK)
  {
    Serial.println("ERREUR configuration PWM");
  }
  else
  {
    Serial.println("PWM initialise");
  }

  servo1_us = SERVO1_MID_US;
  servo2_us = SERVO2_MID_US;
  servo3_us = SERVO3_MID_US;

  writeAllServos();
}

int getServoMicroseconds(int servoNumber)
{
  switch (servoNumber)
  {
    case 1:
      return servo1_us;
    case 2:
      return servo2_us;
    case 3:
      return servo3_us;
    default:
      return 0;
  }
}

bool setServoMicroseconds(int servoNumber, int pulseWidth)
{
  switch (servoNumber)
  {
    case 1:
      servo1_us = constrain(pulseWidth, SERVO1_MIN_US, SERVO1_MAX_US);
      setServoPulseByNumber(1, servo1_us);
      return true;

    case 2:
      servo2_us = constrain(pulseWidth, SERVO2_MIN_US, SERVO2_MAX_US);
      setServoPulseByNumber(2, servo2_us);
      return true;

    case 3:
      servo3_us = constrain(pulseWidth, SERVO3_MIN_US, SERVO3_MAX_US);
      setServoPulseByNumber(3, servo3_us);
      return true;

    default:
      return false;
  }
}

int getServoTicks(int servoNumber)
{
  return servoNumber >= 1 && servoNumber <= 3 ? rawTicks[servoNumber-1] : 0;
}
bool setServoTicks(int servoNumber, int ticks)
{
  if (servoNumber < 1 || servoNumber > 3 || ticks < ServoCalibration::MIN_TICKS || ticks > ServoCalibration::MAX_TICKS) return false;
  rawTicks[servoNumber-1] = ticks;
  int pulse = (int)((uint64_t(ticks)*20000 + 32767)/65535);
  if (servoNumber == 1) servo1_us = pulse;
  if (servoNumber == 2) servo2_us = pulse;
  if (servoNumber == 3) servo3_us = pulse;
  ledcWrite(servoChannel(servoNumber), ticks);
  return true;
}
bool setServoAngle(int servo, float angle) { return ServoCalibration::setServoAngle(servo, angle); }
bool setKinematicAngles(float q1, float q2, float q3) { return ServoCalibration::setJointAngles(q1, q2, q3); }

void writeAllServos()
{
  setServoPulseByNumber(1, servo1_us);
  setServoPulseByNumber(2, servo2_us);
  setServoPulseByNumber(3, servo3_us);
}

bool isDemoRunning()
{
  return demoRunning;
}

void startDemo()
{
  if (ServoCalibration::isActive()) return;
  demoRunning = true;
  waveReady = false;
  wavePhaseMs = 0;
  lastWaveUpdate = millis();
  waitingCalibration = false;
  waveStatus = "Vague roll/pitch: initialisation";
  wavePose = {PlatformControl::heightMm(), 0, 0, false};
  Serial.println("Vague roll/pitch via cinematique inverse et calibrations.");
}

void stopDemo()
{
  demoRunning = false;
  waveReady = false;
  waitingCalibration = false;
  wavePose.valid = false;
  waveStatus = "Vague suspendue (IMU ou calibration)";
}

const char* demoStatus() { return waveStatus; }
DemoPose getDemoPose() { return wavePose; }
bool isDemoWaitingForCalibration() { return demoRunning && waitingCalibration; }

void updateDemo()
{
  if (!demoRunning || ServoCalibration::isActive()) return;
  const unsigned long now = millis();
  const uint32_t elapsed = now-lastWaveUpdate;
  if (elapsed < WAVE_UPDATE_MS) return;
  lastWaveUpdate = now;
  if (waveReady) wavePhaseMs = (wavePhaseMs+elapsed%WAVE_PERIOD_MS)%WAVE_PERIOD_MS;
  const float phase = 2.0f*PI*float(wavePhaseMs)/WAVE_PERIOD_MS;
  wavePose = {PlatformControl::heightMm(), WAVE_ROLL_AMPLITUDE_DEG*sinf(phase), WAVE_PITCH_AMPLITUDE_DEG*cosf(phase), false};
  waitingCalibration = false;
  if (!Kinematics::compute_kinematics(wavePose.z, wavePose.roll, wavePose.pitch))
  {
    waveStatus = "Pose inatteignable: PWM maintenu (voir erreur IK en serie)";
    static unsigned long lastErrorLog = 0;
    if (now-lastErrorLog >= 1000) {
      Serial.printf("IK erreur=%d bras=%d z=%.2f roll=%.2f pitch=%.2f\n", int(Kinematics::lastError), Kinematics::failedLeg, wavePose.z, wavePose.roll, wavePose.pitch);
      lastErrorLog = now;
    }
    return;
  }
  const float angles[3] = {float(Kinematics::q1), float(Kinematics::q2), float(Kinematics::q3)};
  int targets[3];
  for (int i=0;i<3;++i)
  {
    const auto calibration = ServoCalibration::getCalibration(i+1);
    if (!calibration.valid || calibration.joint<1 || calibration.joint>3
        || !ServoCalibration::angleToTicks(calibration, angles[calibration.joint-1], targets[i]))
    {
      waitingCalibration = true;
      waveStatus = "Calibration manquante ou angle hors plage: PWM maintenu";
      return;
    }
  }
  if (!waveReady)
  {
    waveReady = true;
    waveStart = now;
    for (int i=0;i<3;++i) waveStartTicks[i] = rawTicks[i];
    Serial.println("Vague plateforme active: roll/pitch +/-5 deg, periode 4.5 s.");
  }
  wavePose.valid = true;
  waveStatus = "Vague plateforme active (roll/pitch via IK)";
  // Ease into the wave from the last commanded positions on start/resume.
  const unsigned long rampElapsed = now-waveStart;
  float blend = rampElapsed >= WAVE_START_MS ? 1.0f : float(rampElapsed)/WAVE_START_MS;
  blend = blend*blend*(3-2*blend);
  for (int i=0;i<3;++i)
  {
    const int ticks = lroundf(waveStartTicks[i]+blend*(targets[i]-waveStartTicks[i]));
    setServoTicks(i+1, ticks);
  }
}
} // namespace ServoControl
