#include "platform_control.h"
#include "servo_control.h"
#include "servo_calibration.h"
#include "kinematics.h"
#include <math.h>
#include <stdint.h>

namespace PlatformControl
{
namespace
{
State state;
float height = IMU_Z_MM;
unsigned long lastControlMs = 0, lastLogMs = 0;
uint32_t lastSample = UINT32_MAX;
void hold(const char* message)
{
  state.applied = false;
  state.ikError = 0; state.failedLeg = -1;
  state.message = message;
  lastSample = UINT32_MAX;
}
}
State getState() { return state; }
float heightMm() { return height; }
bool setHeightMm(float z)
{
  if (ServoCalibration::isActive() || !isfinite(z) || z < 0) return false;
  const auto demo = ServoControl::getDemoPose();
  const float roll = state.imuMode ? state.roll : demo.roll;
  const float pitch = state.imuMode ? state.pitch : demo.pitch;
  if (!Kinematics::compute_kinematics(z, roll, pitch)) return false;
  const float q[] = {float(Kinematics::q1),float(Kinematics::q2),float(Kinematics::q3)};
  for (int i=1;i<=3;++i) {
    const auto c = ServoCalibration::getCalibration(i); int ticks;
    if (!c.valid || c.joint<1 || c.joint>3 || !ServoCalibration::angleToTicks(c,q[c.joint-1],ticks)) return false;
  }
  height = z;
  lastSample = UINT32_MAX; // Height changes also apply if the latest IMU sample is unchanged.
  return true;
}
void update(const ImuData::State& imu)
{
  const bool modeChanged = state.imuMode != imu.enabled;
  state.imuMode = imu.enabled;
  if (modeChanged) lastSample = UINT32_MAX;
  if (ServoCalibration::isActive())
  {
    if (ServoControl::isDemoRunning()) ServoControl::stopDemo();
    hold("Calibration servos prioritaire");
    return;
  }
  if (!imu.enabled)
  {
    state.applied = false;
    state.ikError = 0; state.failedLeg = -1;
    state.message = "Mode automatique";
    lastSample = UINT32_MAX;
    if (!ServoControl::isDemoRunning()) ServoControl::startDemo();
    ServoControl::updateDemo();
    return;
  }
  if (ServoControl::isDemoRunning()) ServoControl::stopDemo();
  if (!imu.connected) { hold("MPU absent: position maintenue"); return; }
  if (!imu.calibrated) { hold("Calibration IMU: position maintenue"); return; }
  if (!imu.hasData || millis()-imu.lastUpdateMs >= MAX_IMU_AGE_MS)
  { hold("Mesures IMU absentes ou perimees: position maintenue"); return; }
  if (!isfinite(imu.rollDeg) || !isfinite(imu.pitchDeg))
  { hold("Angles IMU invalides: position maintenue"); return; }
  if (imu.samples == lastSample || (!modeChanged && millis()-lastControlMs < CONTROL_PERIOD_MS)) return;
  lastControlMs = millis();
  lastSample = imu.samples;
  state.ikError = 0; state.failedLeg = -1;
  state.z = height;
  state.roll = IMU_ROLL_SIGN*imu.rollDeg;
  state.pitch = IMU_PITCH_SIGN*imu.pitchDeg;
  if (!Kinematics::compute_kinematics(state.z, state.roll, state.pitch))
  {
    state.applied = false;
    state.ikError = int(Kinematics::lastError);
    state.failedLeg = Kinematics::failedLeg;
    state.message = "Consigne IMU inatteignable: position maintenue";
    if (millis()-lastLogMs >= 1000)
    {
      lastLogMs = millis();
      Serial.printf("IMU -> IK: erreur=%d bras=%d roll=%.2f pitch=%.2f\n", state.ikError, state.failedLeg, state.roll, state.pitch);
    }
    return;
  }
  if (!ServoControl::setKinematicAngles(float(Kinematics::q1), float(Kinematics::q2), float(Kinematics::q3)))
  { state.applied = false; state.message = "Calibration servo manquante ou angle hors plage: position maintenue"; return; }
  state.applied = true;
  state.message = "Pilotage IMU actif (roll/pitch -> IK -> PWM)";
}
}
