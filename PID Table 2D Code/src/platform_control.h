#ifndef PLATFORM_CONTROL_H
#define PLATFORM_CONTROL_H
#include "imu_data.h"
namespace PlatformControl
{
constexpr float IMU_Z_MM = 25.0f; // Platform center height = h0 + this value.
constexpr float IMU_ROLL_SIGN = 1.0f;
constexpr float IMU_PITCH_SIGN = 1.0f;
constexpr unsigned long CONTROL_PERIOD_MS = 20;
constexpr unsigned long MAX_IMU_AGE_MS = 250;
constexpr float MANUAL_LIMIT_DEG = 10.0f;
enum class NonImuMode { Wave, Manual };
struct State
{
  bool imuMode = false;
  bool applied = false; // Command accepted, not measured platform feedback.
  NonImuMode nonImuMode = NonImuMode::Wave;
  float manualRoll = 0, manualPitch = 0;
  float z = IMU_Z_MM, roll = 0, pitch = 0;
  int ikError = 0, failedLeg = -1;
  const char* message = "Mode automatique";
};
void update(const ImuData::State& imu);
State getState();
float heightMm();
bool setHeightMm(float z);
// Validate the full pose before changing mode/targets. Caller holds RuntimeLock.
bool setNonImuMode(NonImuMode mode, float roll, float pitch);
}
#endif
