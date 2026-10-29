#ifndef PLATFORM_CONTROL_H
#define PLATFORM_CONTROL_H
#include "imu_data.h"
namespace PlatformControl
{
constexpr float IMU_Z_MM = 0.0f; // Platform center height = h0 + this value.
constexpr float IMU_ROLL_SIGN = 1.0f;
constexpr float IMU_PITCH_SIGN = 1.0f;
constexpr unsigned long CONTROL_PERIOD_MS = 20;
constexpr unsigned long MAX_IMU_AGE_MS = 250;
struct State
{
  bool imuMode = false;
  bool applied = false; // Last IMU command accepted, not measured platform feedback.
  float z = IMU_Z_MM, roll = 0, pitch = 0;
  int ikError = 0, failedLeg = -1;
  const char* message = "Mode automatique";
};
void update(const ImuData::State& imu);
State getState();
float heightMm();
bool setHeightMm(float z);
}
#endif
