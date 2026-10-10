#ifndef IMU_DATA_H
#define IMU_DATA_H
#include <Arduino.h>
namespace ImuData
{
constexpr int MODE_SWITCH_PIN = 32;
struct State
{
  bool enabled = false;
  bool connected = false;
  bool calibrated = false;
  bool hasData = false;
  uint8_t address = 0;
  uint8_t deviceId = 0;
  float pitchDeg = 0;
  float rollDeg = 0;
  float pitchZeroDeg = 0;
  float rollZeroDeg = 0;
  unsigned long lastUpdateMs = 0;
  uint32_t samples = 0;
  uint32_t i2cErrors = 0;
  uint32_t calibrationRestarts = 0;
  unsigned calibrationSamples = 0;
  uint8_t interruptStatus = 0;
  uint8_t lastRegister = 0;
  uint8_t lastI2cError = 0;
  float accelG[3] = {};
  float gyroDps[3] = {};
  float gyroBiasDps[3] = {};
  float gravityG = 0;
  const char* diagnostic = "Interrupteur OFF";
};
void begin();
void update();
// Restart gyro bias and pitch/roll reference calibration only while the IMU mode is active and connected.
bool calibrate();
bool isEnabled();
State getState();
}
#endif
