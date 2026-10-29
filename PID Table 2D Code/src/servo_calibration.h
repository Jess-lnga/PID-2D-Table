#ifndef SERVO_CALIBRATION_H
#define SERVO_CALIBRATION_H
#include <Arduino.h>

namespace ServoCalibration
{
constexpr float ANGLES[4] = {-30, 0, 45, 90};
// Electrical adjustment envelope; tune these for your actual servo/linkage.
constexpr int MIN_PULSE_US = 500;
constexpr int MAX_PULSE_US = 2500;
constexpr int MIN_TICKS = (MIN_PULSE_US * 65535 + 19999) / 20000;
constexpr int MAX_TICKS = MAX_PULSE_US * 65535 / 20000;

struct Calibration
{
  uint8_t valid = 0;
  uint8_t joint = 0; // q1/q2/q3, zero when not assigned.
  uint16_t ticks[4] = {};
};
enum class Stage { Choosing, Adjusting, Testing, Validate, Paused };
struct State
{
  bool menuActive = false;
  Stage stage = Stage::Choosing;
  uint32_t session = 0;
  uint32_t revision = 0;
  int servo = 0;
  int point = 0;
  float targetDeg = -30;
  Calibration draft;
};
void begin();
void update();
bool isActive();
State getState();
Calibration getCalibration(int servo);
const char* error();
bool openMenu();
bool checkCommand(uint32_t session, uint32_t revision);
void heartbeat(uint32_t session);
bool chooseServo(int servo);
bool adjust(int deltaTicks);
bool done();
bool testAgain();
bool save(int joint);
// Change only the saved q assignment, without moving or recalibrating the servo.
bool assignJoint(int servo, int joint);
bool backToSelection();
bool closeMenu();
bool angleToTicks(const Calibration& calibration, float angleDeg, int& ticks);
bool angleToTicks(int servo, float angleDeg, int& ticks);
bool setServoAngle(int servo, float angleDeg);
bool setJointAngles(float q1, float q2, float q3);
}
#endif
