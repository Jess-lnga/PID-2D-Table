#ifndef SERVO_CONTROL_H
#define SERVO_CONTROL_H

#include <Arduino.h>

namespace ServoControl
{
constexpr int SERVO1_PIN = 25;
constexpr int SERVO2_PIN = 26;
constexpr int SERVO3_PIN = 27;

constexpr int SERVO_FREQ = 50;
constexpr int SERVO_RESOLUTION = 16;

constexpr int SERVO1_MAX_US = 1510;
constexpr int SERVO1_MIN_US = 1090;

constexpr int SERVO2_MAX_US = 1465;
constexpr int SERVO2_MIN_US = 965;

constexpr int SERVO3_MAX_US = 1550;
constexpr int SERVO3_MIN_US = 1130;

constexpr int SERVO1_MID_US = (SERVO1_MAX_US + SERVO1_MIN_US) / 2;
constexpr int SERVO2_MID_US = (SERVO2_MAX_US + SERVO2_MIN_US) / 2;
constexpr int SERVO3_MID_US = (SERVO3_MAX_US + SERVO3_MIN_US) / 2;

constexpr float WAVE_CENTER_RATIO = 0.40f;

constexpr int SERVO1_WAVE_CENTER_US =
  SERVO1_MIN_US + (int)((SERVO1_MAX_US - SERVO1_MIN_US) * WAVE_CENTER_RATIO);

constexpr int SERVO2_WAVE_CENTER_US =
  SERVO2_MIN_US + (int)((SERVO2_MAX_US - SERVO2_MIN_US) * WAVE_CENTER_RATIO);

constexpr int SERVO3_WAVE_CENTER_US =
  SERVO3_MIN_US + (int)((SERVO3_MAX_US - SERVO3_MIN_US) * WAVE_CENTER_RATIO);

void begin();

int getServoMicroseconds(int servoNumber);
bool setServoMicroseconds(int servoNumber, int pulseWidth);

void writeAllServos();

bool isDemoRunning();
void startDemo();
void stopDemo();
void updateDemo();
} // namespace ServoControl

#endif
