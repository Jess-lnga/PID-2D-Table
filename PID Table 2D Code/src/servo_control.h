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

// Platform pose wave: z is mm above h0, angles are degrees.
constexpr float WAVE_Z_MM = 0.0f;
constexpr float WAVE_ROLL_AMPLITUDE_DEG = 5.0f;
constexpr float WAVE_PITCH_AMPLITUDE_DEG = 5.0f;
constexpr uint32_t WAVE_PERIOD_MS = 4500;
constexpr uint32_t WAVE_UPDATE_MS = 20;
constexpr uint32_t WAVE_START_MS = 1500;

void begin();

int getServoMicroseconds(int servoNumber);
bool setServoMicroseconds(int servoNumber, int pulseWidth);

// Exact 16-bit LEDC duty ticks at 50 Hz (0..65535).
int getServoTicks(int servoNumber);
bool setServoTicks(int servoNumber, int ticks);
// Calibrated angle commands, including stored q1/q2/q3 assignment.
bool setServoAngle(int servoNumber, float angleDeg);
bool setKinematicAngles(float q1, float q2, float q3);
void writeAllServos();

bool isDemoRunning();
bool isDemoWaitingForCalibration();
const char* demoStatus();
struct DemoPose { float z, roll, pitch; bool valid; };
DemoPose getDemoPose();
void startDemo();
void stopDemo();
void updateDemo();
} // namespace ServoControl

#endif
