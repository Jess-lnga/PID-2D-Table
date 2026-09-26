#ifndef IMU_DATA_H
#define IMU_DATA_H

#include <Arduino.h>

namespace ImuData
{
struct State
{
  bool transmissionEnabled;
  bool hasData;
  float pitchDeg;
  float rollDeg;
  unsigned long lastUpdateMs;
};

void setTransmissionEnabled(bool enabled);
bool isTransmissionEnabled();

void update(float pitchDeg, float rollDeg);
State getState();
} // namespace ImuData

#endif
