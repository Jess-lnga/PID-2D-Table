#include "imu_data.h"

namespace ImuData
{
namespace
{
State state = {
  false,
  false,
  0.0f,
  0.0f,
  0
};
} // namespace

void setTransmissionEnabled(bool enabled)
{
  state.transmissionEnabled = enabled;

  if (!enabled)
  {
    state.hasData = false;
    state.pitchDeg = 0.0f;
    state.rollDeg = 0.0f;
    state.lastUpdateMs = 0;
  }
}

bool isTransmissionEnabled()
{
  return state.transmissionEnabled;
}

void update(float pitchDeg, float rollDeg)
{
  if (!state.transmissionEnabled)
  {
    return;
  }

  state.hasData = true;
  state.pitchDeg = pitchDeg;
  state.rollDeg = rollDeg;
  state.lastUpdateMs = millis();
}

State getState()
{
  return state;
}
} // namespace ImuData

