#ifndef TCH_SCREEN_H
#define TCH_SCREEN_H
#include <Arduino.h>

namespace TchScreen {
constexpr int P1 = 33;
constexpr int P2 = 13;
constexpr int P3 = 23;
constexpr int P4 = 14;
// Extra wire: connect GPIO34 to P2 (GPIO13). ADC2 is unavailable with Wi-Fi.
constexpr int P2_SENSE = 34;
// X endpoints: layer 2-4 at left/right. Y endpoints: layer 1-3 at top/bottom.
// Replace these endpoints after measuring the useful area of your panel.
constexpr int X_MIN = 0, X_MAX = 4095;
constexpr int Y_MIN = 0, Y_MAX = 4095;
struct State {
  bool present = false;
  // Latest completed pre-measurement contact test, also exposed when absent.
  int contactLow13 = 0, contactHigh13 = 0;
  int contactLow34 = 0, contactHigh34 = 0;
  int rawX = 0, rawY = 0;
  float x = 0, y = 0; // Normalized 0..1; X = layer 2-4, Y = layer 1-3.
  uint32_t samples = 0;
  unsigned long lastUpdateMs = 0;
};
void begin();
// Call frequently from the control loop, under RuntimeLock like getState().
void update();
State getState();
}
#endif
