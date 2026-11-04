#include <cassert>
#include <cstdio>
#include "../../src/tch_screen.cpp"
uint32_t fakeUs = 0;
bool pressed = false;
int levels[40] = {};
void advance(unsigned ms) {
  for (unsigned i=0; i<ms; ++i) { fakeUs += 1000; TchScreen::update(); }
}
int main() {
  TchScreen::begin();
  advance(100);
  assert(!TchScreen::getState().present);
  assert(TchScreen::getState().validSamples == 0);
  pressed = true;
  advance(70);
  auto valid = TchScreen::getState();
  assert(valid.present && valid.measurementValid);
  assert(valid.rawX == 900 && valid.rawY == 2000);
  pressed = false;
  advance(15);
  auto held = TchScreen::getState();
  assert(held.present && !held.measurementValid);
  assert(held.lastValidMs == valid.lastValidMs);
  assert(held.validSamples == valid.validSamples);
  assert(held.rawX == valid.rawX && held.rawY == valid.rawY);
  // Resume inside the grace period: one valid cycle restores fresh coordinates.
  pressed = true;
  advance(30);
  assert(TchScreen::getState().present && TchScreen::getState().measurementValid);
  assert(TchScreen::getState().validSamples > valid.validSamples);
  pressed = false;
  advance(60);
  assert(!TchScreen::getState().present);
  assert(!TchScreen::getState().measurementValid);
  pressed = true;
  advance(70);
  assert(TchScreen::getState().present);
  std::puts("Touch acquisition, dropout, recovery and removal: PASS");
}
