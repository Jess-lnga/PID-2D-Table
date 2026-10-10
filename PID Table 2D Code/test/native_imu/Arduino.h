#pragma once
#include <cstdint>
#include <cstddef>
#include <cstdio>
#include <initializer_list>
#include <algorithm>
constexpr float PI = 3.14159265358979323846f;
constexpr int INPUT_PULLDOWN=1, HIGH=1;
extern uint32_t fakeUs;
extern int fakeSwitch;
inline unsigned long millis() { return fakeUs/1000; }
inline uint32_t micros() { return fakeUs; }
inline void delay(unsigned long ms) { fakeUs += ms*1000; }
inline void pinMode(int,int) {}
inline int digitalRead(int) { return fakeSwitch; }
template<class T> T constrain(T v,T lo,T hi) { return std::max(lo,std::min(v,hi)); }
struct SerialStub {
  template<class... T> void printf(const char*,T...) {}
  void println(const char*) {}
};
extern SerialStub Serial;
