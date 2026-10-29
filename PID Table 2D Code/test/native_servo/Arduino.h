#pragma once
#include <cstdint>
#include <cstddef>
#include <cstdio>
#include <cmath>
#include <algorithm>
using std::isfinite;
constexpr float PI = 3.14159265358979323846f;
inline unsigned long fakeMs = 0;
inline unsigned long millis() { return fakeMs; }
template<class T> T constrain(T v,T lo,T hi) { return std::max(lo,std::min(v,hi)); }
struct SerialStub {
  template<class... T> void printf(const char*,T...) {}
  template<class T> void print(T) {}
  template<class T> void println(T) {}
  void println() {}
};
inline SerialStub Serial;
inline uint32_t pwmTicks[32] = {};
inline uint32_t pwmWrites = 0;
inline bool ledcAttach(int,int,int) { return true; }
inline void ledcAttachPin(int,int) {}
inline double ledcSetup(int,double frequency,int) { return frequency; }
inline void ledcWrite(int channel,uint32_t ticks) { pwmTicks[channel]=ticks; ++pwmWrites; }
