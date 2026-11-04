#pragma once
#include <cstdint>
#include <algorithm>
constexpr int INPUT=0, OUTPUT=1, INPUT_PULLDOWN=2, LOW=0, HIGH=1, ADC_11db=3;
extern uint32_t fakeUs;
extern bool pressed;
extern int levels[40];
inline unsigned long millis() { return fakeUs/1000; }
inline uint32_t micros() { return fakeUs; }
inline void pinMode(int,int) {}
inline void digitalWrite(int pin,int level) { levels[pin]=level; }
inline int digitalRead(int) { return pressed && levels[33] == HIGH && levels[23] == HIGH; }
inline int analogRead(int pin) { return pin == 33 ? 900 : 2000; }
inline void analogReadResolution(int) {}
inline void analogSetPinAttenuation(int,int) {}
template<class T> T constrain(T v,T lo,T hi) { return std::max(lo,std::min(v,hi)); }
