#pragma once
#include "Arduino.h"
#include <cstring>
struct WireStub {
  uint8_t regs[256] = {}, buffer[14] = {}, reg=0;
  bool first=true, ready=true, shortRead=false, fail=false, ignoreConfig=false;
  unsigned index=0, availableBytes=0;
  void begin(int,int) {}
  void setClock(int) {}
  void setTimeOut(int) {}
  void beginTransmission(uint8_t) { first=true; }
  void write(uint8_t value) {
    if(first) {reg=value;first=false;}
    else if(!ignoreConfig) {regs[reg]=value;}
  }
  uint8_t endTransmission(bool=true) { return fail ? 2 : 0; }
  size_t requestFrom(uint8_t,uint8_t count) {
    index=0;availableBytes=shortRead ? count-1 : count;
    if(reg==0x3B) std::memcpy(buffer,regs+0x3B,14);
    else buffer[0]=reg==0x3A ? (ready && (regs[0x38]&1) ? 1 : 0) : regs[reg];
    return availableBytes;
  }
  int available() {return availableBytes;}
  uint8_t read() {--availableBytes;return buffer[index++];}
};
extern WireStub Wire;
