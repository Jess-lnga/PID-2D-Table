#include "Wire.h"
#include <cassert>
#include <cmath>
#include <cstring>
uint32_t fakeUs=0;
int fakeSwitch=HIGH;
SerialStub Serial;
WireStub Wire;
#include "../../src/imu_data.cpp"
void word(int reg,int16_t value) {
  Wire.regs[reg]=uint16_t(value)>>8; Wire.regs[reg+1]=uint16_t(value)&255;
}
void tick(unsigned count=1) {
  for(unsigned i=0;i<count;++i) {fakeUs+=10000;ImuData::update();}
}
void initialize() {
  Wire=WireStub{};Wire.regs[0x75]=0x70;
  word(0x3F,16384);word(0x43,655); // 1 g and a stationary 5 deg/s gyro bias.
  ImuData::state=ImuData::State{};
  ImuData::begin();ImuData::update();
  assert(ImuData::getState().connected);
  assert(Wire.regs[0x38]==1);
}
int main() {
  initialize();tick(199);
  auto s=ImuData::getState();
  assert(!s.calibrated && s.calibrationSamples==199 && s.hasData);
  tick();s=ImuData::getState();
  assert(s.calibrated && std::fabs(s.gyroBiasDps[0]-5)<0.001);
  tick(200);assert(std::fabs(ImuData::getState().rollDeg)<0.01);
  assert(ImuData::calibrate());
  s=ImuData::getState();assert(!s.calibrated && s.calibrationSamples==0 && !s.hasData);
  word(0x43,917);tick(200);
  assert(ImuData::getState().calibrated);
  assert(std::fabs(ImuData::getState().gyroBiasDps[0]-7)<0.001);
  word(0x3F,0);tick();assert(!ImuData::getState().hasData);
  // The user's reported stationary pose must become zero after calibration.
  initialize();
  const float roll=6.6f*ImuData::RAD, pitch=22.8f*ImuData::RAD;
  word(0x3B,int16_t(-std::sin(pitch)*16384));
  word(0x3D,int16_t(std::sin(roll)*std::cos(pitch)*16384));
  word(0x3F,int16_t(std::cos(roll)*std::cos(pitch)*16384));
  tick(200);s=ImuData::getState();
  assert(s.calibrated && std::fabs(s.rollZeroDeg-6.6f)<0.02f && std::fabs(s.pitchZeroDeg-22.8f)<0.02f);
  assert(std::fabs(s.rollDeg)<0.02f && std::fabs(s.pitchDeg)<0.02f);
  // A later calibration must replace the reference rather than add to it.
  assert(ImuData::calibrate());
  word(0x3B,0);word(0x3D,0);word(0x3F,16384);tick(200);
  s=ImuData::getState();
  assert(s.calibrated && std::fabs(s.rollZeroDeg)<0.02f && std::fabs(s.pitchZeroDeg)<0.02f);
  assert(std::fabs(s.rollDeg)<0.02f && std::fabs(s.pitchDeg)<0.02f);
  initialize();tick(50);word(0x43,1310);tick();
  assert(ImuData::getState().calibrationSamples==0);
  assert(ImuData::getState().calibrationRestarts>0);
  initialize();
  for(unsigned i=0;i<200;++i) {word(0x43,i%2 ? 734 : 576);tick();}
  s=ImuData::getState();
  assert(!s.calibrated && std::strstr(s.diagnostic,"Gyro instable"));
  initialize();word(0x3F,0);tick();s=ImuData::getState();
  assert(!s.hasData && std::strstr(s.diagnostic,"Acceleration invalide"));
  initialize();Wire.ready=false;tick(51);s=ImuData::getState();
  assert(!s.connected && std::strstr(s.diagnostic,"DATA_RDY"));
  initialize();Wire.shortRead=true;tick();s=ImuData::getState();
  assert(!s.connected && s.i2cErrors && s.lastI2cError==255);
  initialize();Wire.fail=true;tick();s=ImuData::getState();
  assert(!s.connected && s.lastI2cError==2);
  Wire.fail=false;tick(101);assert(ImuData::getState().connected);
  tick(200);assert(ImuData::getState().calibrated);
  fakeSwitch=0;tick(4);assert(!ImuData::isEnabled());
  assert(!ImuData::calibrate());
  auto samples=ImuData::getState().samples;tick(10);
  assert(ImuData::getState().samples==samples);
  fakeSwitch=1;tick(4);assert(ImuData::isEnabled());
  assert(ImuData::getState().calibrated);
  initialize();Wire.ignoreConfig=true;Wire.regs[0x19]=0;
  ImuData::state.connected=false;fakeUs+=1000000;ImuData::update();
  assert(!ImuData::getState().connected);
  assert(!ImuData::calibrate());
  assert(std::strstr(ImuData::getState().diagnostic,"Configuration"));
  puts("Native MPU tests passed: bias, progress, motion, invalid data, DATA_RDY, I2C failures, reconnect and switch.");
}
