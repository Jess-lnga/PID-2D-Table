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
  tick(250); // No automatic calibration, even with many perfectly still frames.
  assert(!ImuData::getState().calibrated && !ImuData::getState().calibrationActive);
  assert(ImuData::getState().calibrationSamples==0);
  assert(ImuData::calibrate());
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
  // Modest 0.6 deg/s noise used to restart the window, now it calibrates.
  initialize();
  for(unsigned i=0;i<200;++i) {word(0x43,i%2 ? 734 : 576);tick();}
  s=ImuData::getState();assert(s.calibrated && std::fabs(s.gyroBiasDps[0]-5)<0.02f);
  // A single severe spike must be discarded, not erase 190 accepted frames.
  initialize();tick(190);word(0x43,10000);tick();
  assert(ImuData::getState().calibrationSamples==190 && !ImuData::getState().calibrated);
  word(0x43,655);tick(10);assert(ImuData::getState().calibrated);
  // Sustained movement still restarts; no forced timeout calibration.
  initialize();tick(50);word(0x43,6000);tick(20);
  assert(!ImuData::getState().calibrated && ImuData::getState().calibrationSamples==0);
  assert(ImuData::getState().calibrationRestarts>0);
  assert(!ImuData::getState().calibrationActive);
  // Strong spread below the spike threshold must fail the whole-window check.
  initialize();
  for(unsigned i=0;i<200;++i) {word(0x43,i%2 ? 1048 : 262);tick();}
  assert(!ImuData::getState().calibrated && ImuData::getState().calibrationRestarts>0);
  // Slowly changing gravity catches real tilt even when the raw gyro is constant.
  initialize();
  for(unsigned i=0;i<200;++i) {
    const float r=i*0.002f;
    word(0x3D,int16_t(std::sin(r)*16384));word(0x3F,int16_t(std::cos(r)*16384));tick();
  }
  assert(!ImuData::getState().calibrated);
  initialize();word(0x3F,0);tick();s=ImuData::getState();
  assert(!s.hasData && std::strstr(s.diagnostic,"Acceleration invalide"));
  initialize();Wire.ready=false;tick(51);s=ImuData::getState();
  assert(!s.connected && std::strstr(s.diagnostic,"DATA_RDY"));
  initialize();Wire.shortRead=true;tick();s=ImuData::getState();
  assert(s.connected && !s.hasData && s.calibrationActive && s.i2cErrors==2 && s.lastI2cError==255);
  Wire.shortRead=false;tick(200);assert(ImuData::getState().calibrated);
  initialize();tick(100);Wire.transientFailures=1;tick();
  assert(ImuData::getState().calibrationSamples==101 && ImuData::getState().i2cRetries==1);
  Wire.fail=true;tick(5);s=ImuData::getState();
  assert(s.calibrationActive && s.calibrationSamples==101 && !s.hasData && s.i2cConsecutiveFailures==5);
  assert(Wire.recoveries>0);
  Wire.fail=false;tick(99);assert(ImuData::getState().calibrated);
  initialize();Wire.fail=true;tick(51);s=ImuData::getState();
  assert(!s.connected && s.lastI2cError==2);
  Wire.fail=false;tick(101);assert(ImuData::getState().connected);
  tick(200);assert(!ImuData::getState().calibrated && !ImuData::getState().calibrationActive);
  assert(ImuData::calibrate());tick(200);assert(ImuData::getState().calibrated);
  float savedZero=ImuData::getState().rollZeroDeg;
  Wire.fail=true;tick();Wire.fail=false;tick(101);tick(200);
  assert(ImuData::getState().calibrated && !ImuData::getState().calibrationActive);
  assert(ImuData::getState().rollZeroDeg==savedZero);
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
