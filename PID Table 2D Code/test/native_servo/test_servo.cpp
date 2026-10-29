#include "Arduino.h"
#include "Preferences.h"
#include "../../src/servo_control.h"
#include "../../src/servo_calibration.h"
#include "../../src/kinematics.h"
#include "../../src/platform_control.h"
#include <cassert>
#include <limits>
using namespace ServoCalibration;
void pulseTo(int target) {
  while(ServoControl::getServoTicks(getState().servo)!=target) {
    const int delta=target-ServoControl::getServoTicks(getState().servo);
    assert(adjust(constrain(delta,-100,100)));
  }
}
void advance(unsigned long ms,bool alive=true) {
  for(unsigned long i=0;i<ms;i+=20) {
    fakeMs+=20;
    if(alive)heartbeat(getState().session);
    update();
  }
}
void calibrate(int servo,const int* points) {
  unsigned writesBefore=nvsWrites;
  auto storedBefore=getCalibration(servo);
  assert(chooseServo(servo));
  for(int i=0;i<4;++i) {pulseTo(points[i]);assert(done());}
  assert(getState().stage==Stage::Testing);
  assert(nvsWrites==writesBefore);
  assert(getCalibration(servo).valid==storedBefore.valid);
  assert(getCalibration(servo).ticks[0]==storedBefore.ticks[0]);
  advance(980);
  assert(ServoControl::getServoTicks(servo)==points[3]);
  advance(20);
  advance(600);assert(ServoControl::getServoTicks(servo)==points[2]);
  advance(1600);assert(ServoControl::getServoTicks(servo)==points[1]);
  advance(1600);assert(ServoControl::getServoTicks(servo)==points[0]);
  advance(1000);assert(getState().stage==Stage::Validate);
}
int main() {
  ServoControl::begin();begin();
  assert(getCalibration(1).valid && getCalibration(1).ticks[0]==2337 && getCalibration(1).joint==2);
  assert(getCalibration(2).ticks[1]==2522&&getCalibration(2).joint==1);
  assert(getCalibration(3).ticks[3]==5068&&getCalibration(3).joint==3);
  assert(nvsWrites==0); // Defaults never overwrite flash at boot.
  auto geometry=Kinematics::parameters;
  Kinematics::parameters.L1=1;
  ServoControl::startDemo();auto missingWrites=pwmWrites;fakeMs+=20;ServoControl::updateDemo();
  assert(!ServoControl::getDemoPose().valid&&pwmWrites==missingWrites);
  Kinematics::parameters=geometry;
  ServoControl::startDemo();assert(openMenu());assert(!ServoControl::isDemoRunning());
  auto first=getState();assert(!openMenu());
  assert(checkCommand(first.session,first.revision));
  assert(chooseServo(1));
  assert(!checkCommand(first.session,first.revision));
  assert(!checkCommand(first.session+1,getState().revision));
  assert(!save(1));assert(!adjust(101));assert(!ServoControl::setServoAngle(1,0));
  int other=ServoControl::getServoTicks(2);
  pulseTo(3300);assert(done());
  assert(!done()); // Equal points rejected, still on the same angle.
  pulseTo(3200);assert(done());pulseTo(3400);assert(!done()); // Reversal rejected.
  assert(backToSelection());
  const int a[]={3300,4000,5000,6000};
  calibrate(1,a);assert(ServoControl::getServoTicks(2)==other);
  assert(testAgain());int before=ServoControl::getServoTicks(1);
  advance(20);assert(ServoControl::getServoTicks(1)>before && ServoControl::getServoTicks(1)<6000);
  advance(8000);assert(getState().stage==Stage::Validate);
  assert(!save(0));
  failNvsWrite=true;assert(!save(2));assert(getCalibration(1).ticks[0]==2337);
  failNvsWrite=false;assert(save(2));assert(nvsWrites==1 && getCalibration(1).joint==2);
  int ticks=0;assert(angleToTicks(1,22.5f,ticks)&&ticks==4500);
  assert(angleToTicks(1,-15,ticks)&&ticks==3650);
  assert(angleToTicks(1,67.5,ticks)&&ticks==5500);
  assert(!angleToTicks(1,91,ticks));assert(!angleToTicks(1,std::numeric_limits<float>::quiet_NaN(),ticks));
  const int b[]={6500,6000,5000,4000};calibrate(2,b);
  assert(!save(0));assert(save(1));
  assert(angleToTicks(2,22.5,ticks)&&ticks==5500);
  const int c[]={3400,4100,5100,6100};calibrate(3,c);assert(save(3));
  assert(closeMenu());assert(ServoControl::setServoAngle(1,0));assert(ServoControl::getServoTicks(1)==4000);
  assert(ServoControl::setKinematicAngles(0,45,90));
  assert(ServoControl::getServoTicks(1)==5000); // physical servo 1 is mapped to q2.
  assert(ServoControl::getServoTicks(2)==6000); // physical servo 2 is mapped to q1.
  assert(ServoControl::getServoTicks(3)==6100);
  uint32_t writes=pwmWrites;
  assert(!ServoControl::setKinematicAngles(0,100,0));assert(pwmWrites==writes);
  begin();assert(getCalibration(1).valid&&getCalibration(1).joint==2);assert(getCalibration(2).ticks[0]==6500);
  assert(openMenu());assert(chooseServo(1));
  advance(5020,false);assert(getState().stage==Stage::Paused);
  assert(!adjust(10));assert(isActive());assert(backToSelection());
  const int revised[]={3500,4200,5200,6200};calibrate(1,revised);
  failNvsWrite=true;assert(!save(2));assert(getCalibration(1).ticks[0]==3300);failNvsWrite=false;
  assert(testAgain());advance(5020,false);assert(getState().stage==Stage::Paused);
  writes=pwmWrites;advance(2000,false);assert(pwmWrites==writes);
  assert(!save(2));assert(backToSelection());assert(closeMenu());
  assert(getCalibration(1).ticks[0]==3300); // Cancel never overwrites the saved curve.
  assert(openMenu());calibrate(1,a);assert(save(1));
  assert(getCalibration(1).joint==1 && getCalibration(2).joint==2); // Atomic ID swap.
  assert(closeMenu());begin();assert(getCalibration(2).joint==2);
  assert(openMenu());writes=pwmWrites;
  auto savedCurve=getCalibration(1);
  assert(assignJoint(1,3));assert(getCalibration(1).joint==3&&getCalibration(3).joint==1);
  assert(getCalibration(1).ticks[0]==savedCurve.ticks[0]&&pwmWrites==writes);
  failNvsWrite=true;assert(!assignJoint(2,1));failNvsWrite=false;
  assert(getCalibration(2).joint==2&&getCalibration(3).joint==1);
  begin();assert(getCalibration(1).joint==3);
  // Exercise more than two uninterrupted cycles, respecting q mapping and calibration.
  ServoControl::startDemo();fakeMs+=20;ServoControl::updateDemo();
  const unsigned long waveOrigin=fakeMs;
  assert(!ServoControl::isDemoWaitingForCalibration());
  for(int step=0;step<600;++step) {
    fakeMs+=20;ServoControl::updateDemo();
    assert(ServoControl::isDemoRunning());
    if(fakeMs-waveOrigin<ServoControl::WAVE_START_MS)continue;
    float phase=2*PI*float((fakeMs-waveOrigin)%ServoControl::WAVE_PERIOD_MS)/ServoControl::WAVE_PERIOD_MS;
    auto pose=ServoControl::getDemoPose();
    assert(pose.valid);
    assert(std::fabs(pose.roll-ServoControl::WAVE_ROLL_AMPLITUDE_DEG*std::sin(phase))<0.001);
    assert(std::fabs(pose.pitch-ServoControl::WAVE_PITCH_AMPLITUDE_DEG*std::cos(phase))<0.001);
    assert(Kinematics::compute_kinematics(pose.z,pose.roll,pose.pitch));
    const float angles[]={float(Kinematics::q1),float(Kinematics::q2),float(Kinematics::q3)};
    for(int servo=1;servo<=3;++servo) {
      auto curve=getCalibration(servo);
      int expected;assert(angleToTicks(curve,angles[curve.joint-1],expected));
      assert(std::abs(ServoControl::getServoTicks(servo)-expected)<=1);
    }
  }
  ServoControl::stopDemo();writes=pwmWrites;fakeMs+=100;ServoControl::updateDemo();assert(pwmWrites==writes);
  ImuData::State imu;
  imu.enabled=true;imu.connected=true;imu.calibrated=true;imu.hasData=true;
  imu.samples=10;imu.lastUpdateMs=fakeMs;imu.rollDeg=2;imu.pitchDeg=-3;
  PlatformControl::update(imu);
  assert(PlatformControl::getState().applied&&!ServoControl::isDemoRunning());
  const float iq[]={float(Kinematics::q1),float(Kinematics::q2),float(Kinematics::q3)};
  for(int servo=1;servo<=3;++servo) {
    auto curve=getCalibration(servo);int expected;
    assert(angleToTicks(curve,iq[curve.joint-1],expected));
    assert(ServoControl::getServoTicks(servo)==expected);
  }
  writes=pwmWrites;PlatformControl::update(imu);assert(pwmWrites==writes);
  assert(PlatformControl::setHeightMm(10));assert(PlatformControl::heightMm()==10&&pwmWrites==writes);
  fakeMs+=20;imu.lastUpdateMs=fakeMs;PlatformControl::update(imu);
  assert(PlatformControl::getState().applied&&PlatformControl::getState().z==10);
  writes=pwmWrites;assert(!PlatformControl::setHeightMm(1000));assert(PlatformControl::heightMm()==10&&pwmWrites==writes);
  assert(!PlatformControl::setHeightMm(-1));
  assert(PlatformControl::setHeightMm(0));
  fakeMs+=250;PlatformControl::update(imu);assert(!PlatformControl::getState().applied&&pwmWrites==writes);
  imu.lastUpdateMs=fakeMs;imu.samples++;PlatformControl::update(imu);assert(PlatformControl::getState().applied);
  imu.calibrated=false;writes=pwmWrites;PlatformControl::update(imu);assert(!PlatformControl::getState().applied&&pwmWrites==writes);
  imu.calibrated=true;assert(openMenu());PlatformControl::update(imu);assert(pwmWrites==writes);assert(closeMenu());
  fakeMs+=20;imu.lastUpdateMs=fakeMs;imu.samples++;imu.rollDeg=85;imu.pitchDeg=85;
  PlatformControl::update(imu);assert(!PlatformControl::getState().applied&&pwmWrites==writes);
  const int ikError=PlatformControl::getState().ikError;assert(ikError!=0);
  PlatformControl::update(imu);assert(PlatformControl::getState().ikError==ikError);
  imu.rollDeg=std::numeric_limits<float>::quiet_NaN();PlatformControl::update(imu);assert(pwmWrites==writes);
  imu.enabled=false;PlatformControl::update(imu);assert(ServoControl::isDemoRunning());ServoControl::stopDemo();
  fakeNvs.back()^=1;begin();assert(getCalibration(1).ticks[0]==2337);
  assert(openMenu());assert(assignJoint(1,2));assert(getCalibration(1).ticks[0]==2337);
  begin();assert(getCalibration(1).joint==2&&getCalibration(1).ticks[0]==2337); // Corrupt checksum ignored.
  puts("Servo tests passed: PWM ticks, four steps, timing, replay, reversed curves, interpolation, q mapping, NVS, cancel, stale commands, timeout.");
}
