#include "../../src/kinematics.h"
#include <cassert>
#include <cmath>
#include <cstdio>
#include <limits>
#include <initializer_list>
constexpr double rad=3.14159265358979323846/180;
void verify(double z,double roll,double pitch) {
  assert(Kinematics::compute_kinematics(z,roll,pitch));
  const auto& p=Kinematics::parameters;
  const double cy=std::cos(Kinematics::yawDeg*rad),sy=std::sin(Kinematics::yawDeg*rad);
  const double cr=std::cos(roll*rad),sr=std::sin(roll*rad),cp=std::cos(pitch*rad),sp=std::sin(pitch*rad);
  const double qs[]={Kinematics::q1,Kinematics::q2,Kinematics::q3};
  for(int i=0;i<3;++i) {
    double a=120*i*rad,c=std::cos(a),s=std::sin(a);
    // Independently apply Rx, then Ry, then Rz to the original platform point.
    double x=p.Rp*c,y=p.Rp*s*cr,h=p.Rp*s*sr;
    double xx=cp*x+sp*h,zz=-sp*x+cp*h;
    double X=cy*xx-sy*y+Kinematics::xP,Y=sy*xx+cy*y+Kinematics::yP,Z=zz+p.h0+z;
    assert(std::fabs(-s*X+c*Y)<1e-8);
    double q=qs[i]*rad;
    double ex=(p.Rb+p.L1*std::cos(q))*c,ey=(p.Rb+p.L1*std::cos(q))*s,ez=p.L1*std::sin(q);
    double length=std::sqrt((X-ex)*(X-ex)+(Y-ey)*(Y-ey)+(Z-ez)*(Z-ez));
    assert(std::fabs(length-p.L2)<1e-7);
  }
}
int main() {
  assert(!Kinematics::compute_kinematics(0,0,0));
  assert(Kinematics::lastError==Kinematics::Error::InvalidParameters);
  auto& p=Kinematics::parameters;p.L1=50;p.L2=120;p.Rb=100;p.Rp=80;p.h0=100;
  verify(0,0,0);
  assert(std::fabs(Kinematics::q1-Kinematics::q2)<1e-9);
  assert(std::fabs(Kinematics::q2-Kinematics::q3)<1e-9);
  assert(std::fabs(Kinematics::xP)+std::fabs(Kinematics::yP)+std::fabs(Kinematics::yawDeg)<1e-9);
  for(int sign : {-1,1}) {
    for(int i=0;i<3;++i)p.elbowSign[i]=sign;
    for(int r=-20;r<=20;r+=5)for(int t=-20;t<=20;t+=5)verify(10,r,t);
  }
  double saved=Kinematics::q1;
  assert(!Kinematics::compute_kinematics(1000,0,0));assert(Kinematics::q1==saved);
  assert(Kinematics::lastError==Kinematics::Error::Unreachable);
  assert(!Kinematics::compute_kinematics(0,0,std::numeric_limits<double>::quiet_NaN()));
  assert(!Kinematics::compute_kinematics(-1,0,0));
  assert(!Kinematics::compute_kinematics(0,0,180));
  assert(Kinematics::lastError==Kinematics::Error::SingularOrientation);
  p.qMinDeg[0]=0;p.qMaxDeg[0]=1;
  assert(!Kinematics::compute_kinematics(0,0,0));assert(Kinematics::lastError==Kinematics::Error::JointLimit);
  assert(Kinematics::q1==saved);
  puts("Kinematics tests passed: 162 combined poses, both branches, plane constraints, link lengths and failure preservation.");
}
