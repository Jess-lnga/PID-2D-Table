#include "kinematics.h"
#include <cmath>
#include <limits>

Kinematics::Parameters Kinematics::parameters;
double Kinematics::q1 = std::numeric_limits<double>::quiet_NaN();
double Kinematics::q2 = std::numeric_limits<double>::quiet_NaN();
double Kinematics::q3 = std::numeric_limits<double>::quiet_NaN();
double Kinematics::xP = 0, Kinematics::yP = 0, Kinematics::yawDeg = 0;
Kinematics::Error Kinematics::lastError = Kinematics::Error::None;
int Kinematics::failedLeg = -1;

bool Kinematics::compute_kinematics(double z, double rollDeg, double pitchDeg)
{
  constexpr double pi = 3.14159265358979323846;
  constexpr double rad = pi / 180.0;
  const auto& p = parameters;
  auto fail = [](Error error, int leg = -1) {
    lastError = error; failedLeg = leg; return false;
  };
  if (!std::isfinite(p.L1) || !std::isfinite(p.L2) || !std::isfinite(p.Rb)
      || !std::isfinite(p.Rp) || !std::isfinite(p.h0)
      || p.L1 <= 0 || p.L2 <= 0 || p.Rb < 0 || p.Rp <= 0 || p.h0 < 0)
    return fail(Error::InvalidParameters);
  for (int i = 0; i < 3; ++i)
    if ((p.elbowSign[i] != -1 && p.elbowSign[i] != 1)
        || !std::isfinite(p.qMinDeg[i]) || !std::isfinite(p.qMaxDeg[i])
        || p.qMinDeg[i] > p.qMaxDeg[i] || p.qMaxDeg[i] - p.qMinDeg[i] > 360)
      return fail(Error::InvalidParameters, i);
  if (!std::isfinite(z) || !std::isfinite(rollDeg) || !std::isfinite(pitchDeg)
      || z < 0 || !std::isfinite(p.h0 + z))
    return fail(Error::InvalidInput);

  const double roll = std::remainder(rollDeg, 360.0)*rad;
  const double pitch = std::remainder(pitchDeg, 360.0)*rad;
  const double sr = std::sin(roll), cr = std::cos(roll);
  const double sp = std::sin(pitch), cp = std::cos(pitch);
  // Sum of the three radial-plane constraints gives R21 = R12:
  // sin(yaw)*(cos(pitch)+cos(roll)) = cos(yaw)*sin(pitch)*sin(roll).
  const double yawA = sp*sr, yawB = cp+cr;
  if (std::hypot(yawA, yawB) < 1e-10) return fail(Error::SingularOrientation);
  // Branch continuous about the level pose; the other solution differs by pi.
  double yaw = std::atan2(yawA, yawB);
  if (yaw > pi/2) yaw -= pi;
  if (yaw < -pi/2) yaw += pi;
  const double sy = std::sin(yaw), cy = std::cos(yaw);
  const double R[3][3] = {
    {cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr},
    {sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr},
    {-sp, cp*sr, cp*cr}
  };
  const double px = 0.5*p.Rp*(R[0][0]-R[1][1]);
  const double py = -p.Rp*R[1][0];
  const double height = p.h0+z;
  const double scale = p.L1+p.L2+p.Rb+p.Rp+height;
  if (!std::isfinite(scale)) return fail(Error::InvalidInput);
  double angles[3];
  for (int i = 0; i < 3; ++i)
  {
    const double alpha = 2*pi*i/3;
    const double c = std::cos(alpha), s = std::sin(alpha);
    const double X = px + p.Rp*(R[0][0]*c+R[0][1]*s);
    const double Y = py + p.Rp*(R[1][0]*c+R[1][1]*s);
    const double Z = height + p.Rp*(R[2][0]*c+R[2][1]*s);
    const double u = c*X+s*Y-p.Rb;
    const double v = Z;
    const double d = std::hypot(u,v);
    if (!std::isfinite(d)) return fail(Error::InvalidInput, i);
    if (d <= 1e-10*scale) return fail(Error::SingularLeg, i);
    if (d > p.L1+p.L2+1e-9*scale || d < std::fabs(p.L1-p.L2)-1e-9*scale)
      return fail(Error::Unreachable, i);
    // |(u,v) - L1*(cos(q),sin(q))|^2 = L2^2.
    // Normalize by d to reduce overflow risk when dimensions are large.
    double cosine = (d/p.L1 + (p.L1/d) - (p.L2/d)*(p.L2/p.L1))/2;
    if (!std::isfinite(cosine)) return fail(Error::InvalidInput, i);
    if (cosine > 1+1e-8 || cosine < -1-1e-8) return fail(Error::Unreachable, i);
    cosine = cosine > 1 ? 1 : cosine < -1 ? -1 : cosine;
    double q = (std::atan2(v,u) + p.elbowSign[i]*std::acos(cosine))/rad;
    // Select an equivalent angle inside the configured interval without changing branch.
    q += 360*std::ceil((p.qMinDeg[i]-q-1e-8)/360);
    if (q < p.qMinDeg[i]-1e-8 || q > p.qMaxDeg[i]+1e-8)
      return fail(Error::JointLimit, i);
    angles[i] = q < p.qMinDeg[i] ? p.qMinDeg[i] : q > p.qMaxDeg[i] ? p.qMaxDeg[i] : q;
  }
  q1 = angles[0]; q2 = angles[1]; q3 = angles[2];
  xP = px; yP = py; yawDeg = yaw/rad;
  lastError = Error::None; failedLeg = -1;
  return true;
}
