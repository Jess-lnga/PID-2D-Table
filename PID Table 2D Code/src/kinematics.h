#ifndef KINEMATICS_H
#define KINEMATICS_H

// Pure geometry: no PWM writes. All lengths use the SAME unit; angles are degrees.
// Bases and platform joints are ordered A/B/C at 0/120/240 degrees.
// R = Rz(yaw) * Ry(pitch) * Rx(roll), applied to column vectors.
// q=0: L1 points horizontally OUTWARD; positive q raises the arm.
class Kinematics
{
public:
  struct Parameters
  {
    double L1 = 74; // Servo axis -> end of palonnier
    double L2 = 77; // Bielle, joint center -> joint center
    double Rb = 58.838; // Base center -> servo axis (all base axes at Z=0)
    double Rp = 132.842; // Platform center -> spherical joint
    double h0 = 60; // Platform reference height; center Z = h0 + z
    int elbowSign[3] = {-1, -1, -1}; // Select fixed assembly branch (+1 or -1).
    double qMinDeg[3] = {-30, -30, -30}; // Replace with measured mechanical limits.
    double qMaxDeg[3] = {90, 90, 90};
  };
  enum class Error
  {
    None, InvalidParameters, InvalidInput, SingularOrientation,
    SingularLeg, Unreachable, JointLimit
  };
  static Parameters parameters; // Must be filled with measured dimensions before use.
  static double q1, q2, q3; // Degrees; initially NaN, unchanged on a failed computation.
  static double xP, yP, yawDeg; // Successful pose's parasitic displacement/yaw.
  static Error lastError;
  static int failedLeg; // -1 for a global error, 0/1/2 for A/B/C.

  // z >= 0 is measured above h0. Returns false for invalid/unreachable geometry.
  // All outputs update together ONLY on success. Check the return before using q1/2/3.
  static bool compute_kinematics(double z, double rollDeg, double pitchDeg);
};

#endif
