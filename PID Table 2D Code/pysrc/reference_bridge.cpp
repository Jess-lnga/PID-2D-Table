// Host-only parity check. Compile with the production kinematics.cpp.
#include "../src/kinematics.h"
#include <iostream>
#include <iomanip>
int main()
{
  double z, r, p;
  std::cout << std::setprecision(17);
  while (std::cin >> z >> r >> p)
  {
    bool valid = Kinematics::compute_kinematics(z, r, p);
    std::cout << static_cast<int>(Kinematics::lastError) << ' ' << Kinematics::failedLeg;
    if (valid)
      std::cout << ' ' << Kinematics::q1 << ' ' << Kinematics::q2 << ' ' << Kinematics::q3
                << ' ' << Kinematics::xP << ' ' << Kinematics::yP << ' ' << Kinematics::yawDeg;
    std::cout << '\n';
  }
}
