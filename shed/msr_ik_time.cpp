#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <set>
#include <string>
#define RAW_KINEMATICS_FUNCTIONS
#include "kinematics/ur/urkin.h"

int main() {
  {
    Joints joints = {0.15, 0.21, 0.32, 0.47, 0.5, 0.6};
    Trafo ee = fk(joints, UR5E_DH);

    double ee_raw[3 * 4] = {};

    for (size_t i = 0; i < 3; i++) {
      for (size_t j = 0; j < 4; j++) {
        ee_raw[i * 4 + j] = ee[i][j];
      }
    }

    double solutions_raw[8 * 6] = {};
    double straw = 0;
    for (size_t n = 0; n < 10000000; n++) {
      size_t nj = ik_raw(ee_raw, solutions_raw, 0, UR5E_DH);
      straw += nj;
    }

    std::cout << straw;
  }

  return EXIT_SUCCESS;
}
