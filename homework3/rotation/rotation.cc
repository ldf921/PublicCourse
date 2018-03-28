// Copyright @2018 Pony AI Inc. All rights reserved.

#include "homework3/rotation/rotation.h"
#include <math.h>
#include <iostream>
#include "Eigen/LU"
#include "Eigen/Eigenvalues"

namespace homework3 {

Eigen::Vector3d ToRollPitchYaw(Eigen::Matrix3d rotation) {
  double roll, pitch, yaw;
  pitch = -asin(rotation(2,0));
  yaw = atan2(rotation(1,0), rotation(0, 0));
  roll = atan2(rotation(2,1), rotation(2, 2));
  return Eigen::Vector3d(yaw, pitch, roll);
}

Eigen::AngleAxisd ToAngleAxis(Eigen::Matrix3d rotation) {
  Eigen::EigenSolver<Eigen::Matrix3d> es(rotation);
  Eigen::Matrix3d diag(es.pseudoEigenvalueMatrix());
  double mx = 0;
  int mi;
  for(int i = 0; i < 3; i++)
  if (diag(i, i) > mx) {
    mx = diag(i, i);
    mi = i;
  }
  int xi = (mi + 1) % 3;
  int yi = (mi + 2) % 3;
  double angle = -asin(diag(xi, yi)) * es.pseudoEigenvectors().determinant();
  Eigen::Vector3d axis(es.pseudoEigenvectors().col(mi));
  return Eigen::AngleAxisd(angle, axis);
}
}  // namespace homework3
