// Copyright @2018 Pony AI Inc. All rights reserved.

#include "homework3/rotation/rotation.h"
#include <math.h>
#include <iostream>

namespace homework3 {

Eigen::Vector3d ToRollPitchYaw(Eigen::Matrix3d rotation) {
  double roll, pitch, yaw;
  pitch = asin(rotation(0,2));
  yaw = atan2(-rotation(0,1), rotation(0, 0));
  roll = atan2(-rotation(1,2), rotation(2, 2));
  return Eigen::Vector3d(yaw, pitch, roll);
}

Eigen::AngleAxisd ToAngleAxis(Eigen::Matrix3d rotation) {
  return Eigen::AngleAxisd(0.1, Eigen::Vector3d::UnitX());
}
}  // namespace homework3
