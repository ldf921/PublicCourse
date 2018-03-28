#include <iostream>
#include "rotation.h"


Eigen::Matrix3d toRotationMatrix(double yaw, double pitch, double roll)
{
  Eigen::Matrix3d rot_1, rot_2, rot_3;
  rot_1  << cos(pitch), 0, sin(pitch),
             0, 1, 0,
             -sin(pitch), 0 , cos(pitch);
  rot_2 << cos(yaw), -sin(yaw), 0,
           sin(yaw), cos(yaw), 0,
           0, 0, 1;
  rot_3 << 1, 0, 0,
           0, cos(roll), -sin(roll),
           0, sin(roll), cos(roll);
  return rot_2 * rot_1 * rot_3;
}

int main() {
  // Eigen::Matrix3d rotation;
  // rotation << 0.97517033, -0.0978434, -0.19866933,
  //             0.03695701, 0.95642509, -0.28962948,
  //             0.21835066, 0.27509585, 0.93629336;
  // rotation << 0.97517033, -0.153792, -0.15934508,
  //             0.0978434, 0.94470249, -0.31299183,
  //             0.19866933, 0.28962948, 0.93629336;
  // rotation = toRotationMatrix(0.1, -0.2, 0.3);
  // std::cout << rotation << std::endl;
  // std::cout << homework3::ToRollPitchYaw(rotation) << std::endl;
  // Eigen::Vector3d  ypr = homework3::ToRollPitchYaw(rotation);
  // std::cout << rotation - toRotationMatrix(ypr.x(), ypr.y(), ypr.z()) << std::endl;
  // std::cout << homework3::ToRollPitchYaw(rotation) << std::endl;
  // Eigen::Matrix3d rotation;
  // rotation << 0.99934681, -0.03017958, 0.0199023,
  //             0.02997888, 0.99949734, 0.01032745,
  //             -0.02020334, -0.00972537, 0.99974821;
  // homework3::ToAngleAxis(rotation);
  return 0;
}
