#pragma once

#include <unordered_map>
#include <string>
#include <vector>
#include <cmath>

namespace tingflau {


class VehicleControl {
public:
  enum Input {
    THROTTLE = 0,
    BRAKE
  };
  static constexpr double step = 0.05;
  static constexpr int sample = 150;
  VehicleControl &fromTextFile(const std::string &filename);
  std::pair<Input, double> lookup(double velocity, double acceleration);
private:
  double lookup(const std::unordered_map<double, std::vector<double> > &responses, double velocity, double acceleration);
  std::unordered_map<double, std::vector<double> > throttle_responses_;
  std::unordered_map<double, std::vector<double> > brake_responses_;
};

class PID
{
public:
  PID() {}
  PID(double kp, double ki) : kp_(kp), ki_(ki) {}
  void set_target(double target) {  target_ = target; }
  double integral() { return integral_; }
  double get_control(double timestamp, double input);
private:
  double kp_, ki_, target_ = 0;
  double state_time_, state_err_;
  double integral_ = 0;
  bool initialized_ = false, captured_ = false;
};

}