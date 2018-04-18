#include "homework6/agents/tlau/control.h"

#include "glog/logging.h"
#include <fstream>
#include <limits>
#include <cmath>

namespace tlau {


VehicleControl& VehicleControl::fromTextFile(const std::string &filename) {
  std::ifstream is(filename.c_str());
  if (!is) {
    LOG(ERROR) << "Cannot open file " << filename << std::endl;
  }
  std::string type;
  double ratio;
  while(true) {
    is >> type >> ratio;
    if (type == "end") {
      break;
    }

    std::vector<double> vec;
    double l;
    vec.reserve(VehicleControl::sample);
    for(int i = 0; i < VehicleControl::sample; i++) {
      is >> l;
      vec.push_back(l);
    }

    if (type == "throttle") {
      throttle_responses_.emplace(ratio, std::move(vec));
    } else if (type == "brake") {
      brake_responses_.emplace(ratio, std::move(vec));
    }  else {
      LOG(ERROR) << "Unkown type of input " << type << std::endl;
    }
  }
  return *this;
}


double VehicleControl::lookup(const std::unordered_map<double, std::vector<double> > &responses, double velocity, double acceleration) {
  double min_err = std::numeric_limits<double>::max();
  double best_ratio = 0;
  double clip_velocity = std::max(0.0, std::min((VehicleControl::sample - 1) * VehicleControl::step, velocity));
  int index = (clip_velocity - 1e-12) / VehicleControl::step;
  double residual = clip_velocity - index * VehicleControl::step;
  for(auto const& kv : responses) {
    double acc = (1 - residual) * kv.second[index] + residual * kv.second[index + 1];
    double err = std::abs(acc - acceleration);
    if (err < min_err) {
      min_err = err;
      best_ratio = kv.first;
    }
  }
  return best_ratio;
}

std::pair<VehicleControl::Input, double> VehicleControl::lookup(double velocity, double acceleration) {
  if (acceleration > 0) {
    return std::make_pair(THROTTLE, lookup(throttle_responses_, velocity, acceleration));
  } else {
    return std::make_pair(BRAKE, lookup(brake_responses_, velocity, acceleration));
  }
}

double PID::get_control(double timestamp, double input) {
  const double err = target_ - input;
  if (std::abs(err) < 0.5) {
    captured_ = true;
  }
  if (initialized_) {
    if (captured_) {
      integral_ += 0.5 * (err + state_err_) * (timestamp - state_time_);
    }
  } else {
    initialized_ = true;
  }
  double ret = kp_ * err + ki_ * integral_;

  // update the state
  state_time_ = timestamp;
  state_err_ = err;
  return ret;
}

}