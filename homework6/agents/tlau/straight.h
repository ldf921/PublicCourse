// Copyright @2018 Pony AI Inc. All rights reserved.

#pragma once

#include "homework6/simulation/vehicle_agent.h"

#include "common/proto/agent_status.pb.h"
#include "common/proto/geometry.pb.h"
#include "common/proto/vehicle_status.pb.h"
#include "common/utils/math/math_utils.h"
#include "homework6/simulation/vehicle_agent_factory.h"
#include "homework6/agents/tlau/control.h"

#include "gflags/gflags.h"
#include "glog/logging.h"

#include <iostream>
#include <fstream>
#include <sstream>
#include <stdlib.h>
#include <unordered_map>
#include <string>

DEFINE_double(tlau_ki, 0.1, "integral coefficient");

namespace tlau {

class LineAgent : public simulation::VehicleAgent {
 public:
  explicit LineAgent(const std::string& name) : VehicleAgent(name) {}

  virtual void Initialize(const interface::agent::AgentStatus& /* agent_status */) {
    LOG(INFO) << "Start" << std::endl;
    control_.fromTextFile("/home/miu/PublicCourse/homework6/table/processed.txt");
    target_speed_ = 5;
    controler_ = tlau::PID(1, FLAGS_tlau_ki);
    controler_.set_target(5);
    logger_ = std::ofstream("/home/miu/PublicCourse/homework6/table/trail.log");
    logger_ << "time,throttle,brake,velocity" << std::endl;
    LOG(INFO) << "Initialized" << std::endl;
  }

  virtual interface::control::ControlCommand RunOneIteration(
      const interface::agent::AgentStatus& agent_status) {
    interface::control::ControlCommand command;
    // Vehicle's current position reaches the destination
    if (CalcDistance(agent_status.vehicle_status().position(),
                     agent_status.route_status().destination()) < 3.0) {
      position_reached_destination_ = true;
    }
    // LOG(INFO) << "Step" << agent_status.simulation_status().simulation_time() << std::endl;
    const double timestamp = agent_status.simulation_status().simulation_time();
    const double velocity = CalcVelocity(agent_status.vehicle_status().velocity());
    VehicleControl::Input action;
    double ratio;
    if (!position_reached_destination_) {
      // PID control
      double acc = controler_.get_control(timestamp, velocity);
      std::tie(action, ratio) = control_.lookup(velocity, acc);
    } else {
      action = VehicleControl::BRAKE;
      ratio = 0.5;
    }

    if (action == VehicleControl::THROTTLE) {
      command.set_throttle_ratio(ratio);      
    } else {
      command.set_brake_ratio(ratio);
    }

    logger_ << timestamp << ',' << command.throttle_ratio() << ',' << command.brake_ratio()
    << ',' << CalcVelocity(agent_status.vehicle_status().velocity()) <<  std::endl;
    
    // LOG(INFO) << timestamp << ',' << command.throttle_ratio() << ',' << command.brake_ratio()
    // << ',' << CalcVelocity(agent_status.vehicle_status().velocity()) << controler_.integral() <<  std::endl;

    return command;
  }

 private:
  double CalcDistance(const interface::geometry::Vector3d& position,
                      const interface::geometry::Point3D& destination) {
    double sqr_sum =
        math::Sqr(position.x() - destination.x()) + math::Sqr(position.y() - destination.y());
    ;
    return std::sqrt(sqr_sum);
  }

  double CalcVelocity(const interface::geometry::Vector3d& velocity) {
    double sqr_sum = math::Sqr(velocity.x()) + math::Sqr(velocity.y());
    ;
    return std::sqrt(sqr_sum);
  }

  // Whether vehicle's current position reaches the destination
  bool position_reached_destination_ = false;
  // Whether vehicle's current velocity reaches 5 m/s
  bool velocity_reached_threshold_ = false;

  VehicleControl control_;
  double target_speed_;
  tlau::PID controler_;
  std::ofstream logger_;
};

}  // namespace sample
