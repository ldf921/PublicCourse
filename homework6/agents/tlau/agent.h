// Copyright @2018 Pony AI Inc. All rights reserved.

#pragma once

#include "homework6/simulation/vehicle_agent.h"

#include "common/proto/agent_status.pb.h"
#include "common/proto/geometry.pb.h"
#include "common/proto/vehicle_status.pb.h"
#include "common/utils/math/math_utils.h"
#include "homework6/simulation/vehicle_agent_factory.h"

#include "gflags/gflags.h"
#include "glog/logging.h"

#include <iostream>
#include <fstream>
#include <sstream>
#include <stdlib.h>

DEFINE_double(tlau_throttle, 0.1, "The throttle ratio");
DEFINE_double(tlau_control_throttle, 0.2, "The throttle ratio");
DEFINE_double(tlau_brake, 1.0, "The brake ratio");
DEFINE_double(tlau_target_speed, 5.0, "Target speed for testing");
DEFINE_double(tlau_test_time, 2.0, "Time for testing");
DEFINE_string(tlau_task, "throttle", "throttle or brake");

namespace tlau {

class SampleVehicleAgent : public simulation::VehicleAgent {
 public:
  explicit SampleVehicleAgent(const std::string& name) : VehicleAgent(name) {}

  virtual void Initialize(const interface::agent::AgentStatus& /* agent_status */) {
    std::ostringstream os;
    os << "/home/miu/PublicCourse/homework6/" << "table/";
    if (FLAGS_tlau_task == "throttle") {
      os << "throttle" << FLAGS_tlau_throttle  << ".txt";
    } else {
      os << "brake" << FLAGS_tlau_brake << ".txt";
    }
     
    LOG(INFO) << os.str() << std::endl;
    of_ = std::ofstream(os.str().c_str());  
    of_ << "time,brake,throttle,velocity,acceleartion_vcs,ay,az" << std::endl;
  }

  virtual interface::control::ControlCommand RunOneIteration(
      const interface::agent::AgentStatus& agent_status) {
    interface::control::ControlCommand command;
    // Vehicle's current position reaches the destination
    if (CalcDistance(agent_status.vehicle_status().position(),
                     agent_status.route_status().destination()) < 3.0) {
      position_reached_destination_ = true;
    }
    // Vehicle's current velocity reaches 5 m/s
    if (CalcVelocity(agent_status.vehicle_status().velocity()) > 50) {
      velocity_reached_threshold_ = true;
    }


    double brake_ratio = 0;
    double throttle_ratio = 0;
    static bool target_speed = false, low_acc = false, stall=false;
    static double time_reach_target = 0, time_low_acc = 0;

    const double timestamp = agent_status.simulation_status().simulation_time();
    static bool closed = false;

    if (FLAGS_tlau_task == "throttle") {
        if (CalcVelocity(agent_status.vehicle_status().velocity()) > FLAGS_tlau_target_speed) {
        if (!target_speed) {
          target_speed = true;
          time_reach_target = timestamp;
        }
      }

      if (agent_status.vehicle_status().acceleration_vcs().x() < 0.05) {
        if (!low_acc) {
          time_low_acc = timestamp;  
          LOG(INFO) << "timestamp" << timestamp << std::endl;
        }
        low_acc = true;
        if (timestamp - time_low_acc > 0.5) {
          stall = true;
        }
      } else {
        low_acc = false;
      }


      if (!target_speed) {
        throttle_ratio = FLAGS_tlau_throttle;
      } else {
        throttle_ratio = 0;
      }

      command.set_brake_ratio(brake_ratio);
      command.set_throttle_ratio(throttle_ratio);


      if (target_speed or stall) {
        double dt = agent_status.simulation_status().simulation_time() - time_reach_target;
        if (CalcVelocity(agent_status.vehicle_status().velocity()) <= 0.02 or dt > FLAGS_tlau_test_time) {
          if (!closed) {
            closed = true;
            of_.close();
            LOG(INFO) << "ouput stream closed" << " " << CalcVelocity(agent_status.vehicle_status().velocity()) << std::endl;
            exit(0);
          }
        }
      }
    } else {
       if (CalcVelocity(agent_status.vehicle_status().velocity()) > FLAGS_tlau_target_speed) {
        if (!target_speed) {
          target_speed = true;
          time_reach_target = timestamp;
        }
      }

      if (!target_speed) {
        throttle_ratio = FLAGS_tlau_control_throttle;
      } else {
        brake_ratio = FLAGS_tlau_brake;
      }

      command.set_brake_ratio(brake_ratio);
      command.set_throttle_ratio(throttle_ratio);


      if (target_speed) {
        double dt = timestamp - time_reach_target;
        if (CalcVelocity(agent_status.vehicle_status().velocity()) <= 0.02 || dt > 100) {
          if (!closed) {
            closed = true;
            of_.close();
            LOG(INFO) << "ouput stream closed" << " " << CalcVelocity(agent_status.vehicle_status().velocity()) << std::endl;
            exit(0);
          }
        }
      }
    }
    
    
    if (!closed) {
      of_ << agent_status.simulation_status().simulation_time() 
        << ',' << brake_ratio
        << ',' << throttle_ratio
        << ',' << CalcVelocity(agent_status.vehicle_status().velocity()) 
        << ',' << agent_status.vehicle_status().acceleration_vcs().x() 
        << ',' << agent_status.vehicle_status().acceleration_vcs().y() 
        << ',' << agent_status.vehicle_status().acceleration_vcs().z() << std::endl;
    }
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

  std::ofstream of_;
};

}  // namespace sample
