// Copyright @2018 Pony AI Inc. All rights reserved.

#pragma once

#include "homework6/simulation/vehicle_agent.h"

#include "common/proto/agent_status.pb.h"
#include "common/proto/geometry.pb.h"
#include "common/proto/route.pb.h"
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
#include <Eigen/Core>
#include <math.h>

DEFINE_double(tlau_look_ahead, 5.0, "Distance to look ahead in direct pursuit steering planning");
DEFINE_double(tlau_steer_kp, 10.0, "Steer Kp");

namespace tlau {

template<typename Tp>
inline Eigen::Vector2d convert2d(const Tp &p)
{
  return Eigen::Vector2d(p.x(), p.y());
}


double cross(const Eigen::Vector2d &a, const Eigen::Vector2d &b) {
  return a.x() * b.y() - a.y() * b.x();
}

int compare(double a, double b, double eps = 1e-8) {
  double diff = a - b;
  if (diff < -eps) return -1;
  else if (diff > eps) return 1;
  else return 0;
}

bool find_landmark(Eigen::Vector2d &ret, const Eigen::Vector2d &x, const Eigen::Vector2d &s, const Eigen::Vector2d &t, double L) {
  const auto dir = (t - s).normalized();
  double od = dir.dot(x - s);
  double squaredLen = math::Sqr(L) - (x - s).squaredNorm() + math::Sqr(od);
  if (squaredLen < 0) return false;
  double len = std::sqrt(squaredLen);
  double r = (t - s).norm();
  if (compare(0, od + len) <= 0 && compare(od + len, r) <= 0) {
    ret = s + dir * (od + len);
    return true;
  } 
  if (compare(0, od - len) <= 0 && compare(od - len, r) <= 0) {
    ret = s + dir * (od - len);
    return true;
  }
  return false;
}

bool find_landmark_in_route(Eigen::Vector2d &ret, const Eigen::Vector2d &x, const interface::route::Route &route, double L) {
  for(int i = route.route_point_size() - 2; i >= 0; i--) {
    if (find_landmark(ret, x, convert2d(route.route_point(i)), convert2d(route.route_point(i + 1)), L)) {
      return true;
    }
  }
  return false; 
}

class CurveAgent : public simulation::VehicleAgent {
 public:
  explicit CurveAgent(const std::string& name) : VehicleAgent(name) {}

  virtual void Initialize(const interface::agent::AgentStatus& agent_status) {
    LOG(INFO) << "Start" << std::endl;
    control_.fromTextFile("/home/miu/PublicCourse/homework6/table/processed.txt");
    target_speed_ = 5;
    controler_ = tlau::PID(1, FLAGS_tlau_ki);
    steer_ = tlau::PID(FLAGS_tlau_steer_kp, 0);
    steer_.set_target(0);
    controler_.set_target(5);

    route_lib_.readMap(map_lib());
    route_.mutable_start_point()->set_x(agent_status.vehicle_status().position().x()); 
    route_.mutable_start_point()->set_y(agent_status.vehicle_status().position().y()); 
    route_.mutable_end_point()->set_x(agent_status.route_status().destination().x());
    route_.mutable_end_point()->set_y(agent_status.route_status().destination().y());
    route_lib_.route(route_);
    
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

    //direct pursuit for wheel
    auto q = agent_status.vehicle_status().orientation();
    double hx = 1 - 2 * math::Sqr(q.y()) - 2 * math::Sqr(q.z()), hy = 2 * q.x() * q.y() + 2 * q.z() * q.w();
    double theta = std::atan2(hy, hx);

    // if (!true) {
      Eigen::Vector2d landmark;
      auto position = convert2d(agent_status.vehicle_status().position());
      CHECK(find_landmark_in_route(landmark, position, route_, FLAGS_tlau_look_ahead))
        << "cannot found landmark";
      auto heading = landmark - position;

      double alpha = std::atan2(heading.y(), heading.x()) - theta;
      if (alpha < -M_PI) {
        alpha += M_PI * 2;
      }
      if (alpha > M_PI) {
        alpha -= M_PI * 2;
      }
    // }
      
    static double steering_angle = 0;
    double target_angular = 2 * velocity * std::sin(alpha) / FLAGS_tlau_look_ahead;
    double angular = agent_status.vehicle_status().angular_velocity_vcs().z();

    steering_angle += 0.01 * M_PI * steer_.get_control(timestamp, angular - target_angular);
    command.set_steering_angle(steering_angle);
    command.set_steering_rate(0);

      // command.set_steering_rate(std::abs(2 * FLAGS_tlau_amp * velocity * std::sin(alpha) / FLAGS_tlau_look_ahead));
    // }
    // else {
    //   command.set_steering_angle(M_PI / 2);
    //   command.set_steering_rate(std::abs(2 * velocity / FLAGS_tlau_look_ahead));      
    // }

    logger_ << timestamp << ',' << command.throttle_ratio() << ',' << command.brake_ratio()
    << ',' << CalcVelocity(agent_status.vehicle_status().velocity()) <<  std::endl;

    
    static double last_record = 0;
    if (timestamp - last_record > 0.25) {
      last_record = timestamp;
      LOG(INFO) << timestamp << ' ' << command.throttle_ratio() << ' ' << command.brake_ratio() 
      << ' ' << command.steering_rate() << " Tw=" << target_angular << " w=" << angular
      << " v=" << CalcVelocity(agent_status.vehicle_status().velocity()) << std::endl;  
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

  VehicleControl control_;
  double target_speed_;
  tlau::PID controler_, steer_;
  std::ofstream logger_;
  homework6::map::RouteLib route_lib_;
  interface::route::Route route_;
};

}  // namespace sample
