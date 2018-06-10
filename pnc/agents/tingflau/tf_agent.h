// Copyright @2018 Pony AI Inc. All rights reserved.

#pragma once

#include "pnc/simulation/vehicle_agent.h"

#include "common/proto/agent_status.pb.h"
#include "common/proto/geometry.pb.h"
#include "common/proto/vehicle_status.pb.h"
#include "common/utils/math/math_utils.h"
#include "pnc/simulation/vehicle_agent_factory.h"


#include "common/proto/agent_status.pb.h"
#include "common/proto/geometry.pb.h"
#include "common/proto/route.pb.h"
#include "common/proto/vehicle_status.pb.h"
#include "common/utils/math/math_utils.h"
#include "pnc/agents/tingflau/control.h"
#include "pnc/agents/tingflau/route.h"


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
DEFINE_string(tlau_log, "/tmp/tlau_homework_log.txt", "Log file path");
DEFINE_double(tlau_ki, 0.1, "integral coefficient");

namespace tingflau {

template<typename Tp>
inline Eigen::Vector2d convert2d(const Tp &p)
{
  return Eigen::Vector2d(p.x(), p.y());
}

template<typename Tp>
inline std::string asStr(const Tp &p)
{
  std::ostringstream os;
  os << p;
  return os.str();
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

double squared_vert(const Eigen::Vector2d &s, const Eigen::Vector2d &t, const Eigen::Vector2d &o) {
  const auto dir = (t - s).normalized();
  double od = dir.dot(o - s);
  double tL = (o - t).squaredNorm();
  double sL = (o - s).squaredNorm();

  double L = std::min(tL, sL);
  if (0 < od && od < (t - s).norm()) {
    L = std::min(L, (o - s).squaredNorm() - math::Sqr(od));
  }

  return L;
}

bool find_landmark_in_route(Eigen::Vector2d &ret, const Eigen::Vector2d &x, const interface::route::Route &route, double L) {
  for(int i = route.route_point_size() - 2; i >= 0; i--) {
    if (find_landmark(ret, x, convert2d(route.route_point(i)), convert2d(route.route_point(i + 1)), L)) {
      return true;
    }
  }
  return false; 
}

std::pair<double, double> intersection_line(const Eigen::Vector2d &a, const Eigen::Vector2d &b, 
  const Eigen::Vector2d &p, const Eigen::Vector2d &dir) {
  Eigen::Vector2d dir2 = b - a;
  double t1 = cross(dir, p - a) / cross(dir, dir2);
  double t2 = cross(dir2, a - p) / cross(dir2, dir);
  return std::make_pair(t1, t2);
}

int locate_in_route(const Eigen::Vector2d &query, const interface::route::Route &route) {
  double min_sqr_distance = std::numeric_limits<double>::max();
  int result;
  
  int i = 0;
  for(; i + 1 < route.route_point_size(); i++) {
    double sqr_d = squared_vert(convert2d(route.route_point(i)), convert2d(route.route_point(i + 1)), query);
    if (sqr_d < min_sqr_distance) {
      min_sqr_distance = sqr_d;
      result = i;
    }
  }
  // CHECK(min_distance < 4) << "The point is not in any lane";
  // if (!(min_distance < 4)) {
  //   result.second = -1;
  // }
  return result;
}


struct ObstacleView {
  double radis, speed;
  Eigen::Vector2d position;
  std::string id;
  ObstacleView() {}
  ObstacleView(const interface::perception::PerceptionObstacle &obstacle) {
    Eigen::Array2d sum(0, 0);
    for(const auto &point : obstacle.polygon_point()) {
      sum = sum + Eigen::Array2d(point.x(), point.y());
    }
    position = sum / obstacle.polygon_point_size();
    radis = 0.5;
    for(const auto &point : obstacle.polygon_point()) {
      radis = std::max(radis, (convert2d(point) - position).norm());
    }
    speed = 6;
    id = obstacle.id();
  }
};

class CurveAgent : public simulation::VehicleAgent {
 public:
  explicit CurveAgent(const std::string& name) : VehicleAgent(name) {}

  static constexpr double speed_limit = 5;

  virtual void Initialize(const interface::agent::AgentStatus& agent_status) {
    LOG(INFO) << "Start" << std::endl;
    control_.fromTextFile("pnc/agents/tingflau/data/processed.txt");
    target_speed_ = 5;
    controler_ = PID(5, 2 * FLAGS_tlau_ki);
    steer_ = PID(FLAGS_tlau_steer_kp, 0);
    steer_.set_target(0);
    controler_.set_target(5);

    route_lib_.readMap(map_lib());
    set_route(agent_status);

    logger_ = std::ofstream(FLAGS_tlau_log.c_str());
    logger_ << "time,throttle,brake,velocity" << std::endl;
    LOG(INFO) << "Initialized" << std::endl;
  }

  void set_route(const interface::agent::AgentStatus& agent_status) {
    route_.mutable_start_point()->set_x(agent_status.vehicle_status().position().x()); 
    route_.mutable_start_point()->set_y(agent_status.vehicle_status().position().y()); 
    route_.mutable_end_point()->set_x(agent_status.route_status().destination().x());
    route_.mutable_end_point()->set_y(agent_status.route_status().destination().y());
    if (!route_lib_.route(route_)) {
        route_.mutable_end_point()->set_x(agent_status.vehicle_status().position().x());
        route_.mutable_end_point()->set_y(agent_status.vehicle_status().position().y());    
    }
  }

  virtual interface::control::ControlCommand RunOneIteration(
      const interface::agent::AgentStatus& agent_status) {

    if (agent_status.route_status().is_new_request()) {
        set_route(agent_status);
        LOG(INFO) << "A new route #" << route_.route_point_size() << std::endl;
        LOG(INFO) << route_.start_point().x() << "," << route_.start_point().y()
            << " " << route_.end_point().x() << "," << route_.end_point().y() << std::endl;
        position_reached_destination_ = false;
    }

    interface::control::ControlCommand command;
    // Vehicle's current position reaches the destination
    if (CalcDistance(agent_status.vehicle_status().position(),
                     agent_status.route_status().destination()) < 3.0) {
      position_reached_destination_ = true;
    }

    double desired_speed = speed_limit;
    // auto v = convert2d(agent_status.vehicle_status().velocity());
    auto p = convert2d(agent_status.vehicle_status().position());
    int ri = locate_in_route(p, route_);
    auto head = (convert2d(route_.route_point(ri + 1)) - convert2d(route_.route_point(ri))).normalized();

    double cloest_distance = 1e20;
    ObstacleView cloest_ov;
    for(const auto &obstacle : agent_status.perception_status().obstacle()) {
      ObstacleView ov(obstacle);

      double speed_l = 0, speed_r = speed_limit;
      double acc = 4;
      double sd;
      while(speed_r - speed_l > 1e-6) {
        double speed = speed_l + (speed_r - speed_l) / 2;
        double length = 3 + math::Sqr(speed) / (2 * acc);
        sd = squared_vert(p, p + head * length, ov.position);
        if (sd > math::Sqr(ov.radis + 1.5 + speed / acc * ov.speed)) speed_l = speed;
        else speed_r = speed;
      }

      auto dist = (ov.position - p).norm();
      if (dist < cloest_distance) {
        cloest_distance = dist;
        cloest_ov = ov;
      }

      desired_speed = std::min(desired_speed, speed_l);
    }

    for(const auto &status : agent_status.perception_status().traffic_light())
      for(const auto &traffic_light : status.single_traffic_light_status()) {
        auto & info = route_lib_.traffic_lights[traffic_light.id().id()];
        // CHECK(info.stop_line().point_size() == 2) << "Error" << info.stop_line().point_size() << " " << info.id().id();
        if (traffic_light.color() == interface::map::Bulb::YELLOW || 
          traffic_light.color() == interface::map::Bulb::RED) {
          auto s = convert2d(info.stop_line().point(0));
          auto t = convert2d(info.stop_line().point(1));
          double acc = 4;
          double dist = squared_vert(s, t, p);
          if (dist < math::Sqr(10)) {
            PublishVariable("TrafficLight", traffic_light.id().id(), utils::display::Color::Red()); 
            double stop_line_intersect, front_dist;
            std::tie(stop_line_intersect, front_dist) = intersection_line(s, t, p, head);
            PublishVariable("FrontDist", asStr(front_dist)); 
            if (front_dist > 0 && 0 <= stop_line_intersect && stop_line_intersect <= 1) {
              double speed_l = std::max(0.0, std::sqrt(2 * acc * (front_dist - 4)));
              desired_speed = std::min(desired_speed, speed_l);  
            }
          }
        }
      }
      
    std::ostringstream os;
    os << desired_speed;
    PublishVariable("desired_speed", os.str());
    PublishVariable("x", asStr(cloest_ov.position.x()));
    PublishVariable("y", asStr(cloest_ov.position.y()));
    PublishVariable("id", cloest_ov.id);

    controler_.set_target(desired_speed);
    // LOG(INFO) << "Step" << agent_status.simulation_status().simulation_time() << std::endl;
    const double timestamp = agent_status.simulation_status().simulation_time();
    const double velocity = CalcVelocity(agent_status.vehicle_status().velocity());

    PublishVariable("diff_speed", asStr(desired_speed - velocity));

    VehicleControl::Input action;
    double ratio;
    if (!position_reached_destination_) {
      // PID control
      double acc = controler_.get_control(timestamp, velocity);
      PublishVariable("acc", asStr(acc), utils::display::Color::Red()); 
      std::tie(action, ratio) = control_.lookup(velocity, acc);
    } else {
      action = VehicleControl::BRAKE;
      ratio = 0.5;
    }


    if (action == VehicleControl::THROTTLE) {
      command.set_throttle_ratio(ratio);      
      PublishVariable("action", asStr(ratio), utils::display::Color::Green()); 
    } else {
      command.set_brake_ratio(ratio);
      PublishVariable("action", asStr(ratio), utils::display::Color::Red()); 
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

    static bool closed = false;
    if (!closed) {
      logger_ << timestamp << ',' << command.throttle_ratio() << ',' << command.brake_ratio()
        << ',' << CalcVelocity(agent_status.vehicle_status().velocity()) <<  std::endl;  
      if (position_reached_destination_ && velocity < 0.1) {
        logger_.close();
        closed = true;  
      }
    }
    
    
    static double last_record = 0;
    if (timestamp - last_record > 0.25) {
      last_record = timestamp;
      // LOG(INFO) << timestamp << ' ' << command.throttle_ratio() << ' ' << command.brake_ratio() 
      // << ' ' << command.steering_rate() << " Tw=" << target_angular << " w=" << angular
      // << " v=" << CalcVelocity(agent_status.vehicle_status().velocity()) << std::endl;  

      // LOG(INFO) << p.x() << " " << p.y() << " " << head.x() << " " << head.y() << " " << desired_speed;
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
  PID controler_, steer_;
  std::ofstream logger_;
  RouteLib route_lib_;
  interface::route::Route route_;
};

}  // namespace sample
