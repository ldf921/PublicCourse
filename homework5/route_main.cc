// Copyright @2018 Tingfung Lau. All rights reserved.

#include <iostream>
#include "glog/logging.h"
#include "common/utils/file/file.h"
#include "gflags/gflags.h"
#include "homework5/map/map_lib.h"
#include <Eigen/Core>
#include <string>

DEFINE_string(route_file_path, "", "Path to route to be completed");

std::string replace(const std::string &str, const std::string &target, const std::string replacement) {
  size_t p = str.find(target);
  if (p != std::string::npos) {
    auto ret = str;
    return ret.replace(p, target.length(), replacement);
  }
  return str;
}

inline Eigen::Vector3d convert(const interface::geometry::Point3D &p)
    {
      return Eigen::Vector3d(p.x(), p.y(), p.z());
    }

int main(int argc, char* argv[]) {
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);

  
  interface::route::Route route;
  CHECK(file::ReadTextFileToProto(FLAGS_route_file_path, &route)) << "Failed to load route file";

  homework5::map::MapLib map;
  map.route(route);
  auto target_path = replace(FLAGS_route_file_path, "request", "result");
  CHECK(file::WriteProtoToTextFile(route, target_path)) << "Failed to write route file";
  return 0;
}