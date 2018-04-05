// Copyright @2018 Tingfung Lau. All rights reserved.

#include <iostream>
#include "glog/logging.h"
#include "common/utils/file/file.h"
#include "gflags/gflags.h"
#include "homework5/map/map_lib.h"
#include <Eigen/Core>

DEFINE_string(route_file_path, "", "Path of displayed route");

// Convert a Protobuf Point3D to Eigen
Eigen::Vector3d convert(const interface::geometry::Point3D &p)
{
  return Eigen::Vector3d(p.x(), p.y(), p.z());
}

// Check if the end point of lane 1 and the starting of line 2 is close enough
bool is_connected(const interface::map::Lane &from, const interface::map::Lane &to)
{
  const auto &from_line = from.central_line();
  double d = (convert(from_line.point(from_line.point_size() - 1)) - convert(to.central_line().point(0))).norm();
  return d < 1e-4;
}

// #define TLAU_DEBUG  

int main(int argc, char* argv[]) {
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);

  
  homework5::map::MapLib map_lib;
  interface::map::Map map(map_lib.map_proto());

  auto lanes = map.mutable_lane();

  for(auto lane_f = lanes->begin(); lane_f != lanes->end(); lane_f++) {
    for(auto lane_s = lanes->begin(); lane_s != lanes->end(); lane_s++) {
      if (is_connected(*lane_f, *lane_s)) {
        lane_f->add_successor()->CopyFrom(lane_s->id());
        lane_s->add_predecessor()->CopyFrom(lane_f->id());
      }
    } 
  }

  #ifdef TLAU_DEBUG
  for(const auto &lane : map.lane()) {
    std::cout << lane.id().id() << ':' << std::endl;
    std::cout << "    ";
    int z = 0;
    for(const auto &id : lane.predecessor()) {
      if (z++ > 0) std::cout << ',';
      std::cout << id.id();
    }
    std::cout << std::endl << "    ";
    z = 0;
    for(const auto &id : lane.successor()) {
      if (z++ > 0) std::cout << ',';
      std::cout << id.id();
    }
    std::cout << std::endl;
  }
  #endif

  CHECK(file::WriteProtoToTextFile(map, "/home/miu/PublicCourse/homework5/processed_map_proto.txt")) << "Fail to write";
  return 0;
}
