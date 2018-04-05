// Copyright @2018 Tingfung Lau. All rights reserved.

#include <iostream>
#include "glog/logging.h"
#include "common/utils/file/file.h"
#include "gflags/gflags.h"
#include "homework5/map/map_lib.h"
#include <Eigen/Core>

DEFINE_string(route_file_path, "", "Path of displayed route");


// #define TLAU_DEBUG  

int main(int argc, char* argv[]) {
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);

  
  homework5::map::MapLib map_lib;
  interface::map::Map map(map_lib.map_proto());

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
