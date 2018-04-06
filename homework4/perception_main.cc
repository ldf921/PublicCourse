#include "homework4/perception_lib.h"

#include <gflags/gflags.h>
#include <glog/logging.h>

DEFINE_string(pony_data_dir, "", "The path of pony data.");

int main(int argc, char* argv[]) {
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);

  PointCloudViewer::Options options;
  PointCloudViewer viewer(options, nullptr, FLAGS_pony_data_dir, false);

  
  return 0;
}
