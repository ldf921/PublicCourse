#include "homework4/perception_lib.h"

#include <vector>
#include <Eigen/Core>
#include <math.h>
#include <iostream>

namespace perception {

struct PointCloudLabel {
  std::vector<Eigen::Vector2d> polygon;
  double floor = std::numeric_limits<double>::infinity();
  double ceiling = -std::numeric_limits<double>::infinity();
};

double cross(const Eigen::Vector2d &a, const Eigen::Vector2d &b)
{
    return a.x() * b.y() - a.y() * b.x(); 
}

bool is_intersection(const Eigen::Vector2d &a, const Eigen::Vector2d &b, 
  const Eigen::Vector2d &p, const Eigen::Vector2d &dir) {
  Eigen::Vector2d dir2 = b - a;
  double t1 = cross(dir, p - a) / cross(dir, dir2);
  double t2 = cross(dir2, a - p) / cross(dir2, dir);
  return t2 > 0 && 0 < t1 && t1 < 1;
}

bool inside_box(const Eigen::Vector3d &point, const PointCloudLabel &label) {
  if (!(label.floor <= point.z() && point.z() <= label.ceiling)) return false;
  Eigen::Vector2d p(point.x(), point.y());

  double alpha = 0.2157;
  Eigen::Vector2d direction(cos(alpha), sin(alpha));
  int c = 0;
  for(int i = 0, j; i < label.polygon.size(); i++) {
    j = i + 1;
    if (j == label.polygon.size()) j = 0;
    if (is_intersection(label.polygon[i], label.polygon[j], p, direction)) ++c;
  }

  return c % 2 == 1;
}

interface::geometry::Point3D convert(const Eigen::Vector3d &point)
{
  interface::geometry::Point3D p;
  p.set_x(point.x());
  p.set_y(point.y());
  p.set_z(point.z()); 
  return p;
}

interface::perception::PerceptionObstacles build(const PointCloud &pointcloud, 
  const interface::object_labeling::ObjectLabels &labels)
{
  std::vector<Eigen::Vector3d> points;
  points.clear();
  points.reserve(pointcloud.points.size());
  for (const auto& point : pointcloud.points) {
    Eigen::Vector3d point_in_world = pointcloud.rotation * point + pointcloud.translation;
    points.push_back(point_in_world);
  }

  interface::perception::PerceptionObstacles obstacles;
  obstacles.set_timestamp(labels.timestamp());
  for(const auto &label : labels.object()) {
    auto obstacle = obstacles.add_obstacle(); 
    obstacle->set_id(label.id()); 
    obstacle->set_heading(label.heading());
    obstacle->set_height(label.height());
    obstacle->set_type(label.type());

    for(const auto &point : label.polygon().point()) {
      obstacle->add_polygon_point()->CopyFrom(point);
    }

    PointCloudLabel prism;
    for (const auto& point : label.polygon().point()) {
      prism.floor = std::min(prism.floor, point.z());
      prism.polygon.emplace_back(point.x(), point.y());
    }
    prism.ceiling = prism.floor + label.height();

    for (const auto& point : points) {
      if (inside_box(point, prism)) {
        obstacle->add_object_points()->CopyFrom(convert(point));
      }
    }

    std::cout << obstacle->id() << ' ' << obstacle->object_points_size() << std::endl;
  }

  return obstacles;
}


}

/*
#include "common/utils/file/file.h"
#include "common/utils/file/path.h"

using perception::PointCloudLabel;
int main()
{
    interface::object_labeling::ObjectLabels object_labels;
  file::ReadFileToProto("/home/miu/PublicCourse/pony_data/sample/label/VelodyneDevice32c/0.label", &object_labels);
  for(const auto &label : object_labels.object()) {
    PointCloudLabel prism;
    for (const auto& point : label.polygon().point()) {
      prism.floor = std::min(prism.floor, point.z());
      prism.polygon.emplace_back(point.x(), point.y());
    }
    prism.ceiling = prism.floor + label.height();

    std::cout << label.id() << std::endl;
    Eigen::Vector3d center(label.center_x(), label.center_y(), (prism.ceiling + prism.floor) / 2);
    std::cout << inside_box(center, prism) << std::endl;
  }
  return 0;
}
*/