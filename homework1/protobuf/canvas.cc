// Copyright @2018 Pony AI Inc. All rights reserved.

#include "homework1/protobuf/canvas.h"

#include <iostream>
#include <glog/logging.h>
#include <math.h>

namespace homework1 {

using homework1::geometry::Point3D;

void Canvas::Draw() const {
  for (const auto& p : polygon_.point()) {
    std::cout << "Point:" << p.x() << ", " << p.y() << ", " << p.z() << std::endl;
  }
}

void Canvas::AddPoint(double x, double y, double z) {
  Point3D point;
  point.set_x(x);
  point.set_y(y);
  point.set_z(z);
  AddPoint(point);
}

void Canvas::AddPoint(const Point3D& p) {
  auto* point = polygon_.add_point();
  point->CopyFrom(p);
}

const Point3D& Canvas::GetPoint(int index) const {
  return polygon_.point(index);
}

void Canvas::ParseFromString(const std::string& serialzation) {
  polygon_.ParseFromString(serialzation);
}

const std::string Canvas::SerializeToString() const {
  std::string serialzation;
  CHECK(polygon_.SerializeToString(&serialzation)) << "Canvas serialization failed.";
  return serialzation;
}

double distance(const homework1::geometry::Point3D &a, const homework1::geometry::Point3D &b)
{
  int x = a.x() - b.x(), y = a.y() - b.y(), z = a.z() - b.z();
  return sqrt(x * x + y * y + z * z);
}

double get_length(const homework1::geometry::Polyline &polyline)
{
  double length = 0;
  for(int i = 0; i < polyline.point_size() - 1; i++ ) length += distance(polyline.point(i), polyline.point(i+1) );
  return length;
}

}  // namespace homework1
