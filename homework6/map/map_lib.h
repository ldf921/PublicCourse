// Copyright @2018 Pony AI Inc. All rights reserved.

#pragma once

#include "common/proto/map.pb.h"
#include "common/proto/route.pb.h"
#include "common/utils/file/file.h"

#include "glog/logging.h"
#include <map>

namespace std {

template <>
struct less<interface::map::Id>
{
 bool operator()(const interface::map::Id &lhs, const interface::map::Id &rhs) const {
  return lhs.id() < rhs.id();
 }
};

} //namespace std

namespace homework6 {
namespace map {


class Lane {
public:
 Lane(const interface::map::Lane *lane) : lane_(lane) {}
 auto successor() const { return lane_->successor(); }
 auto point() const { return lane_->central_line().point(); }
 auto id() const {return lane_->id(); }

 Lane *pred_;
private:
 const interface::map::Lane *lane_;
};

class MapLib {
 public:
  MapLib() { CHECK(file::ReadFileToProto("homework6/map/grid2/map_proto.txt", &map_data_)); process(); }

  const interface::map::Map& map_proto() const { return map_data_; }

  // Given a Route protobuf with a start_point and end_point,
  // find a route connect them
  void route(interface::route::Route &);
 private:

  //Fill in the predecessors and successors in the protobuf
  //Build a map from Id to Lanes
  void process();

  // Given a 2D point, find the closest point on the central line of any lane.
  // Return the lane `id`, and the index `i` of the cloest point such that the cloest point is `lanes_[id]->point(i)`
  std::pair<interface::map::Id, int> locate(const interface::geometry::Point2D &point);

  interface::map::Map map_data_;
  std::map<interface::map::Id, Lane *> lanes_;
};

}  // namespace map
}  // namespace homework5
