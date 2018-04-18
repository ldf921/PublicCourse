#include "homework5/map/map_lib.h"

#include <Eigen/Core>
#include <queue>

namespace homework5 {
namespace map {

// Convert a Protobuf Point3D to Eigen
inline Eigen::Vector3d convert(const interface::geometry::Point3D &p)
{
  return Eigen::Vector3d(p.x(), p.y(), p.z());
}

inline Eigen::Vector2d convert2d(const interface::geometry::Point3D &p)
{
  return Eigen::Vector2d(p.x(), p.y());
}

inline Eigen::Vector2d convert2d(const interface::geometry::Point2D &p)
{
  return Eigen::Vector2d(p.x(), p.y());
}

// Check if the end point of lane 1 and the starting of line 2 is close enough
inline bool is_connected(const interface::map::Lane &from, const interface::map::Lane &to)
{
  const auto &from_line = from.central_line();
  double d = (convert(from_line.point(from_line.point_size() - 1)) - convert(to.central_line().point(0))).norm();
  return d < 1e-4;
}

void MapLib::process() {
  auto lanes = map_data_.mutable_lane();

  //enumerate every pair of lanes to see if one lane is the successor of another lane.
  for(auto lane_f = lanes->begin(); lane_f != lanes->end(); lane_f++) {
    for(auto lane_s = lanes->begin(); lane_s != lanes->end(); lane_s++) {
      if (is_connected(*lane_f, *lane_s)) {
        lane_f->add_successor()->CopyFrom(lane_s->id());
        lane_s->add_predecessor()->CopyFrom(lane_f->id());
      }
    } 
  }

  for(auto lane_f = lanes->pointer_begin(); lane_f != lanes->pointer_end(); lane_f++) {
    interface::map::Lane *ptr = *lane_f;
    lanes_.insert(std::make_pair(ptr->id(), new Lane(ptr)));
  }
}

// Helper class, record a range [s, t] in a lane
// addToRoute will add the points in range[s, t] to a Route protobuf 
struct Path {
  Lane *lane;
  int s, t;
  Path(Lane *lane_) : lane(lane_), s(0), t(lane_->point().size() - 1) {}
  Path(Lane *lane_, int s_, int t_) : lane(lane_), s(s_), t(t_) {}
  void addToRoute(interface::route::Route &route) {
    for(int i = s; i <= t; i++) {
      auto ptr=route.add_route_point();
      ptr->set_x(lane->point().Get(i).x());
      ptr->set_y(lane->point().Get(i).y());
    }  
  }
};

void MapLib::route(interface::route::Route &route) {
  auto sp = locate(route.start_point());
  auto ep = locate(route.end_point());
  route.clear_route_point();
  
  #ifdef TLAU_DEBUG
  std::cout << sp.first.id() << ' ' << ep.first.id() << std::endl;
  #endif

  // end point could be reached within the same lane
  if (sp.first.id() == ep.first.id() && sp.second <= ep.second) {
    Path(lanes_[sp.first], sp.second, ep.second).addToRoute(route);
    return ;
  }

  // at least pass through one other lane
  for(auto &kv : lanes_) {
    kv.second->pred_ = NULL;
  }

  // run a BFS from the starting lane, going through the successors the each lane we visited
  // for every lane reached in BFS, we record the predcessor of the point in `pred_` filed.
  // when we reach the same lane of end point, we get the answer
  std::queue<Lane*> que;
  Lane *dest = NULL;
  que.push(lanes_[sp.first]);
  while(!que.empty()) {
    auto u = que.front();
    que.pop();
    for(const auto &id : u->successor()) {
      auto v = lanes_[id];
      if (v->pred_ == NULL) {
        v->pred_ = u;
        if (v->id().id() == ep.first.id()) {
          dest = v;
          break;
        } 
        que.push(v);
      }
    }

    if (dest != NULL) break;
  }

  CHECK(dest != NULL) << "The map is not connected";
  std::vector<Path> stack;
  auto v = dest;

  // using pred_ to find the route from end to start
  // record all the points in the path
  stack.push_back(Path(v, 0, ep.second));
  v = v->pred_;
  while(v->id().id() != sp.first.id()) 
  {
    stack.push_back(Path(v));
    v = v->pred_;
  }
  stack.push_back(Path(v, sp.second, v->point().size() - 1));

  // Add all the points to the Protobuf in the correct order, from start to end
  for(auto it = stack.rbegin(); it != stack.rend(); it++) {
    it->addToRoute(route);
  }
}

std::pair<interface::map::Id, int> MapLib::locate(const interface::geometry::Point2D &point)
{
  auto query = convert2d(point);
  double min_distance = std::numeric_limits<double>::max();
  std::pair<interface::map::Id, int> result;
  for(const auto &kv : lanes_) {
    int i = 0;
    for(const auto &point : kv.second->point()) {
      double d = (query - convert2d(point)).norm();
      if (d < min_distance) {
        min_distance = d;
        result = std::make_pair(kv.first, i);
      }
      ++i;
    }
  }
  CHECK(min_distance < 4) << "The point is not in any lane";
  return result;
}

} //end of map
} //end of homework 5