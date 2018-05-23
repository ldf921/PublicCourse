// Copyright @2018 Pony AI Inc. All rights reserved.

#include "perception/perception.h"
#include <queue>
#include <vector>
#include <algorithm>

int coord(double x) {
  return int(std::round(x));
}

struct PointCloudLabel {
  std::string id;
  std::vector<math::Vec2d> polygon;
  double floor = std::numeric_limits<double>::infinity();
  double ceiling = -std::numeric_limits<double>::infinity();
  int label;
};

struct cmp {
  bool operator()(const math::Vec2d &a, const math::Vec2d &b) const {
    return a.x < b.x || (a.x == b.x && a.y < b.y);
  }
};


double cross(const math::Vec2d &a, const math::Vec2d &b)
{
    return a.x * b.y - a.y * b.x; 
}

double cross(const Eigen::Vector2d &a, const Eigen::Vector2d &b)
{
    return a.x() * b.y() - a.y() * b.x(); 
}

std::vector<math::Vec2d> convexHull(const std::vector<math::Vec3d> &points) {
  std::vector<math::Vec2d> ret;

  for(const auto &p : points) {
    ret.emplace_back(p.x, p.y);
  }

  std::sort(ret.begin(), ret.end(), cmp());

  std::vector<math::Vec2d> polygon(ret.size() + 1);
  int n = 0;
  for(int i = 0; i < ret.size(); i++) {
    while(n >= 2 && cross(polygon[n - 1] - polygon[n - 2], ret[i] - polygon[n - 1]) <= 0) --n;
    polygon[n++] = ret[i];
  }

  for(int i = ret.size() - 2; i >= 0; i--) {
    while(n >= 2 && cross(polygon[n - 1] - polygon[n - 2], ret[i] - polygon[n - 1]) <= 0) --n;
    // polygon[n++] = ret[i]; 
    polygon.at(n++) = ret[i]; 
  }

  --n;

  polygon.resize(n);
  return polygon;
}

template<typename ContainerType, typename T>
std::pair<double, double> getRange(ContainerType container,  T key) {
  double minv = std::numeric_limits<double>::max(), maxv = std::numeric_limits<double>::lowest();
  for(const auto &elem : container) {
    auto v = key(elem);
    minv = std::min(minv, v);
    maxv = std::max(maxv, v);
  }
  return std::make_pair(minv, maxv);
}

bool is_intersection(const Eigen::Vector2d &a, const Eigen::Vector2d &b, 
  const Eigen::Vector2d &p, const Eigen::Vector2d &dir) {
  Eigen::Vector2d dir2 = b - a;
  double t1 = cross(dir, p - a) / cross(dir, dir2);
  double t2 = cross(dir2, a - p) / cross(dir2, dir);
  return t2 > 0 && 0 < t1 && t1 < 1;
}

auto eigen(const math::Vec2d &p) {
  return Eigen::Vector2d(p.x, p.y);
}

bool inside_box(const math::Vec3d &point, const PointCloudLabel &label) {
  if (!(label.floor <= point.z && point.z <= label.ceiling)) return false;
  Eigen::Vector2d p(point.x, point.y);

  double alpha = 0.2157;
  Eigen::Vector2d direction(cos(alpha), sin(alpha));
  int c = 0;
  for(int i = 0, j; i < label.polygon.size(); i++) {
    j = i + 1;
    if (j == label.polygon.size()) j = 0;
    if (is_intersection(eigen(label.polygon[i]), eigen(label.polygon[j]), p, direction)) ++c;
  }

  return c % 2 == 1;
}

struct Box {
  math::Vec2d center;
  double length, width, theta;
  std::vector<math::Vec2d> asPolygon() const {
    static const int dx[] = {1, 1, -1, -1};
    static const int dy[] = {-1, 1, 1, -1};

    std::vector<math::Vec2d> ret;
    for(int i = 0; i < 4; i++) {
      math::Vec2d d(dx[i] * length / 2, dy[i] * width / 2);
      ret.push_back(center + d.Rotate(theta));  
    }
    return ret;
  }  
  Box(math::Vec2d _center, double _length, double _width, double _theta) : 
    center(_center), length(_length), width(_width), theta(_theta) {}
  Box () {}

  bool isInside(const math::Vec2d &p) const {
    auto p_box = (p - center).Rotate(-theta);
    return -length / 2 <= p_box.x && p_box.x <= length / 2 
      && -width / 2 <= p_box.y && p_box.y <= width / 2;    
  }
};

struct HeightGrid
{
  static const int rows = 200, cols = 200;
  const int cx = rows / 2, cy = cols / 2;
  const double resy = 0.5, resx = 0.5;
  const double inf = 1e8;
  double rx, ry;
  double height[rows][cols], top[rows][cols];
  bool isGround[rows][cols];
  double hlist[rows * cols];
  int head[rows][cols];
  int cnt[rows][cols], cntGround[rows][cols], cntContextGround[rows][cols];
  double contextHeight[rows][cols];
  bool showed[rows][cols];
  double traceHeight[rows][cols];

  double getHegiht(const math::Vec3d &p) {
    const int y = coord((p.y - ry) / resy + cy), x = coord((p.x - rx) / resx + cx);
    if (0 <= y && y < rows && 0 <= x && x < cols) {
      return height[y][x];
    } else return -10;
  }

  double getContextHegiht(const math::Vec3d &p) {
    const int y = coord((p.y - ry) / resy + cy), x = coord((p.x - rx) / resx + cx);
    if (0 <= y && y < rows && 0 <= x && x < cols) {
      return contextHeight[y][x];
    } else return -10;
  }

  math::Vec2d getOrigin() const {
    return {rx, ry};
  }

  std::vector<std::vector<math::Vec3d> > removeGround(const std::vector<math::Vec3d> &pointcloud,
    const math::Vec3d &lidar_pos,
    std::vector<math::Vec3d> &ground_points) {
    
    for(int i = 0; i < rows; i++)
      for(int j = 0; j < cols; j++)
      {
        height[i][j] = inf;
      }

    for(int i = 0; i < rows; i++)
      for(int j = 0; j < cols; j++)
      {
        top[i][j] = -inf;
      }

    for(int i = 0; i < rows; i++)
      for(int j = 0; j < cols; j++)
      {
        cnt[i][j] = 0;
        cntGround[i][j] = 0;
      }

    for(int i = 0; i < rows; i++)
      for(int j = 0; j < cols; j++)
      {
        head[i][j] = -1;
      }

    for(int i = 0; i < rows; i++)
      for(int j = 0; j < cols; j++)
      {
        traceHeight[i][j] = -inf;
      }
    // double sx = 0, sy = 0;
    // for(auto &p : pointcloud) {
    //   sx += p.x;
    //   sy += p.y;
    // }
    // rx = sx / pointcloud.size(), ry = sy / pointcloud.size();
    rx = lidar_pos.x;
    ry = lidar_pos.y;

    for (size_t i = 0; i < pointcloud.size(); i++) {
        auto &p = pointcloud[i];
        const int y = coord((p.y - ry) / resy + cy), x = coord((p.x - rx) / resx + cx);
        if (0 <= y && y < rows && 0 <= x && x < cols)
        {
            height[y][x] = std::min(height[y][x], p.z);
            top[y][x] = std::max(top[y][x], p.z);
            cnt[y][x]++;
        }
    }

    setContextHeightMap(5, 5);

    memset(isGround, false, sizeof(isGround));
    for(int i = 0; i < rows; i++)
      for(int j = 0; j < cols; j++) {
        if (top[i][j] <= height[i][j] + 0.1 && height[i][j] <= contextHeight[i][j] + 0.3) {
          isGround[i][j] = true;
        }
      }

    std::vector<math::Vec3d> select_points;
    for(auto &p : pointcloud) {
      const int y = coord((p.y - ry) / resy + cy), x = coord((p.x - rx) / resx + cx);
      if (0 <= y && y < rows && 0 <= x && x < cols) {
        if (p.z > height[y][x] + 0.08 || p.z > contextHeight[y][x] + 0.3) {
          select_points.push_back(p);
        } else {
          if (isGround[y][x]) {
            ground_points.push_back(p);
            cntGround[y][x]++;
          }
        }
      }
    }

    setContextGround(7, 7);

    std::vector<int> next(select_points.size(), -1);
    std::vector<bool> vis(select_points.size(), false);
    for (size_t i = 0; i < select_points.size(); i++) {
        auto &p = select_points[i];
        const int y = coord((p.y - ry) / resy + cy), x = coord((p.x - rx) / resx + cx);
        if (0 <= y && y < rows && 0 <= x && x < cols)
        {
            next[i] = head[y][x];
            head[y][x] = i;
        }
    }

    std::queue<math::Vec3d> que;
    std::vector<std::vector<math::Vec3d> > groups;
    for(size_t i = 0; i < select_points.size(); i++) {
      if (!vis[i]) {
        auto &sp = select_points[i];
        que.push(sp);
        vis[i] = true;

        groups.emplace_back();
        auto &group = groups.back();
        while(!que.empty()) {
          auto p = que.front();
          group.push_back(p);
          que.pop();
          const int y = coord((p.y - ry) / resy + cy), x = coord((p.x - rx) / resx + cx);
          if (0 <= y && y < rows && 0 <= x && x < cols) {
            for(int dx = -2; dx <= 2; dx++)
              for(int dy = -2; dy <= 2; dy++) {
                const int nx = x + dx, ny = y + dy;
                if (0 <= ny && ny < rows && 0 <= nx && nx < cols) {
                  for(int i = head[ny][nx]; i != -1; i = next[i]) {
                    if (!vis[i] && (select_points[i] - p).Length() < 0.8) {
                      vis[i] = true;
                      que.push(select_points[i]);
                    }
                  }  
                }
              }
          }
        }
      }
    }

    return groups;
  }

  double traceGround(const math::Vec3d &p) {
    double height = inf;
    const int y = coord((p.y - ry) / resy + cy), x = coord((p.x - rx) / resx + cx);
    if (0 <= y && y < rows && 0 <= x && x < cols) {
      if (traceHeight[y][x] != -inf) return traceHeight[y][x];      
      if (std::abs(y - cy) > std::abs(x - cx)) {
          int s = y - cy > 0 ? 1 : -1;
          for(int dy = 0; dy <= std::abs(y - cy); dy++) {
            int ty = cy + dy * s;
            int tx = (ty - cy) * (x - cx) / (y - cy) + cx;

            if (0 <= ty && ty < rows && 0 <= tx && tx < cols && 
              isGround[ty][tx] && contextHeight[ty][tx] < height + 0.3) {
              height = contextHeight[ty][tx];
            }
          }
        } else if (std::abs(x - cx) > 0) {
          // std::cerr << std::abs(x - cx) << std::endl;
          int s = x - cx > 0 ? 1 : -1;
          for(int dx = 0; dx <= std::abs(x - cx); dx++) {
            int tx = cx + dx * s;
            int ty = (tx - cx) * (y - cy) / (x - cx) + cy;

            if (0 <= ty && ty < rows && 0 <= tx && tx < cols && 
              isGround[ty][tx] && contextHeight[ty][tx] < height + 0.3) {
              height = contextHeight[ty][tx];
            }
          }
        }
      traceHeight[y][x] = height;
    }
    return height;
  }

  bool plotGround(PointCloudLabel &label, const math::Vec3d &p) {
      const int y = coord((p.y - ry) / resy + cy), x = coord((p.x - rx) / resx + cx);
      if (!(0 <= y && y < rows && 0 <= x && x < cols)) return false;
      if (showed[y][x])  return false;
      showed[y][x] = true;
      label.floor = height[y][x];
      label.ceiling = label.floor + 0.1;
      std::ostringstream ss;
      ss << "Ground_" << x << "_" << y << std::endl;
      label.id = ss.str();

      
      label.polygon.push_back(math::Vec2d(rx + (x - cx - 0.5) * resx, ry + (y - cy - 0.5) * resy));    
      label.polygon.push_back(math::Vec2d(rx + (x - cx + 0.5) * resx, ry + (y - cy - 0.5) * resy));    
      label.polygon.push_back(math::Vec2d(rx + (x - cx + 0.5) * resx, ry + (y - cy + 0.5) * resy));    
      label.polygon.push_back(math::Vec2d(rx + (x - cx - 0.5) * resx, ry + (y - cy + 0.5) * resy));     
      return true;
  }

  void setContextGround(int contextY, int contextX) {
    for(int y = 0; y < rows; y++)
      for(int x = 0; x < cols; x++) {
        int ans = 0;
        for(int dy = -contextY / 2; dy <= contextY / 2; dy++)
          for(int dx = -contextX / 2; dx <= contextX / 2; dx++) {
            const int ny = y + dy, nx = x + dx;
            if (0 <= ny && ny < rows && 0 <= nx && nx < cols) {
              ans += cntGround[ny][nx];
            }
          } 
        cntContextGround[y][x] = ans;
    }
  }

  int getCntContextGround(const math::Vec3d &p) {
    const int y = coord((p.y - ry) / resy + cy), x = coord((p.x - rx) / resx + cx);
    if (0 <= y && y < rows && 0 <= x && x < cols) {
      return cntContextGround[y][x];
    } else return 0;
  }

  void setContextHeightMap(int contextY = 5, int contextX = 5) {
    for(int y = 0; y < rows; y++)
      for(int x = 0; x < cols; x++) {
        double ans = inf;
        if (height[y][x] != inf) {
         for(int dy = -contextY / 2; dy <= contextY / 2; dy++)
          for(int dx = -contextX / 2; dx <= contextX / 2; dx++) {
            const int ny = y + dy, nx = x + dx;
            if (0 <= ny && ny < rows && 0 <= nx && nx < cols) {
              ans = std::min(ans, height[ny][nx]);  
            }
          } 
        }
        contextHeight[y][x] = ans;
      }
  }

  template<typename T>
  math::Vec2d getPoint(T x, T y) const {
    return {(x - cx) * resx + rx, (y - cy) * resy + ry};
  }

  static const int range = 10;
  struct SearchResult {
    Box box;
    double score;
  };

  struct Constraint {
    double lmin, lmax, wmin, wmax;
  };

  double getAngle(double a, double b) {
    double d = std::abs(a - b);
    return d > M_PI ? M_PI * 2 - d : d;
  }
  
  int sumGround[range * 2 + 2][range * 2 + 2];
  SearchResult searchBox(const Box &box, double lidar_vis, int length, int width) {
    const int ym = coord((box.center.y - ry) / resy + cy), xm = coord((box.center.x - rx) / resx + cx);
    double theta = box.theta;

    memset(sumGround, 0, sizeof(sumGround));
    for(int y = -range; y <= range; y++) 
      for(int x = -range; x <= range; x++) {
        auto p = math::Vec2d(xm, ym) + math::Vec2d(x, y).Rotate(theta);
        int ny = coord(p.y), nx = coord(p.x);
        if (0 <= ny && ny < rows && 0 <= nx && nx < cols) {
          sumGround[y + range + 1][x + range + 1] = cntGround[ny][nx];
        }
      }
    
    // Compute Partial Sum
    for(int y = 1; y < range * 2 + 2; y++)
      for(int x = 1; x < range * 2 + 2; x++) {
        sumGround[y][x] += sumGround[y - 1][x] - sumGround[y - 1][x - 1] + sumGround[y][x - 1];
      }

    double score = -1e6;
    Box ans = box;
    ans.length = length * resx;
    ans.width = width * resy;
    ans.theta = theta;

    auto origin = getPoint(xm, ym);
    auto diff = (box.center - origin).Rotate(-box.theta);

    Constraint c;
    c.lmax = diff.x - box.length / 2.0 + length * resx / 2.0;
    c.lmin = diff.x + box.length / 2.0 - length * resx / 2.0;
    c.wmax = diff.y - box.width  / 2.0 + width  * resy / 2.0;
    c.wmin = diff.y + box.width  / 2.0 - width  * resy / 2.0;

    double margin = 0.5;
    if (getAngle(lidar_vis, theta) <= M_PI / 4 ) {
      c.lmin = std::max(c.lmin, diff.x - box.length / 2.0 + length * resx / 2.0 - margin);
    }
    else if (getAngle(lidar_vis, theta + M_PI / 2) <= M_PI / 4) {
      c.wmin = std::max(c.wmin, diff.y - box.width  / 2.0 + width  * resy / 2.0 - margin);
    }
    else if (getAngle(lidar_vis, theta - M_PI / 2) <= M_PI / 4) {
      c.wmax = std::min(c.wmax, diff.y + box.width  / 2.0 - width  * resy / 2.0 + margin); 
    }
    else {
      c.lmax = std::min(c.lmax, diff.x + box.length / 2.0 - length * resx / 2.0 + margin); 
    }

    double gdx, gdy;
    for(int y = 0; y + width < range * 2 + 2; y++) {
      for(int x = 0; x + length < range * 2 + 2; x++) {
        double dx = (x + (length - 1) / 2.0 - range) * resx, dy = (y + (width - 1) / 2.0 - range) * resy;
        double t = (std::max(c.lmin - dx, 0.0) + std::max(dx - c.lmax, 0.0) +  
        std::max(c.wmin - dy, 0.0) + std::max(dy - c.wmax, 0.0)) * 40;
        int groundPoints = (sumGround[y + width][x + length]
         - sumGround[y + width][x]
         - sumGround[y][x + length]
         + sumGround[y][x]);
        double s = -groundPoints - t;
        if (s > score) {
          score = s;
          ans.center = origin + math::Vec2d(dx,  dy).Rotate(theta);
          gdx = dx;
          gdy = dy;
        }
      } 
    }

    SearchResult r;
    r.box = ans;
    r.score = score + (std::max(box.width - ans.width, 0.0) + std::max(box.length - ans.length, 0.0)) * -100;
    return r;
  }

} hmap;

struct ObjectFeature {
  ObjectFeature(const std::vector<math::Vec3d> &points, const PointCloudLabel &label) {
    area = 0;
    for(int i = 1; i + 1 < label.polygon.size(); i++) {
      area += cross(label.polygon[i] - label.polygon[0], label.polygon[i + 1] - label.polygon[0]);
    }
    area = std::abs(area);

    density = points.size() / area;

    diameter = getDiameter(label.polygon);

    height = label.ceiling - label.floor;
    floor = label.floor;
    ceiling = label.ceiling;

    cirv = area / (diameter * diameter);

    groundLevel = 1e4;
    contextGround = 1e4;
    distance = 0;
    int groundedCnt = 0;
    cntCG = 0;
    for(const auto & point : points) {
      groundLevel = std::min(groundLevel, hmap.getHegiht(point));
      contextGround = std::min(contextGround, hmap.getContextHegiht(point));
      double d = (hmap.getOrigin() - math::Vec2d(point.x, point.y)).Length();
      distance += d;
      cntCG = std::max(cntCG, hmap.getCntContextGround(point));
    }
    distance /= points.size();
    cnt = points.size();
    grounded = groundedCnt / (double) points.size();
  }

  double getDiameter(const std::vector<math::Vec2d> &polygon) {
    double diameter = 0, max_distance = 0;
    const int n = polygon.size();
    for(int k = 0, j = 1; k <= n; k++) {
      int i = k % n;
      auto base = polygon[(i + 1) % n] - polygon[i];
      double len = base.Length();
      max_distance = std::abs(cross(polygon[j] - polygon[i], base) / len);
      for(;;) {
        double distance = std::abs(cross(polygon[(j + 1) % n] - polygon[i], base) / len);
        diameter = std::max(diameter, (polygon[j] - polygon[i]).Length());
        if (distance >= max_distance) {
          max_distance = distance;
          j = (j + 1) % n;
        } else break;
      }
      diameter = std::max(diameter, max_distance);
    }
    return diameter;
  }

  bool isCar() const {
    return area > 1 && cntCG > 10 && area * height > 2 && cirv > 0.2 && height < 2.5 && height > 1 && floor - contextGround < 1;
  }

  bool isPedestrain() const {
    return 0.15 < area && cntCG > 10 && cnt > 20 && area < 0.5 && 0.8 < height && height < 1.5 
      && floor - contextGround < 0.8 && cirv > 0.2;
  }

  std::string repr() const {
    std::ostringstream os;
    os << "area=" << area << " "
    << "density=" << density << "/" << cnt << " " 
    << "diameter=" << diameter << " "
    << "cirv=" << cirv << " "
    << "z=(" << floor - groundLevel << "," << ceiling - groundLevel << ") "
    << "distance=" << distance << " "
    << "ground=" << groundLevel << " "
    << "ctxg=" << contextGround << " " 
    << "tg=" << traceGround << " "
    << "ccg=" << cntCG;
    return os.str();
  }

  double setTraceGround(const std::vector<math::Vec3d> &points) {
    traceGround = 0;
    int cnt = 0;
    for(const auto &point : points) {
      double h = hmap.traceGround(point);
      if (h != hmap.inf) {
        traceGround += h;
        cnt++;
      }
    }
    traceGround /= (cnt + 1e-4);
    return traceGround;
  }

  int cnt, cntCG;
  double area, density, diameter, height, cirv, floor, ceiling;
  double groundLevel, distance, contextGround, grounded;
  double traceGround;
};

std::pair<double, double> getRange(const std::vector<math::Vec2d> &polygon, double theta) {
  double minv = std::numeric_limits<double>::max();
  double maxv = std::numeric_limits<double>::lowest();
  auto dir = math::Vec2d::FromUnit(theta);
  for (const auto &point : polygon) {
    double l = dir.InnerProd(point);
    minv = std::min(minv, l);
    maxv = std::max(maxv, l);
  }
  return std::make_pair(minv, maxv);
}

Box getBoundingBox(const std::vector<math::Vec2d> &polygon) {
  const int n = 10;
  Box bestBox;
  double area = std::numeric_limits<double>::max();
  for(int k = 0; k < n; k++) {
    double theta = M_PI / 2 * (double)k / n;
    double x1, x2, y1, y2;
    std::tie(x1, x2) = getRange(polygon, theta);
    std::tie(y1, y2) = getRange(polygon, theta + M_PI / 2);
    
    Box box(math::Vec2d((x1 + x2) / 2, (y1 + y2) / 2).Rotate(theta),
      x2 - x1, y2 - y1, theta);

    if (box.length * box.width < area) {
      area = box.length * box.width;
      bestBox = box;
    }
  }
  return bestBox;
}

/*
 * Theta is along the direction of long edge 
 */
Box refineBoundingBox(const std::vector<math::Vec2d> &polygon, double theta) {
  const int n = 2;
  HeightGrid::SearchResult best;
  best.score = std::numeric_limits<double>::lowest();
  for(int k = -n; k <= n; k++) {
    double t = theta + M_PI / 20 * k;
    double x1, x2, y1, y2;
    std::tie(x1, x2) = getRange(polygon, t);
    std::tie(y1, y2) = getRange(polygon, t + M_PI / 2);
    
    Box box(math::Vec2d((x1 + x2) / 2, (y1 + y2) / 2).Rotate(t),
      x2 - x1, y2 - y1, t);

    auto res = hmap.searchBox(box, 0, std::max(10.0, std::floor(box.length / 0.5) + 1), 5);
    if (res.score > best.score) {
      best = res;
    }
  }
  return best.box;
}


void setLabel(const std::vector<std::vector<math::Vec3d> > &groups, std::vector<PointCloudLabel> &labels,
  const std::vector<math::Vec3d> &points, 
  const math::Vec3d &lidar_pos) {
  int c = 0;
  // labels.clear();

  // for (int y = 0; y < hmap.rows; y++)
  //   for(int x = 0; x < hmap.cols; x++)
  //   { 
  //     double h = hmap.height[y][x];
  //     if (h < hmap.inf) {
  //       labels.emplace_back();

  //   }

  for (const auto& group : groups) {
    // for (const auto & point : group) {
    //   points.push_back(point);
    // } 
    if (group.size() > 10) {
      

      // labels.emplace_back();
      // auto &label = labels.back();
      PointCloudLabel label;
      
      for (const auto& point : group) {
        label.floor = std::min(label.floor, point.z);
        label.ceiling = std::max(label.ceiling, point.z);
      }
    
      label.polygon = convexHull(group);
      
      ObjectFeature feature(group, label);

      if (!(feature.area < 50
        && feature.floor < 1.0 + feature.groundLevel
        // && feature.ceiling < 4.0 + groundLevel
        && feature.height < 3.0 ) ) {
        // labels.pop_back(); 
      } else {

        int flag = 0;
        std::string gtid;

        if (feature.distance > 32) {
          // labels.pop_back();
          continue;
        }
        std::ostringstream ss;

        int pred = 0;
        if (feature.isCar() && feature.floor - feature.setTraceGround(group) < 1.0) {
          pred = 1;
        } else if (feature.isPedestrain()) {
          pred = 2;
        }
        if (flag == 1 || flag == 2) {
          if (flag != pred) {
            ss << "!#!" << c++ << "_" << gtid << "_" << pred;
            label.id = ss.str();
          } else {
            ss << "@@@" << c++ << "_" << gtid;
            label.id = ss.str();
          }
        } else {
          if (pred == 0) {
            // labels.pop_back();
            continue;
          }  else {
            if (pred == 2) {
              ss << "---" << c++;  
            } else {
              ss << "^^^" << c++;  
            }

            
            label.id = ss.str();
          }
        }

        auto box = getBoundingBox(label.polygon);
        // std::cout << label.id << " " << feature.repr() <<  std::endl;
        if (pred == 2) {
          PointCloudLabel label_box;
          box.width = box.length = 1;
          label_box.id = label.id + "_pedbox";
          label_box.floor = label.floor;
          label_box.ceiling = label.ceiling;
          label_box.polygon = box.asPolygon();
          label_box.label = pred;
          labels.push_back(label_box);
        }
        else if (pred == 1) {
          auto vis_dir = box.center - lidar_pos.xy();
          double vis_angle = std::atan2(vis_dir.y, vis_dir.x);
          // std::cout << vis_dir.x << " " << vis_dir.y << " " << box.length << " " << box.width << std::endl;
          // for(const auto &point : box.asPolygon()) {
          //   std::cout << "(" << point.x << "," << point.y << ")" << std::endl;
          // }
          double rot = std::min(std::abs(vis_angle - box.theta), std::min(std::abs(vis_angle - box.theta + M_PI),
              std::abs(vis_angle - box.theta - M_PI)));

          double r = box.length / box.width;

          auto x_vertical   = hmap.searchBox(box, vis_angle, 5, std::max(10.0, std::floor(box.width / 0.5) + 1));
          auto x_horizontal = hmap.searchBox(box, vis_angle, std::max(10.0, std::floor(box.length / 0.5) + 1), 5);
          
          if (r <= 2 && r >= 1 / 2) {
            if (rot < M_PI / 4) x_horizontal.score += 20;
            else x_vertical.score += 20;
          }

          // std::cout<< x_vertical.score << " " << x_horizontal.score << std::endl;

          Box x;
          if (x_vertical.score > x_horizontal.score) {
            x = x_vertical.box;
          }
          else {
            x = x_horizontal.box;
          } 

          // double theta = x.theta;
          // if (x.length < x.width) {
          //   theta -= M_PI / 2;
          // }

          // box = refineBoundingBox(label.polygon, theta);

          PointCloudLabel label_box;
          label_box.id = label.id + "_carbox";
          label_box.floor = label.floor;
          label_box.ceiling = label.ceiling;
          label_box.polygon = x.asPolygon();
          label_box.label = pred;
          labels.push_back(label_box);
        }

        
        // if  (flag) {
        //   PointCloudViewer::PointCloudLabel groundLabel;
        //   for(const auto & point : group) {
        //    if (hmap.plotGround(groundLabel, point)) {
        //     labels.push_back(groundLabel);
        //    }
        //   }
        // }

        // for (const auto & point : group) {
        //   points.push_back(point);
        // } 
      }
    }
  }
}

interface::perception::PerceptionObstacles convertPerceptionProto(
    const std::vector<PointCloudLabel> &labels) {

  interface::perception::PerceptionObstacles perception_result;
  // Add a mocked up obstacle.

  for(const auto &label : labels) {
    auto *obstacle = perception_result.add_obstacle();
    if (label.label == 1) {
      obstacle->set_type(interface::perception::ObjectType::CAR);  
    } else if (label.label == 2) {
      obstacle->set_type(interface::perception::ObjectType::PEDESTRIAN);  
    } else {
      obstacle->set_type(interface::perception::ObjectType::UNKNOWN_TYPE);  
    }
    
    for(const auto &point : label.polygon) {
      auto *polygon_point = obstacle->add_polygon_point();
      polygon_point->set_x(point.x);
      polygon_point->set_y(point.y);
      polygon_point->set_z(label.floor);
    }
    obstacle->set_height(label.ceiling - label.floor);
    obstacle->set_id(label.id);
  }

  return perception_result;
}


interface::perception::PerceptionObstacles Perception::RunPerception(
    const PointCloud& pointcloud_, const utils::Optional<cv::Mat>& image) {
  std::vector<math::Vec3d> points_, ground_points_;
  // points_.reserve(pointcloud_.points.size());
  math::Vec3d lidar_pos(pointcloud_.translation.x(),
    pointcloud_.translation.y(),
    pointcloud_.translation.z());
  for (const auto& point : pointcloud_.points) {
    Eigen::Vector3d point_in_world = pointcloud_.rotation * point + pointcloud_.translation;
    points_.emplace_back(point_in_world.x(), point_in_world.y(), point_in_world.z());
  }

  auto groups = hmap.removeGround(points_, lidar_pos, ground_points_);
  points_.clear();
  for(auto &group : groups)
    for(auto &p : group)
      points_.push_back(p);
  std::vector<PointCloudLabel> labels_;
  setLabel(groups, labels_, points_, lidar_pos);

  if (image) {
    // Remove me if you don't want to pause the program every time.
    cv::namedWindow("camera");
    imshow("camera", *image);
    cv::waitKey(0);
  }

  auto perception_result = convertPerceptionProto(labels_);
  static int c = 0;
  std::cout << "Frame" << c++ << std::endl;
  LOG(INFO) << "Perception done.";
  return perception_result;
}