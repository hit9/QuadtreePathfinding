// Source Code: https://github.com/hit9/quadtree-pathfinding
// License: BSD. Chao Wang, hit9[At]icloud.com.

#include "grid_map.hpp"

#include <algorithm>
#include <cstdlib>

namespace qdpf {
namespace naive {

// Array of { dx, dy }
const int DIRECTIONS[8][2]{
    // right,right-bottom,bottom,left-bottom
    {1, 0},
    {1, 1},
    {0, 1},
    {-1, 1},
    // left,left-top,top,top-right
    {-1, 0},
    {-1, -1},
    {0, -1},
    {1, -1},
};

NaiveGridMap::NaiveGridMap(int w, int h, ObstacleChecker isObstacle, DistanceCalculator distance)
    : w(w), h(h), s(std::max(w, h)), isObstacle(isObstacle), distance(distance) {
  g.Init();
  g.Resize(s * s);

  int costUnitHV = distance(0, 0, 0, 1);
  int costUnitDiagonal = distance(0, 0, 1, 1);

  for (int i = 0; i < 8; ++i) {
    int dx = directions[i][0] = DIRECTIONS[i][0];
    int dy = directions[i][1] = DIRECTIONS[i][1];
    directions[i][2] = (dx == 0 || dy == 0) ? costUnitHV : costUnitDiagonal;
  }
}

bool NaiveGridMap::IsObstacle(int x, int y) const {
  if (!(x >= 0 && x < w && y >= 0 && y < h)) return true;
  return isObstacle(x, y);
}

bool NaiveGridMap::IsObstacle(const Cell& c) const { return IsObstacle(c.first, c.second); }

int NaiveGridMap::Distance(int x1, int y1, int x2, int y2) const {
  return distance(x1, y1, x2, y2);
}

int NaiveGridMap::Distance(const Cell& c1, const Cell& c2) const {
  return distance(c1.first, c1.second, c2.first, c2.second);
}

int NaiveGridMap::Distance(int u, int v) const {
  auto [x1, y1] = UnpackXY(u);
  auto [x2, y2] = UnpackXY(v);
  return distance(x1, y1, x2, y2);
}

int NaiveGridMap::PackXY(int x, int y) const { return s * x + y; }
Cell NaiveGridMap::UnpackXY(int v) const {
  auto dv = std::div(v, s);
  return {dv.quot, dv.rem};  // {x,y}
}

void NaiveGridMap::Build() {
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      // We can check only the right | bottom directions
      // Since we're iterating from left-top to right-bottom.
      Update(x, y, 4);
    }
  }
}

void NaiveGridMap::Update(int x, int y, int maxd) {
  int u = PackXY(x, y);

  if (isObstacle(x, y)) {
    // can't walk from/to obstacles.
    g.ClearEdgeTo(u);
    g.ClearEdgeFrom(u);
  } else {
    for (int i = 0; i < maxd; ++i) {
      const auto& d = directions[i];
      int dx = d[0], dy = d[1], cost = d[2];
      int x1 = x + dx, y1 = y + dy;

      // can't walk from/to obstacles (including boundry).
      if (IsObstacle(x1, y1)) continue;

      int v = PackXY(x1, y1);
      g.AddEdge(u, v, cost);
      g.AddEdge(v, u, cost);
    }
  }
}

}  // namespace naive
}  // namespace qdpf
