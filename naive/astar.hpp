// Source Code: https://github.com/hit9/quadtree-pathfinding
// License: BSD. Chao Wang, hit9[At]icloud.com.

// Naive A* path finding implementation on 2D grid map.
// **NOTE**: This file is NOT required to use quadtree-pathfinding.
// It's for debuging and testing purpose.

#ifndef QDPF_NAIVE_ASTAR_HPP
#define QDPF_NAIVE_ASTAR_HPP

#include <functional>

#include "../internal/base.hpp"
#include "grid_map.hpp"

namespace qdpf {

namespace naive {

using internal::Cell;

using Path = std::vector<Cell>;
using PathCollector = std::function<void(int x, int y, int cost)>;

class NaiveAStarPathFinder {
 public:
  // Find a shortest path from (x1,y1) to (x2,y2) on given map m.
  // Returns 0 on success.
  // Returns -1 on failure.
  int Compute(const NaiveGridMap* m, int x1, int y1, int x2, int y2, PathCollector& collector);
  int Compute(const NaiveGridMap* m, int x1, int y1, int x2, int y2, Path& path);
};

}  // namespace naive

}  // namespace qdpf
#endif
