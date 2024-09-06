// Source Code: https://github.com/hit9/quadtree-pathfinding
// License: BSD. Chao Wang, hit9[At]icloud.com.

// 2D grid map in format of a graph.
// **NOTE**: This file is NOT required to use quadtree-pathfinding.
// It's for debuging and testing purpose.

#ifndef QDPF_NAIVE_MAP_HPP
#define QDPF_NAIVE_MAP_HPP

#include "../internal/base.hpp"
#include "../internal/graph.hpp"
#include "../internal/quadtree_map.hpp"

namespace qdpf {
namespace naive {

using internal::Cell;
using internal::DistanceCalculator;
using internal::ObstacleChecker;
using internal::PairHasher;
using NaiveGridGraph = internal::SimpleDirectedGraph;

class NaiveGridMap {
 public:
  NaiveGridMap(int w, int h, ObstacleChecker isObstacle, DistanceCalculator distance);

  bool IsObstacle(int x, int y) const;
  bool IsObstacle(const Cell& c) const;

  int Distance(int x1, int y1, int x2, int y2) const;
  int Distance(const Cell& c1, const Cell& c2) const;
  int Distance(int u, int v) const;

  const NaiveGridGraph& GetGraph() const { return g; }

  int PackXY(int x, int y) const;
  Cell UnpackXY(int v) const;

  // Build on a an existing 2D grid map.
  void Build();
  void Update(int x, int y);

 private:
  const int w, h, s;
  NaiveGridGraph g;

  ObstacleChecker isObstacle;
  DistanceCalculator distance;

  // directions[i] => {dx, dy, cost}
  int directions[8][3];
};

}  // namespace naive
}  // namespace qdpf

#endif
