// Hierarchical astar path finding on quadtree for equal-weighted 2D grid map.
// Source Code: https://github.com/hit9/quadtree-astar
// License: BSD. Version: 0.1.0. Author: Chao Wang, hit9[At]icloud.com.
// Quadtree reference: https://github.com/hit9/quadtree-hpp
// 1. An abstract graph is maintained dynamically based on a quadtree.
// 2. A* performas path finding on the abstract graph.

#ifndef QUADTREE_ASTAR_HPP
#define QUADTREE_ASTAR_HPP

#include <cmath>
#include <functional>
#include <unordered_map>
#include <utility>
#include <vector>

#include "quadtree-hpp/quadtree.hpp"

namespace quadtree_astar {

// QdTree is the type alias of a quadtree.
using QdTree = quadtree::Quadtree<bool>;
// QdNode is the type alias of a quadtree node.
using QdNode = quadtree::Node<bool>;

// ObstacleChecker is the type of the function that returns true if the given
// cell (x,y) is an obstacle.
using ObstacleChecker = std::function<bool(int x, int y)>;

// DistanceCalculator is the type of the function that calculates the distance
// from cell (x1,y1) to (x2,y2).
// The distance calculator should guarantee that the distance between (x1,y1)
// and (x2,y2) always equals to the distance between (x2,y2) and (x1,y1).
using DistanceCalculator = std::function<int(int x1, int y1, int x2, int y2)>;

// CellCollector is the type of the function that collects points on a path.
// The argument (x,y) is a cell in the grid map.
using CellCollector = std::function<void(int x, int y)>;

// QdNodeCollector is the type of the function that collects quadtree nodes of the path finder.
using QdNodeCollector = std::function<void(const QdNode *node)>;

// Euclidean distance calculator with given cost unit.
template <int CostUnit>
int EuclideanDistance(int x1, int y1, int x2, int y2) {
  return std::floor(std::hypot(abs(x1 - x2), abs(y1 - y2)) * CostUnit);
}

// Gate (a => b), where a and b is the cell id.
// A Gate always belongs to a node, here is the aNode,
// and is determined by the key (a, bNode).
//
//  +-------+--------+
//  |    [a => b]    |
//  +-------+--------+
//    aNode   bNode
struct Gate {
  // cell in aNode.
  int a;
  // the other node.
  QdNode *bNode;
  // the other connected end in b.
  int b;
  // the key is (a, bNode).
  bool operator==(const Gate &other) const { return a == other.a && bNode == other.bNode; }
};

// Hashing support for gate, uniqued by (a, bNode).
struct GateHasher {
  std::size_t operator()(const Gate &g) const;
};

// A set of gates inside a node.
using GateSet = std::unordered_set<Gate, GateHasher>;

class PathFinder {
 public:
  PathFinder(
      int w, int h,                 // width and height of the grid map
      ObstacleChecker isObstacle,   // isObstacle(x,y) returns true if cell (x,y) is an obstacle
      DistanceCalculator distance,  // the function calculates the distance between too cells
      int step = 1,                 // number of interval cells when picking gate cells.
      bool use4directions = false   // use 4 (N,E,S,W) or 8 directinos (including diagonal)
  );
  // Build the underlying quadtree right after construction.
  void Build();
  // Update should be called after a cell (x,y)'s value is changed.
  void Update(int x, int y);
  // ComputeRoutes computes the route cells from (x1,y1) to (x2,y2) based on A* algorithm.
  // The route cells are composed of three kinds of cells:
  // start(x1,y1), gate cells, target(x2,y2).
  void ComputeRoutes(int x1, int y1, int x2, int y2, CellCollector &collector);
  // ComputePathToNextRoute computes the detailed cells from current route (x1,y1) to next route
  // (x2,y2). Notes that the (x1,y1) won't be collected, and the (x2,y2) will.
  void ComputePathToNextRoute(int x1, int y1, int x2, int y2, CellCollector &collector) const;

  // ~~~~~~~~~~ Debuging Purpose ~~~~~~~~~~~~
  // Collects the quadtree's leaf nodes.
  void Nodes(QdNodeCollector &collector) const;
  // Collects all gate cells.
  void Gates(CellCollector &collector) const;

 private:
  int packxy(int x, int y) const;
  std::pair<int, int> unpackxy(int v) const;
  int unpackx(int v) const;
  int unpacky(int v) const;

  int calcDistance(int u, int v) const;
  void addVertex(QdNode *node, int v);
  void removeVertex(QdNode *node, int v);
  void addConnection(QdNode *aNode, int a, QdNode *bNode, int b);
  void handleNewNode(QdNode *aNode);
  void handleRemovedNode(QdNode *aNode);
  void getNeighbourCellsDiagonal(int direction, QdNode *aNode, int &a, int &b);
  void getNeighbourCellsHV(int direction, QdNode *aNode, QdNode *bNode,
                           std::vector<std::pair<int, int>> &ncs);

 private:
  using P = std::pair<int, int>;

  const int w, h, step;
  const int m;  // max of (w,h)
  const int n;  // w x h
  const bool use4directions;

  ObstacleChecker isObstacle;
  DistanceCalculator distance;

  // the quadtree on this grid map.
  QdTree tree;

  // ~~~~~~~ Graph ~~~~~~~~
  // edges[v] => { v =>  cost(u => v) }
  std::vector<std::unordered_map<int, int>> edges;

  // ~~~~~~~ Gates (group by nodes) ~~~~~~~~
  // Stores all the gate cells, groupped by node.
  std::unordered_map<QdNode *, GateSet> gates;

  // ~~~~~~~ A* context (for reusing memory allocation purpose) ~~~~~~~~
  // f[v] is the shortest path from s to v.
  std::vector<int> f;
  // vis[v] is true if A* has marked v.
  std::vector<bool> vis;
  // from[v] stores the upstream vertex for v.
  std::vector<int> from;
};

}  // namespace quadtree_astar

#endif
