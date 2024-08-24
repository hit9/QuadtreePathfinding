// Hierarchical path finding on quadtree for equal-weighted 2D grid map.
// License: BSD. Version: 0.2.0. Author: Chao Wang, hit9[At]icloud.com.
// Source Code: https://github.com/hit9/quadtree-pathfinding
// Quadtree reference: https://github.com/hit9/quadtree-hpp

// Concepts and Mechanisms
// ~~~~~~~~~~~~~~~~~~~~~~~
// 1. A QuadtreeMap is a 2D grid map maintained by a quadtree.
// 2. The quadtree splits the grid map into multiple sections.
// 3. A section contains no obstacles or all obstacles.
// 4. The shortest path inside a section without obstacles will be a straight line.
// 5. Adjacent quadtree nodes are connected by multiple gates.
// 6. A gate is composed of two adjacent cells, one on each side, directed.
// 7. All nodes compose the 1st level abstract graph.
// 8. All gates compose the 2nd level abstract graph.
// 9. Path finding performs on the 2 or 3 levels graphs:
//      1. Find the node path on the 1st level graph (it's optional, faster but less optimal).
//      2. Find the gate path on the 2nd level graph.
//      3. Fill the straight lines between gate cells.

// Coordinates
// ~~~~~~~~~~~
//    0      w
//  0 +---------------> y
//    |
// h  |
//    |
//    v
//    x

// Code Example
// ~~~~~~~~~~~~
//
//   // Setup the map
//   auto isObstacle = [](int x, int y) { return grid[x][y]; };
//   auto distance = quadtree_astar::EuclideanDistance<10>;
//   quadtree_astar::QuadtreeMap m(w, h, isObstacle, distance);
//
//   // Setup an A* path finder.
//   quadtree_astar::AStarPathFinder pf(m);
//
//   // Register the path finder and build the map.
//   m.RegisterGateGraph(&pf);
//   m.Build();
//
//   // Resets the start(x1,y1) and target(x2,y2).
//   pf.Reset(x1,y1,x2,y2);
//
//   // Compute the node path (optional).
//   // Returns -1 if unreachable.
//   pf.ComputeNodeRoutes();
//
//   // Compute the gate routes.
//   // the 2nd parameter specifics whether to use the computed node path.
//   pf.ComputeGateRoutes(collector, true);
//
//   // Fill the detailed path from current position to next route cell.
//   pf.ComputePathToNextRoute(x1,y1, x2,y2);

#ifndef QDPF_HPP
#define QDPF_HPP

#include <functional>  // for std::function, std::hash

#include "qdpf_internal.hpp"

namespace qdpf {

using internal ::inf;

//////////////////////////////////////
/// Graph
//////////////////////////////////////

// IDirectedGraph is the interface of a directed graph.
// It's a pure virtual class so that the subclasses should implement all the virtual methods.
// The parameter Vertex is the type of vertex (e.g. int).
template <typename Vertex>
using IDirectedGraph = internal::IDirectedGraph<Vertex>;

//////////////////////////////////////
/// QuadtreeMap
//////////////////////////////////////

// ObstacleChecker is the type of the function that returns true if the given
// cell (x,y) is an obstacle.
// std::function<bool(int x, int y)>
using ObstacleChecker = internal::ObstacleChecker;

// DistanceCalculator is the type of the function that calculates the distance
// from cell (x1,y1) to (x2,y2).
// The distance calculator should guarantee that the distance between (x1,y1)
// and (x2,y2) always equals to the distance between (x2,y2) and (x1,y1).
// We can just use quadtree_astar::EuclideanDistance<CostUnit> to build a euclidean distance
// calculator.
// std::function<int(int x1, int y1, int x2, int y2)>;
using DistanceCalculator = internal::DistanceCalculator;

// Euclidean distance calculator with a given cost unit.
template <int CostUnit>
int EuclideanDistance(int x1, int y1, int x2, int y2) {
  return std::floor(std::hypot(x1 - x2, y1 - y2) * CostUnit);
}

// CellCollector is the type of the function that collects points on a path.
// The argument (x,y) is a cell in the grid map.
// std::function<void(int x, int y)>;
using CellCollector = internal::CellCollector;

// StepFunction is the type of a function to specific a dynamic gate picking step.
// The argument length is the length (number of cells) of the adjacent side of two neighbor nodes.
// We should make sure the return value is always > 0.
// An example: [](int length) { return length / 8 + 1; }
// For this example, we use larger step on large rectangles, and smaller step on small rectangles.
// std::function<int(int length)>;
using StepFunction = internal::StepFunction;

// NodeVisitor is the type of a function to visit quadtree nodes.
// Where (x1,y1) and (x2,y2) are the left-top and right-bottom corner cells of the visited node.
using NodeVisitor = std::function<void(int x1, int y1, int x2, int y2)>;

// GateVisitor is the type of a function to visit gates.
// Where (x1,y1) and (x2,y2) are the start and end cell of the gate.
using GateVisitor = std::function<void(int x1, int y1, int x2, int y2)>;

// Graph of gate cells.
using GateGraph = internal::GateGraph;

// QuadtreeMap maintains a 2D grid map by a quadtree.
class QuadtreeMap {
 public:
  // * w, h: width and height of the grid map.
  // * isObstacle(x,y) returns true if cell (x,y) is an obstacle, it should be fast.
  // * distance: the function calculates the distance between two cells.
  // * step: number of interval cells when picking gate cells at N(0)/E(1)/S(2)/W(3) sides.
  // * stepf: dynamic step function, checkout StepFunction's document. Use it if it's set,
  //   otherwise use the constant step instead.
  // * maxNodeWidth, maxNodeHeight: the max width and height of a quadtree node's rectangle.
  QuadtreeMap(int w, int h, ObstacleChecker isObstacle, DistanceCalculator distance, int step = 1,
              StepFunction stepf = nullptr, int maxNodeWidth = -1, int maxNodeHeight = -1);
  ~QuadtreeMap();

  // ~~~~~~~~~~~~~ Basic methods ~~~~~~~~~~~~~~~~~

  // Returns the width of the map.
  int W() const;
  // Returns the height of the map.
  int H() const;
  // Returns the number of cells of the map.
  int N() const;
  // Returns the number of nodes.
  int NumNodes() const;
  // Returns the number of leaf nodes.
  int NumLeafNodes() const;
  // Returns the depth of the tree.
  int TreeDepth() const;

  // Register a directed gate graph and keep it updated synchronously with the quadtree map
  // A quadtree map can register multiple gate graphs.
  void RegisterGateGraph(GateGraph *g);

  // ~~~~~~~~~~~~~ Debuging Purpose ~~~~~~~~~~~~~~~~~
  // Visit all the quadtree's leaf nodes.
  void Nodes(NodeVisitor &visitor) const;
  // Visit all gate cells.
  // Note that dual gates (a => b) and (b => a) are visited twice (once for each).
  void Gates(GateVisitor &visitor) const;

  // ~~~~~~~~~~~~~ Graphs Maintaining ~~~~~~~~~~~~~~~~~

  // Build the underlying quadtree right after construction.
  void Build();
  // Update should be called after any cell (x,y)'s value is changed.
  void Update(int x, int y);

 private:
  internal::QuadtreeMapImpl *pImpl;
  friend class IPathFinder;
};

//////////////////////////////////////
/// PathFinder
//////////////////////////////////////

// IPathFinder is the base class for all path finder implementations.
class IPathFinder {
 public:
  IPathFinder(const QuadtreeMap &m);
  // Returns the pointer to the path finder's gate graph.
  // We allow different path finders maintain their own gate graphs.
  virtual IDirectedGraph<int> *GetGateGraph() = 0;

 protected:
  const internal::QuadtreeMapImpl *QuadtreeMapImpl();

 private:
  const QuadtreeMap &m;
};

//////////////////////////////////////
/// AStarPathFinder
//////////////////////////////////////

class AStarPathFinder : public IPathFinder {
 public:
  AStarPathFinder(const QuadtreeMap &m);
  ~AStarPathFinder();
  // ~~~~~~~~~~~~~~ Implements IPathFinder ~~~~~~~~~~~~~~
  IDirectedGraph<int> *GetGateGraph() override;
  // ~~~~~~~~~~~~~~ API ~~~~~~~~~~~~~~
  // Returns the count of quadtree nodes on the path.
  std::size_t NodePathSize() const;
  // Sets the start(x1,y1) and target(x2,y2).
  void Reset(int x1, int y1, int x2, int y2);
  // ComputeNodeRoutes computes the path of quadtree nodes from the start cell's node to the target
  // cell's node on the node graph.
  // Returns -1 if unreachable.
  // Returns the approximate cost to target node on the node graph level.
  // This step is optional, the benefits to use it ahead of ComputeGateRoutes:
  // 1. faster (but less optimal).
  // 2. fast checking if the target is reachable.
  int ComputeNodeRoutes();
  // Visit computed node path.
  void VisitComputedNodeRoutes(NodeVisitor &visitor) const;
  // ComputeGateRoutes computes the route cells from (x1,y1) to (x2,y2).
  // Sets useNodePath to true to use the previous ComputeNodeRoutes results, it will find path
  // only over gate cells on the node path, this the path finding is much faster, but less optimal.
  // Sets useNodePath to false to disable this optimization, it will find path over all gate cells.
  // The route cells are composed of three kinds of cells: start(x1,y1), gate cells in the middle
  // and target(x2,y2).
  // Returns -1 if the path finding is failed.
  // Returns the distance of the shortest path on success (>=0).
  int ComputeGateRoutes(CellCollector &collector, bool useNodePath = true);
  // ComputePathToNextRoute computes the detail cells from current route cell (x1,y1) to next route
  // cell (x2,y2). Note that the (x1,y1) and the (x2,y2) will both be collected. The default
  // implementation is based on Bresenham's line algorithm.
  // Ref: https://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm
  // Ref: https://members.chello.at/easyfilter/bresenham.html
  void ComputePathToNextRouteCell(int x1, int y1, int x2, int y2, CellCollector &collector) const;

 private:
  internal::AStarPathFinderImpl *pImpl;
};

}  // namespace qdpf

#endif
