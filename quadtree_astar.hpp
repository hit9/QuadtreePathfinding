// Hierarchical path finding on quadtree for equal-weighted 2D grid map.
// License: BSD. Version: 0.1.0. Author: Chao Wang, hit9[At]icloud.com.
// Source Code: https://github.com/hit9/quadtree-astar
// Quadtree reference: https://github.com/hit9/quadtree-hpp
//
// Concepts and Mechanisms
// ~~~~~~~~~~~~~~~~~~~~~~~
//
// 1. A QuadtreeMap is a 2D grid map maintained by a quadtree.
// 2. The quadtree splits the grid map into multiple sections.
// 3. A section contains no obstacles or all obstacles.
// 4. In a section without any obstacles, the shortest path inside it will be a straight line.
// 4. There are multiple connections between adjacent quadtree nodes (aka sections).
// 5. And a connection is composed of two adjacent gate cells, one on each side.
// 6. All gates compose an abstract graph, and the path finder works on it.
// 7. We first find the route cells (start, gates and target), and the fill the straight lines
//    between them.
//
// Coordinates
// ~~~~~~~~~~~
//    0      w
//  0 +---------------> y
//    |
// h  |
//    |
//    v
//    x
//
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
//   // Bind them and build the tree.
//   m.RegisterGraph(pf.GetGraph());
//   m.Build();
//
//   // Compute routes
//   pf.ComputeRoutes(x1,y1,x2,y2, collector);
//
//   // Fill the detailed path.
//   pf.ComputePathToNextRoute()

#ifndef QUADTREE_ASTAR_HPP
#define QUADTREE_ASTAR_HPP

#include <cmath>          // for std::floor, std::hypot
#include <functional>     // for std::function
#include <unordered_map>  // for std::unordered_map
#include <utility>        // for std::pair
#include <vector>         // for std::vector

#include "quadtree-hpp/quadtree.hpp"

namespace quadtree_astar {

// QdTree is the type alias of a quadtree.
using QdTree = quadtree::Quadtree<bool>;
// QdNode is the type alias of a quadtree node.
using QdNode = quadtree::Node<bool>;

// ObstacleChecker is the type of the function that returns true if the given
// cell (x,y) is an obstacle.
using ObstacleChecker = std::function<bool(int x, int y)>;

// Euclidean distance calculator with a given cost unit.
template <int CostUnit>
int EuclideanDistance(int x1, int y1, int x2, int y2) {
  return std::floor(std::hypot(abs(x1 - x2), abs(y1 - y2)) * CostUnit);
}

// DistanceCalculator is the type of the function that calculates the distance
// from cell (x1,y1) to (x2,y2).
// The distance calculator should guarantee that the distance between (x1,y1)
// and (x2,y2) always equals to the distance between (x2,y2) and (x1,y1).
// We can just use quadtree_astar::EuclideanDistance<CostUnit> to build a euclidean distance
// calculator.
using DistanceCalculator = std::function<int(int x1, int y1, int x2, int y2)>;

// CellCollector is the type of the function that collects points on a path.
// The argument (x,y) is a cell in the grid map.
using CellCollector = std::function<void(int x, int y)>;

// QdNodeCollector is the type of the function that collects quadtree nodes of the path finder.
using QdNodeCollector = std::function<void(const QdNode *node)>;

// NeighbourVertexVisitor is the type of the function that visit neighbor vertices of a given
// vertex in a directed graph.
using NeighbourVertexVisitor = std::function<void(int v, int cost)>;

// IDirectedGraph is the interface of a directed graph.
// It's a pure virtual class so that the subclasses should implement all the virtual methods.
class IDirectedGraph {
 public:
  // Initialize the graph, where the n is the total number of vertices.
  // This method should be called by a path finder.
  virtual void Init(int n) = 0;
  // Add an edge from vertex u to v with given cost.
  virtual void AddEdge(int u, int v, int cost) = 0;
  // Remove an edge from vertex u to v.
  virtual void RemoveEdge(int u, int v) = 0;
  // Clears all edges starting from vertex u.
  virtual void ClearEdgeFrom(int u) = 0;
  // Call given visitor function for each neighbor vertex of given vertex u.
  virtual void ForEachNeighbours(int u, NeighbourVertexVisitor &visitor) const = 0;
  // Clears all edges.
  virtual void Clear() = 0;
};

// SimpleDirectedGraph is a simple implementation of IDirectedGraph.
class SimpleDirectedGraph : public IDirectedGraph {
 public:
  void Init(int n) override;
  void AddEdge(int u, int v, int cost) override;
  void RemoveEdge(int u, int v) override;
  void ClearEdgeFrom(int u) override;
  void ForEachNeighbours(int u, NeighbourVertexVisitor &visitor) const override;
  void Clear() override;

 protected:
  // edges[v] => { v =>  cost(u => v) }
  std::vector<std::unordered_map<int, int>> edges;
};

// SimpleUnorderedMapDirectedGraph is a simple directed graph storing in an unordered_map.
// This uses less memory than SimpleDirectedGraph for sparse graph.
class SimpleUnorderedMapDirectedGraph : public IDirectedGraph {
 public:
  void Init(int n) override;
  void AddEdge(int u, int v, int cost) override;
  void RemoveEdge(int u, int v) override;
  void ClearEdgeFrom(int u) override;
  void ForEachNeighbours(int u, NeighbourVertexVisitor &visitor) const override;
  void Clear() override;

 protected:
  // edges[v] => { v =>  cost(u => v) }
  std::unordered_map<int, std::unordered_map<int, int>> edges;
};

// QuadtreeMap maintains a 2D grid map by a quadtree.
class QuadtreeMap {
 public:
  // Parameters:
  // * w, h: width and height of the grid map.
  // * isObstacle(x,y) returns true if cell (x,y) is an obstacle, it should be fast.
  // * distance: the function calculates the distance between two cells.
  // * step: number of interval cells when picking gate cells at N(0)/E(1)/S(2)/W(3) sides.
  // * maxNodeWidth, maxNodeHeight: the max width and height of a quadtree node's rectangle.
  QuadtreeMap(int w, int h, ObstacleChecker isObstacle, DistanceCalculator distance, int step = 1,
              int maxNodeWidth = -1, int maxNodeHeight = -1);
  // ~~~~~~~~~~~~~~~ Cell ID Packing ~~~~~~~~~~~
  // PackXY packs a cell position (x,y) to an integral id v.
  int PackXY(int x, int y) const;
  // UnpackXY unpacks a vertex id v to a two-dimensional position (x,y).
  std::pair<int, int> UnpackXY(int v) const;
  // Unpacks a cell id v's x axis.
  int UnpackX(int v) const;
  // Unpacks a cell id v's y axis.
  int UnpackY(int v) const;
  // ~~~~~~~~~~~~~ Basic methods ~~~~~~~~~~~~~~~~~
  // Returns the width of the map.
  int W() const { return w; }
  // Returns the height of the map.
  int H() const { return h; }
  // Returns the number of cells of the map.
  int N() const { return n; }
  // Returns the distance between two vertices u and v.
  int Distance(int u, int v) const;
  // Returns true if the given cell (x,y) is an obstacle.
  bool IsObstacle(int x, int y) const;
  // Register a directed graph and keep it updated synchronously with the quadtree map
  // A quadtree map can register multiple directed graphs.
  void RegisterGraph(IDirectedGraph *g);
  // ~~~~~~~~~~~~~ Gates and Nodes ~~~~~~~~~~~~~~~~~
  // Get the quadtree node for given cell (x,y).
  QdNode *FindNode(int x, int y) const;
  // Is given cell u locating at given node a gate?
  bool IsGate(QdNode *node, int u) const;
  // Visit each gate cell inside a node and call given visitor with it.
  void ForEachGateInNode(QdNode *node, std::function<void(int u)> &visitor) const;
  // ~~~~~~~~~~~~~ Map Maintaining ~~~~~~~~~~~~~~~~~
  // Build the underlying quadtree right after construction.
  void Build();
  // Update should be called after any cell (x,y)'s value is changed.
  void Update(int x, int y);
  // ~~~~~~~~~~ Debuging Purpose ~~~~~~~~~~~~
  // Collects the quadtree's leaf nodes.
  void Nodes(QdNodeCollector &collector) const;
  // Collects all gate cells.
  void Gates(CellCollector &collector) const;

 private:
  const int w, h, step;
  const int s;  // max side of (w,h)
  const int n;  // w x h
  const int maxNodeWidth, maxNodeHeight;
  ObstacleChecker isObstacle;
  DistanceCalculator distance;
  // the quadtree on this grid map.
  QdTree tree;
  // the abstract graphs to update
  std::vector<IDirectedGraph *> graphs;

  // ~~~~~~~ Gates (group by nodes) ~~~~~~~~
  // Stores all the gate cells, group by node.
  //
  // What a gate looks like:
  //
  //  +-------+--------+
  //  |    [a => b]    |
  //  +-------+--------+
  //    aNode   bNode
  //
  // Format: gates[aNode][a][b] => bNode.
  std::unordered_map<QdNode *, std::unordered_map<int, std::unordered_map<int, QdNode *>>> gates;

  // ~~~~~~~~~~~~~~ Internals ~~~~~~~~~~~~~~~
  void addVertex(QdNode *node, int v);
  void addConnection(QdNode *aNode, int a, QdNode *bNode, int b);
  void handleNewNode(QdNode *aNode);
  void handleRemovedNode(QdNode *aNode);
  void getNeighbourCellsDiagonal(int direction, QdNode *aNode, int &a, int &b) const;
  void getNeighbourCellsHV(int direction, QdNode *aNode, QdNode *bNode,
                           std::vector<std::pair<int, int>> &ncs) const;
};

// IPathFinder is the base class for all path finder implementations.
class IPathFinder {
 public:
  // Returns the pointer to the path finder's graph .
  virtual IDirectedGraph *GetGraph() = 0;
};

// PathFinderHelper is a mixin class to provide some util functions.
class PathFinderHelper {
 public:
  PathFinderHelper(const QuadtreeMap &m, IDirectedGraph *g);
  // ComputePathToNextRoute computes the detail cells from current route (x1,y1) to next route
  // (x2,y2). Note that the (x1,y1) and the (x2,y2) will both be collected.
  // The default implementation is based on Bresenham's line algorithm.
  // You can override it with a custom implementation.
  // Ref: https://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm
  // Ref: https://members.chello.at/easyfilter/bresenham.html
  virtual void ComputePathToNextRoute(int x1, int y1, int x2, int y2,
                                      CellCollector &collector) const;

 protected:
  const QuadtreeMap &m;
  IDirectedGraph *g;
  // tmp graph is to store edges between start/target and other gate cells.
  SimpleUnorderedMapDirectedGraph tmp;

  // BuildTmpGraph builds the temporary graph to store edges between start/target and other gates
  // in the same nodes. This is a helper function.
  // Parameters:
  // 1. s and t are the ids of cell start and target.
  // 2. (x1,y1) and (x2,y2) are the positions of cell start and target.
  void BuildTmpGraph(int s, int t, int x1, int y1, int x2, int y2);
  // ForEachNeighboursWithST iterates each neighbor vertex connecting from given vertex u.
  // What's the deference with the graph's ForEachNeighbours is: it will check both the grid map's
  // abstract graph and the temporary graph (storing the start,target info).
  void ForEachNeighboursWithST(int u, NeighbourVertexVisitor &visitor) const;
  // helper function to add a vertex u to the given node in the temporary graph.
  void addVertexToTmpGraph(int u, QdNode *node);
};

// AStarPathFinder implements path finding on a QuadtreeMap based on A* algorithm.
class AStarPathFinder : public IPathFinder, public PathFinderHelper {
 public:
  AStarPathFinder(const QuadtreeMap &m);
  // ~~~~~~~~~~~~~~ Implements IPathFinder ~~~~~~~~~~~~~~
  IDirectedGraph *GetGraph() override;
  // ~~~~~~~~~~~~~~ API ~~~~~~~~~~~~~~
  // ComputeRoutes computes the route cells from (x1,y1) to (x2,y2).
  // The route cells are composed of three kinds of cells:
  // start(x1,y1), gate cells in the middle and target(x2,y2).
  // Returns -1 if the path finding is failed.
  // Returns the distance of the shortest path on success (>=0).
  int ComputeRoutes(int x1, int y1, int x2, int y2, CellCollector &collector);

 protected:
  // P is a pair of integers.
  using P = std::pair<int, int>;

  SimpleDirectedGraph g;

  // ~~~~~~~~ A* context (reusing for less reallocation) ~~~~~~~
  std::vector<int> f;
  std::vector<bool> vis;
  std::vector<int> from;
};

}  // namespace quadtree_astar

#endif
