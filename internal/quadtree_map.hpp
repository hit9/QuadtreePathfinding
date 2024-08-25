// Source Code: https://github.com/hit9/quadtree-pathfinding
// License: BSD. Chao Wang, hit9[At]icloud.com.

#ifndef QDPF_INTERNAL_QUADTREE_MAP_HPP
#define QDPF_INTERNAL_QUADTREE_MAP_HPP

#include <functional>  // for std::function

#include "graph.hpp"
#include "quadtree-hpp/quadtree.hpp"

// QuadtreeMap
// ~~~~~~~~~~~
// 2d grid map maintained by a quadtree, for:
// 1. 1x1 sized pathfinding agents.
// 2. 01 binary terrain types.

namespace qdpf {
namespace internal {

using ObstacleChecker = std::function<bool(int x, int y)>;
using DistanceCalculator = std::function<int(int x1, int y1, int x2, int y2)>;
using CellCollector = std::function<void(int x, int y)>;
using StepFunction = std::function<int(int length)>;

// QdTree is the type alias of a quadtree.
using QdTree = quadtree::Quadtree<bool>;
// QdNode is the type alias of a quadtree node.
using QdNode = quadtree::Node<bool>;
// QdNodeVisitor is the type of the function that visits quadtree nodes of the path finder.
using QdNodeVisitor = std::function<void(const QdNode *node)>;

// Gate between two adjacent quadtree nodes from cell a in aNode to cell b in bNode.
//  +-------+--------+
//  |    [a => b]    |
//  +-------+--------+
//    aNode   bNode
// 1. a and b are called 'gate cell's.
struct Gate {
  QdNode *aNode, *bNode;
  int a, b;
  Gate(QdNode *aNode, QdNode *bNode, int a, int b);
};

// GateVisitor the type of the function to visit gates.
using GateVisitor = std::function<void(const Gate *)>;

// Graph of gate cells.
using GateGraph = SimpleDirectedGraph;

class QuadtreeMapImpl {
 public:
  QuadtreeMapImpl(int w, int h, ObstacleChecker isObstacle, DistanceCalculator distance,
                  int step = 1, StepFunction stepf = nullptr, int maxNodeWidth = -1,
                  int maxNodeHeight = -1);
  ~QuadtreeMapImpl();

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
  int W() const { return w; }
  int H() const { return h; }
  int N() const { return n; }
  const QdTree &Tree() const { return tree; }
  // Returns the distance between two vertices u and v.
  int Distance(int u, int v) const;
  // Returns true if the given cell (x,y) is an obstacle.
  bool IsObstacle(int x, int y) const { return isObstacle(x, y); }
  // Approximate distance between two quadtree nodes.
  // Using the provided distance calculator on their center cells.
  int DistanceBetweenNodes(QdNode *aNode, QdNode *bNode) const;
  const GateGraph &GetGateGraph() const { return g2; }

  // ~~~~~~~~~~~~~ Visits and Reads ~~~~~~~~~~~~~~~~~

  // Get the quadtree node where the given cell (x,y) locates.
  QdNode *FindNode(int x, int y) const;
  // Is given cell u locating at given node a gate cell?
  bool IsGateCell(QdNode *node, int u) const;
  // Is given cell u is a gate?
  // higher level version method based on IsGateCell(node, u).
  bool IsGateCell(int u) const;
  // Visit each gate cell inside a node and call given visitor with it.
  void ForEachGateInNode(QdNode *node, GateVisitor &visitor) const;
  // Visit all the quadtree's leaf nodes.
  void Nodes(QdNodeVisitor &visitor) const;
  // Visit all gate cells.
  // Note that dual gates (a => b) and (b => a) are visited twice (once for each).
  void Gates(GateVisitor &visitor) const;
  // Visit reachable neighbor nodes for given node on the node graph.
  void ForEachNeighbourNodes(QdNode *node, NeighbourVertexVisitor<QdNode *> &visitor) const;

  // ~~~~~~~~~~~~~ Graphs Maintaining ~~~~~~~~~~~~~~~~~

  // Build the underlying quadtree right after construction.
  void Build();
  // Update should be called after any cell (x,y)'s value is changed.
  void Update(int x, int y);

 private:
  using NodeGraph = SimpleUnorderedMapDirectedGraph<QdNode *>;

  const int w, h, step;
  const int s;  // max side of (w,h)
  const int n;  // w x h, number of cells
  const int maxNodeWidth, maxNodeHeight;

  ObstacleChecker isObstacle;
  DistanceCalculator distance;
  StepFunction stepf;

  // the quadtree on this grid map.
  QdTree tree;

  // ~~~~~~~~~~~~~~~ Graphs ~~~~~~~~~~~
  // the 1st level abstract graph: graph  of nodes.
  NodeGraph g1;
  // the 2st level abstract graph: graph of gate cells.
  GateGraph g2;

  // ~~~~~~~~~~~~~~ Gates ~~~~~~~~~~~~~
  // manages memory of gates.
  std::unordered_set<Gate *> gates;
  // gates group by node for faster quering.
  // gates1[node][a] => set of gates
  // there may exist 1~3 gates starting from a cell.
  // for an example below, the 3 gates are:
  // (a => c), (a => b) and (a => d).
  //   b | c
  //   --+--
  //   a | d
  using GateSet = std::unordered_set<Gate *>;
  std::unordered_map<QdNode *, std::unordered_map<int, GateSet>> gates1;

  // ~~~~~~~~~~~~~~~~ Internals ~~~~~~~~~~~~~~~
  void forEachGateInNode(QdNode *node, std::function<void(Gate *)> &visitor) const;
  void handleNewNode(QdNode *aNode);
  void handleRemovedNode(QdNode *aNode);
  void connectCellsInGateGraphs(int u, int v);
  void connectGateCellsInNodeToNewGateCell(QdNode *aNode, int a);
  void disconnectCellInGateGraphs(int u);
  void disconnectCellsInNodeFromGateGraphs(QdNode *aNode);
  void connectNodesOnNodeGraph(QdNode *aNode, QdNode *bNode);
  void disconnectNodeFromNodeGraph(QdNode *aNode);
  void createGate(QdNode *aNode, int a, QdNode *bNode, int b);
  void removeGateInNode(QdNode *aNode, int a, QdNode *bNode, int b);
  void getNeighbourCellsDiagonal(int direction, QdNode *aNode, int &a, int &b) const;
  void getNeighbourCellsHV(int direction, QdNode *aNode, QdNode *bNode,
                           std::vector<std::pair<int, int>> &ncs) const;
};

}  // namespace internal
}  // namespace qdpf

#endif
