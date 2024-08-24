// Source Code: https://github.com/hit9/quadtree-pathfinding
// License: BSD. Chao Wang, hit9[At]icloud.com.

#ifndef QDPF_INTERNAL_HPP
#define QDPF_INTERNAL_HPP

#include <functional>     // for std::function, std::hash
#include <queue>          // for std::priority_queue
#include <unordered_map>  // for std::unordered_map
#include <unordered_set>  // for std::unordered_set
#include <utility>        // for std::pair
#include <vector>         // for std::vector

#include "qdpf.hpp"

namespace qdpf {

//////////////////////////////////////
/// Graph
//////////////////////////////////////

// NeighbourVertexVisitor is the type of the function that visit neighbor vertices of a given
// vertex in a directed graph.
template <typename Vertex>
using NeighbourVertexVisitor = std::function<void(Vertex v, int cost)>;

template <typename Vertex>
class IDirectedGraph {
 public:
  // Initialize the graph, where the n is the total number of vertices.
  // This method should be called by a path finder.
  virtual void Init(int n) = 0;
  // Add an edge from vertex u to v with given cost.
  virtual void AddEdge(Vertex u, Vertex v, int cost) = 0;
  // Remove an edge from vertex u to v.
  virtual void RemoveEdge(Vertex u, Vertex v) = 0;
  // Clears all edges starting from vertex u.
  virtual void ClearEdgeFrom(Vertex u) = 0;
  // Clears all edges connectinig to vertex v.
  virtual void ClearEdgeTo(Vertex v) = 0;
  // Call given visitor function for each neighbor vertex v connecting from given vertex u.
  virtual void ForEachNeighbours(Vertex u, NeighbourVertexVisitor<Vertex> &visitor) const = 0;
  // Clears all edges.
  virtual void Clear() = 0;
};

// SimpleDirectedGraph is a simple implementation of IDirectedGraph, using integral vertex.
class SimpleDirectedGraph : public IDirectedGraph<int> {
 public:
  void Init(int n) override;
  void AddEdge(int u, int v, int cost) override;
  void RemoveEdge(int u, int v) override;
  void ClearEdgeFrom(int u) override;
  void ClearEdgeTo(int v) override;
  void ForEachNeighbours(int u, NeighbourVertexVisitor<int> &visitor) const override;
  void Clear() override;

 protected:
  // edges[from] => { to => cost }
  std::vector<std::unordered_map<int, int>> edges;
  // predecessors[to] => { from .. }
  std::vector<std::unordered_set<int>> predecessors;
};

// SimpleUnorderedMapDirectedGraph is a simple directed graph storing in an unordered_map.
// This uses less memory than SimpleDirectedGraph for sparse graph.
template <typename Vertex, typename VertexHasher = std::hash<Vertex>>
class SimpleUnorderedMapDirectedGraph : public IDirectedGraph<Vertex> {
 public:
  void Init(int n) override;
  void AddEdge(Vertex u, Vertex v, int cost) override;
  void RemoveEdge(Vertex u, Vertex v) override;
  void ClearEdgeFrom(Vertex u) override;
  void ClearEdgeTo(Vertex v) override;
  void ForEachNeighbours(Vertex u, NeighbourVertexVisitor<Vertex> &visitor) const override;
  void Clear() override;

 protected:
  using M = std::unordered_map<Vertex, int, VertexHasher>;
  using ST = std::unordered_set<Vertex, VertexHasher>;
  // edges[v] => { v =>  cost(u => v) }
  std::unordered_map<Vertex, M, VertexHasher> edges;
  // predecessors[to] => { from .. }
  std::unordered_map<Vertex, ST, VertexHasher> predecessors;
};

//////////////////////////////////////
/// QuadtreeMap
//////////////////////////////////////
///
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

class QuadtreeMap::Impl {
 public:
  Impl(int w, int h, ObstacleChecker isObstacle, DistanceCalculator distance, int step = 1,
       StepFunction stepf = nullptr, int maxNodeWidth = -1, int maxNodeHeight = -1);
  ~Impl();

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
  // Returns the distance between two vertices u and v.
  int Distance(int u, int v) const;
  // Returns true if the given cell (x,y) is an obstacle.
  bool IsObstacle(int x, int y) const { return isObstacle(x, y); }
  // Register a directed gate graph and keep it updated synchronously with the quadtree map
  // A quadtree map can register multiple gate graphs.
  void RegisterGateGraph(GateGraph *g);
  // Approximate distance between two quadtree nodes.
  // Using the provided distance calculator on their center cells.
  int DistanceBetweenNodes(QdNode *aNode, QdNode *bNode) const;

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
  // the 2st level abstract graphs: graph of gate cells.
  std::vector<GateGraph *> g2s;

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

  friend QuadtreeMap;

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

//////////////////////////////////////
/// Algorithm AStar
//////////////////////////////////////

template <typename K, typename V, V DefaultValue>
class KVContainer {
 public:
  virtual void Resize(std::size_t n);
  virtual V &operator[](K k);              // set k by reference
  virtual const V &operator[](K k) const;  // get value
  virtual void Clear();
};

// a handy util kv container with default value support.
template <typename K, typename V, V DefaultValue>
class DefaultedUnorderedMap : KVContainer<K, V, DefaultValue> {
 public:
  void Resize(std::size_t _ignoredn) override {}  // ignore
  V &operator[](K k) override;
  const V &operator[](K k) const override;
  void Clear() override { m.clear(); }

 private:
  V defaultValue = DefaultValue;
  std::unordered_map<K, V> m;
};

template <typename K, int DefaultValue>
using DefaultedUnorderedMapInt = DefaultedUnorderedMap<K, int, DefaultValue>;

template <typename K, bool DefaultValue>
using DefaultedUnorderedMapBool = DefaultedUnorderedMap<K, bool, DefaultValue>;

template <typename V, V DefaultValue>
class DefaultedVector : KVContainer<int, V, DefaultValue> {
 public:
  void Resize(std::size_t n) override { vec.resize(n, defaultValue); }
  V &operator[](int k) override { return vec[k]; }
  const V &operator[](int k) const override { return vec[k]; }
  void Clear() override { vec.clear(); }

 private:
  V defaultValue = DefaultValue;
  std::vector<V> vec;
};

template <int DefaultValue>
using DefaultedVectorInt = DefaultedVector<int, DefaultValue>;

// avoid using std::vector<bool>
template <bool DefaultValue>
using DefaultedVectorBool = DefaultedVector<unsigned char, DefaultValue>;

// AStar algorithm on a directed graph.
template <typename Vertex, Vertex NullVertex, typename F = DefaultedUnorderedMapInt<Vertex, inf>,
          typename Vis = DefaultedUnorderedMapBool<Vertex, false>,
          typename From = DefaultedUnorderedMap<Vertex, Vertex, NullVertex>>
class AStar {
 public:
  // Collects the result path and total cost to it.
  using PathCollector = std::function<void(Vertex v, int cost)>;
  // Returns the distance between two vertices u and v.
  using Distance = std::function<int(Vertex u, Vertex v)>;
  // Collects the neighbor vertices from u.
  using NeighboursCollector =
      std::function<void(Vertex u, NeighbourVertexVisitor<Vertex> &visitor)>;
  // Filter a neighbor vertex, returns true for cared neighbor.
  using NeighbourFilterTester = std::function<bool(Vertex)>;
  // Pair of { cost, vertex}.
  using P = std::pair<int, Vertex>;
  // The n is the upper bound of number of vertices on the graph.
  AStar(int n);
  void SetDistanceFunc(Distance f) { distance = f; }
  // Computes astar shortest path on given graph from start s to target t.
  // The collector will be called with each vertex on the result path,
  // along with the cost walking to it.
  // Returns -1 if the target is unreachable.
  // Returns the total cost to the target on success.
  int Compute(NeighboursCollector &neighborsCollector, Vertex s, Vertex t,
              PathCollector &collector, NeighbourFilterTester neighborTester);

 protected:
  int n;  // upper bound of vertices
  Distance distance;
  // store containers to avoid memory reallocation.
  F f;
  Vis vis;
  From from;
};

//////////////////////////////////////
/// PathFinding
//////////////////////////////////////

// PathFinderHelper is a mixin class to provide some util functions.
class PathFinderHelper : virtual public IPathFinder {
 public:
  // parameter g2 is the gate graph of the path finder.
  PathFinderHelper(const QuadtreeMap &m, IDirectedGraph<int> *g2);
  // Bresenham's line algorithm.
  // You can override it with a custom implementation.
  // Ref: https://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm
  // Ref: https://members.chello.at/easyfilter/bresenham.html
  virtual void ComputePathToNextRouteCell(int x1, int y1, int x2, int y2,
                                          CellCollector &collector) const;
  IDirectedGraph<int> *GetGateGraph() override { return g2; }

 protected:
  IDirectedGraph<int> *g2;
  // tmp gate graph is to store edges between start/target and other gate cells.
  SimpleUnorderedMapDirectedGraph<int> tmp;
  // BuildTmpGateGraph builds a temporary gate graph to store edges between start/target and
  // other gates in the same nodes.
  // This is a helper function.
  // Parameters:
  // 1. s and t are the ids of cell start and target.
  // 2. (x1,y1) and (x2,y2) are the positions of cell start and target.
  void BuildTmpGateGraph(int s, int t, int x1, int y1, int x2, int y2, QdNode *sNode,
                         QdNode *tNode);
  // ForEachNeighbourGateCellWithST iterates each neighbor gate cell connecting from given gate
  // cell u. What's the deference with the gate graph's ForEachNeighbours is: it will check both
  // the QuadtreeMap's gate cell graph and the temporary gate graph,
  // where stores the start, target informations.
  void ForEachNeighbourGateWithST(int u, NeighbourVertexVisitor<int> &visitor) const;
  // helper function to add a cell u to the given node in the temporary graph.
  void addCellToTmpGateGraph(int u, QdNode *node);
};

class AStarPathFinder::Impl : public PathFinderHelper {
 public:
  Impl(const QuadtreeMap &m);
  void Reset(int x1, int y1, int x2, int y2);
  int ComputeNodeRoutes();
  void VisitComputedNodeRoutes(QdNodeVisitor &visitor) const;
  int ComputeGateRoutes(CellCollector &collector, bool useNodePath = true);

 private:
  SimpleDirectedGraph g;

  // Astar for computing node path.
  using A1 = AStar<QdNode *, nullptr>;
  AStar<QdNode *, nullptr> astar1;

  // Astar for computing gate cell path.
  using A2 = AStar<int, inf, DefaultedVectorInt<inf>, DefaultedVectorBool<false>,
                   DefaultedVectorInt<inf>>;
  A2 astar2;

  // stateful values for current round compution.
  int x1, y1, x2, y2;
  int s, t;
  QdNode *sNode, *tNode;
  // the path of nodes if ComputeNodeRoutes is called successfully.
  using P = std::pair<QdNode *, int>;  // { node, cost }
  std::vector<P> nodePath;
  // the gate cells on the node path if ComputeNodeRoutes is called successfully.
  std::unordered_set<int> gateCellsOnNodePath;

  friend AStarPathFinder;

  void collectGateCellsOnNodePath();
};

//////////////////////////////////////////
/// Implementation for Templated Functions
//////////////////////////////////////////

// ~~~~~~~~~~~ Implements  SimpleUnorderedMapDirectedGraph ~~~~~~~~~~~~~~

template <typename Vertex, typename VertexHasher>
void SimpleUnorderedMapDirectedGraph<Vertex, VertexHasher>::Init(int n) {}

template <typename Vertex, typename VertexHasher>
void SimpleUnorderedMapDirectedGraph<Vertex, VertexHasher>::AddEdge(Vertex u, Vertex v, int cost) {
  edges[u].insert({v, cost});
  predecessors[v].insert(u);
}

template <typename Vertex, typename VertexHasher>
void SimpleUnorderedMapDirectedGraph<Vertex, VertexHasher>::RemoveEdge(Vertex u, Vertex v) {
  // Remove from edges.
  auto it = edges.find(u);
  if (it != edges.end()) {
    // m is edges[u]
    auto &m = it->second;
    m.erase(v);
    if (m.empty()) edges.erase(it);
  }
  // Remove from predecessors.
  auto it1 = predecessors.find(v);
  if (it1 != predecessors.end()) {
    // st is predecessors[v]
    auto &st = it->second;
    st.erase(u);
    if (st.empty()) predecessors.erase(it1);
  }
}

template <typename Vertex, typename VertexHasher>
void SimpleUnorderedMapDirectedGraph<Vertex, VertexHasher>::ClearEdgeFrom(Vertex u) {
  auto it = edges.find(u);
  if (it == edges.end()) return;
  // m is edges[u]
  auto &m = it->second;
  // Remove (u => v) in predecessors in advance.
  for (auto [v, _] : m) {
    auto it1 = predecessors.find(v);
    if (it1 != predecessors.end()) {
      // st is predecessors[v]
      auto &st = it1->second;
      st.erase(u);
      if (st.empty()) predecessors.erase(it1);
    }
  }
  // Clear and remove edges from u.
  m.clear();
  edges.erase(it);
}

template <typename Vertex, typename VertexHasher>
void SimpleUnorderedMapDirectedGraph<Vertex, VertexHasher>::ClearEdgeTo(Vertex v) {
  auto it = predecessors.find(v);
  if (it == predecessors.end()) return;
  // st is predecessors[v]
  auto &st = it->second;
  // Remove (u => v) in edges in advance.
  for (auto u : st) {
    auto it1 = edges.find(u);
    if (it1 != edges.end()) {
      // m is edges[u]
      auto &m = it1->second;
      m.erase(v);
      if (m.empty()) edges.erase(it1);
    }
  }
  // Clear and remove predecessors to v.
  st.clear();
  predecessors.erase(it);
}

template <typename Vertex, typename VertexHasher>
void SimpleUnorderedMapDirectedGraph<Vertex, VertexHasher>::ForEachNeighbours(
    Vertex u, NeighbourVertexVisitor<Vertex> &visitor) const {
  auto it = edges.find(u);
  if (it == edges.end()) return;
  // m is edges[u]
  const auto &m = it->second;
  for (const auto [v, cost] : m) visitor(v, cost);
}

template <typename Vertex, typename VertexHasher>
void SimpleUnorderedMapDirectedGraph<Vertex, VertexHasher>::Clear() {
  edges.clear();
  predecessors.clear();
}

// ~~~~~~~~~~~ Implements AStar ~~~~~~~~~~~~~~

template <typename K, typename V, V DefaultValue>
V &DefaultedUnorderedMap<K, V, DefaultValue>::operator[](K k) {
  return m.try_emplace(k, defaultValue).first->second;
}

template <typename K, typename V, V DefaultValue>
const V &DefaultedUnorderedMap<K, V, DefaultValue>::operator[](K k) const {
  auto it = m.find(k);
  if (it == m.end()) return defaultValue;
  return it->second;
}

template <typename Vertex, Vertex NullVertex, typename F, typename Vis, typename From>
AStar<Vertex, NullVertex, F, Vis, From>::AStar(int n) : n(n) {
  f.Resize(n), vis.Resize(n), from.Resize(n);
}

// A* search algorithm.
template <typename Vertex, Vertex NullVertex, typename F, typename Vis, typename From>
int AStar<Vertex, NullVertex, F, Vis, From>::Compute(NeighboursCollector &neighborsCollector,
                                                     Vertex s, Vertex t, PathCollector &collector,
                                                     NeighbourFilterTester neighborTester) {
  f.Clear(), vis.Clear(), from.Clear();
  f.Resize(n), vis.Resize(n);
  from[t] = NullVertex;

  // A* smallest-first queue, where P is { cost, vertex }
  std::priority_queue<P, std::vector<P>, std::greater<P>> q;
  f[s] = 0;
  q.push({f[s], s});

  Vertex u;

  // expand from u to v with cost c
  NeighbourVertexVisitor<Vertex> expand = [&u, &neighborTester, &q, &t, this](Vertex v, int c) {
    if (neighborTester != nullptr && !neighborTester(v)) return;
    auto g = f[u] + c;
    auto h = distance(v, t);
    auto cost = g + h;
    if (f[v] > g) {
      f[v] = g;
      q.push({cost, v});
      from[v] = u;
    }
  };

  while (q.size()) {
    u = q.top().second;
    q.pop();
    if (u == t) break;  // found
    if (vis[u]) continue;
    vis[u] = true;
    neighborsCollector(u, expand);
  }
  if (from[t] == NullVertex) return -1;  // fail

  // collects the path backward on from.
  std::vector<Vertex> path;
  path.push_back(t);
  auto v = t;
  while (v != s) {
    v = from[v];
    path.push_back(v);
  }
  for (int i = path.size() - 1; i >= 0; --i) collector(path[i], f[path[i]]);
  return f[t];
}

//////////////////////////////////////
}  // namespace qdpf

#endif
