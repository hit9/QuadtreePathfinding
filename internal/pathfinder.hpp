// Source Code: https://github.com/hit9/quadtree-pathfinding
// License: BSD. Chao Wang, hit9[At]icloud.com.

#ifndef QDPF_INTERNAL_PATHFINDER_HPP
#define QDPF_INTERNAL_PATHFINDER_HPP

#include <functional>     // for std::function, std::hash
#include <queue>          // for std::priority_queue
#include <unordered_map>  // for std::unordered_map
#include <utility>        // for std::pair
#include <vector>         // for std::vector

#include "base.hpp"
#include "graph.hpp"
#include "quadtree_map.hpp"

namespace qdpf {
namespace internal {

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
class PathFinderHelper {
 public:
  // parameter g2 is the gate graph of the path finder.
  PathFinderHelper(const QuadtreeMapImpl &m, IDirectedGraph<int> *g2);
  // Bresenham's line algorithm.
  // You can override it with a custom implementation.
  // Ref: https://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm
  // Ref: https://members.chello.at/easyfilter/bresenham.html
  virtual void ComputePathToNextRouteCell(int x1, int y1, int x2, int y2,
                                          CellCollector &collector) const;

 protected:
  const QuadtreeMapImpl &m;
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

class AStarPathFinderImpl : public PathFinderHelper {
 public:
  // the path of nodes if ComputeNodeRoutes is called successfully.
  using P = std::pair<QdNode *, int>;  // { node, cost }

  AStarPathFinderImpl(const QuadtreeMapImpl &m);
  GateGraph *GetGateGraph() { return &g; }
  const std::vector<P> &NodePath() { return nodePath; }
  void Reset(int x1, int y1, int x2, int y2);
  int ComputeNodeRoutes();
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
  std::vector<P> nodePath;
  // the gate cells on the node path if ComputeNodeRoutes is called successfully.
  std::unordered_set<int> gateCellsOnNodePath;

  void collectGateCellsOnNodePath();
};

//////////////////////////////////////////
/// Implementation for Templated Functions
//////////////////////////////////////////

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
}  // namespace internal
}  // namespace qdpf

#endif
