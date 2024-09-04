// Source Code: https://github.com/hit9/quadtree-pathfinding
// License: BSD. Chao Wang, hit9[At]icloud.com.

#ifndef QDPF_INTERNAL_PATHFINDER_ASTAR_HPP
#define QDPF_INTERNAL_PATHFINDER_ASTAR_HPP

#include <functional>  // for std::function, std::hash
#include <queue>       // for std::priority_queue
#include <utility>     // for std::pair
#include <vector>      // for std::vector

#include "base.hpp"
#include "graph.hpp"
#include "pathfinder_helper.hpp"
#include "quadtree_map.hpp"

// AStarPathFinder
// ~~~~~~~~~~~~~~~
// Implements A* pathfinder on a agent-size and terrain-types relateless quadtree map.

namespace qdpf {
namespace internal {

//////////////////////////////////////
/// Algorithm AStar
//////////////////////////////////////

// AStar algorithm on a directed graph.
template <typename Vertex, Vertex NullVertex, typename F = DefaultedUnorderedMapInt<Vertex, inf>,
          typename Vis = DefaultedUnorderedMapBool<Vertex, false>,
          typename From = DefaultedUnorderedMap<Vertex, Vertex, NullVertex>>
class AStar {
 public:
  using NeighboursCollectorT = NeighboursCollector<Vertex>;
  using NeighbourFilterTesterT = NeighbourFilterTester<Vertex>;

  // Collects the result path and total cost to it.
  using PathCollector = std::function<void(Vertex v, int cost)>;

  // Returns the distance between two vertices u and v.
  using Distance = std::function<int(Vertex u, Vertex v)>;

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
  int Compute(Vertex s, Vertex t, PathCollector &collector,
              NeighboursCollectorT &neighborsCollector, NeighbourFilterTesterT neighborTester);

 protected:
  // upper bound of vertices
  int n;
  // returns the distance bwteen two vertices.
  Distance distance;
  // store containers to avoid memory reallocation.
  F f;
  Vis vis;
  From from;
};

//////////////////////////////////////
/// AStarPathFinder
//////////////////////////////////////

// the type of node path, a vector if { node, cost to target }.
using NodePath = std::vector<std::pair<QdNode *, int>>;

// the type of the function to collect computed gate cells.
using GateRouteCollector = std::function<void(int x, int y, int cost)>;

// AStar PathFinder.
// how to:
// 1. Resets the map to use and start, target cells: Reset(m, x1,y1, x2, y2)
// 2. Computes on the 1st level node graph (optional): ComputeNodeRoutes().
// 3. Computes on the 2nd level gate graph: ComputeGateRoutes().
// 4. Fill the detailed cells from current to next route cell: ComputeStraightLine().
class AStarPathFinderImpl : public PathFinderHelper {
 public:
  // n is the upper bound of the number of vertices of  gate graph and node graph.
  AStarPathFinderImpl(int n) : astar1(A1(n)), astar2(A2(n)) {}

  // Resets current working context: the map instance, start(x1,y1) and target (x2,y2);
  void Reset(const QuadtreeMap *m, int x1, int y1, int x2, int y2);

  // Compute the node path.
  // Returns 0 on success.
  // Returns -1 on failure (unreachable).
  int ComputeNodeRoutes(NodePath &nodePath);

  // Compute the gate cell path.
  // Returns 0 on success.
  // Returns -1 on failure (unreachable).
  int ComputeGateRoutes(GateRouteCollector &collector);

  // Compute the gate cell path, based on computed node path.
  // Returns 0 on success.
  // Returns -1 on failure (unreachable).
  int ComputeGateRoutes(GateRouteCollector &collector, const NodePath &nodePath);

 private:
  // the quadtree map current working on
  const QuadtreeMap *m = nullptr;

  // Astar for computing node path.
  using A1 = AStar<QdNode *, nullptr>;
  A1 astar1;

  // Astar for computing gate cell path.
  using A2 = AStar<int, inf, DefaultedVectorInt<inf>, DefaultedVectorBool<false>,
                   DefaultedVectorInt<inf>>;
  A2 astar2;

  // stateful values for current round compution.
  int x1, y1, x2, y2;
  int s, t;
  QdNode *sNode = nullptr, *tNode = nullptr;

  void collectGateCellsOnNodePath(std::unordered_set<int> &gateCellsOnNodePath,
                                  const NodePath &nodePath);
};

//////////////////////////////////////////
/// Implementation for Templated Functions
//////////////////////////////////////////

// ~~~~~~~~~~~ Implements AStar ~~~~~~~~~~~~~~

template <typename Vertex, Vertex NullVertex, typename F, typename Vis, typename From>
AStar<Vertex, NullVertex, F, Vis, From>::AStar(int n) : n(n) {
  f.Resize(n), vis.Resize(n), from.Resize(n);
}

// A* search algorithm.
template <typename Vertex, Vertex NullVertex, typename F, typename Vis, typename From>
int AStar<Vertex, NullVertex, F, Vis, From>::Compute(Vertex s, Vertex t, PathCollector &collector,
                                                     NeighboursCollectorT &neighborsCollector,
                                                     NeighbourFilterTesterT neighborTester) {
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
