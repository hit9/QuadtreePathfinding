// Source Code: https://github.com/hit9/quadtree-pathfinding
// License: BSD. Chao Wang, hit9[At]icloud.com.

#ifndef QDPF_INTERNAL_PATHFINDER_FLOW_FIELD_HPP
#define QDPF_INTERNAL_PATHFINDER_FLOW_FIELD_HPP

#include <functional>
#include <queue>
#include <unordered_map>
#include <unordered_set>

#include "base.hpp"
#include "pathfinder_helper.hpp"
#include "quadtree_map.hpp"

// FlowFieldPathFinder
// ~~~~~~~~~~~~~~~~~~~
// Implements FlowField pathfinder on a agent-size and terrain-types relateless quadtree map.

namespace qdpf {
namespace internal {

//////////////////////////////////////
/// FlowField (Container)
//////////////////////////////////////

// FlowField is a data container that stores the direction from a vertex to next vertex,
// along with the cost to the target.
template <typename Vertex, Vertex NullVertex, typename Hasher = std::hash<Vertex>>
class FlowField {
 public:
  using Visitor = std::function<void(Vertex v, Vertex next, int cost)>;
  // Set cost to the target for given vertex v.
  void SetCost(Vertex v, int cost);
  // Returns the cost from v to target.
  // Returns inf if the v is unreachable to target.
  int GetCost(Vertex v) const;
  // Returns the next vertex for vertex v.
  // Returns NullVertex if not found.
  Vertex GetNext(Vertex v) const;
  // Sets the next vertex for v to u.
  void SetNext(Vertex v, Vertex u);
  // Iterates each vertex along with its next vertex and cost informations.
  void ForEach(Visitor& visitor) const;
  void ForEach(Visitor&& visitor) const { ForEach(visitor); }  // trasfering rvalue
  // Clears the field.
  void Clear();

 private:
  // costs[v] stores the cost of vertex v to target.
  std::unordered_map<Vertex, int, Hasher> costs;
  // nexts[v] stores the next vertex of vertex v.
  std::unordered_map<Vertex, Vertex, Hasher> nexts;
};

// FlowField of quadtree nodes.
using NodeFlowField = FlowField<QdNode*, nullptr>;
// FlowField of cells.
using CellFlowField = FlowField<int, inf>;

// UnpackedCellFlowFieldVisitor is a functino to visit each item in a CellFlowField.
// The (x,y) is current cell, (xNext, yNext) is the next cell that current cell pointing to.
using UnpackedCellFlowFieldVisitor =
    std::function<void(int x, int y, int xNext, int yNext, int cost)>;

//////////////////////////////////////
/// FlowField (Algorithm)
//////////////////////////////////////

// flow field algorithm:
// 1. Compute the cost field by reverse-traversing  from the target, using the dijkstra
// algorithm.
// 2. Compute the flow field by comparing each vertex with its neighours vertices.

template <typename Vertex, Vertex NullVertex,
          typename Vis = DefaultedUnorderedMapBool<Vertex, false>>
class FlowFieldAlgorithm {
 public:
  using FlowFieldT = FlowField<Vertex, NullVertex>;
  using NeighboursCollectorT = NeighboursCollector<Vertex>;
  using NeighbourFilterTesterT = NeighbourFilterTester<Vertex>;

  // Test on a vertex, after its processing, whether to stop the whole flowfield processing.
  using StopAfterFunction = std::function<bool(Vertex)>;

  // The n is the upper bound of number of vertices on the graph.
  FlowFieldAlgorithm(int n);

  // Compute flowfield on given graph to target t.
  // Parameters:
  // 1. neighborsCollector is a function that gives the neighbor vertices of a vertex.
  // 2. t is the target vertex.
  // 3. field is the destination field to fill results.
  // 4. neighborTester is to filter neighbor.
  //
  // Note: the neighborsCollector should use the negative direction of the edges in the original
  // directed graph. But specially speaking, for our case, on the grid map, either the gate graph
  // or the node graph are both double-directed graph. That's passing in a function visiting the
  // original direction is just ok.
  void Compute(Vertex t, FlowFieldT& field, NeighboursCollectorT& neighborsCollector,
               NeighbourFilterTesterT neighborTester, StopAfterFunction& stopAfterTester);

 private:
  // Pair of { cost, vertex}.
  using P = std::pair<int, Vertex>;
  int n;
  // avoid reallocations..
  Vis vis;  // visit array.
};

//////////////////////////////////////
/// FlowFieldPathFinder
//////////////////////////////////////

// FlowField pathfinder.
// how to:
// 1. Resets current map to use and destination rectangle to fill the field, and the target cell.
// 2. Compute the flow field on the node graph level (optional).
// 3. Compute the flow field on the gate graph level.
// 4. Compute the detailed flow field for each cell in the destination flow field.
class FlowFieldPathFinderImpl : public PathFinderHelper {
 public:
  // n is the max number of vertex in the graphs.
  FlowFieldPathFinderImpl(int n);

  // Resets current working context: the the map instance, target (x2,y2) and flow field dest
  // rectangle to fill results.
  void Reset(const QuadtreeMap* m, int x2, int y2, const Rectangle& dest);
  // Computes the node flow field.
  // Returns -1 on failure
  int ComputeNodeFlowField();
  // Computes the gate cell flow field.
  // Returns -1 on failure
  int ComputeGateFlowField(bool useNodeFlowField = true);
  // Computes the final cell flow field for destination rectangle.
  // Returns -1 on failure
  int ComputeCellFlowFieldInDestRectangle();

  // Visits the computed node flow field.
  const NodeFlowField& GetNodeFlowField() const { return nodeFlowField; }
  // Visits the computed gate flow field.
  const CellFlowField& GetGateFlowField() const { return gateFlowField; }
  // Visits the computed detailed flow field for the destination rectangle.
  // The final flow field.
  const CellFlowField& GetFinalFlowFieldInDestRectangle() const { return finalFlowField; }
  // Helps to visit the cell flow field via (x,y,nextX,nextY,cost)
  void VisitCellFlowField(const CellFlowField& cellFlowField,
                          UnpackedCellFlowFieldVisitor& visitor) const;

 private:
  // ~~~~~~~  algorithm handlers ~~~~~~~~
  // for computing node flow field.
  using FFA1 = FlowFieldAlgorithm<QdNode*, nullptr>;
  FFA1 ffa1;

  // for computing gate flow field.
  using FFA2 = FlowFieldAlgorithm<int, inf, DefaultedVectorBool<false>>;
  FFA2 ffa2;

  // ~~~~~~~  stateful values for current round compution.~~~~~~~~
  // the quadtree map current working on
  const QuadtreeMap* m = nullptr;
  // compution results ared limited within this rectangle.
  Rectangle dest;
  // target.
  int x2, y2;
  int t;
  QdNode* tNode = nullptr;

  // nodes inside the dest rectangle.
  std::unordered_set<const QdNode*> nodesInDest;
  // gate cells inside the dest rectangle
  std::unordered_set<int> gatesInDest;
  // the gate cells on the node field if ComputeNodeFlowField is called successfully.
  std::unordered_set<int> gateCellsOnNodeFields;

  // ~~~~~~~~~~ computed results ~~~~~~~~~~~~~
  // results for node flow field.
  NodeFlowField nodeFlowField;
  // results for gate flow field.
  CellFlowField gateFlowField;
  // results for the final cell flow field.
  CellFlowField finalFlowField;

  void collectGateCellsOnNodeField();
};

//////////////////////////////////////
/// Implements templated functions
//////////////////////////////////////

// ~~~~~~~~~~~~~~~ Implements FlowField Container ~~~~~~~~~~~

template <typename Vertex, Vertex NullVertex, typename Hasher>
void FlowField<Vertex, NullVertex, Hasher>::SetCost(Vertex v, int cost) {
  costs[v] = cost;
}

template <typename Vertex, Vertex NullVertex, typename Hasher>
int FlowField<Vertex, NullVertex, Hasher>::GetCost(Vertex v) const {
  auto it = costs.find(v);
  if (it == costs.end()) return inf;
  return it->second;
}

template <typename Vertex, Vertex NullVertex, typename Hasher>
Vertex FlowField<Vertex, NullVertex, Hasher>::GetNext(Vertex v) const {
  auto it = nexts.find(v);
  if (it == nexts.end()) return NullVertex;
  return it->second;
}

template <typename Vertex, Vertex NullVertex, typename Hasher>
void FlowField<Vertex, NullVertex, Hasher>::SetNext(Vertex v, Vertex u) {
  nexts[v] = u;
}

template <typename Vertex, Vertex NullVertex, typename Hasher>
void FlowField<Vertex, NullVertex, Hasher>::ForEach(Visitor& visitor) const {
  for (auto [v, u] : nexts) {
    visitor(v, u, GetCost(v));
  }
}

template <typename Vertex, Vertex NullVertex, typename Hasher>
void FlowField<Vertex, NullVertex, Hasher>::Clear() {
  costs.clear();
  nexts.clear();
}

// ~~~~~~~~~~~~~~~ Implements FlowField Algorithm ~~~~~~~~~~~

template <typename Vertex, Vertex NullVertex, typename Vis>
FlowFieldAlgorithm<Vertex, NullVertex, Vis>::FlowFieldAlgorithm(int n) : n(n) {
  vis.Resize(n);
}

template <typename Vertex, Vertex NullVertex, typename Vis>
void FlowFieldAlgorithm<Vertex, NullVertex, Vis>::Compute(Vertex t, FlowFieldT& field,

                                                          NeighboursCollectorT& neighborsCollector,
                                                          NeighbourFilterTesterT neighborTester,
                                                          StopAfterFunction& stopAfterTester) {
  // dijkstra
  vis.Clear();
  vis.Resize(n);

  // smallest-first queue, where P is { cost, vertex }
  std::priority_queue<P, std::vector<P>, std::greater<P>> q;

  field.SetCost(t, 0);
  q.push({0, t});

  Vertex u;

  // expand from u to v with cost c
  NeighbourVertexVisitor<Vertex> expand = [&u, &neighborTester, &q, &t, &field, this](Vertex v,
                                                                                      int c) {
    if (neighborTester != nullptr && !neighborTester(v)) return;
    int fu = field.GetCost(u);
    int fv = field.GetCost(v);
    if (fv > fu + c) {
      fv = fu + c;
      q.push({fv, v});
      field.SetCost(v, fv);
      // v comes from u, that is.
      // In inversing view, u is the next way to go.
      field.SetNext(v, u);
    }
  };

  while (q.size()) {
    u = q.top().second;
    q.pop();
    if (vis[u]) continue;
    vis[u] = true;
    if (stopAfterTester != nullptr && stopAfterTester(u)) break;
    neighborsCollector(u, expand);
  }
}

}  // namespace internal
}  // namespace qdpf

#endif
