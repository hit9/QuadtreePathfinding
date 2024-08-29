// Source Code: https://github.com/hit9/quadtree-pathfinding
// License: BSD. Chao Wang, hit9[At]icloud.com.

#ifndef QDPF_INTERNAL_PATHFINDER_FLOW_FIELD_HPP
#define QDPF_INTERNAL_PATHFINDER_FLOW_FIELD_HPP

#include <functional>
#include <queue>  // for std::priority_queue
#include <unordered_set>
#include <vector>

#include "base.hpp"
#include "pathfinder_helper.hpp"
#include "quadtree_map.hpp"

// FlowFieldPathFinder
// ~~~~~~~~~~~~~~~~~~~
// Implements FlowField pathfinder on an agent-size and terrain-types relateless quadtree map.

namespace qdpf {
namespace internal {

//////////////////////////////////////
/// FlowField (Container)
//////////////////////////////////////

// FlowField is a simple data container that stores the direction from a vertex to next vertex,
// along with the cost to the target.
template <typename Vertex, Vertex NullVertex, typename Hasher = std::hash<Vertex>>
struct FlowField {
  using CostFieldT = DefaultedUnorderedMap<Vertex, int, inf>;
  using NextFieldT = DefaultedUnorderedMap<Vertex, Vertex, NullVertex>;

  bool Exist(Vertex v) const { return costs.Exist(v); }
  void Clear() { costs.Clear(), nexts.Clear(); }

  // costs[v] stores the cost of vertex v to target.
  CostFieldT costs;
  // nexts[v] stores the next vertex of vertex v.
  // next for target is itself.
  NextFieldT nexts;
};

// FlowField of quadtree nodes.
using NodeFlowField = FlowField<QdNode*, nullptr>;
// FlowField of cells.
using CellFlowField = FlowField<int, inf>;

// UnpackedCellFlowFieldVisitor is a function to visit each item in a CellFlowField.
// The (x,y) is current cell, (xNext, yNext) is the next cell that current cell pointing to.
using UnpackedCellFlowFieldVisitor =
    std::function<void(int x, int y, int xNext, int yNext, int cost)>;

//////////////////////////////////////
/// FlowField (Algorithm)
//////////////////////////////////////

// flow field algorithm:
// 1. Compute the cost field by reverse-traversing from the target, using the dijkstra algorithm.
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

  // The n is the upper bound of the number of vertices on the graph.
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
  // or the node graph are both bidirectional graph. That is, passing in a function visiting the
  // original direction is just ok.
  void Compute(Vertex t, FlowFieldT& field, NeighboursCollectorT& neighborsCollector,
               NeighbourFilterTesterT neighborTester, StopAfterFunction& stopAfterTester);

 private:
  // Pair of { cost, vertex}.
  using P = std::pair<int, Vertex>;
  // upper bound of the number of vertices on the graph.
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
  // Returns -1 on failure (unreachable).
  int ComputeNodeFlowField();
  // Computes the gate cell flow field.
  // Returns -1 on failure (unreachable).
  int ComputeGateFlowField(bool useNodeFlowField = true);
  // Computes the final cell flow field for destination rectangle.
  // Returns -1 on failure (unreachable).
  int ComputeFinalFlowFieldInDestRectangle();

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

  // ~~~~~~~ stateful values for current round compution.~~~~~~~~
  // ~~~~~~~ they should be cleared on every Reset call ~~~~~~
  // the quadtree map current working on
  const QuadtreeMap* m = nullptr;
  // compution results ared limited within this rectangle.
  Rectangle dest;
  // target.
  int x2, y2;
  int t;
  QdNode* tNode = nullptr;
  // ~~~~~ for earlier quit ~~~~~~~
  // nodes inside the dest rectangle.
  std::unordered_set<const QdNode*> nodesInDest;
  // gate cells inside the dest rectangle
  std::unordered_set<int> gatesInDest;

  // ~~~~~ for reducing the number of gates that participating the ComputeGateFlowField(). ~~~~~~~
  // the gate cells on the node field if ComputeNodeFlowField is called successfully.
  std::unordered_set<int> gateCellsOnNodeFields;

  // ~~~~~~~~~~ computed results ~~~~~~~~~~~~~
  // results for node flow field.
  NodeFlowField nodeFlowField;
  // results for gate flow field.
  CellFlowField gateFlowField;
  // results for the final cell flow field.
  CellFlowField finalFlowField;

  // ~~~~~~~~ compution lambdas (optimization for reuses) ~~~~~~~~~
  // lambda to collects quadtree nodes inside given dest rectangle.
  QdNodeVisitor nodesInDestCollector = nullptr;
  // lambda to collects gate cells inside given dest rectangle.
  GateVisitor gatesInDestCollector = nullptr;

  FFA1::NeighboursCollectorT ffa1NeighborsCollector = nullptr;
  FFA2::NeighboursCollectorT ffa2NeighborsCollector = nullptr;

  void collectGateCellsOnNodeField();

  // DP value container of f for ComputeFinalFlowFieldInDestRectangle()
  using Final_F = NestedDefaultedUnorderedMap<int, int, int, inf>;
  // DP value container of from for ComputeFinalFlowFieldInDestRectangle()
  using Final_From = NestedDefaultedUnorderedMap<int, int, int, inf>;
  // B[x][y] indicates whether (x,y) is a computed gate cell for
  // ComputeFinalFlowFieldInDestRectangle().
  using Final_B = NestedDefaultedUnorderedMap<int, int, bool, false>;

  void computeFinalFlowFieldDP1(const QdNode* node, Final_F& f, Final_From& from, Final_B& b,
                                int c1, int c2);
  void computeFinalFlowFieldDP2(const QdNode* node, Final_F& f, Final_From& from, Final_B& b,
                                int c1, int c2);
};

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

  field.costs[t] = 0;
  // Notes that the target's next is itself.
  field.nexts[t] = t;
  q.push({0, t});

  Vertex u;

  // expand from u to v with cost c
  NeighbourVertexVisitor<Vertex> expand = [&u, &neighborTester, &q, &t, &field, this](Vertex v,
                                                                                      int c) {
    if (neighborTester != nullptr && !neighborTester(v)) return;
    int fu = field.costs[u];
    int fv = field.costs[v];
    if (fv > fu + c) {
      fv = fu + c;
      q.push({fv, v});
      field.costs[v] = fv;
      // v comes from u, that is.
      // In inversing view, u is the next way to go.
      field.nexts[v] = u;
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
