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
// 1. Compute the cost field by reverse-traversing from the target, using the astar algorithm.
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

  // The heuristic function for astar, it's optional.
  using HeuristicFunction = std::function<int(Vertex u)>;

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

  void SetHeuristicFunction(HeuristicFunction f) { heuristic = f; }

 private:
  // Pair of { cost, vertex}.
  using P = std::pair<int, Vertex>;
  // upper bound of the number of vertices on the graph.
  int n;
  // avoid reallocations..
  Vis vis;  // visit array.

  HeuristicFunction heuristic = nullptr;
};

//////////////////////////////////////
/// FlowFieldPathFinder
//////////////////////////////////////

// FlowField pathfinder.
// how to:
// 1. Resets current map to use, the query range, and the target cell.
// 2. Compute the flow field on the node graph level (optional).
// 3. Compute the flow field on the gate graph level.
// 4. Compute the detailed flow field for each cell in the query range.
class FlowFieldPathFinderImpl : public PathFinderHelper {
 public:
  // n is the max number of vertex in the graphs.
  FlowFieldPathFinderImpl(int n);

  // Resets current working context:
  // * the the map instance
  // * target (x2,y2)
  // * the query range rectangle to fill results.
  void Reset(const QuadtreeMap* m, int x2, int y2, const Rectangle& qrange);

  // Computes the node flow field.
  // Returns -1 on failure (unreachable).
  int ComputeNodeFlowField();

  // Computes the gate cell flow field.
  // Returns -1 on failure (unreachable).
  int ComputeGateFlowField(bool useNodeFlowField = true);

  // Computes the final cell flow field for the query range.
  // Returns -1 on failure (unreachable).
  int ComputeFinalFlowFieldInQueryRange();

  // Visits the computed node flow field.
  const NodeFlowField& GetNodeFlowField() const { return nodeFlowField; }

  // Visits the computed gate flow field.
  const CellFlowField& GetGateFlowField() const { return gateFlowField; }

  // Visits the computed detailed flow field for the query range.
  // The final flow field.
  const CellFlowField& GetFinalFlowFieldInQueryRange() const { return finalFlowField; }

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

  // final compution results ared limited within this rectangle.
  Rectangle qrange;
  int qrangeCenterX, qrangeCenterY;
  // target.
  int x2, y2;
  int t;
  QdNode* tNode = nullptr;

  // ~~~~~ for earlier quit ~~~~~~~
  // nodes overlapping with the query range.
  // this node collection is to stop the ffa1 pathfinder's compution earlier once
  // all the related nodes are marked via flowfield algorithm.
  std::unordered_set<const QdNode*> nodesOverlappingQueryRange;

  // this gate collection includes two parts:
  // 1. gate cells inside the nodes within nodesOverlappingQueryRange.
  // 2. virtual gate cells inside the tmp graph.
  // Its purpose is also to stop the ffa2 pathfinder's compution earlier once all the
  // related gates are marked via flowfield algorithm.
  std::unordered_set<int> gatesInNodesOverlappingQueryRange;

  // to reduce the number of gates that participating the ComputeGateFlowField():
  // we collect the gate cells on the computed node fields, only gate inside this collection will
  // participate the further ComputeGateFlowField() if there was a previous successful
  // ComputeNodeFlowField() call.
  std::unordered_set<int> gateCellsOnNodeFields;

  // ~~~~~~~~~~ computed results ~~~~~~~~~~~~~
  // results for node flow field.
  NodeFlowField nodeFlowField;
  // results for gate flow field.
  CellFlowField gateFlowField;
  // results for the final cell flow field.
  CellFlowField finalFlowField;

  // ~~~~~~~~ compution lambdas (optimization for reuses) ~~~~~~~~~
  // lambda to collect quadtree nodes overlapping with the qrange.
  QdNodeVisitor nodesOverlappingQueryRangeCollector = nullptr;
  // lambda to collect gate cells inside a node within nodesOverlappingQueryRange.
  GateVisitor gatesInNodesOverlappingQueryRangeCollector = nullptr;
  // neighbor queriers for the flowfield algorithm.
  FFA1::NeighboursCollectorT ffa1NeighborsCollector = nullptr;
  FFA2::NeighboursCollectorT ffa2NeighborsCollector = nullptr;

  // ~~~~~~~~ internal functions ~~~~~~~~~~~

  void collectGateCellsOnNodeField();

  // DP value container of f for ComputeFinalFlowFieldInQueryRange()
  using Final_F = NestedDefaultedUnorderedMap<int, int, int, inf>;
  // DP value container of from for ComputeFinalFlowFieldInQueryRange()
  using Final_From = NestedDefaultedUnorderedMap<int, int, int, inf>;
  // B[x][y] is the container indicates that whether cell (x,y) is on the computed gate flow field.
  // which is a helper typing for ComputeFinalFlowFieldInQueryRange().
  using Final_B = NestedDefaultedUnorderedMap<int, int, bool, false>;

  void findNeighbourCellByNext(int x, int y, int x1, int y1, int& x2, int& y2);
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
  // astar
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
    int fu = field.costs[u];  // readonly
    int fv = field.costs[v];  // readonly
    auto g = fu + c;          // existing real cost
    auto cost = g;            // future estimation cost
    if (heuristic != nullptr) cost += heuristic(v);
    if (fv > g) {
      fv = g;
      q.push({cost, v});
      field.costs[v] = g;  // real cost
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
