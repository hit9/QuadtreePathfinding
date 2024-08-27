// Source Code: https://github.com/hit9/quadtree-pathfinding
// License: BSD. Chao Wang, hit9[At]icloud.com.

#ifndef QDPF_INTERNAL_PATHFINDER_FLOW_FIELD_HPP
#define QDPF_INTERNAL_PATHFINDER_FLOW_FIELD_HPP

#include <unordered_map>

#include "base.hpp"
#include "pathfinder_helper.hpp"
#include "quadtree_map.hpp"

// FlowFieldPathFinder
// ~~~~~~~~~~~~~~~~~~~
// Implements FlowField pathfinder on a agent-size and terrain-types relateless quadtree map.

namespace qdpf {
namespace internal {

// FlowField is a data container that stores the direction from a vertex to next vertex,
// along with the cost to the target.
template <typename Vertex, Vertex NullVertex>
class FlowField {
 public:
  // Returns the cost from v to target.
  // Returns inf if the v is unreachable to target.
  int Cost(Vertex v);
  // Returns the next vertex for vertex v.
  // Returns NullVertex if not found.
  Vertex Next(Vertex v);

 private:
  // costs[v] stores the cost of vertex v to target.
  std::unordered_map<Vertex, int> costs;
  // nexts[v] stores the next vertex of vertex v.
  std::unordered_map<Vertex, Vertex> nexts;
};

using NodeFlowField = FlowField<QdNode*, nullptr>;
using CellFlowField = FlowField<int, inf>;

//////////////////////////////////////
/// FlowFieldPathFinder
//////////////////////////////////////

// FlowField pathfinder.
// how to:
// 1. Resets current map to use and destination rectangle to fill the field, and the target cell.
// 2. Compute the flow field on the node graph level (optional).
// 3. Compute the flow field on the gate graph level.
// 4. Compute the detailed flow field for each cell in the destination flow field.
class FlowFieldPAthFinderImpl : public PathFinderHelper {
 public:
  // Resets current working context: the the map instance, target (x2,y2) and flow field dest
  // rectangle to fill results.
  void Reset(const QuadtreeMap* m, int x2, int y2, const Rectangle& dest);
  // Computes the node flow field.
  void ComputeNodeFlowField();
  // Computes the gate cell flow field.
  void ComputeGateFlowField();
  // Computes the final cell flow field for destination rectangle.
  void ComputeCellFlowFieldInDestRectangle();
  // Visits the computed node flow field.
  const NodeFlowField& GetNodeFlowField() const;
  // Visits the computed gate flow field.
  const CellFlowField& GetGateFlowField() const;
  // Visits the computed detailed flow field for the destination rectangle.
  const CellFlowField& GetFlowFieldInDestRectangle() const;
};

}  // namespace internal
}  // namespace qdpf

#endif
