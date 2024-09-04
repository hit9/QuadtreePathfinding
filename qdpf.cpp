// Source Code: https://github.com/hit9/quadtree-pathfinding
// License: BSD. Chao Wang, hit9[At]icloud.com.

#include "qdpf.hpp"

namespace qdpf {

//////////////////////////////////////
/// QuadtreeMapX
//////////////////////////////////////

QuadtreeMapX::QuadtreeMapX(int w, int h, DistanceCalculator distance,
                           TerrainTypesChecker terrainChecker, QuadtreeMapXSettings settings,
                           int step, StepFunction stepf, int maxNodeWidth, int maxNodeHeight)
    : impl(internal::QuadtreeMapXImpl(w, h, distance, terrainChecker, settings, step, stepf,
                                      maxNodeWidth, maxNodeHeight)) {}

void QuadtreeMapX::Build() { impl.Build(); }
void QuadtreeMapX::Update(int x, int y) { impl.Update(x, y); }
void QuadtreeMapX::Compute() { impl.Compute(); }
const internal::QuadtreeMap *QuadtreeMapX::Get(int agentSize, int terrainTypes) const {
  return impl.Get(agentSize, terrainTypes);
}

//////////////////////////////////////
/// AStarPathFinder
//////////////////////////////////////

AStarPathFinder::AStarPathFinder(const QuadtreeMapX &mx)
    : mx(mx), impl(internal::AStarPathFinderImpl(mx.impl.N())) {}

int AStarPathFinder::Reset(int x1, int y1, int x2, int y2, int agentSize, int terrainTypes) {
  auto m = mx.Get(agentSize, terrainTypes);
  if (m == nullptr) return -1;
  impl.Reset(m, x1, y1, x2, y2);
  return 0;
}

int AStarPathFinder::ComputeNodeRoutes(NodePath &nodePath) {
  return impl.ComputeNodeRoutes(nodePath);
}

int AStarPathFinder::ComputeGateRoutes(GateRouteCollector &collector, const NodePath &nodePath) {
  return impl.ComputeGateRoutes(collector, nodePath);
}

int AStarPathFinder::ComputeGateRoutes(GateRouteCollector &collector) {
  return impl.ComputeGateRoutes(collector);
}

int AStarPathFinder::ComputeGateRoutes(GatePath &path, const NodePath &nodePath) {
  GateRouteCollector collector = [&path](int x, int y, int cost) { path.push_back({x, y, cost}); };
  return ComputeGateRoutes(collector, nodePath);
}

int AStarPathFinder::ComputeGateRoutes(GatePath &path) {
  GateRouteCollector collector = [&path](int x, int y, int cost) { path.push_back({x, y, cost}); };
  return ComputeGateRoutes(collector);
}

//////////////////////////////////////
/// FlowFieldPathFinder
//////////////////////////////////////

FlowFieldPathFinder::FlowFieldPathFinder(const QuadtreeMapX &mx)
    : mx(mx), impl(internal::FlowFieldPathFinderImpl(mx.impl.N())) {}

int FlowFieldPathFinder::Reset(int x2, int y2, const Rectangle &dest, int agentSize,
                               int walkableterrainTypes) {
  auto m = mx.Get(agentSize, walkableterrainTypes);
  if (m == nullptr) return -1;
  impl.Reset(m, x2, y2, dest);
  return 0;
}

int FlowFieldPathFinder::ComputeNodeFlowField() { return impl.ComputeNodeFlowField(); }

int FlowFieldPathFinder::ComputeGateFlowField(bool useNodeFlowField) {
  return impl.ComputeGateFlowField(useNodeFlowField);
}

int FlowFieldPathFinder::ComputeFinalFlowFieldInQueryRange() {
  return impl.ComputeFinalFlowFieldInQueryRange();
}

void FlowFieldPathFinder::VisitComputedNodeFlowField(NodeFlowFieldVisitor &visitor) {
  const auto &field = impl.GetNodeFlowField();
  for (auto [node, cost] : field.costs.GetUnderlyingUnorderedMap()) {
    auto next = field.nexts[node];
    visitor(node, next, cost);
  }
}

void FlowFieldPathFinder::VisitComputedGateFlowField(CellFlowFieldVisitor &visitor) {
  impl.VisitCellFlowField(impl.GetGateFlowField(), visitor);
}

void FlowFieldPathFinder::VisitComputedCellFlowFieldInQueryRange(CellFlowFieldVisitor &visitor) {
  impl.VisitCellFlowField(impl.GetFinalFlowFieldInQueryRange(), visitor);
}

}  // namespace qdpf
