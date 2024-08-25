// Source Code: https://github.com/hit9/quadtree-pathfinding
// License: BSD. Chao Wang, hit9[At]icloud.com.

#include "qdpf.hpp"

namespace qdpf {

//////////////////////////////////////
/// QuadtreeMap
//////////////////////////////////////

QuadtreeMap::QuadtreeMap(int w, int h, ObstacleChecker isObstacle, DistanceCalculator distance,
                         int step, StepFunction stepf, int maxNodeWidth, int maxNodeHeight)
    : pImpl(new internal::QuadtreeMapImpl(w, h, isObstacle, distance, step, stepf, maxNodeWidth,
                                          maxNodeHeight)) {}
QuadtreeMap::~QuadtreeMap() { delete pImpl; }
int QuadtreeMap::W() const { return pImpl->W(); }
int QuadtreeMap::H() const { return pImpl->H(); }
void QuadtreeMap::RegisterGateGraph(GateGraph *g) { return pImpl->RegisterGateGraph(g); }
void QuadtreeMap::Build() { pImpl->Build(); }
void QuadtreeMap::Update(int x, int y) { pImpl->Update(x, y); }

void QuadtreeMap::Nodes(NodeVisitor &visitor) const {
  internal::QdNodeVisitor visitor1 = [this, &visitor](const internal::QdNode *node) {
    visitor(node->x1, node->y1, node->x2, node->y2);
  };
  pImpl->Nodes(visitor1);
}

void QuadtreeMap::Gates(GateVisitor &visitor) const {
  internal::GateVisitor visitor1 = [this, &visitor](const internal::Gate *gate) {
    auto [x1, y1] = this->pImpl->UnpackXY(gate->a);
    auto [x2, y2] = this->pImpl->UnpackXY(gate->b);
    visitor(x1, y1, x2, y2);
  };
  pImpl->Gates(visitor1);
}

//////////////////////////////////////
/// IPathFinder
//////////////////////////////////////

IPathFinder::IPathFinder(const QuadtreeMap &m) : m(m) {}
const internal::QuadtreeMapImpl *IPathFinder::QuadtreeMapImpl() { return m.pImpl; }

//////////////////////////////////////
/// AStarPathFinder
//////////////////////////////////////

AStarPathFinder::AStarPathFinder(const QuadtreeMap &m) : pImpl(), IPathFinder(m) {
  pImpl = new internal::AStarPathFinderImpl(*QuadtreeMapImpl());
}
AStarPathFinder::~AStarPathFinder() { delete pImpl; }
IDirectedGraph<int> *AStarPathFinder::GetGateGraph() { return pImpl->GetGateGraph(); }
std::size_t AStarPathFinder::NodePathSize() const { return pImpl->NodePath().size(); }
void AStarPathFinder::Reset(int x1, int y1, int x2, int y2) { pImpl->Reset(x1, y1, x2, y2); }
int AStarPathFinder::ComputeNodeRoutes() { return pImpl->ComputeNodeRoutes(); }

void AStarPathFinder::VisitComputedNodeRoutes(NodeVisitor &visitor) const {
  for (auto [node, cost] : pImpl->NodePath()) visitor(node->x1, node->y1, node->x2, node->y2);
}

int AStarPathFinder::ComputeGateRoutes(CellCollector &collector, bool useNodePath) {
  return pImpl->ComputeGateRoutes(collector, useNodePath);
}

void AStarPathFinder::ComputePathToNextRouteCell(int x1, int y1, int x2, int y2,
                                                 CellCollector &collector) const {
  pImpl->ComputePathToNextRouteCell(x1, y1, x2, y2, collector);
}

}  // namespace qdpf
