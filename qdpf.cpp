#include "qdpf.hpp"

#include "qdpf_internal.hpp"

namespace qdpf {

//////////////////////////////////////
/// QuadtreeMap
//////////////////////////////////////

QuadtreeMap::QuadtreeMap(int w, int h, ObstacleChecker isObstacle, DistanceCalculator distance,
                         int step, StepFunction stepf, int maxNodeWidth, int maxNodeHeight)
    : pImpl(new QuadtreeMap::Impl(w, h, isObstacle, distance, step, stepf, maxNodeWidth,
                                  maxNodeHeight)) {}
QuadtreeMap::~QuadtreeMap() { delete pImpl; }
int QuadtreeMap::W() const { return pImpl->w; }
int QuadtreeMap::H() const { return pImpl->h; }
int QuadtreeMap::N() const { return pImpl->n; }
int QuadtreeMap::NumNodes() const { return pImpl->tree.NumNodes(); }
int QuadtreeMap::NumLeafNodes() const { return pImpl->tree.NumLeafNodes(); }
int QuadtreeMap::TreeDepth() const { return pImpl->tree.Depth(); }
void QuadtreeMap::RegisterGateGraph(GateGraph *g) { return pImpl->RegisterGateGraph(g); }
void QuadtreeMap::Nodes(QdNodeVisitor &visitor) const { pImpl->Nodes(visitor); }
void QuadtreeMap::Build() { pImpl->Build(); }
void QuadtreeMap::Update(int x, int y) { pImpl->Update(x, y); }
void QuadtreeMap::Gates(CellCollector &visitor) const {
  GateVisitor visitor1 = [this, &visitor](const Gate *gate) {
    auto [x, y] = this->pImpl->UnpackXY(gate->a);
    visitor(x, y);
  };
  pImpl->Gates(visitor1);
}

//////////////////////////////////////
/// AStarPathFinder
//////////////////////////////////////

AStarPathFinder::AStarPathFinder(const QuadtreeMap &m)
    : pImpl(new AStarPathFinder::Impl(m)), IPathFinder(m) {}
AStarPathFinder::~AStarPathFinder() { delete pImpl; }
IDirectedGraph<int> *AStarPathFinder::GetGateGraph() { return pImpl->GetGateGraph(); }
std::size_t AStarPathFinder::NodePathSize() const { return pImpl->nodePath.size(); }
void AStarPathFinder::Reset(int x1, int y1, int x2, int y2) { pImpl->Reset(x1, y1, x2, y2); }
int AStarPathFinder::ComputeNodeRoutes() { return pImpl->ComputeNodeRoutes(); }
void AStarPathFinder::VisitComputedNodeRoutes(QdNodeVisitor &visitor) const {
  pImpl->VisitComputedNodeRoutes(visitor);
}
int AStarPathFinder::ComputeGateRoutes(CellCollector &collector, bool useNodePath) {
  return pImpl->ComputeGateRoutes(collector, useNodePath);
}
void AStarPathFinder::ComputePathToNextRouteCell(int x1, int y1, int x2, int y2,
                                                 CellCollector &collector) const {
  pImpl->ComputePathToNextRouteCell(x1, y1, x2, y2, collector);
}

//////////////////////////////////////
/// SimpleDirectedGraph
//////////////////////////////////////

void SimpleDirectedGraph::Init(int n) { edges.resize(n), predecessors.resize(n); }

void SimpleDirectedGraph::AddEdge(int u, int v, int cost) {
  edges[u].insert({v, cost});
  predecessors[v].insert(u);
}

void SimpleDirectedGraph::RemoveEdge(int u, int v) {
  edges[u].erase(v);
  predecessors[v].erase(u);
}

void SimpleDirectedGraph::ClearEdgeFrom(int u) {
  for (auto [v, _] : edges[u]) predecessors[v].erase(u);
  edges[u].clear();
}

void SimpleDirectedGraph::ClearEdgeTo(int v) {
  for (auto u : predecessors[v]) edges[u].erase(v);
  predecessors[v].clear();
}

void SimpleDirectedGraph::ForEachNeighbours(int u, NeighbourVertexVisitor<int> &visitor) const {
  for (const auto [v, cost] : edges[u]) visitor(v, cost);
}

void SimpleDirectedGraph::Clear() {
  edges.clear();
  predecessors.clear();
}

}  // namespace qdpf
