// Source Code: https://github.com/hit9/quadtree-pathfinding
// License: BSD. Chao Wang, hit9[At]icloud.com.

#include "pathfinder_helper.hpp"

#include <cassert>

namespace qdpf {
namespace internal {

void PathFinderHelper::Reset(const QuadtreeMap *mPtr) {
  tmp.Clear();
  m = mPtr;
}

void PathFinderHelper::ConnectCellsOnTmpGraph(int u, int v) {
  assert(m != nullptr);
  if (u != v) {
    int dist = m->Distance(u, v);
    tmp.AddEdge(u, v, dist);
    tmp.AddEdge(v, u, dist);
  }
}

void PathFinderHelper::AddCellToNodeOnTmpGraph(int u, QdNode *node) {
  GateVisitor visitor = [this, u](const Gate *gate) { ConnectCellsOnTmpGraph(u, gate->a); };
  m->ForEachGateInNode(node, visitor);
}

void PathFinderHelper::ForEachNeighbourGateWithST(int u,
                                                  NeighbourVertexVisitor<int> &visitor) const {
  m->GetGateGraph().ForEachNeighbours(u, visitor);
  tmp.ForEachNeighbours(u, visitor);
}

}  // namespace internal
}  // namespace qdpf
