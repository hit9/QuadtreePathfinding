// Source Code: https://github.com/hit9/quadtree-pathfinding
// License: BSD. Chao Wang, hit9[At]icloud.com.

#include "pathfinder_flowfield.hpp"

#include <algorithm>
#include <cassert>

namespace qdpf {
namespace internal {

void FlowFieldPathFinderImpl::Reset(const QuadtreeMap* m, int x2, int y2, const Rectangle& dest) {
  // debug mode, checks m, it's nullptr if mapx didn't find one.
  assert(m != nullptr);

  // resets the attributes.
  this->m = m;
  this->x2 = x2, this->y2 = y2;
  this->dest = dest;  // copy updated

  t = m->PackXY(x2, y2);
  tNode = m->FindNode(x2, y2);
  if (tNode == nullptr) return;

  // clears the old results.
  nodeFlowField.Clear();
  gateFlowField.Clear();
  finalFlowField.Clear();

  // Rebuild the tmp graph.
  PathFinderHelper::Reset(this->m);
  // Add the target cell to the gate graph
  bool tIsGate = m->IsGateCell(tNode, t);
  if (!tIsGate) AddCellToNodeOnTmpGraph(t, tNode);

  if (IsOverlap(tNode->x1, tNode->y1, tNode->x2, tNode->y2, dest.x1, dest.y1, dest.x2, dest.y2)) {
    // Special case:
    // if the target node overlaps the dest rectangle, we should connects the overlaping cells to
    // the target, since the best path is a straight line then.
    int x3 = std::max(tNode->x1, dest.x1);
    int y3 = std::max(tNode->y1, dest.y1);
    int x4 = std::min(tNode->x2, dest.x2);
    int y4 = std::min(tNode->y2, dest.y2);
    for (int x = x3; x <= x4; ++x) {
      for (int y = y3; y <= y4; ++y) {
        int u = m->PackXY(x, y);
        // detail notice is: u should not be a gate cell,
        // since we already connect all gate cells with t.
        if (u != t && m->IsGateCell(tNode, u)) ConnectCellsOnTmpGraph(u, t);
      }
    }
  }
}

}  // namespace internal
}  // namespace qdpf
