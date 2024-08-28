// Source Code: https://github.com/hit9/quadtree-pathfinding
// License: BSD. Chao Wang, hit9[At]icloud.com.

#include "pathfinder_flowfield.hpp"

#include <cassert>

namespace qdpf {
namespace internal {

FlowFieldPathFinderImpl::FlowFieldPathFinderImpl(int n) : ffa1(FFA1(n)), ffa2(FFA2(n)) {}

void FlowFieldPathFinderImpl::Reset(const QuadtreeMap *m, int x2, int y2, const Rectangle &dest) {
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

  // find all nodes in the dest range.
  QdNodeVisitor nodeVisitor = [this](const QdNode *node) { nodesInDest.insert(node); };
  m->NodesInRange(dest, nodeVisitor);

  // find all gates in the dest range.
  GateVisitor gateVisitor = [this, m, &dest](const Gate *gate) {
    auto [x, y] = m->UnpackXY(gate->a);
    if (x >= dest.x1 && x <= dest.x2 && y >= dest.y1 && y <= dest.y2) {
      gatesInDest.insert(gate->a);
    }
  };
  for (auto node : nodesInDest) m->ForEachGateInNode(node, gateVisitor);

  // Rebuild the tmp graph.
  PathFinderHelper::Reset(this->m);
  // Add the target cell to the gate graph
  bool tIsGate = m->IsGateCell(tNode, t);
  if (!tIsGate) AddCellToNodeOnTmpGraph(t, tNode);

  // Special case:
  // if the target node overlaps the dest rectangle, we should connects the overlaping cells to
  // the target, since the best path is a straight line then.
  Rectangle tNodeRectangle{tNode->x1, tNode->y1, tNode->x2, tNode->y2};
  Rectangle overlap;
  auto hasOverlap = GetOverlap(tNodeRectangle, dest, overlap);
  if (hasOverlap) {
    for (int x = overlap.x1; x <= overlap.x2; ++x) {
      for (int y = overlap.y1; y <= overlap.y2; ++y) {
        int u = m->PackXY(x, y);
        // detail notice is: u should not be a gate cell,
        // since we already connect all gate cells with t.
        if (u != t && m->IsGateCell(tNode, u)) {
          ConnectCellsOnTmpGraph(u, t);
          // We should consider u as a new tmp "gate" cell.
          gatesInDest.insert(u);
        }
      }
    }
  }
}

int FlowFieldPathFinderImpl::ComputeNodeFlowField() {
  // unreachable
  if (tNode == nullptr) return -1;
  if (m->IsObstacle(x2, y2)) return -1;

  // collector for neighbour qd nodes.
  FFA1::NeighboursCollectorT neighborsCollector =
      [this](QdNode *u, NeighbourVertexVisitor<QdNode *> &visitor) {
        m->ForEachNeighbourNodes(u, visitor);
      };

  // stops earlier if all nodes inside the dest rectangle are checked.
  int numNodesInDestChecked = 0;
  FFA1::StopAfterFunction stopf = [&numNodesInDestChecked, this](QdNode *node) {
    if (nodesInDest.find(node) != nodesInDest.end()) ++numNodesInDestChecked;
    return numNodesInDestChecked >= nodesInDest.size();
  };

  // Compute flowfield on node graph.
  ffa1.Compute(tNode, nodeFlowField, neighborsCollector, nullptr, stopf);
  return 0;
}

int FlowFieldPathFinderImpl::ComputeGateFlowField() {
  if (tNode == nullptr) return -1;
  if (m->IsObstacle(x2, y2)) return -1;

  // collects neighbour on the {tmp + map } 's gate graph.
  FFA2::NeighboursCollectorT neighborsCollector = [this](int u,
                                                         NeighbourVertexVisitor<int> &visitor) {
    ForEachNeighbourGateWithST(u, visitor);
  };

  // stops earlier if all gates inside the dest rectangle are checked.
  int numGatesInDestChecked = 0;
  FFA2::StopAfterFunction stopf = [this, &numGatesInDestChecked](int u) {
    if (gatesInDest.find(u) != gatesInDest.end()) ++numGatesInDestChecked;
    return numGatesInDestChecked >= gatesInDest.size();
  };

  // TODO: filter the gates !!!!!!!!!!!!!!
  ffa2.Compute(t, gateFlowField, neighborsCollector, nullptr, stopf);
  return 0;
}

int FlowFieldPathFinderImpl::ComputeCellFlowFieldInDestRectangle() {
  // TODO:
  return 0;
}

}  // namespace internal
}  // namespace qdpf
