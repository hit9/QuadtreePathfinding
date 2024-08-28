// Source Code: https://github.com/hit9/quadtree-pathfinding
// License: BSD. Chao Wang, hit9[At]icloud.com.

#include "pathfinder_flowfield.hpp"

#include <cassert>
#include <utility>
#include <vector>

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
  nodesInDest.clear();
  QdNodeVisitor nodeVisitor = [this](const QdNode *node) { nodesInDest.insert(node); };
  m->NodesInRange(dest, nodeVisitor);

  // find all gates in the dest range.
  gatesInDest.clear();
  GateVisitor gateVisitor = [this, m, &dest](const Gate *gate) {
    auto [x, y] = m->UnpackXY(gate->a);
    if (x >= dest.x1 && x <= dest.x2 && y >= dest.y1 && y <= dest.y2) {
      gatesInDest.insert(gate->a);
    }
  };
  for (auto node : nodesInDest) m->ForEachGateInNode(node, gateVisitor);

  gateCellsOnNodeFields.clear();

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

// collects the gate cells on the node flow field if ComputeNodeFlowField is successfully called
// and ComputeGateFlowField is called with useNodeFlowField is set true.
void FlowFieldPathFinderImpl::collectGateCellsOnNodeField() {
  gateCellsOnNodeFields.insert(t);
  // gateVisitor collects the gate between node and nextNode.
  QdNode *node = nullptr, *nextNode = nullptr;

  GateVisitor gateVisitor = [this, &node, &nextNode](const Gate *gate) {
    // tNode has no next.
    if (node == tNode || node == nullptr || nextNode == nullptr) return;
    // collect only the gates between node and next node.
    if (gate->bNode == nextNode) {
      gateCellsOnNodeFields.insert(gate->a);
      gateCellsOnNodeFields.insert(gate->b);
    };
  };

  // nodeVisitor visits each node inside the nodeField.
  NodeFlowField::Visitor nodeVisitor = [this, &node, &nextNode, &gateVisitor](
                                           QdNode *v, QdNode *next, int cost) {
    node = v;
    nextNode = next;
    m->ForEachGateInNode(node, gateVisitor);
  };
  nodeFlowField.ForEach(nodeVisitor);
}

int FlowFieldPathFinderImpl::ComputeGateFlowField(bool useNodeFlowField) {
  if (tNode == nullptr) return -1;
  if (m->IsObstacle(x2, y2)) return -1;

  if (useNodeFlowField) collectGateCellsOnNodeField();

  // collects neighbour on the { tmp + map } 's gate graph.
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

  // if useNodeFlowField is true, we visit only the gate cells on the node field.
  FFA2::NeighbourFilterTesterT neighbourTester = [this, useNodeFlowField](int v) {
    if (useNodeFlowField && gateCellsOnNodeFields.find(v) == gateCellsOnNodeFields.end())
      return false;
    return true;
  };

  ffa2.Compute(t, gateFlowField, neighborsCollector, neighbourTester, stopf);
  return 0;
}

// Computes the final flow field via dynamic programming.
// Time Complexity O(dest.w * dest.h);
// 1. Suppose the cost to target for cell (x,y) is f[x][y].
// 2. Scan from left to right, up to bottom:
//      // directions: left-up, up, left, right-up
//      f[x][y] <= min(f[x][y], f[x-1][y-1], f[x-1][y], f[x][y-1], f[x-1][y+1]) + cost
// 3. Scan from right to left, bottom to up:
//      // directions: right-bottom, bottom, right, left-bottom
//      f[x][y] <= min(f[x][y], f[x+1][y+1], f[x+1][y], f[x][y+1], f[x+1][y-1]) + cost
// 4. Then calculates the flow field via f.
int FlowFieldPathFinderImpl::ComputeCellFlowFieldInDestRectangle() {
  // width and height of rectangle.
  int w = dest.y2 - dest.y1 + 1, h = dest.x2 - dest.x1 + 1;

  // f[x][y] is the cost from (x+dest.x1,y+dest.x2) to target.
  std::vector<std::vector<int>> f(h, std::vector<int>(w, inf));

  // from[x][y] stores where the min value comes from.
  std::vector<std::vector<std::pair<int, int>>> from(
      h, std::vector<std::pair<int, int>>(w, {-1, -1}));

  // initialize f from gate flow field.
  CellFlowField::Visitor visitor = [this, &f](int v, int next, int cost) {
    auto [x, y] = m->UnpackXY(v);
    x -= dest.x1;
    y -= dest.y1;
    f[x][y] = cost;
  };

  // cost unit on HV(horizonal and vertical) and diagonal directions.
  int c1 = m->Distance(0, 0, 0, 1), c2 = m->Distance(0, 0, 1, 1);

  // DP 1: left-top corner to right-bottom corner.
  for (int x = 0; x < h; ++x) {
    for (int y = 0; y < w; ++y) {
      if (x > 0 && y > 0 && f[x][y] > f[x - 1][y - 1] + c2) {  // left-up
        f[x][y] = f[x - 1][y - 1] + c2;
        from[x][y] = {x - 1, y - 1};
      }
      if (y > 0 && f[x][y] > f[x - 1][y] + c1) {  // up
        f[x][y] = f[x - 1][y] + c1;
        from[x][y] = {x - 1, y};
      }
      if (x > 0 && f[x][y] > f[x][y - 1] + c1) {  // left
        f[x][y] = f[x][y - 1] + c1;
        from[x][y] = {x, y - 1};
      }
      if (x > 0 && y < w - 1 && f[x][y] > f[x - 1][y + 1] + c2) {  // right-up
        f[x][y] = f[x - 1][y + 1] + c2;
        from[x][y] = {x - 1, y + 1};
      }
    }
  }

  // DP 2: right-bottom corner to left-top corner.
  for (int x = h - 1; x >= 0; --x) {
    for (int y = w - 1; y >= 0; --y) {
      if (x < h - 1 && y < w - 1 && f[x][y] > f[x + 1][y + 1] + c2) {  // right-bottom
        f[x][y] = f[x + 1][y + 1] + c2;
        from[x][y] = {x + 1, y + 1};
      }
      if (x < h - 1 && f[x][y] > f[x + 1][y] + c1) {  // bottom
        f[x][y] = f[x + 1][y] + c1;
        from[x][y] = {x + 1, y};
      }
      if (y < w - 1 && f[x][y] > f[x][y + 1] + c1) {  // right
        f[x][y] = f[x][y + 1] + c1;
        from[x][y] = {x, y + 1};
      }
      if (x < h - 1 && y > 0 && f[x][y] > f[x + 1][y - 1] + c2) {  // left-bottom
        f[x][y] = f[x + 1][y - 1] + c2;
        from[x][y] = {x + 1, y - 1};
      }
    }
  }

  // computes the flow field.
  for (int x = 0; x < h; ++x) {
    for (int y = 0; y < w; ++y) {
      auto [fromX, fromY] = from[x][y];
      if (fromX == -1 || fromY == -1) continue;
      int v = m->PackXY(x + dest.x1, y + dest.y1);
      int next = m->PackXY(fromX + dest.x1, fromY + dest.y1);
      finalFlowField.SetCost(v, f[x][y]);
      finalFlowField.SetNext(v, next);
    }
  }

  return 0;
}

void FlowFieldPathFinderImpl::VisitCellFlowField(const CellFlowField &cellFlowField,
                                                 UnpackedCellFlowFieldVisitor &visitor) const {
  CellFlowField::Visitor visitor1 = [&visitor, this](int v, int next, int cost) {
    auto [x, y] = m->UnpackXY(v);
    auto [xNext, yNext] = m->UnpackXY(next);
    visitor(x, y, xNext, yNext, cost);
  };
  cellFlowField.ForEach(visitor1);
}

}  // namespace internal
}  // namespace qdpf
