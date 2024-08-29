// Source Code: https://github.com/hit9/quadtree-pathfinding
// License: BSD. Chao Wang, hit9[At]icloud.com.

#include "pathfinder_astar.hpp"

#include <cassert>

namespace qdpf {
namespace internal {

void AStarPathFinderImpl::Reset(const QuadtreeMap *m, int x1, int y1, int x2, int y2) {
  // debug mode, checks m, it's nullptr if mapx didn't find one.
  assert(m != nullptr);

  // resets the attributes.
  this->x1 = x1, this->y1 = y1, this->x2 = x2, this->y2 = y2;
  this->m = m;
  s = m->PackXY(x1, y1), t = m->PackXY(x2, y2);

  // finding a node is very fast: we find the start and target node without caring whether the
  // ComputeNodeRoutes is used in the future.
  sNode = m->FindNode(x1, y1), tNode = m->FindNode(x2, y2);

  // happen when: any of them out of map bound.
  // stop further handlings.
  if (sNode == nullptr || tNode == nullptr) return;

  // reset the distance function.
  A1::Distance distance1 = [this](QdNode *a, QdNode *b) {
    return this->m->DistanceBetweenNodes(a, b);
  };
  A2::Distance distance2 = [this](int u, int v) { return this->m->Distance(u, v); };
  astar1.SetDistanceFunc(distance1);
  astar2.SetDistanceFunc(distance2);

  // clear old results.
  nodePath.clear();
  gateCellsOnNodePath.clear();

  // Rebuild the tmp graph. And add the start and target cells to the gate graph.
  PathFinderHelper::Reset(this->m);

  // Is the start and target a gate cell?
  bool sIsGate = m->IsGateCell(sNode, s);
  bool tIsGate = m->IsGateCell(tNode, t);

  // Add s and t to the tmp graph, if they are not gates.
  if (!sIsGate) AddCellToNodeOnTmpGraph(s, sNode);
  if (!tIsGate) AddCellToNodeOnTmpGraph(t, tNode);

  // A special case:
  // if s and t are in the same node, and both of them aren't gates,
  // we should connect them with edges, the shortest path should be dist itself.
  // TODO: should we just stop the path finding on tihs case?
  if (tNode == sNode && s != t && !sIsGate && !tIsGate) ConnectCellsOnTmpGraph(s, t);
}

int AStarPathFinderImpl::ComputeNodeRoutes() {
  // any one of start and target are out of map bounds.
  if (sNode == nullptr || tNode == nullptr) return -1;
  // Can't route to or start from obstacles.
  if (m->IsObstacle(x1, y1) || m->IsObstacle(x2, y2)) return -1;
  // Same node.
  if (sNode == tNode) {
    nodePath.push_back({sNode, 0});
    return 0;
  }
  // collector for path result.
  A1::PathCollector collector = [this](QdNode *node, int cost) {
    nodePath.push_back({node, cost});
  };
  // collector for neighbour qd nodes.
  A1::NeighboursCollectorT neighborsCollector = [this](QdNode *u,
                                                       NeighbourVertexVisitor<QdNode *> &visitor) {
    m->ForEachNeighbourNodes(u, visitor);
  };
  // compute
  return astar1.Compute(sNode, tNode, collector, neighborsCollector, nullptr);
}

// Collects the gate cells on node path if ComputeNodeRoutes is successfully called and any further
// ComputeGateRoutes call specifics the useNodePath true.
// Notes that the start and target should be also collected.
void AStarPathFinderImpl::collectGateCellsOnNodePath() {
  gateCellsOnNodePath.insert(s);
  gateCellsOnNodePath.insert(t);
  // A visitor to collect all gate cells of a node.
  int i = 0;
  GateVisitor visitor = [this, &i](const Gate *gate) {
    // collect only the gates between aNode and next node on the path.
    if (i != nodePath.size() - 1 && gate->bNode == nodePath[i + 1].first) {
      gateCellsOnNodePath.insert(gate->a);
      gateCellsOnNodePath.insert(gate->b);
    };
  };
  for (; i != nodePath.size(); ++i) m->ForEachGateInNode(nodePath[i].first, visitor);
}

int AStarPathFinderImpl::ComputeGateRoutes(CellCollector &collector, bool useNodePath) {
  // any one of start and target are out of map bounds.
  if (sNode == nullptr || tNode == nullptr) return -1;
  // Can't route to or start from obstacles.
  if (m->IsObstacle(x1, y1) || m->IsObstacle(x2, y2)) return -1;
  // Same point.
  if (x1 == x2 && y1 == y2) {
    collector(x1, y1);
    return 0;
  }
  // if useNodePath then collect all gate cells for these node.
  if (useNodePath) collectGateCellsOnNodePath();
  // collector for path result.
  A2::PathCollector collector1 = [this, &collector](int u, int cost) {
    auto [x, y] = m->UnpackXY(u);
    collector(x, y);
  };

  A2::NeighbourFilterTesterT neighbourTester = [this, useNodePath](int v) {
    if (useNodePath && gateCellsOnNodePath.find(v) == gateCellsOnNodePath.end()) return false;
    return true;
  };
  // collector for neighbour gate cells.
  A2::NeighboursCollectorT neighborsCollector = [this](int u,
                                                       NeighbourVertexVisitor<int> &visitor) {
    ForEachNeighbourGateWithST(u, visitor);
  };
  // compute
  return astar2.Compute(s, t, collector1, neighborsCollector, neighbourTester);
}

}  // namespace internal
}  // namespace qdpf
