// Source Code: https://github.com/hit9/quadtree-pathfinding
// License: BSD. Chao Wang, hit9[At]icloud.com.

#include "pathfinder_astar.hpp"

namespace qdpf {
namespace internal {

void AStarPathFinderImpl::Reset(const QuadtreeMapImpl *mPtr, int x1_, int y1_, int x2_, int y2_) {
  // resets the attributes.
  x1 = x1_, y1 = y1_, x2 = x2_, y2 = y2_;
  m = mPtr;
  s = m->PackXY(x1, y1), t = m->PackXY(x2, y2);
  sNode = m->FindNode(x1, y1), tNode = m->FindNode(x2, y2);
  // reset the distance function.
  A1::Distance distance1 = [this](QdNode *a, QdNode *b) { return m->DistanceBetweenNodes(a, b); };
  A2::Distance distance2 = [this](int u, int v) { return m->Distance(u, v); };
  astar1.SetDistanceFunc(distance1);
  astar2.SetDistanceFunc(distance2);
  // clear old results.
  nodePath.clear();
  gateCellsOnNodePath.clear();
  // Rebuild the tmp graph.
  PathFinderHelper::Reset(mPtr);
  BuildTmpGateGraph(s, t, x1, y1, x2, y2, sNode, tNode);
}

int AStarPathFinderImpl::ComputeNodeRoutes() {
  if (sNode == tNode) {
    // Same node.
    nodePath.push_back({sNode, 0});
    return 0;
  }
  // collector for path result.
  A1::PathCollector collector = [this](QdNode *node, int cost) {
    nodePath.push_back({node, cost});
  };
  // collector for neighbour qd nodes.
  A1::NeighboursCollector neighborsCollector = [this](QdNode *u,
                                                      NeighbourVertexVisitor<QdNode *> &visitor) {
    m->ForEachNeighbourNodes(u, visitor);
  };
  // compute
  return astar1.Compute(neighborsCollector, sNode, tNode, collector, nullptr);
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

  A2::NeighbourFilterTester neighbourTester = [this, useNodePath](int v) {
    if (useNodePath && gateCellsOnNodePath.find(v) == gateCellsOnNodePath.end()) return false;
    return true;
  };
  // collector for neighbour gate cells.
  A2::NeighboursCollector neighborsCollector = [this](int u,
                                                      NeighbourVertexVisitor<int> &visitor) {
    ForEachNeighbourGateWithST(u, visitor);
  };
  // compute
  return astar2.Compute(neighborsCollector, s, t, collector1, neighbourTester);
}

}  // namespace internal
}  // namespace qdpf
