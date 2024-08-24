#include "qdpf.hpp"
#include "qdpf_internal.hpp"

namespace qdpf {

//////////////////////////////////////
/// IPathFinder
//////////////////////////////////////

IPathFinder::IPathFinder(const QuadtreeMap &m) : m(m) {}
const QuadtreeMap::Impl *IPathFinder::QuadtreeMapImpl() { return m.pImpl; }

//////////////////////////////////////
/// PathFinderHelper
//////////////////////////////////////

PathFinderHelper::PathFinderHelper(const QuadtreeMap &m, IDirectedGraph<int> *g2)
    : g2(g2), IPathFinder(m) {}

// Add given cell u to given node temporarily to the tmp gate graph.
// It will establish edges between u and all gate cells in node.
void PathFinderHelper::addCellToTmpGateGraph(int u, QdNode *node) {
  GateVisitor visitor = [this, u](const Gate *gate) {
    auto v = gate->a;
    if (u != v) {
      int dist = QuadtreeMapImpl()->Distance(u, v);
      tmp.AddEdge(u, v, dist);  // (u => v)
      tmp.AddEdge(v, u, dist);  // (v => u)
    }
  };
  QuadtreeMapImpl()->ForEachGateInNode(node, visitor);
}

void PathFinderHelper::BuildTmpGateGraph(int s, int t, int x1, int y1, int x2, int y2,
                                         QdNode *sNode, QdNode *tNode) {
  // Is the start and target a gate cell?
  bool sIsGate = QuadtreeMapImpl()->IsGateCell(sNode, s);
  bool tIsGate = QuadtreeMapImpl()->IsGateCell(tNode, t);

  // Add s and t to the tmp graph, if they are not gates.
  if (!sIsGate) addCellToTmpGateGraph(s, sNode);
  if (!tIsGate) addCellToTmpGateGraph(t, tNode);

  // A special case:
  // if s and t are in the same node, and both of them aren't gates,
  // we should connect them with edges, the shortest path should be dist itself.
  if (tNode == sNode && s != t && !sIsGate && !tIsGate) {
    // TODO: should we just stop the path finding on tihs case?
    int dist = QuadtreeMapImpl()->Distance(s, t);
    tmp.AddEdge(s, t, dist);
    tmp.AddEdge(t, s, dist);
  }
}

void PathFinderHelper::ForEachNeighbourGateWithST(int u,
                                                  NeighbourVertexVisitor<int> &visitor) const {
  g2->ForEachNeighbours(u, visitor);
  tmp.ForEachNeighbours(u, visitor);
}

void PathFinderHelper::ComputePathToNextRouteCell(int x1, int y1, int x2, int y2,
                                                  CellCollector &collector) const {
  int dx = abs(x2 - x1), sx = x1 < x2 ? 1 : -1;
  int dy = -abs(y2 - y1), sy = y1 < y2 ? 1 : -1;
  int err = dx + dy, e2;
  while (true) {
    collector(x1, y1);
    e2 = 2 * err;
    if (e2 >= dy) {
      if (x1 == x2) break;
      err += dy;
      x1 += sx;
    }
    if (e2 <= dx) {
      if (y1 == y2) break;
      err += dx;
      y1 += sy;
    }
  }
}

//////////////////////////////////////
/// AStarPathFinder
//////////////////////////////////////

AStarPathFinder::Impl::Impl(const QuadtreeMap &m)
    : PathFinderHelper(m, &g), astar1(A1(m.N())), astar2(A2(m.N())), IPathFinder(m) {
  g.Init(m.N());
  A1::Distance distance1 = [&m, this](QdNode *a, QdNode *b) {
    return this->QuadtreeMapImpl()->DistanceBetweenNodes(a, b);
  };
  A2::Distance distance2 = [&m, this](int u, int v) {
    return this->QuadtreeMapImpl()->Distance(u, v);
  };
  astar1.SetDistanceFunc(distance1);
  astar2.SetDistanceFunc(distance2);
}

void AStarPathFinder::Impl::Reset(int x1_, int y1_, int x2_, int y2_) {
  x1 = x1_, y1 = y1_, x2 = x2_, y2 = y2_;
  s = QuadtreeMapImpl()->PackXY(x1, y1), t = QuadtreeMapImpl()->PackXY(x2, y2);
  sNode = QuadtreeMapImpl()->FindNode(x1, y1), tNode = QuadtreeMapImpl()->FindNode(x2, y2);
  nodePath.clear();
  gateCellsOnNodePath.clear();
  // Rebuild the tmp graph.
  tmp.Clear();
  BuildTmpGateGraph(s, t, x1, y1, x2, y2, sNode, tNode);
}

int AStarPathFinder::Impl::ComputeNodeRoutes() {
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
    QuadtreeMapImpl()->ForEachNeighbourNodes(u, visitor);
  };
  // compute
  return astar1.Compute(neighborsCollector, sNode, tNode, collector, nullptr);
}

void AStarPathFinder::Impl::VisitComputedNodeRoutes(QdNodeVisitor &visitor) const {
  for (const auto [node, cost] : nodePath) visitor(node);
}

// Collects the gate cells on node path if ComputeNodeRoutes is successfully called and any further
// ComputeGateRoutes call specifics the useNodePath true.
// Notes that the start and target should be also collected.
void AStarPathFinder::Impl::collectGateCellsOnNodePath() {
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
  for (; i != nodePath.size(); ++i)
    QuadtreeMapImpl()->ForEachGateInNode(nodePath[i].first, visitor);
}

int AStarPathFinder::Impl::ComputeGateRoutes(CellCollector &collector, bool useNodePath) {
  // Can't route to or start from obstacles.
  if (QuadtreeMapImpl()->IsObstacle(x1, y1) || QuadtreeMapImpl()->IsObstacle(x2, y2)) return -1;
  // Same point.
  if (x1 == x2 && y1 == y2) {
    collector(x1, y1);
    return 0;
  }
  // if useNodePath then collect all gate cells for these node.
  if (useNodePath) collectGateCellsOnNodePath();
  // collector for path result.
  A2::PathCollector collector1 = [this, &collector](int u, int cost) {
    auto [x, y] = QuadtreeMapImpl()->UnpackXY(u);
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
}  // namespace qdpf
