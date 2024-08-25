// Source Code: https://github.com/hit9/quadtree-pathfinding
// License: BSD. Chao Wang, hit9[At]icloud.com.

#include "pathfinder_helper.hpp"

namespace qdpf {
namespace internal {

void PathFinderHelper::Reset(const QuadtreeMap *mPtr) {
  tmp.Clear();
  m = mPtr;
}

// Add given cell u to given node temporarily to the tmp gate graph.
// It will establish edges between u and all gate cells in node.
void PathFinderHelper::addCellToTmpGateGraph(int u, QdNode *node) {
  GateVisitor visitor = [this, u](const Gate *gate) {
    auto v = gate->a;
    if (u != v) {
      int dist = m->Distance(u, v);
      tmp.AddEdge(u, v, dist);  // (u => v)
      tmp.AddEdge(v, u, dist);  // (v => u)
    }
  };
  m->ForEachGateInNode(node, visitor);
}

void PathFinderHelper::BuildTmpGateGraph(int s, int t, int x1, int y1, int x2, int y2,
                                         QdNode *sNode, QdNode *tNode) {
  // Is the start and target a gate cell?
  bool sIsGate = m->IsGateCell(sNode, s);
  bool tIsGate = m->IsGateCell(tNode, t);

  // Add s and t to the tmp graph, if they are not gates.
  if (!sIsGate) addCellToTmpGateGraph(s, sNode);
  if (!tIsGate) addCellToTmpGateGraph(t, tNode);

  // A special case:
  // if s and t are in the same node, and both of them aren't gates,
  // we should connect them with edges, the shortest path should be dist itself.
  if (tNode == sNode && s != t && !sIsGate && !tIsGate) {
    // TODO: should we just stop the path finding on tihs case?
    int dist = m->Distance(s, t);
    tmp.AddEdge(s, t, dist);
    tmp.AddEdge(t, s, dist);
  }
}

void PathFinderHelper::ForEachNeighbourGateWithST(int u,
                                                  NeighbourVertexVisitor<int> &visitor) const {
  m->GetGateGraph().ForEachNeighbours(u, visitor);
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

}  // namespace internal
}  // namespace qdpf
