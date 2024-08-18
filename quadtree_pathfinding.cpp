// License: BSD. Version: 0.1.0. Author: Chao Wang, hit9[At]icloud.com.
// Source Code: https://github.com/hit9/quadtree-pathfinding

#include "quadtree_pathfinding.hpp"

#include <algorithm>   // for std::max, std::min
#include <functional>  // for std::function, std::greater
#include <utility>
#include <vector>

namespace quadtree_pathfinding {

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

//////////////////////////////////////
/// QuadtreeMap
//////////////////////////////////////

QuadtreeMap::QuadtreeMap(int w, int h, ObstacleChecker isObstacle, DistanceCalculator distance,
                         int step, StepFunction stepf, int maxNodeWidth, int maxNodeHeight)
    : w(w),
      h(h),
      step(step),
      s(std::max(w, h)),
      n(w * h),
      maxNodeWidth(maxNodeWidth == -1 ? w : maxNodeWidth),
      maxNodeHeight(maxNodeHeight == -1 ? h : maxNodeHeight),
      isObstacle(isObstacle),
      distance(distance),
      stepf(stepf),
      tree(QdTree(w, h)) {
  g1.Init(n);  // actually optional
  // ssf returns true to stop a quadtree node to continue to split.
  // Where w and h are the width and height of the node's region.
  // n is the number of obstacles in this node.
  // This ssf will stop the spliting if:
  // 1. there's no obstacles in this node.
  // 2. or, the cells in this nodes are all obstacles.
  // The quadtree node will always to split if its width and height exceeds
  // maxNodeWidth and maxNodeHeight.
  tree.SetSsf([this](int w, int h, int n) {
    return (w <= this->maxNodeWidth && h <= this->maxNodeHeight) && (n == 0 || (w * h == n));
  });
  // handleRemovedNode and handleNewNode maintain the sections and gates on quadtree adjustments.
  tree.SetAfterLeafRemovedCallback([this](QdNode *node) { handleRemovedNode(node); });
  tree.SetAfterLeafCreatedCallback([this](QdNode *node) { handleNewNode(node); });
}

QuadtreeMap::~QuadtreeMap() {
  // free all gate pointers.
  for (auto gate : gates) delete gate;
  gates.clear();
}

// ~~~~~~~~~~~~~~~ QuadtreeMap :: Cell ID Packing ~~~~~~~~~~~

int QuadtreeMap::PackXY(int x, int y) const { return s * x + y; }
std::pair<int, int> QuadtreeMap::UnpackXY(int v) const { return {v / s, v % s}; }
int QuadtreeMap::UnpackX(int v) const { return v / s; }
int QuadtreeMap::UnpackY(int v) const { return v % s; }

// ~~~~~~~~~~~~~~~ QuadtreeMap :: Basic methods ~~~~~~~~~~~

int QuadtreeMap::Distance(int u, int v) const {
  auto [x1, y1] = UnpackXY(u);
  auto [x2, y2] = UnpackXY(v);
  return distance(x1, y1, x2, y2);
}

void QuadtreeMap::RegisterGateGraph(GateGraph *g) { g2s.push_back(g); }

int QuadtreeMap::DistanceBetweenNodes(QdNode *aNode, QdNode *bNode) const {
  int aNodeCenterX = aNode->x1 + (aNode->x2 - aNode->x1) / 2;
  int aNodeCenterY = aNode->y1 + (aNode->y2 - aNode->y1) / 2;
  int bNodeCenterX = bNode->x1 + (bNode->x2 - bNode->x1) / 2;
  int bNodeCenterY = bNode->y1 + (bNode->y2 - bNode->y1) / 2;
  return distance(aNodeCenterX, aNodeCenterY, bNodeCenterX, bNodeCenterY);
}

// ~~~~~~~~~~~~~ QuadtreeMap :: Visits and Reads ~~~~~~~~~~~~~~~~~

QdNode *QuadtreeMap::FindNode(int x, int y) const { return tree.Find(x, y); }

bool QuadtreeMap::IsGateCell(QdNode *node, int u) const {
  auto it = gates1.find(node);
  // the node is not found.
  if (it == gates1.end()) return false;
  // m is gates1[node], format: {u => set of gates}
  auto &m = it->second;
  return m.find(u) != m.end();
}

bool QuadtreeMap::IsGateCell(int u) const {
  auto [x, y] = UnpackXY(u);
  auto node = tree.Find(x, y);  // O(log Depth).
  return IsGateCell(node, u);
}

// Visit each gate cell inside a node and call given visitor with it.
void QuadtreeMap::ForEachGateInNode(QdNode *node, GateVisitor &visitor) const {
  std::function<void(Gate *)> visitor1 = [&visitor](Gate *gate) {
    // Won't allow user to modify gate's content.
    visitor(const_cast<const Gate *>(gate));
  };
  forEachGateInNode(node, visitor1);
}

void QuadtreeMap::Nodes(QdNodeVisitor &visitor) const {
  QdTree::VisitorT visitor1 = [&visitor](QdNode *node) {
    if (node->isLeaf) visitor(node);
  };
  tree.ForEachNode(visitor1);
}

void QuadtreeMap::Gates(GateVisitor &visitor) const {
  for (const auto &gate : gates) visitor(gate);
}

void QuadtreeMap::ForEachNeighbourNodes(QdNode *node,
                                        NeighbourVertexVisitor<QdNode *> &visitor) const {
  g1.ForEachNeighbours(node, visitor);
}

// ~~~~~~~~~~~~~ QuadtreeMap :: Graphs Maintaining ~~~~~~~~~~~~~~~~~

void QuadtreeMap::Build() {
  tree.Build();
  for (int x = 0; x < h; x++) {
    for (int y = 0; y < w; y++) {
      // On the first build, we care only about the obstacles.
      // the grid map will be splited into multiple sections,
      // and gates will be created for the first time.
      if (isObstacle(x, y)) Update(x, y);
    }
  }
}

void QuadtreeMap::Update(int x, int y) {
  // Special case:
  //   When the (x,y) always locates at a single-cell 1x1 node before and after the tree
  //   adjustment. Changing this cell's value won't trigger spliting and merging, the ssf
  //   determines this fact. In this case, we should call handleNewNode and handleRemovedNode
  //   manually to ensure the gates are still maintained in this scenario, as if this node is
  //   removed or created.
  auto b = isObstacle(x, y);
  auto node = tree.Find(x, y);
  // Is it 1x1 node before?
  auto before1x1 = (node->x1 == node->x2 && node->y1 == node->y2);
  if (b)
    tree.Add(x, y, true);
  else
    tree.Remove(x, y, true);

  // Refresh the node it locates.
  node = tree.Find(x, y);
  // Is it 1x1 node after?
  auto after1x1 = (node->x1 == node->x2 && node->y1 == node->y2);
  if (before1x1 && after1x1) {
    if (b)
      handleRemovedNode(node);
    else
      handleNewNode(node);
  }
}

// ~~~~~~~~~~~~~ QuadtreeMap :: Internals ~~~~~~~~~~~~~~~~~

// visits each gate of a given node.
void QuadtreeMap::forEachGateInNode(QdNode *node, std::function<void(Gate *)> &visitor) const {
  auto it = gates1.find(node);
  if (it == gates1.end()) return;
  // m is gates1[node]
  const auto &m = it->second;
  for (const auto &[_, st] : m) {
    // for each gate inside node.
    for (auto gate : st) visitor(gate);
  }
}

// Connects given two cells in the gate graphs by establishing bidirectional edges between them.
void QuadtreeMap::connectCellsInGateGraphs(int u, int v) {
  int dist = Distance(u, v);
  for (auto g : g2s) {
    g->AddEdge(u, v, dist);
    g->AddEdge(v, u, dist);
  }
}

// Connects bidirectional edges between the new gate cell a and all other existing gate cells in
// this node. The given node must not be an obstacle node. Hint: all cells inside a non-obstacle
// node are reachable to each other.
void QuadtreeMap::connectGateCellsInNodeToNewGateCell(QdNode *aNode, int a) {
  auto it = gates1.find(aNode);
  // the aNode is not found.
  if (it == gates1.end()) return;
  // m is gates1[aNode], map of { a => gate set }
  const auto &m = it->second;
  for (const auto &[u, _] : m) {
    if (u != a) connectCellsInGateGraphs(u, a);
  }
}

// Creates a gate between aNode and bNode through cell a and b.
//  +-------+-------+
//  |     [a|b]     |
//  +-------+-------+
//    aNode   bNode
// Steps:
// 1. Connect edges with existing gate cells inside each node.
// 2. Connects a and b and add the created gate into management.
void QuadtreeMap::createGate(QdNode *aNode, int a, QdNode *bNode, int b) {
  // bidirection edges between new gate cell and existing gate cells inside each node.
  connectGateCellsInNodeToNewGateCell(aNode, a);
  connectGateCellsInNodeToNewGateCell(bNode, b);
  // connects a and b.
  connectCellsInGateGraphs(a, b);
  // creates a gate and maintain into container gates and gates.
  auto gate1 = new Gate(aNode, bNode, a, b);  // a => b
  auto gate2 = new Gate(bNode, aNode, b, a);  // b => a
  gates.insert(gate1);
  gates.insert(gate2);
  gates1[aNode][a].insert(gate1);
  gates1[bNode][b].insert(gate2);
}

// Disconnects all edges connecting with given gate cell u.
void QuadtreeMap::disconnectCellInGateGraphs(int u) {
  for (auto g : g2s) {
    g->ClearEdgeTo(u);
    g->ClearEdgeFrom(u);
  }
}

// Disconnects all gate cells in aNode from the gate graphs.
void QuadtreeMap::disconnectCellsInNodeFromGateGraphs(QdNode *aNode) {
  auto it = gates1.find(aNode);
  // node is not found.
  if (it == gates1.end()) return;
  // m is gates1[aNode], map of { a => set of gates }
  auto &m = it->second;
  for (auto &[u, _] : m) {
    disconnectCellInGateGraphs(u);
  }
}

//  Connects given two nodes on the node graph.
void QuadtreeMap::connectNodesOnNodeGraph(QdNode *aNode, QdNode *bNode) {
  // use the distance betwen the two nodes's center cells
  int dist = DistanceBetweenNodes(aNode, bNode);
  g1.AddEdge(aNode, bNode, dist);
  g1.AddEdge(bNode, aNode, dist);
}

// Disconnects the given node from the node graphs.
void QuadtreeMap::disconnectNodeFromNodeGraph(QdNode *aNode) {
  g1.ClearEdgeTo(aNode);
  g1.ClearEdgeFrom(aNode);
}

// Remove gate (a => b) from given node aNode.
void QuadtreeMap::removeGateInNode(QdNode *aNode, int a, QdNode *bNode, int b) {
  auto it = gates1.find(aNode);
  if (it != gates1.end()) {
    // m is gates1[aNode]
    auto &m = it->second;
    auto it1 = m.find(a);
    if (it1 != m.end()) {
      // st is gates1[aNode][a], set of gate pointers starting at cell a.
      // we should find the gate points from a to b.
      auto &st = it1->second;

      // the set should be very small.
      Gate *target = nullptr;
      for (auto gate : st) {
        if (gate->bNode == bNode && gate->b == b) {
          target = gate;
          break;
        }
      }

      if (target != nullptr) {
        // remove the gate (a => b)
        gates.erase(target);
        st.erase(target);
        // remove the container st and m if empty.
        if (st.empty()) m.erase(a);
        if (m.empty()) gates1.erase(aNode);

        delete target;
      }
    }
  }
}

// Handle the node graph and all gate graphs changes on a quadtree node is removed.
// 1. (gate graphs) Remove all edges connected to any gate cells in aNode (from and to).
// 2. (node graphs) Remove all edges connected aNode (from and to aNode).
// 3. (gates) Remove all gates inside the node.
void QuadtreeMap::handleRemovedNode(QdNode *aNode) {
  disconnectNodeFromNodeGraph(aNode);
  disconnectCellsInNodeFromGateGraphs(aNode);
  // ~~~~~~~~ disposes all gates for this node. ~~~~~~~~~

  // we first collect all gates in this node.
  std::vector<Gate *> aNodeGates;
  std::function<void(Gate *)> visitor = [&aNodeGates](Gate *gate) { aNodeGates.push_back(gate); };
  forEachGateInNode(aNode, visitor);

  // remove gates in each adjacent node b.
  for (const auto gate : aNodeGates) {
    auto a = gate->a, b = gate->b;
    auto bNode = gate->bNode;
    removeGateInNode(bNode, b, aNode, a);
  }

  // remove all gates inside aNode.
  for (auto gate : aNodeGates) {
    gates.erase(gate);
    delete gate;
  }
  gates1.erase(aNode);
}

// Handle the node graph and all gate graphs changes on a quadtree node is created.
// 1. find all neighbour leaf nodes around this node.
// 2. pick some neighbour cells to create gates.
// 3. and finally establish the edges in all graphs.
void QuadtreeMap::handleNewNode(QdNode *aNode) {
  // ignores if it's a obstacle node.
  if (aNode->objects.size()) return;

  // ~~~~~~ find neighbours nodes ~~~~~~~

  // format: neighbours[direction] => list of neighbour nodes
  std::vector<QdNode *> neighbours[8];
  // direction: 0~3 NESW; 4~7 Diagonal
  int d;
  quadtree::Visitor<bool> visitor = [this, &neighbours, &d](QdNode *bNode) {
    // cares only about non-obstacle nodes.
    if (bNode->objects.empty()) neighbours[d].push_back(bNode);
  };
  for (d = 0; d < 8; d++) {
    tree.FindNeighbourLeafNodes(aNode, d, visitor);
  }

  // Diagonal directions.
  for (d = 4; d < 8; d++) {
    if (neighbours[d].size()) {
      auto bNode = neighbours[d][0];
      // connects a and b on the node graph.
      connectNodesOnNodeGraph(aNode, bNode);
      // connects a and b on the gate graph:
      // for each diagonal direction, there is at most one neighbour node.
      // and we just need to pick only one pair of cells to create a connection.
      int a, b;
      getNeighbourCellsDiagonal(d, aNode, a, b);
      createGate(aNode, a, bNode, b);
    }
  }

  // Horizonal and Vertical directions.
  std::vector<std::pair<int, int>> ncs;
  for (d = 0; d < 4; d++) {
    for (auto bNode : neighbours[d]) {
      // connects a and b on the node graph.
      connectNodesOnNodeGraph(aNode, bNode);
      // connects a and b on the gate graph:
      // this is more complex than the diagonal's case.
      // for each HV direction, there may be multiple neighbour nodes.
      // and for each neighbour, we pick multiple pairs of adjacent cells to create connections.
      // the following ncs is to collect picked gate cells, a vector of {x,y} pairs.
      ncs.clear();
      getNeighbourCellsHV(d, aNode, bNode, ncs);
      for (auto [a, b] : ncs) createGate(aNode, a, bNode, b);
    }
  }
}

// getNeighbourCellsDiagonal sets given a and b by reference for given direction.
// Where the a and b are the gate cell ids to derive.
// In the diagram below, '(e,f),(g,h),(i,j),(kl)' are (a,b)'s positions for each direction
// (4,5,6,7).
//
//         y1    y2
//     4  f|     |h   5
//       --e-----g--    x1
//         |     |
//         |     |
//       --k-----i--    x2
//     7  l|     |j   6
void QuadtreeMap::getNeighbourCellsDiagonal(int direction, QdNode *aNode, int &a, int &b) const {
  int x1 = aNode->x1, y1 = aNode->y1, x2 = aNode->x2, y2 = aNode->y2;
  switch (direction) {
    case 4:  // e,f
      a = PackXY(x1, y1), b = PackXY(x1 - 1, y1 - 1);
      return;
    case 5:  // g,h
      a = PackXY(x1, y2), b = PackXY(x1 - 1, y2 + 1);
      return;
    case 6:  // i,j
      a = PackXY(x2, y2), b = PackXY(x2 + 1, y2 + 1);
      return;
    case 7:  // k,l
      a = PackXY(x2, y1), b = PackXY(x2 + 1, y1 - 1);
      return;
  }
}

// getNeighbourCellsHV collects neighbour gate cells for given direction between the two adjacent
// nodes aNode and bNode.
// Where the target gate cells locate like diagram below:
//
//            N:0
//         y1    y2
//         |     |
//         a     b
//       -g+-----+c-    x1
//   W:3   |     |        E:1
//         |     |
//       -h+-----+d-    x2
//         e     f
//         |     |
//           S:2
void QuadtreeMap::getNeighbourCellsHV(int direction, QdNode *aNode, QdNode *bNode,
                                      std::vector<std::pair<int, int>> &ncs) const {
  int x1, y1, x2, y2, d;
  switch (direction) {
    case 0:  // N
      x1 = aNode->x1, y1 = std::max(aNode->y1, bNode->y1), y2 = std::min(aNode->y2, bNode->y2);
      d = stepf == nullptr ? step : stepf(y2 - y1 + 1);
      for (int y = y1; y <= y2; y += d) ncs.push_back({PackXY(x1, y), PackXY(x1 - 1, y)});
      break;
    case 1:  // E
      y2 = aNode->y2, x1 = std::max(aNode->x1, bNode->x1), x2 = std::min(aNode->x2, bNode->x2);
      d = stepf == nullptr ? step : stepf(x2 - x1 + 1);
      for (int x = x1; x <= x2; x += d) ncs.push_back({PackXY(x, y2), PackXY(x, y2 + 1)});
      break;
    case 2:  // S
      x2 = aNode->x2, y1 = std::max(aNode->y1, bNode->y1), y2 = std::min(aNode->y2, bNode->y2);
      d = stepf == nullptr ? step : stepf(y2 - y1 + 1);
      for (int y = y1; y <= y2; y += d) ncs.push_back({PackXY(x2, y), PackXY(x2 + 1, y)});
      break;
    case 3:  // W
      y1 = aNode->y1, x1 = std::max(aNode->x1, bNode->x1), x2 = std::min(aNode->x2, bNode->x2);
      d = stepf == nullptr ? step : stepf(x2 - x1 + 1);
      for (int x = x1; x <= x2; x += d) ncs.push_back({PackXY(x, y1), PackXY(x, y1 - 1)});
      break;
  }
}

//////////////////////////////////////
/// PathFinderHelper
//////////////////////////////////////

PathFinderHelper::PathFinderHelper(const QuadtreeMap &m, IDirectedGraph<int> *g2) : m(m), g2(g2) {}

// Add given cell u to given node temporarily to the tmp gate graph.
// It will establish edges between u and all gate cells in node.
void PathFinderHelper::addCellToTmpGateGraph(int u, QdNode *node) {
  GateVisitor visitor = [this, u](const Gate *gate) {
    auto v = gate->a;
    if (u != v) {
      int dist = m.Distance(u, v);
      tmp.AddEdge(u, v, dist);  // (u => v)
      tmp.AddEdge(v, u, dist);  // (v => u)
    }
  };
  m.ForEachGateInNode(node, visitor);
}

void PathFinderHelper::BuildTmpGateGraph(int s, int t, int x1, int y1, int x2, int y2,
                                         QdNode *sNode, QdNode *tNode) {
  // Is the start and target a gate cell?
  bool sIsGate = m.IsGateCell(sNode, s), tIsGate = m.IsGateCell(tNode, t);

  // Add s and t to the tmp graph, if they are not gates.
  if (!sIsGate) addCellToTmpGateGraph(s, sNode);
  if (!tIsGate) addCellToTmpGateGraph(t, tNode);

  // A special case:
  // if s and t are in the same node, and both of them aren't gates,
  // we should connect them with edges, the shortest path should be dist itself.
  if (tNode == sNode && s != t && !sIsGate && !tIsGate) {
    // TODO: should we just stop the path finding on tihs case?
    int dist = m.Distance(s, t);
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

AStarPathFinder::AStarPathFinder(const QuadtreeMap &m) : PathFinderHelper(m, &g) { g.Init(m.N()); }
IDirectedGraph<int> *AStarPathFinder::GetGateGraph() { return &g; }

void AStarPathFinder::Reset(int x1_, int y1_, int x2_, int y2_) {
  x1 = x1_, y1 = y1_, x2 = x2_, y2 = y2_;
  s = m.PackXY(x1, y1), t = m.PackXY(x2, y2);
  sNode = m.FindNode(x1, y1), tNode = m.FindNode(x2, y2);
  nodePath.clear();
  gateCellsOnNodePath.clear();
  // Rebuild the tmp graph.
  tmp.Clear();
  BuildTmpGateGraph(s, t, x1, y1, x2, y2, sNode, tNode);
}

int AStarPathFinder::ComputeNodeRoutes() {
  if (sNode == tNode) {
    // Same node.
    nodePath.push_back({sNode, 0});
    return 0;
  }
  using A = AStar<QdNode *, nullptr>;
  A astar;
  // collector for path result.
  A::PathCollector collector = [this](QdNode *node, int cost) {
    nodePath.push_back({node, cost});
  };
  // collector for neighbour qd nodes.
  A::NeighboursCollector neighborsCollector = [this](QdNode *u,
                                                     NeighbourVertexVisitor<QdNode *> &visitor) {
    m.ForEachNeighbourNodes(u, visitor);
  };
  // distance calculator.
  A::Distance distance = [this](QdNode *a, QdNode *b) { return m.DistanceBetweenNodes(a, b); };
  A::NeighbourFilterTester neighbourTester = nullptr;
  // compute
  return astar.Compute(m.NumLeafNodes() + 1, neighborsCollector, distance, sNode, tNode, collector,
                       neighbourTester);
}

void AStarPathFinder::VisitComputedNodeRoutes(QdNodeVisitor &visitor) const {
  for (const auto [node, cost] : nodePath) visitor(node);
}

// Collects the gate cells on node path if ComputeNodeRoutes is successfully called and any further
// ComputeGateRoutes call specifics the useNodePath true.
// Notes that the start and target should be also collected.
void AStarPathFinder::collectGateCellsOnNodePath() {
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
  for (; i != nodePath.size(); ++i) m.ForEachGateInNode(nodePath[i].first, visitor);
}

int AStarPathFinder::ComputeGateRoutes(CellCollector &collector, bool useNodePath) {
  // Can't route to or start from obstacles.
  if (m.IsObstacle(x1, y1) || m.IsObstacle(x2, y2)) return -1;
  // Same point.
  if (x1 == x2 && y1 == y2) {
    collector(x1, y1);
    return 0;
  }
  if (useNodePath) collectGateCellsOnNodePath();
  // If useNodePath and nodePath is empty, return -1;
  // if useNodePath then collect all gate cells for these node.
  // Path finding on gate cells.
  using A = AStar<int, inf, std::vector<int>, std::vector<bool>, std::vector<int>>;
  A astar;
  // collector for path result.
  A::PathCollector collector1 = [this, &collector](int u, int cost) {
    auto [x, y] = m.UnpackXY(u);
    collector(x, y);
  };

  A::NeighbourFilterTester neighbourTester = [this, useNodePath](int v) {
    if (useNodePath && gateCellsOnNodePath.find(v) == gateCellsOnNodePath.end()) return false;
    return true;
  };
  // collector for neighbour gate cells.
  A::NeighboursCollector neighborsCollector = [this](int u, NeighbourVertexVisitor<int> &visitor) {
    ForEachNeighbourGateWithST(u, visitor);
  };
  // distance calculator.
  A::Distance distance1 = [this](int u, int v) { return m.Distance(u, v); };
  // compute
  return astar.Compute(m.N(), neighborsCollector, distance1, s, t, collector1, neighbourTester);
}

}  // namespace quadtree_pathfinding
