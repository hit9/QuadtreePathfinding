// Source Code: https://github.com/hit9/quadtree-pathfinding
// License: BSD. Chao Wang, hit9[At]icloud.com.

#include "quadtree_map.hpp"

#include <cassert>

namespace qdpf {
namespace internal {

Gate::Gate(QdNode *aNode, QdNode *bNode, int a, int b) : aNode(aNode), bNode(bNode), a(a), b(b) {}

// ~~~~~~~~~~~~~~~ QuadtreeMap::Impl  ~~~~~~~~~~~

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
  assert(s > 0);  // debug mode: avoid s == 0, in case of division error
  g1.Init(n);
  g2.Init(n);
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

// ~~~~~~~~~~~~~~~ QuadtreeMap::Impl :: Cell Id Packing ~~~~~~~~~~~

// Use std::div for faster divide and module operations, than / and % operators.
//
// From https://en.cppreference.com/w/cpp/numeric/math/div:
//
//   > On many platforms, a single CPU instruction obtains both the quotient and the remainder,
//   > and this function may leverage that, although compilers are generally able to merge
//   > nearby / and % where suitable.
static std::pair<int, int> __div(int n, int k) {
  // from cppreference: the returned std::ldiv_t might be either form of
  // { int quot; int rem; } or { int rem; int quot; }, the order of its members is undefined,
  // we have to pack them into a pair for further structured bindings.
  auto dv = std::div(n, k);
  return {dv.quot, dv.rem};
}

int QuadtreeMap::PackXY(int x, int y) const { return s * x + y; }
std::pair<int, int> QuadtreeMap::UnpackXY(int v) const { return __div(v, s); }
int QuadtreeMap::UnpackX(int v) const { return v / s; }
int QuadtreeMap::UnpackY(int v) const { return v % s; }

// ~~~~~~~~~~~~~~~ QuadtreeMap::Impl :: Basic methods ~~~~~~~~~~~

int QuadtreeMap::Distance(int x1, int y1, int x2, int y2) const {
  return distance(x1, y1, x2, y2);
}

int QuadtreeMap::Distance(int u, int v) const {
  if (u == v) return 0;  // avoid further calculation.
  auto [x1, y1] = UnpackXY(u);
  auto [x2, y2] = UnpackXY(v);
  return distance(x1, y1, x2, y2);
}

int QuadtreeMap::DistanceBetweenNodes(QdNode *aNode, QdNode *bNode) const {
  if (aNode == bNode) return 0;
  int aNodeCenterX = aNode->x1 + (aNode->x2 - aNode->x1) / 2;
  int aNodeCenterY = aNode->y1 + (aNode->y2 - aNode->y1) / 2;
  int bNodeCenterX = bNode->x1 + (bNode->x2 - bNode->x1) / 2;
  int bNodeCenterY = bNode->y1 + (bNode->y2 - bNode->y1) / 2;
  return distance(aNodeCenterX, aNodeCenterY, bNodeCenterX, bNodeCenterY);
}

// ~~~~~~~~~~~~~ QuadtreeMap::Impl :: Visits and Reads ~~~~~~~~~~~~~~~~~

QdNode *QuadtreeMap::FindNode(int x, int y) const { return tree.Find(x, y); }

bool QuadtreeMap::IsGateCell(QdNode *node, int u) const { return gates1[node][u].Size() > 0; }

bool QuadtreeMap::IsGateCell(int u) const {
  auto [x, y] = UnpackXY(u);
  auto node = tree.Find(x, y);  // O(log Depth).
  return IsGateCell(node, u);
}

// Visit each gate cell inside a node and call given visitor with it.
void QuadtreeMap::ForEachGateInNode(const QdNode *node, GateVisitor &visitor) const {
  if (visitor == nullptr) return;
  std::function<void(Gate *)> visitor1 = [&visitor](Gate *gate) {
    // Won't allow user to modify gate's content.
    visitor(const_cast<const Gate *>(gate));
  };
  forEachGateInNode(const_cast<QdNode *>(node), visitor1);
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

void QuadtreeMap::NodesInRange(const Rectangle &rect, QdNodeVisitor &visitor) const {
  tree.QueryLeafNodesInRange(rect.x1, rect.y1, rect.x2, rect.y2, visitor);
}

// ~~~~~~~~~~~~~ QuadtreeMap::Impl :: Graphs Maintaining ~~~~~~~~~~~~~~~~~

void QuadtreeMap::Build() {
  // debug: the tree's size should be 0 before build.
  // If it isn't (failed here), checks if BuildTree() is called for at least twice.
  assert(tree.NumNodes() == 0);

  // build the empty tree, which creates the root node.
  tree.Build();

  std::vector<quadtree::BatchOperationItem<bool>> items;

  for (int x = 0; x < h; x++) {
    for (int y = 0; y < w; y++) {
      // On the first build, we care only about the obstacles.
      // the grid map will be splited into multiple sections,
      // and gates will be created for the first time.
      if (isObstacle(x, y)) items.push_back({x, y, true});
    }
  }

  tree.BatchAddToLeafNode(tree.GetRootNode(), items);
}

void QuadtreeMap::Update(int x, int y) {
  // ndebug, let's do nothing, don't crash the program.
  // since the (x,y) is passed in by a user-level programmer.
  if (!(x >= 0 && x < h)) return;
  if (!(y >= 0 && y < w)) return;

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

// ~~~~~~~~~~~~~ QuadtreeMap::Impl :: Internals ~~~~~~~~~~~~~~~~~

// visits each gate of a given node.
void QuadtreeMap::forEachGateInNode(QdNode *node, std::function<void(Gate *)> &visitor) const {
  if (gates1[node].Size() == 0) return;
  for (auto &[a, m] : gates1[node].GetUnderlyingUnorderedMap()) {  // gates1[aNode][a]
    for (auto &[b, gate] : m.GetUnderlyingUnorderedMap()) {        // gates1[aNode][a][b]
      visitor(gate);
    }
  }
}

// Connects given two cells in the gate graphs by establishing bidirectional edges between them.
void QuadtreeMap::connectCellsInGateGraphs(int u, int v) {
  int dist = Distance(u, v);
  g2.AddEdge(u, v, dist);
  g2.AddEdge(v, u, dist);
}

// Connects bidirectional edges between the new gate cell a and all other existing gate cells in
// this node. The given node must not be an obstacle node. Hint: all cells inside a non-obstacle
// node are reachable to each other.
void QuadtreeMap::connectGateCellsInNodeToNewGateCell(QdNode *aNode, int a) {
  for (auto &[u, m] : gates1[aNode].GetUnderlyingUnorderedMap()) {
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
  // idempotent: if aNode[a][b] => bNode exist
  auto gt1 = gates1[aNode][a][b];
  if (gt1 != nullptr && gt1->bNode == bNode) return;

  // idempotent: if bNode[b][a] => aNode exist
  auto gt2 = gates1[bNode][b][a];
  if (gt2 != nullptr && gt2->bNode == aNode) return;

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

  gates1[aNode][a][b] = gate1;
  gates1[bNode][b][a] = gate2;
}

// Disconnects all edges connecting with given gate cell u.
void QuadtreeMap::disconnectCellInGateGraphs(int u) {
  g2.ClearEdgeTo(u);
  g2.ClearEdgeFrom(u);
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

// Handle the node graph and all gate graphs changes on a quadtree node is removed.
// 1. (node graphs) Remove all edges connected aNode (from and to aNode).
// 2. (gate graphs) Remove all edges connected to any gate cells in aNode (from and to).
// 3. (gates) Remove all gates inside the node.
void QuadtreeMap::handleRemovedNode(QdNode *aNode) {
  disconnectNodeFromNodeGraph(aNode);

  // we first collect all gates in this node.
  std::vector<Gate *> aNodeGates;
  std::function<void(Gate *)> visitor = [&aNodeGates](Gate *gate) { aNodeGates.push_back(gate); };
  forEachGateInNode(aNode, visitor);

  // for each gate cell a inside aNode, disconnect a from the gate graph.
  for (auto gate : aNodeGates) disconnectCellInGateGraphs(gate->a);

  // remove the dual gate in each adjacent node b.
  for (const auto aGate : aNodeGates) {
    auto a = aGate->a, b = aGate->b;
    auto bNode = aGate->bNode;

    auto bGate = gates1[bNode][b][a];

    // remove bGate from bNode.
    gates1[bNode][b].Erase(a);
    gates.erase(bGate);
    delete bGate;

    // shrink the unordered_maps if empty.
    if (gates1[bNode][b].Size() == 0) {
      // that is, b is the gate cell which only points to a.
      // disconncts it from the gate graph.
      disconnectCellInGateGraphs(b);
      gates1[bNode].Erase(b);
    }

    if (gates1[bNode].Size() == 0) gates1.Erase(bNode);
  }

  // remove all gates inside aNode.
  for (auto gate : aNodeGates) {
    gates.erase(gate);
    delete gate;
  }
  gates1.Erase(aNode);
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
}  // namespace internal

}  // namespace qdpf
