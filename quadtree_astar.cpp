#include "quadtree_astar.hpp"

#include <algorithm>
#include <functional>
#include <queue>
#include <unordered_map>
#include <utility>
#include <vector>

namespace quadtree_astar {

static const int inf = 0x3f3f3f3f;

// ssf returns true to stop a quadtree node to split.
// Where w and h are the width and height of the node's region.
// n is the number of obstacles in this node.
// This ssf will stop the spliting if:
// 1. there's no obstacles in this node.
// 2. or, the cells in this nodes are all obstacles.
bool ssf(int w, int h, int n) { return n == 0 || (w * h == n); };

// packxy packs a cell (x,y) to an integeral id v.
int PathFinder::packxy(int x, int y) const { return m * x + y; }
// unpackxy unpacks a cell id v to a two-dimensional position (x,y).
std::pair<int, int> PathFinder::unpackxy(int v) const { return {v / m, v % m}; }
// unpackx unpacks a cell id v's x axis.
int PathFinder::unpackx(int v) const { return v / m; }
// unpacky unpacks a cell id v's y axis.
int PathFinder::unpacky(int v) const { return v % m; }

static const std::size_t __FNV_BASE = 14695981039346656037ULL;
static const std::size_t __FNV_PRIME = 1099511628211ULL;

// Hashing for gate.
std::size_t GateHasher::operator()(const Gate &g) const {
  // combine them via FNV hash.
  std::size_t h = __FNV_BASE;
  h ^= std::hash<int>{}(g.a);
  h *= __FNV_PRIME;
  h ^= std::hash<QdNode *>{}(g.bNode);
  h *= __FNV_PRIME;
  return h;
}

PathFinder::PathFinder(int w, int h, ObstacleChecker isObstacle, DistanceCalculator distance,
                       int step, bool use4directions)
    : w(w),
      h(h),
      step(step),
      m(std::max(w, h)),
      n(w * h),
      use4directions(use4directions),
      isObstacle(isObstacle),
      distance(distance),
      tree(QdTree(w, h, ssf)) {
  edges.resize(n), f.resize(n), vis.resize(n), from.resize(n);
  tree.SetAfterLeafRemovedCallback([this](QdNode *node) { handleRemovedNode(node); });
  tree.SetAfterLeafCreatedCallback([this](QdNode *node) { handleNewNode(node); });
}

void PathFinder::Gates(CellCollector &collector) const {
  for (const auto it : gates) {  // it is [aNode, gate set]
    for (const auto &gate : it.second) {
      auto [x, y] = unpackxy(gate.a);
      collector(x, y);
    }
  }
}

void PathFinder::Nodes(QdNodeCollector &collector) const {
  QdTree::VisitorT visitor = [&collector](QdNode *node) {
    if (node->isLeaf) collector(node);
  };
  tree.ForEachNode(visitor);
}

// helper function to calculate distance between cell u and v by passing their cell ids.
int PathFinder::calcDistance(int u, int v) const {
  auto [x1, y1] = unpackxy(u);
  auto [x2, y2] = unpackxy(v);
  return distance(x1, y1, x2, y2);
}

void PathFinder::Build() {
  tree.Build();
  for (int x = 0; x < h; x++) {
    for (int y = 0; y < w; y++) {
      // On the first build, we care only about the obstacles.
      // the grid map will be splited into multiple sections,
      // and gates will be created for the first time.
      if (isObstacle(x, y)) tree.Add(x, y, true);
    }
  }
}

void PathFinder::Update(int x, int y) {
  // Special case:
  //   When the (x,y) always locates in a single-cell 1x1 node before and after the tree
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

// Add a cell v locating at given node to the abstract graph as a vertex.
// It will create edges between this cell and other existing gate cells in this node.
// The given node must not be a obstacle node.
// All cells inside a non-obstacle node are reachable to each other.
void PathFinder::addVertex(QdNode *node, int v) {
  // v is reachable to any existing gate cells in the same node.
  for (auto &gate : gates[node]) {
    auto u = gate.a;
    if (u != v) {
      int dist = calcDistance(u, v);
      // add edge (u => v)
      edges[u].insert({v, dist});
      // add edge (v => u)
      edges[v].insert({u, dist});
    }
  }
}

// Remove a cell v locating at given node from the abstract graph.
// It will remove all edges between this cell and other existing gate cells in this node.
void PathFinder::removeVertex(QdNode *node, int v) {
  // remove edges from other gate cell to v inside node.
  for (auto gate : gates[node]) {
    auto u = gate.a;
    edges[u].erase(v);
  }
  // remove all edges starting from v.
  edges[v].clear();
}

// Add a connection between aNode and bNode through cell a and b.
// A connection is composed of two adjacent gates a and b.
//
//  +-------+-------+
//  |     [a|b]     |
//  +-------+-------+
//    aNode   bNode
//
// Steps:
// 1. Create edges inside each node.
// 2. Connects a and b and add them to the gate table.
void PathFinder::addConnection(QdNode *aNode, int a, QdNode *bNode, int b) {
  // add edges inside the node itself.
  addVertex(aNode, a);
  addVertex(bNode, b);

  int dist = calcDistance(a, b);
  // gate (a => b)
  gates[aNode].insert({a, bNode, b});
  edges[a].insert({b, dist});
  // gate (b => a)
  gates[bNode].insert({b, aNode, a});
  edges[b].insert({a, dist});
}

// Maintain gates and the abstract graph on a node is removed.
// Steps:
// 1. Remove all edges and vertices inside this node.
// 2. Remove all gates in this node from gate table.
void PathFinder::handleRemovedNode(QdNode *aNode) {
  // We should check this node no matter it contains obstacles or not.
  auto it = gates.find(aNode);
  if (it == gates.end()) return;

  for (auto [a, bNode, b] : it->second) {  // it is [aNode, set of gates in aNode]
    // remove gate (b => a) in bNode.
    edges[b].erase(a);
    gates[bNode].erase({b, aNode, a});
    // clears all edges starting from a.
    edges[a].clear();
  }

  // remove all gates in aNode.
  gates.erase(it);
}

// Maintain the gates and abstract graph on a leaf node is created.
// We first find all neighbour leaf nodes around this node, and then pick some neighbour cells as
// gates, and add connections.
void PathFinder::handleNewNode(QdNode *aNode) {
  // Ignore if it's a obstacle node.
  if (aNode->objects.size()) return;

  // find neighbours nodes.
  // neighbours[direction] => list of neighbour nodes
  std::vector<QdNode *> neighbours[8];
  // d is direction (0~3 NESW; 4~7 Diagonal).
  int d;
  quadtree::Visitor<bool> visitor = [this, &neighbours, &d](QdNode *bNode) {
    // cares only about non-obstacle nodes.
    if (bNode->objects.empty()) neighbours[d].push_back(bNode);
  };
  int maxd = use4directions ? 4 : 8;
  for (d = 0; d < maxd; d++) {
    tree.FindNeighbourLeafNodes(aNode, d, visitor);
  }

  // Diagonal directions.
  // For each diagonal direction, there is at most one neighbour node.
  // And we just need to pick only one pair of cells to create a connection.
  for (d = 4; d < maxd; d++) {
    if (neighbours[d].size()) {
      auto bNode = neighbours[d][0];
      int a, b;
      getNeighbourCellsDiagonal(d, aNode, a, b);
      addConnection(aNode, a, bNode, b);
    }
  }

  // Horizonal and Vertical directions.
  // This is more complex than the diagonal's case.
  // For each HV direction, there may be multiple neighbour nodes.
  // And for each neighbour, we pick multiple pairs of adjacent cells to create connections.
  // The following ncs is to collect picked gate cells, a vector of {x,y} pairs.
  std::vector<std::pair<int, int>> ncs;
  for (d = 0; d < 4; d++) {
    for (auto bNode : neighbours[d]) {
      ncs.clear();
      getNeighbourCellsHV(d, aNode, bNode, ncs);
      for (auto [a, b] : ncs) addConnection(aNode, a, bNode, b);
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
void PathFinder::getNeighbourCellsDiagonal(int direction, QdNode *aNode, int &a, int &b) {
  int x1 = aNode->x1, y1 = aNode->y1, x2 = aNode->x2, y2 = aNode->y2;
  switch (direction) {
    case 4:  // e,f
      a = packxy(x1, y1), b = packxy(x1 - 1, y1 - 1);
      return;
    case 5:  // g,h
      a = packxy(x1, y2), b = packxy(x1 - 1, y2 + 1);
      return;
    case 6:  // i,j
      a = packxy(x2, y2), b = packxy(x2 + 1, y2 + 1);
      return;
    case 7:  // k,l
      a = packxy(x2, y1), b = packxy(x2 + 1, y1 - 1);
      return;
  }
}

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
void PathFinder::getNeighbourCellsHV(int direction, QdNode *aNode, QdNode *bNode,
                                     std::vector<std::pair<int, int>> &ncs) {
  int x1, y1, x2, y2;
  switch (direction) {
    case 0:  // N
      x1 = aNode->x1, y1 = std::max(aNode->y1, bNode->y1), y2 = std::min(aNode->y2, bNode->y2);
      for (int y = y1; y <= y2; y += step) ncs.push_back({packxy(x1, y), packxy(x1 - 1, y)});
      break;
    case 1:  // E
      y2 = aNode->y2, x1 = std::max(aNode->x1, bNode->x1), x2 = std::min(aNode->x2, bNode->x2);
      for (int x = x1; x <= x2; x += step) ncs.push_back({packxy(x, y2), packxy(x, y2 + 1)});
      break;
    case 2:  // S
      x2 = aNode->x2, y1 = std::max(aNode->y1, bNode->y1), y2 = std::min(aNode->y2, bNode->y2);
      for (int y = y1; y <= y2; y += step) ncs.push_back({packxy(x2, y), packxy(x2 + 1, y)});
    case 3:  // W
      y1 = aNode->y1, x1 = std::max(aNode->x1, bNode->x1), x2 = std::min(aNode->x2, bNode->x2);
      for (int x = x1; x <= x2; x += step) ncs.push_back({packxy(x, y1), packxy(x, y1 - 1)});
      break;
  }
}

// A* shortest path.
void PathFinder::ComputeRoutes(int x1, int y1, int x2, int y2, CellCollector &collector) {
  // Can't route to or start from obstacles.
  if (isObstacle(x1, y1) || isObstacle(x2, y2)) return;

  int s = packxy(x1, y1), t = packxy(x2, y2);
  // find the quadtree node where the s and t locate.
  auto sNode = tree.Find(x1, y1), tNode = tree.Find(x2, y2);

  // add s and t to the abstract graph temporarily.
  addVertex(sNode, s);
  addVertex(tNode, t);

  // resets context
  std::fill(f.begin(), f.end(), inf);
  std::fill(vis.begin(), vis.end(), false);
  std::fill(from.begin(), from.end(), inf);

  // A* smallest-first queue.
  // P is { cost, cell id }
  std::priority_queue<P, std::vector<P>, std::greater<P>> q;

  f[s] = 0;
  q.push({f[s], s});

  while (q.size()) {
    auto [_, u] = q.top();
    q.pop();
    if (u == t) break;  // found
    if (vis[u]) continue;
    vis[u] = true;
    for (const auto [v, w] : edges[u]) {
      auto g = f[u] + w;
      auto h = calcDistance(v, t);
      auto cost = g + h;
      if (f[v] > g) {
        f[v] = g;
        q.push({cost, v});
        from[v] = u;
      }
    }
  }

  // remove s and t from the graph
  removeVertex(sNode, s);
  removeVertex(tNode, t);

  if (from[t] == inf) return;

  // collects route cells backward.
  std::vector<int> routes;
  routes.push_back(t);
  int v = t;
  while (v != s) {
    v = from[v];
    routes.push_back(v);
  }
  for (int i = routes.size() - 1; i >= 0; --i) {
    auto [x, y] = unpackxy(routes[i]);
    collector(x, y);
  }
}

}  // namespace quadtree_astar
