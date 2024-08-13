#include "quadtree_astar.hpp"

#include <algorithm>
#include <cmath>
#include <functional>  // for std::function, std::greater
#include <iostream>
#include <queue>  // for std::priority_queue
#include <unordered_map>
#include <utility>
#include <vector>

namespace quadtree_astar {

static const int inf = 0x3f3f3f3f;

//////////////////////////////////////
/// SimpleDirectedGraph
//////////////////////////////////////

void SimpleDirectedGraph::Init(int n) { edges.resize(n); }
void SimpleDirectedGraph::AddEdge(int u, int v, int cost) { edges[u].insert({v, cost}); }
void SimpleDirectedGraph::RemoveEdge(int u, int v) { edges[u].erase(v); }
void SimpleDirectedGraph::ClearEdgeFrom(int u) { edges[u].clear(); }
void SimpleDirectedGraph::ForEachNeighbours(int u, NeighbourVertexVisitor &visitor) const {
  for (const auto [v, cost] : edges[u]) visitor(v, cost);
}
void SimpleDirectedGraph::Clear() { edges.clear(); }

//////////////////////////////////////
/// SimpleUnorderedMapDirectedGraph
//////////////////////////////////////

void SimpleUnorderedMapDirectedGraph::Init(int n) {}

void SimpleUnorderedMapDirectedGraph::AddEdge(int u, int v, int cost) {
  edges[u].insert({v, cost});
}

void SimpleUnorderedMapDirectedGraph::RemoveEdge(int u, int v) {
  auto it = edges.find(u);
  if (it == edges.end()) return;
  auto &m = it->second;
  m.erase(v);
  if (m.empty()) edges.erase(it);
}

void SimpleUnorderedMapDirectedGraph::ClearEdgeFrom(int u) {
  auto it = edges.find(u);
  if (it == edges.end()) return;
  auto &m = it->second;
  m.clear();
  edges.erase(it);
}

void SimpleUnorderedMapDirectedGraph::ForEachNeighbours(int u,
                                                        NeighbourVertexVisitor &visitor) const {
  auto it = edges.find(u);
  if (it == edges.end()) return;
  for (const auto [v, cost] : it->second) visitor(v, cost);
}

void SimpleUnorderedMapDirectedGraph::Clear() { edges.clear(); }

//////////////////////////////////////
/// QuadtreeMap
//////////////////////////////////////

QuadtreeMap::QuadtreeMap(int w, int h, ObstacleChecker isObstacle, DistanceCalculator distance,
                         int step, int maxNodeWidth, int maxNodeHeight)
    : w(w),
      h(h),
      step(step),
      s(std::max(w, h)),
      n(w * h),
      maxNodeWidth(maxNodeWidth == -1 ? w : maxNodeWidth),
      maxNodeHeight(maxNodeHeight == -1 ? h : maxNodeHeight),
      isObstacle(isObstacle),
      distance(distance),
      tree(QdTree(w, h)) {
  // ssf returns true to stop a quadtree node to split.
  // Where w and h are the width and height of the node's region.
  // n is the number of obstacles in this node.
  // This ssf will stop the spliting if:
  // 1. there's no obstacles in this node.
  // 2. or, the cells in this nodes are all obstacles.
  // The quadtree node will always to split if its width and height exceeds
  // maxNodeWidth and maxNodeHeight.
  tree.SetSsf([this](int w, int h, int n) {
    return (w <= this->maxNodeWidth && h <= this->maxNodeWidth) && (n == 0 || (w * h == n));
  });
  // handleRemovedNode and handleNewNode maintains the sections and gates on quadtree adjustments.
  tree.SetAfterLeafRemovedCallback([this](QdNode *node) { handleRemovedNode(node); });
  tree.SetAfterLeafCreatedCallback([this](QdNode *node) { handleNewNode(node); });
}

int QuadtreeMap::PackXY(int x, int y) const { return s * x + y; }
std::pair<int, int> QuadtreeMap::UnpackXY(int v) const { return {v / s, v % s}; }
int QuadtreeMap::UnpackX(int v) const { return v / s; }
int QuadtreeMap::UnpackY(int v) const { return v % s; }

int QuadtreeMap::Distance(int u, int v) const {
  auto [x1, y1] = UnpackXY(u);
  auto [x2, y2] = UnpackXY(v);
  return distance(x1, y1, x2, y2);
}

bool QuadtreeMap::IsObstacle(int x, int y) const { return isObstacle(x, y); }

void QuadtreeMap::RegisterGraph(IDirectedGraph *g) {
  g->Init(n);
  graphs.push_back(g);
}

QdNode *QuadtreeMap::FindNode(int x, int y) const { return tree.Find(x, y); }

bool QuadtreeMap::IsGate(QdNode *node, int u) const {
  auto it = gates.find(node);
  if (it == gates.end()) return false;
  return it->second.find(u) != it->second.end();
}

void QuadtreeMap::ForEachGateInNode(QdNode *node, std::function<void(int u)> &visitor) const {
  auto it = gates.find(node);
  if (it == gates.end()) return;
  auto &m = it->second;
  for (const auto &[u, _] : m) visitor(u);
}

void QuadtreeMap::Gates(CellCollector &collector) const {
  for (const auto &[aNode, m] : gates) {
    for (const auto &[a, _] : m) {
      auto [x, y] = UnpackXY(a);
      collector(x, y);
    }
  }
}

void QuadtreeMap::Nodes(QdNodeCollector &collector) const {
  QdTree::VisitorT visitor = [&collector](QdNode *node) {
    if (node->isLeaf) collector(node);
  };
  tree.ForEachNode(visitor);
}

void QuadtreeMap::Build() {
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

// Add a cell v locating at given node to the abstract graph as a vertex.
// It will create edges between v and other existing gate cells in this node.
// The given node must not be a obstacle node.
// All cells inside a non-obstacle node are reachable to each other.
void QuadtreeMap::addVertex(QdNode *node, int v) {
  auto it = gates.find(node);
  if (it == gates.end()) return;
  auto &m = it->second;
  // v is reachable to any existing gate cells in the same node.
  for (const auto &[a, _] : m) {
    if (a != v) {
      int dist = Distance(a, v);
      for (auto g : graphs) {
        g->AddEdge(a, v, dist);  // (a => v)
        g->AddEdge(v, a, dist);  // (v => a)
      }
    }
  }
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
void QuadtreeMap::addConnection(QdNode *aNode, int a, QdNode *bNode, int b) {
  // add edges inside the node itself.
  addVertex(aNode, a);
  addVertex(bNode, b);

  // add gates (a => b) and (b => a)
  int dist = Distance(a, b);
  gates[aNode][a].insert({b, bNode});
  gates[bNode][b].insert({a, aNode});

  for (auto g : graphs) {
    g->AddEdge(a, b, dist);  // (a => b)
    g->AddEdge(b, a, dist);  // (b => a)
  }
}

// Maintain gates and the abstract graph on a node is removed.
// Steps:
// 1. Remove all edges and vertices inside this node.
// 2. Remove all gates in this node from gate table.
void QuadtreeMap::handleRemovedNode(QdNode *aNode) {
  // We should check this node no matter it contains obstacles or not.
  auto it = gates.find(aNode);
  if (it == gates.end()) return;
  auto &m = it->second;

  // Remove any bNode's edges pointing to any one of aNode's gates a.
  for (const auto &[a, c] : m) {
    for (auto [b, bNode] : c) {
      // remove gate (b => a) in bNode.
      gates[bNode][b].erase(a);
      for (auto g : graphs) {
        g->RemoveEdge(b, a);
      }
    }
  }

  for (const auto &[a, _] : m) {
    // clears all edges starting form a.
    for (auto g : graphs) {
      g->ClearEdgeFrom(a);
    }
  }
  gates.erase(it);
}

// Maintain the gates and abstract graph on a leaf node is created.
// We first find all neighbour leaf nodes around this node, and then pick some neighbour cells as
// gates, and add connections.
void QuadtreeMap::handleNewNode(QdNode *aNode) {
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
  for (d = 0; d < 8; d++) {
    tree.FindNeighbourLeafNodes(aNode, d, visitor);
  }

  // Diagonal directions.
  // For each diagonal direction, there is at most one neighbour node.
  // And we just need to pick only one pair of cells to create a connection.
  for (d = 4; d < 8; d++) {
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
      for (auto [a, b] : ncs) {
        addConnection(aNode, a, bNode, b);
      }
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
  int x1, y1, x2, y2;
  switch (direction) {
    case 0:  // N
      x1 = aNode->x1, y1 = std::max(aNode->y1, bNode->y1), y2 = std::min(aNode->y2, bNode->y2);
      for (int y = y1; y <= y2; y += step) ncs.push_back({PackXY(x1, y), PackXY(x1 - 1, y)});
      break;
    case 1:  // E
      y2 = aNode->y2, x1 = std::max(aNode->x1, bNode->x1), x2 = std::min(aNode->x2, bNode->x2);
      for (int x = x1; x <= x2; x += step) ncs.push_back({PackXY(x, y2), PackXY(x, y2 + 1)});
      break;
    case 2:  // S
      x2 = aNode->x2, y1 = std::max(aNode->y1, bNode->y1), y2 = std::min(aNode->y2, bNode->y2);
      for (int y = y1; y <= y2; y += step) ncs.push_back({PackXY(x2, y), PackXY(x2 + 1, y)});
    case 3:  // W
      y1 = aNode->y1, x1 = std::max(aNode->x1, bNode->x1), x2 = std::min(aNode->x2, bNode->x2);
      for (int x = x1; x <= x2; x += step) ncs.push_back({PackXY(x, y1), PackXY(x, y1 - 1)});
      break;
  }
}

//////////////////////////////////////
/// IPathFinder
//////////////////////////////////////

void IPathFinder::ComputePathToNextRoute(int x1, int y1, int x2, int y2,
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

AStarPathFinder::AStarPathFinder(const QuadtreeMap &m) : m(m) {
  // allocates memory
  f.resize(m.N()), from.resize(m.N()), vis.resize(m.N());
}

IDirectedGraph *AStarPathFinder::GetGraph() { return &g; }

// Add given vertex u to locating at given node the temporarily graph tmp.
// It will establish edges between u and all gate cells in node.
void AStarPathFinder::addVertexToTmpGraph(int u, QdNode *node) {
  std::function<void(int)> visitor = [this, u](int v) {
    if (u != v) {
      int dist = m.Distance(u, v);
      tmp.AddEdge(u, v, dist);  // (u => v)
      tmp.AddEdge(v, u, dist);  // (v => u)
    }
  };
  m.ForEachGateInNode(node, visitor);
}

void AStarPathFinder::BuildTmpGraph(int s, int t, int x1, int y1, int x2, int y2) {
  // Find the quadtree node where the s and t locate.
  auto sNode = m.FindNode(x1, y1), tNode = m.FindNode(x2, y2);

  // Is the start and target a gate cell?
  bool sIsGate = m.IsGate(sNode, s), tIsGate = m.IsGate(tNode, t);

  // Add s and t to the tmp graph, if they are not gates.
  if (!sIsGate) addVertexToTmpGraph(s, sNode);
  if (!tIsGate) addVertexToTmpGraph(t, tNode);

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

void AStarPathFinder::ForEachNeighboursWithST(int u, NeighbourVertexVisitor &visitor) const {
  g.ForEachNeighbours(u, visitor);
  tmp.ForEachNeighbours(u, visitor);
}

void AStarPathFinder::ComputeRoutes(int x1, int y1, int x2, int y2, CellCollector &collector) {
  // Can't route to or start from obstacles.
  if (m.IsObstacle(x1, y1) || m.IsObstacle(x2, y2)) return;
  // Same point.
  if (x1 == x2 && y1 == y2) {
    collector(x1, y1);
    return;
  }

  int s = m.PackXY(x1, y1), t = m.PackXY(x2, y2);

  // (re)Build the tmp graph.
  tmp.Clear();
  BuildTmpGraph(s, t, x1, y1, x2, y2);

  // Resets A* context.
  std::fill(f.begin(), f.end(), inf);
  std::fill(vis.begin(), vis.end(), false);
  from[t] = inf;

  // collects neighbour vertices into a temporary vector, where P is { v, cost }.
  std::vector<P> neighbours;
  NeighbourVertexVisitor visitor = [&neighbours](int v, int cost) {
    neighbours.push_back({v, cost});
  };

  // A* smallest-first queue, where P is { cost, cell id }
  std::priority_queue<P, std::vector<P>, std::greater<P>> q;

  // A* search algorithm.
  f[s] = 0;
  q.push({f[s], s});

  while (q.size()) {
    auto [_, u] = q.top();
    q.pop();
    if (u == t) break;  // found
    if (vis[u]) continue;
    vis[u] = true;
    ForEachNeighboursWithST(u, visitor);
    for (const auto [v, c] : neighbours) {
      auto g = f[u] + c;
      auto h = m.Distance(v, t);
      auto cost = g + h;
      if (f[v] > g) {
        f[v] = g;
        q.push({cost, v});
        from[v] = u;
      }
    }
    neighbours.clear();
  }

  // Not found.
  if (from[t] == inf) return;

  // Collects route cells backward.
  std::vector<int> routes;
  routes.push_back(t);
  int v = t;
  while (v != s) {
    std::cout << m.UnpackX(v) << "," << m.UnpackY(v) << std::endl;
    v = from[v];
    routes.push_back(v);
  }
  for (int i = routes.size() - 1; i >= 0; --i) {
    auto [x, y] = m.UnpackXY(routes[i]);
    collector(x, y);
  }
}

}  // namespace quadtree_astar
