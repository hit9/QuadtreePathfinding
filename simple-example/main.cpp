// how to build: make

#include <iostream>
#include <utility>
#include <vector>

#include "qdpf.hpp"

const int N = 8;

int grid[N][N] = {
    // 8x8
    // clang-format off
    {0, 0, 0, 0, 0, 0, 0, 0},
    {1, 1, 1, 1, 1, 0, 0, 0},
    {0, 0, 0, 0, 1, 0, 0, 0},
    {0, 0, 0, 0, 1, 0, 0, 0},
    {0, 0, 0, 0, 1, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0},
    // clang-format on
};

int main(void) {
  int w = 8, h = 8;
  // Setup the map
  auto isObstacle = [](int x, int y) { return grid[x][y]; };
  auto distance = qdpf::EuclideanDistance<10>;
  qdpf::QuadtreeMap m(w, h, isObstacle, distance);

  // Setup an A* path finder.
  qdpf::AStarPathFinder pf(w * h);

  // Bind them and build the tree.
  m.Build();

  // Add an obstacle
  grid[1][5] = 1;
  m.Update(1, 5);

  // Resets the path finder.
  pf.Reset(&m, 0, 0, 7, 7);

  std::cout << "node route path:" << std::endl;
  // ComputeNodeRoutes is much faster than ComputeGateRoutes to test whether the target is
  // reachable.
  if (pf.ComputeNodeRoutes() == -1) {
    std::cout << "unreachable!" << std::endl;
    return -1;
  }
  qdpf::NodeVisitor visitor1 = [](int x1, int y1, int x2, int y2) {
    std::cout << x1 << "," << y1 << " " << x2 << "," << y2 << std::endl;
  };
  pf.VisitComputedNodeRoutes(visitor1);

  // Compute gate cell routes.
  std::cout << "collect route gate cell path..." << std::endl;
  std::vector<std::pair<int, int>> routes;
  qdpf::CellCollector collector = [&routes](int x, int y) { routes.push_back({x, y}); };
  // The second boolean argument specifics whether to use only the gate cells on the computed node
  // path. This will make the ComputeGateRoutes runs much faster, but less optimal.
  // We can just use ComputeGateRoutes(collector, false) directly without calling ComputeGateRoutes
  // in advance if we don't use the node path.
  pf.ComputeGateRoutes(collector, true);
  for (auto [x, y] : routes) std::cout << x << "," << y << std::endl;

  std::cout << "collect detailed path..." << std::endl;
  // Fill the detailed path (straight lines).
  std::vector<std::pair<int, int>> path;
  qdpf::CellCollector collector1 = [&path](int x, int y) {
    if (path.size()) {
      // skip duplicates end of previous route.
      auto [x2, y2] = path.back();
      if ((x2 == x && y2 == y)) return;
    }
    path.push_back({x, y});
  };
  auto [x, y] = routes[0];
  for (int i = 1; i < routes.size(); i++) {
    auto [x2, y2] = routes[i];
    pf.ComputePathToNextRouteCell(x, y, x2, y2, collector1);
    x = x2, y = y2;
  }

  // Print the path.
  for (auto [x, y] : path) std::cout << x << "," << y << std::endl;
  return 0;
}
