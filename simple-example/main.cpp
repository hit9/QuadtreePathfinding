// how to build:
// clang++ main.cpp ../quadtree_astar.cpp  -I ../3rd-party/
// ./a.out

#include <iostream>
#include <utility>
#include <vector>

#include "quadtree_astar.hpp"

const int N = 8;

const int grid[N][N] = {
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
  auto distance = quadtree_astar::EuclideanDistance<10>;
  quadtree_astar::QuadtreeMap m(w, h, isObstacle, distance);

  // Setup an A* path finder.
  quadtree_astar::AStarPathFinder pf(m);

  // Bind them and build the tree.
  m.RegisterGraph(pf.GetGraph());
  m.Build();

  // Compute routes
  std::vector<std::pair<int, int>> routes;
  quadtree_astar::CellCollector collector = [&routes](int x, int y) { routes.push_back({x, y}); };
  int x1 = 0, y1 = 0, x2 = 7, y2 = 7;
  pf.ComputeRoutes(x1, y1, x2, y2, collector);

  // Fill the detailed path (straight lines).
  std::vector<std::pair<int, int>> path;
  quadtree_astar::CellCollector collector1 = [&path](int x, int y) {
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
    pf.ComputePathToNextRoute(x, y, x2, y2, collector1);
    x = x2, y = y2;
  }

  // Print the path.
  for (auto [x, y] : path) std::cout << x << "," << y << std::endl;
  return 0;
}
