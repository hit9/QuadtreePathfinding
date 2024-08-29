#include <iostream>
#include <utility>
#include <vector>

#include "qdpf.hpp"

const int N = 8;

enum Terrain {
  Land = 0b001,      // 1
  Water = 0b010,     // 2
  Building = 0b100,  // 4
};

int grid[N][N] = {
    // 8x8
    // clang-format off
    {1, 1, 1, 1, 1, 1, 1, 1},
    {4, 4, 4, 4, 4, 1, 1, 1},
    {1, 2, 2, 2, 4, 2, 1, 1},
    {1, 2, 2, 2, 4, 2, 1, 1},
    {1, 2, 2, 2, 2, 1, 1, 1},
    {1, 1, 1, 2, 1, 2, 2, 2},
    {1, 1, 1, 1, 1, 1, 1, 1},
    {1, 1, 1, 1, 1, 1, 1, 1},
    // clang-format on
};

int main(void) {
  int w = 8, h = 8;

  // Setup a QuadtreeMapX.
  qdpf::TerrainTypesChecker terrainChecker = [](int x, int y) { return grid[x][y]; };
  auto distance = qdpf::EuclideanDistance<10>;
  qdpf::QuadtreeMapXSettings settings{
      {10, Terrain::Land},                   // e.g. soldiers
      {20, Terrain::Land},                   // e.g. tanks
      {10, Terrain::Land | Terrain::Water},  // e.g. seals
      {20, Terrain::Water},                  // e.g. boats
  };
  qdpf::QuadtreeMapX mx(w, h, distance, terrainChecker, settings);
  mx.Build();

  // Setup an A* path finder.
  qdpf::AStarPathFinder pf(mx);

  // Change terrain.
  grid[6][6] = Terrain::Water;
  mx.Update(6, 6);
  mx.Compute();

  // Resets the path finder.
  // Find path from (0,0) to (7,7), agent size is 10, we can only walk on { Land }.
  if (-1 == pf.Reset(0, 0, 7, 7, 10, Terrain::Land)) {
    std::cout << "reset failed!" << std::endl;
    return -1;
  }

  std::cout << "node route path:" << std::endl;
  // ComputeNodeRoutes is much faster than ComputeGateRoutes to test whether the target is
  // reachable.
  if (pf.ComputeNodeRoutes() == -1) {
    std::cout << "unreachable!" << std::endl;
    return -1;
  }
  qdpf::NodeVisitor visitor1 = [](const qdpf::QdNode* node) {
    std::cout << node->x1 << "," << node->y1 << " " << node->x2 << "," << node->y2 << std::endl;
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
  int cost = pf.ComputeGateRoutes(collector, true);
  std::cout << "cost: " << cost << std::endl;

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
    qdpf::ComputeStraightLine(x, y, x2, y2, collector1);
    x = x2, y = y2;
  }

  // Print the path.
  for (auto [x, y] : path) std::cout << x << "," << y << std::endl;
  return 0;
}
