#include <iomanip>
#include <iostream>

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
    {1, 1, 1, 1, 1, 1, 1, 1},
    {1, 1, 1, 1, 1, 1, 1, 1},
    {4, 4, 4, 4, 4, 4, 1, 1},
    {1, 2, 2, 2, 1, 1, 1, 1},
    {1, 1, 1, 1, 1, 2, 2, 2},
    {1, 1, 1, 4, 4, 1, 1, 1},
    {1, 1, 1, 1, 1, 1, 1, 1},
    // clang-format on
};

const char DIRECTION_REPL[9][4]{
    "↖",  // 0
    "↑",  // 1
    "↗",  // 2
    "←",  // 3
    "☆",  // 4
    "→",  // 5
    "↙",  // 6
    "↓",  // 7
    "↘",  // 8
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

  // Setup a flow-field path finder.
  qdpf::FlowFieldPathFinder pf(mx);

  // Change terrain.
  grid[6][6] = Terrain::Water;
  mx.Update(6, 6);
  mx.Compute();

  // Resets the path finder.
  // Find path to (7,7), agent size is 10, we can only walk on { Land }.
  // Computes the flow fields for all cells inside rectangle dest.
  qdpf::Rectangle dest{0, 0, 7, 7};
  if (-1 == pf.Reset(7, 7, dest, 10, Terrain::Land)) {
    std::cout << "reset failed!" << std::endl;
    return -1;
  }

  // Computes the node flow field.
  std::cout << "1st step: ComputeNodeFlowField..." << std::endl;
  if (-1 == pf.ComputeNodeFlowField()) {
    std::cout << "ComputeNodeFlowField failed!" << std::endl;
    return -1;
  }

  // Don't need this in real usage, here we just print it out for having a look.
  qdpf::NodeFlowFieldVisitor visitor1 = [](const qdpf::QdNode *node, const qdpf::QdNode *nextNode,
                                           int cost) {
    std::cout << "node : " << node->x1 << "," << node->y1 << " " << node->x2 << "," << node->y2;
    std::cout << " node's next : " << nextNode->x1 << "," << nextNode->y1 << " " << nextNode->x2
              << "," << nextNode->y2;
    std::cout << " cost : " << cost << std::endl;
  };
  pf.VisitComputedNodeFlowField(visitor1);

  // Computes the gate flow field.
  std::cout << "2nd step: ComputeGateFlowField..." << std::endl;
  if (-1 == pf.ComputeGateFlowField(true)) {
    std::cout << "ComputeGateFlowField failed!" << std::endl;
    return -1;
  }

  // Don't need this in real usage, here we just print it out for having a look.
  qdpf::CellFlowFieldVisitor visitor2 = [](int x, int y, int xNext, int yNext, int cost) {
    std::cout << x << "," << y;
    std::cout << " next : " << xNext << "," << yNext;
    std::cout << " cost : " << cost << std::endl;
  };
  pf.VisitComputedGateFlowField(visitor2);

  // Computes the final flow field.
  std::cout << "3rd step: ComputeCellFlowFieldInDestRectangle..." << std::endl;
  if (-1 == pf.ComputeFinalFlowFieldInDestRectangle()) {
    std::cout << "ComputeCellFlowFieldInDestRectangle failed!" << std::endl;
    return -1;
  }

  qdpf::CellFlowFieldVisitor visitor3 = [](int x, int y, int xNext, int yNext, int cost) {
    std::cout << x << "," << y;
    std::cout << " next : " << xNext << "," << yNext;
    std::cout << " cost : " << cost << std::endl;
  };
  pf.VisitComputedCellFlowFieldInDestRectangle(visitor3);

  // Let's draw a flow field.
  // direction encoding: (dx+1)*3+(dy+1)
  int direction_fields[N][N];
  memset(direction_fields, 0xff, sizeof direction_fields);  // -1
  qdpf::CellFlowFieldVisitor visitor4 = [&direction_fields](int x, int y, int xNext, int yNext,
                                                            int cost) {
    int dx = xNext - x, dy = yNext - y;
    int d = (dx + 1) * 3 + (dy + 1);
    if (d >= 0 && d <= 8) direction_fields[x][y] = d;
  };
  pf.VisitComputedCellFlowFieldInDestRectangle(visitor4);

  for (int x = 0; x < N; ++x) {
    for (int y = 0; y < N; ++y) {
      int d = direction_fields[x][y];
      if (d >= 0)
        std::cout << std::setw(6) << DIRECTION_REPL[d];
      else
        std::cout << std::setw(4) << "-";
    }
    std::cout << std::endl;
  }

  return 0;
}
