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
    // ----------------------> x
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
  // (Note that the y is the row, x is the column)
  qdpf::TerrainTypesChecker terrainChecker = [](int x, int y) { return grid[y][x]; };
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
  // Computes the flow fields for all cells inside the query range.
  qdpf::Rectangle qrange{0, 0, 7, 7};
  if (-1 == pf.Reset(7, 7, qrange, 10, Terrain::Land)) {
    std::cout << "reset failed!" << std::endl;
    return -1;
  }

  // Computes the node flow field.
  std::cout << "1st step: ComputeNodeFlowField..." << std::endl;

  qdpf::NodeFlowField nodeFlowField;
  if (-1 == pf.ComputeNodeFlowField(nodeFlowField)) {
    std::cout << "ComputeNodeFlowField failed!" << std::endl;
    return -1;
  }

  // Don't need this in real usage, here we just print it out for having a look.
  for (auto& [node, p] : nodeFlowField.GetUnderlyingMap()) {
    auto [nextNode, cost] = p;
    std::cout << "node : " << node->x1 << "," << node->y1 << " " << node->x2 << "," << node->y2;
    std::cout << " node's next : " << nextNode->x1 << "," << nextNode->y1 << " " << nextNode->x2
              << "," << nextNode->y2;
    std::cout << " cost : " << cost << std::endl;
  }

  // Computes the gate flow field.
  std::cout << "2nd step: ComputeGateFlowField..." << std::endl;
  qdpf::GateFlowField gateFlowField;
  if (-1 == pf.ComputeGateFlowField(gateFlowField, nodeFlowField)) {
    std::cout << "ComputeGateFlowField failed!" << std::endl;
    return -1;
  }

  // Don't need this in real usage, here we just print it out for having a look.
  for (auto& [gate, p] : gateFlowField.GetUnderlyingMap()) {
    auto [nextGate, cost] = p;
    auto [x, y] = gate;
    auto [xNext, yNext] = nextGate;
    std::cout << x << "," << y;
    std::cout << " next : " << xNext << "," << yNext;
    std::cout << " cost : " << cost << std::endl;
  }

  // Computes the final flow field.
  std::cout << "3rd step: ComputeCellFlowFieldInDestRectangle..." << std::endl;
  qdpf::FinalFlowField finalFlowfield;
  if (-1 == pf.ComputeFinalFlowField(finalFlowfield, gateFlowField)) {
    std::cout << "ComputeCellFlowFieldInDestRectangle failed!" << std::endl;
    return -1;
  }

  // Don't need this in real usage, here we just print it out for having a look.
  for (auto& [cell, p] : finalFlowfield.GetUnderlyingMap()) {
    auto [nextCell, cost] = p;
    auto [x, y] = cell;
    auto [xNext, yNext] = nextCell;
    std::cout << x << "," << y;
    std::cout << " next : " << xNext << "," << yNext;
    std::cout << " cost : " << cost << std::endl;
  }

  // Let's draw a flow field.
  // direction encoding: (dy+1)*3+(dx+1)
  int direction_fields[N][N];
  memset(direction_fields, 0xff, sizeof direction_fields);  // -1

  for (auto& [cell, p] : finalFlowfield.GetUnderlyingMap()) {
    auto [nextCell, cost] = p;
    auto [x, y] = cell;
    auto [xNext, yNext] = nextCell;
    int dx = xNext - x, dy = yNext - y;
    int d = (dy + 1) * 3 + (dx + 1);
    if (dx >= -1 && dx <= 1 && dy >= -1 && dy <= 1) direction_fields[x][y] = d;
  };

  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
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
