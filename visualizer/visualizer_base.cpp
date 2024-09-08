#include <spdlog/spdlog.h>

#include <chrono>

#include "qdpf.hpp"
#include "visualizer.hpp"

using qdpf::internal::IsInsideRectangle;

// ~~~~~~~~~~ Agent ~~~~~~~~~~

void Agent::Reset() {
  size = COST_UNIT;
  capability = Terrain::Land;
}

// ~~~~~~~~~~ Map ~~~~~~~~~~

Map::Map(int w, int h, int gridSize, int step) : w(w), h(h), gridSize(gridSize), step(step) {
  // Inits the map, by default:
  // 1. the center are water.
  // 2. there's 2 walls.
  // 3. other cells are land.
  memset(grids, 0, sizeof grids);
  memset(changes, 0, sizeof changes);

  qdpf::Rectangle center{2 * h / 7, 2 * w / 7, h * 5 / 7, w * 5 / 7};
  qdpf::Rectangle wall1{center.x2 + 4, center.y1, center.x2 + 4, center.y2 + 4};
  qdpf::Rectangle wall2{center.x1, center.y2 + 4, center.x2 + 4, center.y2 + 4};
  qdpf::Rectangle wall3{center.x1 - 4, center.y1 - 4, center.x1 - 4, center.y2};
  qdpf::Rectangle wall4{center.x1 - 4, center.y1 - 4, center.x2, center.y1 - 4};

  for (int x = 0; x < h; ++x) {
    for (int y = 0; y < w; ++y) {
      if (IsInsideRectangle(x, y, center))
        grids[x][y] = Terrain::Water;
      else if (IsInsideRectangle(x, y, wall1))
        grids[x][y] = Terrain::Building;
      else if (IsInsideRectangle(x, y, wall2))
        grids[x][y] = Terrain::Building;
      else if (IsInsideRectangle(x, y, wall3))
        grids[x][y] = Terrain::Building;
      else if (IsInsideRectangle(x, y, wall4))
        grids[x][y] = Terrain::Building;
      else
        grids[x][y] = Terrain::Land;
    }
  }
}

Map::~Map() {
  delete qmx;
  qmx = nullptr;
  delete naiveMap;
  ;
}

void Map::Build() {
  // Build QuadtreeMapX.
  auto stepf = (step == -1) ? [](int z) -> int { return z / 8 + 1; } : nullptr;
  qdpf::QuadtreeMapXSettings settings{
      {COST_UNIT, Terrain::Land},
      {2 * COST_UNIT, Terrain::Land},
      {3 * COST_UNIT, Terrain::Land},
      {COST_UNIT, Terrain::Water},
      {2 * COST_UNIT, Terrain::Water},
      {3 * COST_UNIT, Terrain::Water},
      {COST_UNIT, Terrain::Land | Terrain::Water},
      {2 * COST_UNIT, Terrain::Land | Terrain::Water},
      {3 * COST_UNIT, Terrain::Land | Terrain::Water},
  };
  auto distance = qdpf::EuclideanDistance<COST_UNIT>;
  auto terrianChecker = [this](int x, int y) { return grids[x][y]; };
  qmx = new qdpf::QuadtreeMapX(w, h, distance, terrianChecker, settings, step, stepf);
  qmx->Build();
  spdlog::info("Build quadtree maps done");

  // Build naive map.
  auto isObstacle = [this](int x, int y) { return grids[x][y] != Terrain::Land; };
  naiveMap = new qdpf::naive::NaiveGridMap(w, h, isObstacle, distance);
  naiveMap->Build();
  spdlog::info("Build naive map done");
}

void Map::WantChangeTerrain(const Cell& cell, Terrain to) {
  auto [x, y] = cell;
  changes[x][y] = to;
}

void Map::ApplyChangeTerrain(const std::vector<Cell>& cells) {
  for (auto [x, y] : cells) {
    grids[x][y] = changes[x][y];
    changes[x][y] = 0;
    qmx->Update(x, y);
    naiveMap->Update(x, y);
  }
  qmx->Compute();
}

void Map::ClearAllTerrains() {
  for (int x = 0; x < h; ++x) {
    for (int y = 0; y < w; ++y) {
      grids[x][y] = Terrain::Land;
      qmx->Update(x, y);
      naiveMap->Update(x, y);
    }
  }
  qmx->Compute();
}

// ~~~~~~~~~~ AStarContext ~~~~~~~~~~

void NaiveAStarContext::Reset() {
  timeCost = std::chrono::microseconds(0);
  path.clear();
}

void AStarContext::InitPf(qdpf::QuadtreeMapX* qmx) { pf = new qdpf::AStarPathFinder(*qmx); }

AStarContext::~AStarContext() {
  delete pf;
  pf = nullptr;
}

void AStarContext::ClearResults() { nodePath.clear(), gatePath.clear(), finalPath.clear(); }

void AStarContext::Reset() {
  ClearResults();
  x1 = y1 = x2 = y2 = 0;
  isPfReset = false;
  timeCost = std::chrono::microseconds(0);
}

int AStarContext::ResetPf(int agentSize, int capabilities) {
  if (isPfReset) return 0;
  auto startAt = std::chrono::high_resolution_clock::now();
  auto ret = pf->Reset(x1, y1, x2, y2, agentSize, capabilities);
  isPfReset = true;
  auto endAt = std::chrono::high_resolution_clock::now();
  timeCost += std::chrono::duration_cast<std::chrono::microseconds>(endAt - startAt);
  return ret;
}

// ~~~~~~~~~~ FlowFieldContext ~~~~~~~~~~

void NaiveFlowFieldContext::Reset() {
  timeCost = std::chrono::microseconds(0);
  finalFlowField.Clear();
  testPaths.clear();
}

FlowFieldContext::~FlowFieldContext() {
  delete pf;
  pf = nullptr;
}

void FlowFieldContext::InitPf(qdpf::QuadtreeMapX* qmx) {
  pf = new qdpf::FlowFieldPathFinder(*qmx);
}

int FlowFieldContext::ResetPf(int agentSize, int capabilities) {
  if (isPfReset) return 0;
  auto startAt = std::chrono::high_resolution_clock::now();
  auto ret = pf->Reset(x2, y2, qrange, agentSize, capabilities);
  isPfReset = true;
  auto endAt = std::chrono::high_resolution_clock::now();
  timeCost += std::chrono::duration_cast<std::chrono::microseconds>(endAt - startAt);
  return ret;
}

void FlowFieldContext::ClearResults() {
  nodeFlowField.Clear(), gateFlowField.Clear(), finalFlowField.Clear();
  testPaths.clear();
}

void FlowFieldContext::Reset() {
  ClearResults();
  x2 = y2 = 0;
  qrange = {0, 0, 0, 0};
  isPfReset = false;
  timeCost = std::chrono::microseconds(0);
}

// ~~~~~~~~~~ Camera ~~~~~~~~~~

Camera::Camera(int w, int h, int mpw, int mph) : w(w), h(h), mpw(mpw), mph(mph) {}

void Camera::MoveUp(int k) { dy -= k; }
void Camera::MoveDown(int k) { dy += k; }
void Camera::MoveLeft(int k) { dx -= k; }
void Camera::MoveRight(int k) { dx += k; }
void Camera::MoveToLeftMost() { x = 0; }
void Camera::MoveToRightMost() { x = mpw - w - 1; }

void Camera::Update() {
  if (dx == 0 && dy == 0) return;
  x += dx;
  y += dy;
  // bounds check.
  x = std::max(0, x);
  y = std::max(0, y);
  x = std::min(x, mpw - w - 1);
  y = std::min(y, mph - h - 1);
  // clears the delta changes.
  dx = dy = 0;
}
