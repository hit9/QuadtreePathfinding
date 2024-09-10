// Dynamical clearance field (minimum obstacle distance) on 2D grid map.
// Source code: https://github.com/hit9/clearance-field
// Author: hit9[At]icloud.com, License: BSD

#include "clearance_field.hpp"

#include <cassert>

namespace clearance_field {

// 8 directions of (dx, dy, cost)
// 0,1,2 left-top.
// 4,5,6 right-bottom
//      1| 2(N)| 3
//     --+-----+--
//   0(W)|     | 4(E)
//     --+-----+--
//      7| 6(S)| 5
static const int DEFAULT_DIRECTIONS[8][3] = {
    // {dx, dy, IsDiagonal}
    {-1, 0, 0},   // 0, W, HV
    {-1, -1, 1},  // 1, NW, Diagonal
    {0, -1, 0},   // 2, N, HV
    {1, -1, 1},   // 3, NE, Diagonal
    {1, 0, 0},    // 4, E, HV
    {1, 1, 1},    // 5, ES, Diagonal
    {0, 1, 0},    // 6, S, HV
    {-1, 1, 1},   // 7, SW, Diagonal
};

////////////////////////////////////
/// LPAClearanceFieldAlgorithm
////////////////////////////////////

LPAClearanceFieldAlgorithm::LPAClearanceFieldAlgorithm(int w, int h, int u,
                                                       ObstacleChecker isObstacle,
                                                       PredecessorsVisitor predecessorsVisitor,
                                                       SuccessorsVisitor successorVisitor)
    : w(w),
      h(h),
      u(u),
      isObstacle(isObstacle),
      predecessorsVisitor(predecessorsVisitor),
      successorVisitor(successorVisitor) {
  g.resize(w, std::vector<int>(h, inf));
  rhs.resize(w, std::vector<int>(h, inf));
}

LPAClearanceFieldAlgorithm::K LPAClearanceFieldAlgorithm::k(int x, int y) const {
  return {std::min(g[x][y], rhs[x][y]), x, y};
}

int LPAClearanceFieldAlgorithm::Get(int x, int y) const { return g[x][y]; }

void LPAClearanceFieldAlgorithm::Update(int x, int y) {
  auto k0 = k(x, y);

  // update (x,y)'s rhs value by its predecessor (x1,y1).
  NeighbourCellVisitor update = [this, x, y](int x1, int y1, int cost) {
    if (x1 >= 0 && x1 < w && y1 >= 0 && y1 < h) {
      rhs[x][y] = std::min(rhs[x][y], g[x1][y1] + cost);
    }
  };

  if (isObstacle(x, y)) {
    // for an obstacle, g is going to be updating to rhs=0.
    rhs[x][y] = 0;
  } else {
    // for a non-obstacle, g is going to be updated.
    rhs[x][y] = inf;
    // for predecessors
    predecessorsVisitor(x, y, update);
  }

  // requeue (x,y) if g and rhs is inconsistent
  q.erase(k0);
  if (g[x][y] != rhs[x][y]) q.insert({k(x, y), {x, y}});
}

int LPAClearanceFieldAlgorithm::Compute() {
  int n = 0;

  // current handling cell.
  int x, y;

  // update (x,y)'s successor (x1,y1)
  CellVisitor update = [this, &x, &y](int x1, int y1) {
    if (x1 >= 0 && x1 < w && y1 >= 0 && y1 < h) {
      Update(x1, y1);
    }
  };

  while (q.size()) {
    auto it = q.begin();
    x = it->second.first;
    y = it->second.second;
    q.erase(it);
    ++n;

    if (g[x][y] > rhs[x][y]) {
      // local over-consistency
      g[x][y] = rhs[x][y];
    } else {
      // local under-consistency
      g[x][y] = inf;
      Update(x, y);
    }

    // A cell's g value can only be updated in the q's consuming progress.
    if (updatedCellVisitor != nullptr && x < w && y < h) updatedCellVisitor(x, y);

    // if the (x,y) is in consistent and g value >= bound u.
    // there's no need to propagate it to its successors.
    // A non-inf value means the g value is up to date, it's an accurate value.
    if (g[x][y] != inf && g[x][y] == rhs[x][y] && g[x][y] >= u) continue;

    // update all successors's value from (x,y)
    successorVisitor(x, y, update);
  }
  return n;
}

////////////////////////////////////
/// ClearanceFieldBase
////////////////////////////////////

ClearanceFieldBase::ClearanceFieldBase(int w, int h, int u, int costUnit, int diagonalCostUnit,
                                       ObstacleChecker isObstacle)
    : w(w),
      h(h),
      u(u),
      costUnit(costUnit),
      diagonalCostUnit(diagonalCostUnit),
      originalIsObstacle(isObstacle) {
  // initial directions.
  for (int i = 0; i < 8; ++i) {
    directions[i][0] = DEFAULT_DIRECTIONS[i][0];
    directions[i][1] = DEFAULT_DIRECTIONS[i][1];
    directions[i][2] = DEFAULT_DIRECTIONS[i][2] ? diagonalCostUnit : costUnit;
  }
}

// Proxy the algorithm's functions.
void ClearanceFieldBase::SetUpdatedCellVisistor(CellVisitor f) { lpa->SetUpdatedCellVisistor(f); }
int ClearanceFieldBase::Compute() { return lpa->Compute(); }

// makePredecessorsVisitor returns a PredecessorsVisitor function.
// Parameter directionStart and directionEnd are the index of the directions array,
// they indicates the predecessor directions's start and end.
PredecessorsVisitor ClearanceFieldBase::makePredecessorsVisitor(int directionStart,
                                                                int directionEnd) {
  return [this, directionStart, directionEnd](int x, int y, NeighbourCellVisitor& visitor) {
    // for predecessors (right-bottom neigbours)
    for (int i = directionStart; i <= directionEnd; ++i) {
      const auto& d = directions[i];
      int dx = d[0], dy = d[1], cost = d[2];
      int x1 = x + dx, y1 = y + dy;
      visitor(x1, y1, cost);
    }
  };
}

// makeSuccessorsVisitor returns a SuccessorsVisitor function.
// Parameter directionStart and directionEnd are the index of the directions array,
// they indicates the successor directions's start and end.
SuccessorsVisitor ClearanceFieldBase::makeSuccessorsVisitor(int directionStart, int directionEnd) {
  return [this, directionStart, directionEnd](int x, int y, CellVisitor& visitor) {
    for (int i = directionStart; i <= directionEnd; ++i) {
      const auto& d = directions[i];
      int dx = d[0], dy = d[1];
      int x1 = x + dx, y1 = y + dy;
      visitor(x1, y1);
    }
  };
}

////////////////////////////////////
/// TrueClearanceField
////////////////////////////////////

TrueClearanceField::TrueClearanceField(int w, int h, int u, int costUnit, int diagonalCostUnit,
                                       ObstacleChecker isObstacle)
    : ClearanceFieldBase(w, h, u, costUnit, diagonalCostUnit, isObstacle) {
  // We use a larger map for LPAClearanceFieldAlgorithm.
  // Assuming the grid map is surrounded by walls at the right and bottom directions.
  // In the diagram below, 'X' composes the virtual walls (obstacle)
  //          w+1
  //       +-----------X
  //       |-----------X
  //   h+1 |-----------X
  //       |-----------X
  //       XXXXXXXXXXXXX
  ObstacleChecker obstacleChecker = [this, w, h](int x, int y) {
    if (x == w || y == h) return true;
    return originalIsObstacle(x, y);
  };

  // predecessorsVisitor is to visit predecessors on the right-bottom directions.
  //
  // In the diagram below, a, b, c are predecessors of current cell (x,y), their clearance values
  // affect (x,y)'s value.
  //
  //  (x,y) ----+ a
  //    |       |
  //    +-------+ c
  //    b
  PredecessorsVisitor predecessorsVisitor = makePredecessorsVisitor(4, 6);

  // successorVisitor is to visitor successors on the left-top directions.
  //
  // In the diagram below, a, b, c are successors of current cell (x,y), their clearance values
  // are affected by (x,y)'s value.
  //
  //   a
  //   +-------+ b
  //   |       |
  //   +---- (x,y)
  //   c
  SuccessorsVisitor successorVisitor = makeSuccessorsVisitor(0, 2);

  lpa = new LPAClearanceFieldAlgorithm(w + 1, h + 1, u, obstacleChecker, predecessorsVisitor,
                                       successorVisitor);
}

TrueClearanceField::~TrueClearanceField() {
  delete lpa;
  lpa = nullptr;
}

void TrueClearanceField::Build() {
  assert(lpa != nullptr);
  // right wall of the larger map.
  for (int y = 0; y <= h; ++y) lpa->Update(w, y);
  // bottom wall of the larger map.
  // avoid visiting the right-bottom corner (w,h) twice
  for (int x = 0; x < w; ++x) lpa->Update(x, h);
  // initial the map.
  lpa->Compute();
}

int TrueClearanceField::Get(int x, int y) const { return lpa->Get(x, y); }
void TrueClearanceField::Update(int x, int y) { lpa->Update(x, y); }

////////////////////////////////////
/// BrushfireClearanceField
////////////////////////////////////

BrushfireClearanceField::BrushfireClearanceField(int w, int h, int u, int costUnit,
                                                 int diagonalCostUnit, ObstacleChecker isObstacle)
    : ClearanceFieldBase(w, h, u, costUnit, diagonalCostUnit, isObstacle) {
  // We use a larger map for LPAClearanceFieldAlgorithm.
  // Assuming the grid map is surrounded by walls at 4 directions.
  // In the diagram below, 'X' composes the virtual walls (obstacle)
  //           w+2
  //       XXXXXXXXXXXXX
  //       X-----------X
  //   h+2 X-----------X
  //       X-----------X
  //       XXXXXXXXXXXXX
  ObstacleChecker obstacleChecker = [this, w, h](int x, int y) {
    if (x == 0 || y == 0) return true;
    if (x == w + 1 || y == h + 1) return true;
    return originalIsObstacle(x - 1, y - 1);
  };

  // Function to visit predecessors on 8 directions.
  PredecessorsVisitor predecessorsVisitor = makePredecessorsVisitor(0, 7);

  // Function to visit successors on 8 directions.
  SuccessorsVisitor successorVisitor = makeSuccessorsVisitor(0, 7);

  lpa = new LPAClearanceFieldAlgorithm(w + 2, h + 2, u, obstacleChecker, predecessorsVisitor,
                                       successorVisitor);
}

BrushfireClearanceField::~BrushfireClearanceField() {
  delete lpa;
  lpa = nullptr;
}

void BrushfireClearanceField::Build() {
  assert(lpa != nullptr);
  // left wall of the larger map.
  for (int y = 0; y <= h + 1; ++y) lpa->Update(0, y);

  // top wall of the larger map.
  // avoid visiting the left-top corner (0,0) twice.
  for (int x = 1; x <= w + 1; ++x) lpa->Update(x, 0);

  // right wall of the larger map.
  // avoid visiting the right-top corner (0, w+1) twice
  for (int y = 1; y <= h + 1; ++y) lpa->Update(w + 1, y);

  // bottom wall of the larger map.
  // avoid visiting the left-bottom (0,h+1) and right-bottom (w+1,h+1) corners twice.
  for (int x = 1; x < w + 1; ++x) lpa->Update(x, h + 1);

  // initial the map.
  lpa->Compute();
}

int BrushfireClearanceField::Get(int x, int y) const { return lpa->Get(x + 1, y + 1); }
void BrushfireClearanceField::Update(int x, int y) { lpa->Update(x + 1, y + 1); }

}  // namespace clearance_field
