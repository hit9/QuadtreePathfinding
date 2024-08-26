#include "true_clearance_field.hpp"

#include <algorithm>  // for std::max, std::min

namespace true_clearance_field {

static const int DEFAULT_DIRECTIONS[8][3] = {
    // {dx, dy, IsDiagonal}
    {0, -1, 0},   // 0, W, HV
    {-1, -1, 1},  // 1, NW, Diagonal
    {-1, 0, 0},   // 2, N, HV
    {-1, 1, 1},   // 3, NE, Diagonal
    {0, 1, 0},    // 4, E, HV
    {1, 1, 1},    // 5, ES, Diagonal
    {1, 0, 0},    // 6, S, HV
    {1, -1, 1},   // 7, SW, Diagonal
};

TrueClearanceField::TrueClearanceField(int w, int h, int u, int costUnit, int diagonalCostUnit,
                                       ObstacleChecker originalIsObstacle)
    : w(w),
      h(h),
      u(u),
      costUnit(costUnit),
      diagonalCostUnit(diagonalCostUnit),
      originalIsObstacle(originalIsObstacle) {
  // We work on a larger map (w+1)x(h+1).
  // Assume that the right and bottom of the map are surrounded by virtual walls
  g.resize(h + 1, std::vector<int>(w + 1, inf));
  rhs.resize(h + 1, std::vector<int>(w + 1, inf));
  // initial directions.
  for (int i = 0; i < 8; ++i) {
    directions[i][0] = DEFAULT_DIRECTIONS[i][0];
    directions[i][1] = DEFAULT_DIRECTIONS[i][1];
    directions[i][2] = DEFAULT_DIRECTIONS[i][2] ? diagonalCostUnit : costUnit;
  }
}

TrueClearanceField::K TrueClearanceField::k(int x, int y) const {
  return {std::min(g[x][y], rhs[x][y]), x, y};
}

// Returns true if given cell (x,y) is an obstacle on the larger map.
// If the cell locates within the vritual wall, it's considered an obstacle.
bool TrueClearanceField::isObstacle(int x, int y) const {
  if (x == h || y == w) return true;
  return originalIsObstacle(x, y);
}

void TrueClearanceField::Build() {
  // right wall of the larger map.
  for (int x = 0; x <= h; ++x) Update(x, w);
  // bottom wall of the larger map.
  // avoid visiting the right-bottom corner (h,w) twice
  for (int y = 0; y < w; ++y) Update(h, y);
  // initial the map.
  Compute();
}

int TrueClearanceField::Get(int x, int y) const { return g[x][y]; }

void TrueClearanceField::Update(int x, int y) {
  auto k0 = k(x, y);

  if (isObstacle(x, y)) {
    // for an obstacle, g is going to be updating to rhs=0.
    rhs[x][y] = 0;
  } else {
    // for a non-obstacle, g is going to be updated.
    rhs[x][y] = inf;
    // for predecessors (right-bottom neigbours)
    for (int i = 4; i <= 6; ++i) {
      const auto &d = directions[i];
      int dx = d[0], dy = d[1], cost = d[2];
      int x1 = x + dx, y1 = y + dy;
      if (x1 <= h && y1 <= w) {
        rhs[x][y] = std::min(rhs[x][y], g[x1][y1] + cost);
      }
    }
  }

  // requeue (x,y) if g and h is inconsistent
  q.erase(k0);
  if (g[x][y] != rhs[x][y]) q.insert({k(x, y), {x, y}});
}

int TrueClearanceField::Compute() {
  int n = 0;
  while (q.size()) {
    auto it = q.begin();
    auto [x, y] = it->second;
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
    if (updatedCellVisitor != nullptr && x < h && y < w) updatedCellVisitor(x, y);

    // if the (x,y) is in consistent and g value >= bound u.
    // there's no need to propagate it to its successors.
    // A non-inf value means the g value is up to date, it's an accurate value.
    if (g[x][y] != inf && g[x][y] == rhs[x][y] && g[x][y] >= u) continue;

    // update successors (left-top neigbours)
    for (int i = 0; i <= 2; ++i) {
      const auto &d = directions[i];
      int dx = d[0], dy = d[1];
      int x1 = x + dx, y1 = y + dy;
      if (x1 >= 0 && y1 >= 0) Update(x1, y1);
    }
  }
  return n;
}

}  // namespace true_clearance_field
