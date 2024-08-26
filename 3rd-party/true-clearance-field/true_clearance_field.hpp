// A simple dynamical minimum obstacle distance (to the right and bottom) field library on
// equal-weighted 2D grid map.
// Source code: https://github.com/hit9/true-clearance-field
// Author: hit9[At]icloud.com, License: BSD
// Version: 0.1.2
//
// Notes:
// 1. The true-clearance-distance concept can be found at:
//    https://web.archive.org/web/20190411040123/http://aigamedev.com/open/article/clearance-based-pathfinding/
// 2. The incremental updating mechanism is inspired by LPAStar algorithm.
//    ref: https://en.wikipedia.org/wiki/Lifelong_Planning_A*

#ifndef TRUE_CLEARANCE_FIELD_HPP
#define TRUE_CLEARANCE_FIELD_HPP

#include <functional>  // for std::function
#include <map>
#include <tuple>
#include <utility>  // for std::pair

namespace true_clearance_field {

static const int inf = 0x3f3f3f3f;

// ObstacleChecker is the type of the function that returns true if the given
// cell (x,y) is an obstacle.
using ObstacleChecker = std::function<bool(int x, int y)>;

// UpdatedCellVisistor is a function that visits the cell of which the value is updated by function
// Compute.
using UpdatedCellVisistor = std::function<void(int x, int y)>;

class TrueClearanceField {
 public:
  // Paramaters:
  // * w and h are the width and height of the grid map
  // * u is the upper bound of obstacle distance to maintain, usually setting this to (or larger
  //   than) the size of moving unit.
  // * costUnit is the unit cost moving from one cell to neighbour cells on horizontal and vertical
  //   directions. diagonalCostUnit is the unit cost on diagonal directions, generally it can be
  //   set to 1.414 times of costUnit. We stick to use integers instead of floating point numbers
  //   for calculations, for better performance, accuracy and convenience. So it's recommended to
  //   set costUnit to a value larger than 10.
  // * isObstacle(x,y) returns true if the cell (x,y) is an obstacle.
  TrueClearanceField(int w, int h, int u, int costUnit, int diagonalCostUnit,
                     ObstacleChecker isObstacle);
  // Sets a function to listen the cells of which the value is updated by Compute.
  void SetUpdatedCellVisistor(UpdatedCellVisistor f) { updatedCellVisitor = f; }
  // Build should be called on an **empty** map before any further feature is used.
  void Build();
  // Returns the pre-calculated minimum distance from cell (x,y) to the nearest obstacle locating
  // in the right-bottom directions.
  // Returns a number > u or just inf if the distance is larger than u.
  // That is, if you provided a upper bound u, then if the value is inf, which means the minimum
  // distance will be larger than u, but the accurate value is unknown and not maintained. But if
  // the value is not inf, which means the value is the extact minimum distance.
  // We won't check whether the (x,y) is out of boundy.
  int Get(int x, int y) const;
  // Update should be called after an obstacle is added or removed at cell (x,y).
  // The nearby cells on the left-top quadrant within a distance of u will be updated.
  // We won't check whether the (x,y) is out of boundy.
  void Update(int x, int y);
  // Compute should be called after any changes.
  // Returns the number of cells updated.
  int Compute();

 private:
  const int w, h;
  const int u, costUnit, diagonalCostUnit;
  ObstacleChecker originalIsObstacle;
  UpdatedCellVisistor updatedCellVisitor = nullptr;

  // 8 directions of (dx, dy, cost)
  // 0,1,2 left-top.
  // 4,5,6 right-bottom
  //      1| 2(N)| 3
  //     --+-----+--
  //   0(W)|     | 4(E)
  //     --+-----+--
  //      7| 6(S)| 5
  int directions[8][3];

  using K = std::tuple<int, int, int>;  // { cost, x, y }

  // g[x][y] and rhs[x][y] are the old and new value for cell (x,y)
  // local over-consistency: g > rhs
  // local under-consistency: g < rhs
  // local consistency: g == rhs
  std::vector<std::vector<int>> g, rhs;

  // use ordered map instead std::priority_queue
  // because we need to support "update", "erase" operations.
  // K => {x, y}
  std::map<K, std::pair<int, int>> q;

  // ~~~~~~~~~~~ Internals ~~~~~~~~~~~~~~~
  K k(int x, int y) const;
  bool isObstacle(int x, int y) const;
};

}  // namespace true_clearance_field

#endif
