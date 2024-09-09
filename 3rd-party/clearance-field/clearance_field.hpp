// Incremental clearance field (minimum obstacle distance) on 2D grid map.
// Source code: https://github.com/hit9/clearance-field
// Author: hit9[At]icloud.com, License: BSD

// Version: 0.2.0

// Contents
// ~~~~~~~~
// 1. IClearanceField: the interface of all clearance field implementations.
// 2. LPAClearanceFieldAlgorithm: the incremental updating algorithm.
// 3. TrueClearanceField implementer.
// 4. BrushfireClearanceField implementer.
// 5. Ref:
// https://web.archive.org/web/20190411040123/http://aigamedev.com/open/article/clearance-based-pathfinding/

#ifndef CLEARANCE_FIELD_HPP
#define CLEARANCE_FIELD_HPP

#include <functional>  // for std::function
#include <map>
#include <tuple>
#include <utility>  // for std::pair
#include <vector>

namespace clearance_field {

static const int inf = 0x3f3f3f3f;

// CellVisitor is a function to visit a cell.
using CellVisitor = std::function<void(int x, int y)>;

// NeighbourCellVisitor is a function to visit a neighbour cell, the third parameter is the cost
// from current cell to (x,y).
using NeighbourCellVisitor = std::function<void(int x, int y, int cost)>;

// ObstacleChecker is the type of the function that returns true if the given
// cell (x,y) is an obstacle.
using ObstacleChecker = std::function<bool(int x, int y)>;

////////////////////////////////////
/// IClearanceField
////////////////////////////////////

// Abstract interface class of clearance fields.
// Any implementers should guarantee that the clearance values are computed incrementally.
class IClearanceField {
 public:
  virtual ~IClearanceField() {}
  // SetUpdatedCellVisistor sets a function to listen cells of which the clearance value
  // are changed by a Compute() call.
  virtual void SetUpdatedCellVisistor(CellVisitor f) = 0;

  // Build the clearance field on an **empty** 2D grid map.
  // This method should be called before any further feature is used.
  virtual void Build() = 0;

  // Returns the clearance value at cell (x,y);
  // The implementation should guarantee the returned value is not smaller than the actual value.
  // That is the specific implementers may returns unreal distance values, but the values will not
  // smaller than the real value.
  virtual int Get(int x, int y) const = 0;

  // Update should be called after an obstacle is added or removed at cell (x,y).
  virtual void Update(int x, int y) = 0;

  // Compute should be called after any changes.
  // Returns the number of cells updated.
  virtual int Compute() = 0;
};

////////////////////////////////////
/// LPAClearanceFieldAlgorithm
////////////////////////////////////

// PredecessorsVisitor is a function to visit cell (x,y)'s predecessors.
// The (x1,y1) is a predecessor of current cell (x,y),
// the visitor is the function to visit (x1,y1).
using PredecessorsVisitor = std::function<void(int x1, int y1, NeighbourCellVisitor& visitor)>;

// SuccessorsVisitor is a function to visit cell (x,y)'s successors.
// The (x1,y1) is a successor of current cell (x,y),
// the visitor is the function to visit (x1,y1).
using SuccessorsVisitor = std::function<void(int x1, int y1, CellVisitor& visitor)>;

// LPAClearanceFieldAlgorithm is an algorithm inspired by LPA* for
// incremental clearance value updating.
// Ref: https://en.wikipedia.org/wiki/Lifelong_Planning_A*
class LPAClearanceFieldAlgorithm {
 public:
  LPAClearanceFieldAlgorithm(int w, int h, int u, ObstacleChecker isObstacle,
                             PredecessorsVisitor predecessorsVisitor,
                             SuccessorsVisitor successorVisitor);
  // Sets a callback function to visit the updated cells.
  void SetUpdatedCellVisistor(CellVisitor f) { updatedCellVisitor = f; }

  // Returns the clearance value for cell (x,y).
  int Get(int x, int y) const;

  // Update should be called after cell (x,y)'s obstacle property is changed.
  void Update(int x, int y);

  // Compute should be called after several Update() calls.
  int Compute();

 private:
  const int w, h, u;
  ObstacleChecker isObstacle;
  PredecessorsVisitor predecessorsVisitor;
  SuccessorsVisitor successorVisitor;

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

  CellVisitor updatedCellVisitor = nullptr;

  // ~~~~~~~~~~~ Internals ~~~~~~~~~~~~~~~
  K k(int x, int y) const;
};

////////////////////////////////////
/// ClearanceFieldBase
////////////////////////////////////

class ClearanceFieldBase : public IClearanceField {
 public:
  ClearanceFieldBase(int w, int h, int u, int costUnit, int diagonalCostUnit,
                     ObstacleChecker isObstacle);

  // Sets a function to listen the cells of which the value is updated by Compute.
  void SetUpdatedCellVisistor(CellVisitor f) override;

  // Compute should be called after any changes.
  // Returns the number of cells updated.
  int Compute() override;

 protected:
  const int w, h;
  const int u, costUnit, diagonalCostUnit;
  ObstacleChecker originalIsObstacle;
  // 8 directions of (dx, dy, cost)
  int directions[8][3];

  // this pointer should be maintained by the implementer class.
  LPAClearanceFieldAlgorithm* lpa;

  // ~~~~~~~ util functions ~~~~~~
  PredecessorsVisitor makePredecessorsVisitor(int directionStart, int directionEnd);
  SuccessorsVisitor makeSuccessorsVisitor(int directionStart, int directionEnd);
};

////////////////////////////////////
/// TrueClearanceField
////////////////////////////////////

// TrueClearanceField maintains the minimum distance to the obstacles on the right-bottom
// directions incrementally.
// Ref:
// https://web.archive.org/web/20190411040123/http://aigamedev.com/open/article/clearance-based-pathfinding/
class TrueClearanceField : public ClearanceFieldBase {
 public:
  // * w and h are the width and height of the grid map
  // * u is the upper bound of obstacle distance to maintain, usually setting this to (or larger
  //   than) the size of moving agents, the position should be the left-top corner of the agent.
  // * costUnit is the unit cost moving from one cell to neighbour cells on horizontal and vertical
  //   directions. diagonalCostUnit is the unit cost on diagonal directions, generally it can be
  //   set to 1.414 times of costUnit. We stick to use integers instead of floating point numbers
  //   for calculations, for better performance, accuracy and convenience. So it's recommended to
  //   set costUnit to a value larger than 10.
  // * isObstacle(x,y) returns true if the cell (x,y) is an obstacle.
  // * a is the pointer to the algorithm handler, should be maintained by the implementer class.
  TrueClearanceField(int w, int h, int u, int costUnit, int diagonalCostUnit,
                     ObstacleChecker isObstacle);
  ~TrueClearanceField();

  // Build should be called on an **empty** map before any further feature is used.
  void Build() override;

  // Returns the pre-calculated minimum distance from cell (x,y) to the nearest obstacle at the
  // right-bottom directions.
  // Returns a number > u or just inf if the distance is larger than u.
  // That is, if you provided a upper bound u, then if the value is inf, which means the minimum
  // distance will be larger than u, but the accurate value is unknown and not maintained. But if
  // the value is not inf, which means the value is the extact minimum distance.
  // We won't check whether the (x,y) is out of boundry.
  int Get(int x, int y) const override;

  // Update should be called after an obstacle is added or removed at cell (x,y).
  // The nearby cells in the left-top quadrant within a distance of u will be updated.
  // We won't check whether the (x,y) is out of boundry.
  void Update(int x, int y) override;
};

////////////////////////////////////
/// BrushfireClearanceField
////////////////////////////////////

// BrushfireClearanceField maintains the minimum distance to the obstacles around incrementally.
class BrushfireClearanceField : public ClearanceFieldBase {
 public:
  // * w and h are the width and height of the grid map
  // * u is the upper bound of obstacle distance to maintain, usually setting this to (or larger
  //   than) the radius of moving agents, the moving position should be the center of the agent.
  // * costUnit is the unit cost moving from one cell to neighbour cells on horizontal and vertical
  //   directions. diagonalCostUnit is the unit cost on diagonal directions, generally it can be
  //   set to 1.414 times of costUnit. We stick to use integers instead of floating point numbers
  //   for calculations, for better performance, accuracy and convenience. So it's recommended to
  //   set costUnit to a value larger than 10.
  // * isObstacle(x,y) returns true if the cell (x,y) is an obstacle.
  // * a is the pointer to the algorithm handler, should be maintained by the implementer class.
  BrushfireClearanceField(int w, int h, int u, int costUnit, int diagonalCostUnit,
                          ObstacleChecker isObstacle);
  ~BrushfireClearanceField();

  // Build should be called on an **empty** map before any further feature is used.
  void Build() override;

  // Returns the pre-calculated minimum distance from cell (x,y) to the nearest obstacle around.
  // Returns a number > u or just inf if the distance is larger than u.
  // That is, if you provided a upper bound u, then if the value is inf, which means the minimum
  // distance will be larger than u, but the accurate value is unknown and not maintained. But if
  // the value is not inf, which means the value is the extact minimum distance.
  // We won't check whether the (x,y) is out of boundry.
  int Get(int x, int y) const override;

  // Update should be called after an obstacle is added or removed at cell (x,y).
  // The nearby cells within a distance of u will be updated.
  // We won't check whether the (x,y) is out of boundry.
  void Update(int x, int y) override;
};

}  // namespace clearance_field

#endif
