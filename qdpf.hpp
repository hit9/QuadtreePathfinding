// Hierarchical path finding on quadtree for equal-weighted 2D grid map.
// License: BSD. Version: 0.2.0. Author: Chao Wang, hit9[At]icloud.com.
// Source Code: https://github.com/hit9/quadtree-pathfinding
// Quadtree reference: https://github.com/hit9/quadtree-hpp

// Concepts and Mechanisms
// ~~~~~~~~~~~~~~~~~~~~~~~
// 1. A QuadtreeMap is a 2D grid map maintained by a quadtree.
// 2. The quadtree splits the grid map into multiple sections.
// 3. A section on a quadtree map contains no obstacles or all obstacles.
// 4. The shortest path inside a section without obstacles will be a straight line.
// 5. Adjacent quadtree nodes are connected by multiple gates.
// 6. A gate is composed of two adjacent cells, one on each side, directed.
// 7. All nodes compose the 1st level abstract graph.
// 8. All gates compose the 2nd level abstract graph.
// 9. Path finding performs on the 2 or 3 levels graphs:
//      1. Find the node path on the 1st level graph (it's optional, faster but less optimal).
//      2. Find the gate path on the 2nd level graph.
//      3. Fill the straight lines between gate cells.
// 10. A QuadtreeMapX is a manager of multiple quadtree maps for different agent sizes and terrain
//     types supports.
// 11. A PathFinder always works on a single QuadtreeMap the same time. A pathfinding request is
//     reduced into a progress without the agent-size and terrain factors.

// Coordinates
// ~~~~~~~~~~~
//    0      w
//  0 +---------------> y
//    |
// h  |
//    |
//    v
//    x

#ifndef QDPF_HPP
#define QDPF_HPP

#include <cmath>
#include <functional>  // for std::function

#include "internal/base.hpp"
#include "internal/pathfinder_astar.hpp"
#include "internal/pathfinder_flowfield.hpp"
#include "internal/quadtree_map.hpp"
#include "internal/quadtree_mapx.hpp"

namespace qdpf {

using internal::inf;
using internal::Rectangle;

using internal::QdNode;  // the quadtree node.

// CellCollector is the type of the function that collects points on a path.
// The argument (x,y) is a cell in the grid map.
//
// Signature: std::function<void(int x, int y)>;
using CellCollector = internal::CellCollector;

// ComputeStraightLine computes the straight line from (x1,y1) to (x2,y2) based on Bresenham's line
// algorithm. Ref: https://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm Ref:
// https://members.chello.at/easyfilter/bresenham.html
// Signature: void ComputeStraightLine(int x1, int y1, int x2, int y2, CellCollector &collector);
using internal::ComputeStraightLine;

//////////////////////////////////////
/// QuadtreeMapX
//////////////////////////////////////

// DistanceCalculator is the type of the function that calculates the distance
// from cell (x1,y1) to (x2,y2).
//
// The distance calculator should guarantee that:
// 1. the distance between (x1,y1) and (x2,y2) always equals to the distance between (x2,y2) and
//    (x1,y1).
// 2. the distance between two same cells is always 0.
//
// We can just use quadtree_astar::EuclideanDistance<CostUnit> to build a euclidean distance
// calculator.
//
// Signature: std::function<int(int x1, int y1, int x2, int y2)>;
using DistanceCalculator = internal::DistanceCalculator;

// Euclidean distance calculator with a given cost unit.
template <int CostUnit>
int EuclideanDistance(int x1, int y1, int x2, int y2) {
  return std::floor(std::hypot(x1 - x2, y1 - y2) * CostUnit);
}

// StepFunction is the type of a function to specific a dynamic gate picking step.
// The argument length is the length (number of cells) of the adjacent side of two neighbor nodes.
// We should make sure the return value is always > 0.
//
// Signature: std::function<int(int length)>;
//
// An example: [](int length) { return length / 8 + 1; }
// For this example, we use larger step on large rectangles, and smaller step on small rectangles.
using StepFunction = internal::StepFunction;

// QuadtreeMapXSetting is a struct to specific an agent size along with its terrain types
// capabilities.
//
// struct {
//   // the size of pathfinding agent, usually set to the maximum side length of the agent.
//   // Note that it is relative to the unit of the distance function structure, not the number of
//   // cells.
//   int AgentSize;
//
//   // the terrain types to support, represented as the OR sum result of terrain type integers.
//   // It must be an integer > 0.
//   // e.g. Land | Water, where the Land and Water are positive integers, pow of 2.
//   int TerrainTypes;
// };
//
using QuadtreeMapXSetting = internal::QuadtreeMapXSetting;

// QuadtreeMapXSettings is a list of QuadtreeMapXSetting.
// std::initializer_list<QuadtreeMapXSetting>;
//
// Example:
//
//   enum Terrain {
//     Land = 0b001,      // 1
//     Water = 0b010,     // 2
//     Building = 0b100,  // 4
//   };
//
//  qdpf::QuadtreeMapXSettings settings{
//      {1, Terrain::Land},                   // e.g. soldiers
//      {1, Terrain::Land | Terrain::Water},  // e.g. seals
//      {2, Terrain::Water},                  // e.g. boats
//  };
//
using QuadtreeMapXSettings = internal::QuadtreeMapXSettings;

// TerrainTypesChecker is to check the terrain type value for given cell (x,y).
// A terrain type value should be a positive integer, and must be power of 2 integer,
// e.g. 0b1, 0b10, 0b100 etc.
// Signature: std::function<int(int x, int y)>;
using TerrainTypesChecker = internal::TerrainTypesChecker;

// QuadtreeMapX is a manager of multiple 2D grid maps maintained by quadtrees.
class QuadtreeMapX {
 public:
  // Parameters:
  // * w and h are the width and height of the map, the number of cells in y and x axis.
  // * distance is a function that calculates the distance between two given cells.
  //    there's a builtin helper function template EuclideanDistance<CostUnit> to use
  //    for euclidean distance.
  // * settings is the list of agent sizes along with terrain types to support.
  //    If you pass n settings, we will create n quadtree maps.
  // * step is the number of interval cells when picking gate cells in a quadtree map.
  // * stepf is a function to specific dynamic gate picking steps. We will use this function
  //   instead of the constant step if it's provided.
  // * maxNodeWidth and maxNodeHeight the max width and height of a quadtree node's rectangle.
  QuadtreeMapX(int w, int h, DistanceCalculator distance, TerrainTypesChecker terrainChecker,
               QuadtreeMapXSettings settings, int step = 1, StepFunction stepf = nullptr,
               int maxNodeWidth = -1, int maxNodeHeight = -1);

  // Build all managed quadtree maps, this should be called for only once right after the
  // construction. And it will traverse each cell in the w * h map and call Update on it.
  void Build();

  // Update should be called if cell (x,y)'s terrain value is changed.
  // Then Compute should be called to apply these changes.
  void Update(int x, int y);

  // Compute should be called after one or multiple Update calls.
  // It will apply all chanegs to all related quadtree maps.
  void Compute();

  // ~~~~~~~~~~ Debuging purpose ~~~~~~~~~~~
  // Find a quadtree map supporting given agent size and terrain types.
  // Returns nullptr if not found.
  // If there are multiple maps support the given walkableTerrainTypes, the one with largest subset
  // of terrain types support will be returned.
  // This is a debuging purpose api.
  [[nodiscard]] const internal::QuadtreeMap *Get(int agentSize, int walkableTerrainTypes) const;

 private:
  internal::QuadtreeMapXImpl impl;

  // friend with all path finders.
  friend class AStarPathFinder;
  friend class FlowFieldPathFinder;
};

//////////////////////////////////////
/// PathFinding
//////////////////////////////////////

// NodeVisitor is the type of a function to visit quadtree nodes.
// Signature:
// std::function<void(const QdNode *node)>;
using NodeVisitor = internal::QdNodeVisitor;

//////////////////////////////////////
/// AStarPathFinder
//////////////////////////////////////

// A* path finder (stateful).
class AStarPathFinder {
 public:
  // AStarPathFinder is bound to a quadtree map manager.
  AStarPathFinder(const QuadtreeMapX &mx);

  // ~~~~~~~~~~~~~~ API ~~~~~~~~~~~~~~

  // Resets the current working context of this path finder.
  // Returns 0 for success.
  // Returns -1 if there's no quadtree map was found.
  //
  // A path finder always works on a single QuadtreeMap at the same time.
  // We must call Reset() before changing to another kind of {agent-size, terrains, start and
  // target}.
  //
  // Parameters:
  // The cell (x1,y1) and (x2,y2) are start and target cells.
  // The agentSize is the size of the pathfinding agent.
  // The walkableTerrainTypes is the bitwise OR sum of all terrain type values that the pathfinding
  // agent can walk.
  [[nodiscard]] int Reset(int x1, int y1, int x2, int y2, int agentSize,
                          int walkableterrainTypes = 1);

  // ComputeNodeRoutes computes the path of quadtree nodes from the start cell's node to the target
  // cell's node on the node graph.
  // Returns -1 if unreachable.
  // Returns -1 if either of start and target cells are out of bound.
  // Returns the approximate cost to target node on the node graph level.
  // Reset() should be called in advance to call this api.
  // This step is optional, the benefits to use it ahead of ComputeGateRoutes:
  // 1. faster (but less optimal).
  // 2. fast checking if the target is reachable.
  // 3. optimize the following ComputeGateRoutes(useNodePath=true) call.
  [[nodiscard]] int ComputeNodeRoutes();

  // Returns the count of quadtree nodes on the computed node path.
  // ComputeNodeRoutes() should be called in advance to call this api.
  std::size_t NodePathSize() const;

  // Visit the computed node path.
  // ComputeNodeRoutes() should be called in advance to call this api.
  void VisitComputedNodeRoutes(NodeVisitor &visitor) const;

  // ComputeGateRoutes computes the route cells from (x1,y1) to (x2,y2).
  // Sets useNodePath to true to use the previous ComputeNodeRoutes results, it will find path
  // only over gate cells on the node path, this the path finding is much faster, but less optimal.
  // Sets useNodePath to false to disable this optimization, it will find path over all gate cells.
  //
  // The route cells are composed of three kinds of cells: start(x1,y1), gate cells in the middle
  // and target(x2,y2).
  // Returns -1 if the path finding is failed.
  // Returns -1 if either of start and target cells are out of bound.
  // Returns the distance of the shortest path on success (>=0).
  //
  // Reset() should be called in advance to call this api.
  [[nodiscard]] int ComputeGateRoutes(CellCollector &collector, bool useNodePath = true);

 private:
  const QuadtreeMapX &mx;
  internal::AStarPathFinderImpl impl;
};

//////////////////////////////////////
/// FlowFieldPathFinder
//////////////////////////////////////

// NodeFlowFieldVisitor is the function to visit each node in the computed node flow field.
//
// Parameters:
// * node is current quadtree node.
// * nextNode is the next node that the current node points to.
// * cost is the total cost from current node to the target node.
using NodeFlowFieldVisitor =
    std::function<void(const QdNode *node, const QdNode *nextNode, int cost)>;

// CellFlowFieldVisitor is the function to visit each cell in the computed cell level flow field.
// A gate flow field is a cell-based flow field, and the final grid-map-level flow field is also a
// cell-based flow field.
//
// Parameters:
// * (x,y) is the current cell.
// * (xNext,yNext) is the next gate cell that the current cell points to.
// * cost is the total cost from current cell to the target cell.
// Signature:: std::function<void(int x, int y, int xNext, int yNext, int cost)>;
using CellFlowFieldVisitor = internal::UnpackedCellFlowFieldVisitor;

// FlowField (stateful)
class FlowFieldPathFinder {
 public:
  // FlowFieldPathFinder should be bound to a quadtree map manager.
  FlowFieldPathFinder(const QuadtreeMapX &mx);

  // ~~~~~~~~~~~~~~ API ~~~~~~~~~~~~~~

  // Resets the current working context of this path finder.
  // Returns 0 for success.
  // Returns -1 if there's no quadtree map was found.
  //
  // A path finder always works on a single QuadtreeMap at the same time.
  // We must call Reset() before changing to another kind of {agent-size, terrains, destination
  // rectangle and target}.
  //
  // For the case: if there are different sized or differen terrain capabilities agents in the
  // destination rectangle, we should group them by {agent size, terrain types}, and call flow path
  // finder for each.
  //
  // Parameters:
  // * cell (x2,y2) is the target.
  // * dest is the destination rectangle, we will fill the flow field results into this region.
  //   It's better to use a rectangle that covers all the path finding agents.
  //   This struct will be copied into the path finder (and reset existing one).
  // * The agentSize is the size of the pathfinding agents.
  // * The walkableTerrainTypes is the bitwise OR sum of all terrain type values that the
  //    pathfinding agents can walk.
  [[nodiscard]] int Reset(int x2, int y2, const Rectangle &dest, int agentSize,
                          int walkableterrainTypes = 1);

  // ~~~~~~~~~~~~~~~~~~~~~~~ Node Graph Level (Optional) ~~~~~~~~~~~~~~

  // Computes the node flow field.
  // Returns -1 if the target cell is out of bound.
  //
  // In a node flow field, a node points to another field, finally points to the node where the
  // target cell locates.
  //
  // This step is optional, the benefits to use it ahead of ComputeGateFlowField:
  // 1. faster (but less optimal).
  // 2. fast checking if the target is reachable for an agent.
  // 3. optimize the following ComputeGateFlowField(useNodeFlowField=true) call.
  // Reset() should be called in advance to call this api.
  [[nodiscard]] int ComputeNodeFlowField();

  // Visits the computed node flow field.
  // Make sure the ComputeNodeFlowField has been called before calling this function.
  void VisitComputedNodeFlowField(NodeFlowFieldVisitor &visitor);

  // ~~~~~~~~~~~~~~~~~~~~~~~ Gate Graph Level (Required) ~~~~~~~~~~~~~~

  // Computes the gate flow field.
  // Returns -1 if the target cell is out of bound.
  // Setting useNodeFlowField to true to use node flow field results of ComputeNodeFlowField().
  // This makes ComputeGateFlowField() runs faster.
  //
  // In a gate flow field, a gate cell points to another gate cell, finally points to the target
  // cell.
  //
  // This step is required.
  // Reset() should be called in advance to call this api.
  [[nodiscard]] int ComputeGateFlowField(bool useNodeFlowField = true);

  // Visits the computed gate flow field
  // Make sure the ComputeGateFlowField has been called before calling this function.
  void VisitComputedGateFlowField(CellFlowFieldVisitor &visitor);

  // ~~~~~~~~~~~~~~~~~~~~~~~  Grid Map Level  (Required) ~~~~~~~~~~~~~~

  // Computes the final flow field for all cells in the destination rectangle.
  // Returns -1 if the target cell is out of bound.
  //
  // In this flow field, a cell inside the destination rectangle points to a gate cell to go,
  // finally points to the target.
  // If we want to show the detailed path, call ComputeStraightLine between current cell and the
  // next gate cell.
  //
  // Reset() should be called in advance to call this api.
  [[nodiscard]] int ComputeCellFlowFieldInDestRectangle();

  // Visits the computed cell flow field in the destination rectangle.
  // Make sure the ComputeCellFlowFieldInDestRectangle has been called before calling this
  // function.
  void VisitComputedCellFlowFieldInDestRectangle(CellFlowFieldVisitor &visitor);

 private:
  const QuadtreeMapX &mx;
  internal::FlowFieldPathFinderImpl impl;
};

}  // namespace qdpf

#endif
