// Source Code: https://github.com/hit9/quadtree-pathfinding
// License: BSD. Chao Wang, hit9[At]icloud.com.

#ifndef QDPF_INTERNAL_QUADTREE_MAPX_HPP
#define QDPF_INTERNAL_QUADTREE_MAPX_HPP

// QuadtreeMapX
// ~~~~~~~~~~~~
// A manager of multiple quadtree maps to support different agent sizes and terrain types.

#include <initializer_list>
#include <unordered_map>

#include "quadtree_map.hpp"
#include "true-clearance-field/true_clearance_field.hpp"

namespace qdpf {
namespace internal {

struct QuadtreeMapXSetting {
  int AgentSize;
  // OR result of terrain type integers.
  int TerrainTypes;
};

using QuadtreeMapXSettings = std::initializer_list<QuadtreeMapXSetting>;

// TerrainTypesChecker is to check the terrain type value for given cell (x,y)
using TerrainTypesChecker = std::function<int(int x, int y)>;

class QuadtreeMapXImpl {
 public:
  QuadtreeMapXImpl(int w, int h, DistanceCalculator distance, TerrainTypesChecker terrainChecker,
                   QuadtreeMapXSettings settings, int step = 1, StepFunction stepf = nullptr,
                   int maxNodeWidth = -1, int maxNodeHeight = -1);
  ~QuadtreeMapXImpl();

  // Return the number of cells.
  int N() const { return w * h; }
  int W() const { return w; }
  int H() const { return h; }

  // Creates quadtree maps, clearance fields and call Update on existing grid map for each cell.
  void Build();

  // Find a quadtree map by agent size and walkable terrain types.
  // Returns nullptr on not found.
  [[nodiscard]] const QuadtreeMap* Get(int agentSize, int walkableTerrainTypes) const;

  // Update should be called if cell (x,y)'s terrain is changed.
  // If the (x,y) is out of bound, nothing happens.
  void Update(int x, int y);
  // Compute should be called after one or more Update calls, to apply the changes to all related
  // quadtree maps.
  void Compute();

 private:
  const int w, h, maxNodeWidth, maxNodeHeight;
  const int step;
  StepFunction stepf;
  DistanceCalculator distance;
  TerrainTypesChecker terrainChecker;
  const QuadtreeMapXSettings settings;

  // ~~~~~~~ clearance fields ~~~~~~~~~~~
  // tfs[terrainTypes] => tf.
  std::unordered_map<int, true_clearance_field::TrueClearanceField*> tfs;

  // ~~~~~~~~~~~ quadtree maps ~~~~~~~~~~~
  // maps[agentSize][terrainTypes] => map.
  std::unordered_map<int, std::unordered_map<int, QuadtreeMap*>> maps;
  // redundancy map for: maps1[terrainTypes] => list of quadtree map pointers.
  std::unordered_map<int, std::vector<QuadtreeMap*>> maps1;

  // records the dirty cells where the clearance value changed.
  // they are cleared after Compute().
  // dirties[terrainTypes] => {(x,y), ...}
  std::unordered_map<int, std::vector<std::pair<int, int>>> dirties;

  void buildClearanceFields();
  void buildClearanceFieldForTerrainTypes(int agentSizeBound, int costUnit, int costUnitDiagonal,
                                          int terrainTypes);
  void buildQuadtreeMaps();
  void buildQuadtreeMapsForSetting(int agentSize, int terrainTypes);
};

}  // namespace internal
}  // namespace qdpf

#endif
