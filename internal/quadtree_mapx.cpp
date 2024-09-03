// Source Code: https://github.com/hit9/quadtree-pathfinding
// License: BSD. Chao Wang, hit9[At]icloud.com.

#include "quadtree_mapx.hpp"

#include <algorithm>
#include <cassert>
#include <unordered_set>

#include "base.hpp"
#include "quadtree_map.hpp"
#include "true-clearance-field/true_clearance_field.hpp"

namespace qdpf {
namespace internal {

QuadtreeMapXImpl::QuadtreeMapXImpl(int w, int h, DistanceCalculator distance,
                                   TerrainTypesChecker terrainChecker,
                                   QuadtreeMapXSettings settings, int step, StepFunction stepf,
                                   int maxNodeWidth, int maxNodeHeight)
    : w(w),
      h(h),
      distance(distance),
      terrainChecker(terrainChecker),
      settings(settings),
      step(step),
      stepf(stepf),
      maxNodeWidth(maxNodeWidth),
      maxNodeHeight(maxNodeHeight) {
  assert(w > 0);
  assert(h > 0);
}

QuadtreeMapXImpl::~QuadtreeMapXImpl() {
  // free all maps.
  for (auto [_, d] : maps) {
    for (auto [_, m] : d) delete m;
  }
  maps.clear();
  maps1.clear();

  // free the clearance field.
  for (auto [_, tf] : tfs) delete tf;
  tfs.clear();

  // clear dirties.
  dirties.clear();
}

// Query a QuadtreeMap by given agent size and walkable terrain types (capablities).
const QuadtreeMap* QuadtreeMapXImpl::Get(int agentSize, int walkableTerrainTypes) const {
  auto it = maps.find(agentSize);
  if (it == maps.end()) return nullptr;
  const auto& d = it->second;

  // best case: find a map extactly for these walkable terrain types.
  auto it1 = d.find(walkableTerrainTypes);
  if (it1 != d.end()) return it1->second;

  // find subsets of walkableTerrainTypes, the larger the better.
  QuadtreeMap* ans = nullptr;
  int terrainTypesNumBits = 0;

  for (auto [terrainTypes, m] : d) {
    // If all true bits in terrainTypes are also set in walkableTerrainTypes,
    // then the terrainTypes is a subset of given walkableTerrainTypes.
    if (terrainTypes == (terrainTypes & walkableTerrainTypes)) {
      // the terrainTypes with the most number of true bits wins.
      int nbits = CountBits(terrainTypes);
      if (nbits > terrainTypesNumBits) {
        terrainTypesNumBits = nbits;
        ans = m;
      }
    }
  }
  return ans;
}

void QuadtreeMapXImpl::Update(int x, int y) {
  // Update the clearance values
  for (auto [_, tf] : tfs) tf->Update(x, y);
}

void QuadtreeMapXImpl::Compute() {
  // Apply the clearance updates for each field.
  for (auto [_, tf] : tfs) tf->Compute();

  // Update all cells in related quadtree maps.
  // Of which the clearance value is recomputed, we should maintain the gate cells etc.
  for (auto& [terrainTypes, vec] : dirties) {
    for (auto m : maps1[terrainTypes]) {
      for (auto [x, y] : vec) m->Update(x, y);
    }
  }

  dirties.clear();
}

void QuadtreeMapXImpl::Build() {
  // Creates a quadtree map for each pair of {agentSize, terrainTypes}.
  createQuadtreeMaps();
  // Creates a clearance field for each terrainTypes.
  createClearanceFields();
  // Initial the clearance fields.
  buildClearanceFields();
  // Build the quadtree maps on existing terrains.
  buildQuadtreeMaps();
  // Bind them via a queue.
  bindClearanceFieldAndQuadtreeMaps();
}

// Creates a clearance field for each terrainTypes integer.
void QuadtreeMapXImpl::createClearanceFields() {
  // find the max value of agentSize.
  int maxAgentSize = 0;
  for (auto [agentSize, _] : settings) maxAgentSize = std::max(agentSize, maxAgentSize);

  // cost units.
  int costUnit = distance(0, 0, 0, 1);
  int costUnitDiagonal = distance(0, 0, 1, 1);

  // collects different unique terrainType
  std::unordered_set<int> st;
  for (auto [_, terrainTypes] : settings) st.insert(terrainTypes);

  // for each unique terrainTypes, build a clearance field.
  for (auto terrainTypes : st) {
    createClearanceFieldForTerrainTypes(maxAgentSize, costUnit, costUnitDiagonal, terrainTypes);
  }
}

// Creates a clearance field for given terrainTypes integer.
// We will create a clearance field, and bound it to all quadtree maps related to the given
// terrainTypes integer.
void QuadtreeMapXImpl::createClearanceFieldForTerrainTypes(int agentSizeBound, int costUnit,
                                                           int costUnitDiagonal,
                                                           int terrainTypes) {
  // rarely happens, but ensure that we won't reset an allocated clearance field, which makes
  // memory leakings.
  if (tfs.find(terrainTypes) != tfs.end()) return;

  true_clearance_field::ObstacleChecker isObstacle = [this, terrainTypes](int x, int y) {
    // if the terrain type value of cell (x,y) dismatches any of required terrain types, it's
    // considered an obstacle.
    return (terrainChecker(x, y) & terrainTypes) == 0;
  };
  // creates a clearance field.
  auto tf = new true_clearance_field::TrueClearanceField(w, h, agentSizeBound, costUnit,
                                                         costUnitDiagonal, isObstacle);
  tfs[terrainTypes] = tf;
}

// Creates a quadtree map for each setting.
void QuadtreeMapXImpl::createQuadtreeMaps() {
  for (auto [agentSize, terrainTypes] : settings) {
    createQuadtreeMapsForSetting(agentSize, terrainTypes);
  }
}

// Build each of clearance field.
void QuadtreeMapXImpl::buildClearanceFields() {
  for (auto [_, tf] : tfs) {
    // here: just build on an **empty** map.
    tf->Build();

    // Let's update each cell.
    for (int x = 0; x < h; ++x) {
      for (int y = 0; y < w; ++y) tf->Update(x, y);
    }

    // Finally, call Compute for the initial clearance field.
    tf->Compute();
  }
}

// Creates a quadtree map for given setting { agentSize, terrainTypes }.
void QuadtreeMapXImpl::createQuadtreeMapsForSetting(int agentSize, int terrainTypes) {
  // rarely happens, but ensure that we won't reset an allocated map, which makes
  // memory leakings.
  if (maps[agentSize].find(terrainTypes) != maps[agentSize].end()) return;

  ObstacleChecker isObstacle = [this, agentSize, terrainTypes](int x, int y) {
    // If the terrain type value of cell (x,y) dismatches any of required terrain types, it's an
    // obstacle.
    if ((terrainChecker(x, y) & terrainTypes) == 0) return true;
    // If the clearance distance dismatches the agent's size, it's an obstacle, we can't walk into
    // this cell.
    if (tfs[terrainTypes]->Get(x, y) < agentSize) return true;
    return false;
  };
  auto m = new QuadtreeMap(w, h, isObstacle, distance, step, stepf, maxNodeWidth, maxNodeHeight);

  maps[agentSize][terrainTypes] = m;
  maps1[terrainTypes].push_back(m);
}

// Build each quadtree map with existing obstacles (different for different terrains).
// This should be most slow step of the whole Build().
void QuadtreeMapXImpl::buildQuadtreeMaps() {
  for (auto [_, d] : maps) {
    for (auto [_, m] : d) m->Build();
  }
}

// Bind the clearance field of terrainTypes to all quadtree maps of the same collection of
// terrainTypes. In detail is: bind a listener for each quadtree map to listen updates from the
// related clearance field, and the bridge is a queue named "dirties".
void QuadtreeMapXImpl::bindClearanceFieldAndQuadtreeMaps() {
  for (auto [terrainTypes, tf] : tfs) {
    int t = terrainTypes;  // avoid clang++ warning: capture structure binding ... hahaha
    // when a terrain value is changed, it may affect the clearan values of cells around.
    // so we make a listener to collect them, and they will be applied to quadtree map's Updates in
    // the later Compute() call.
    tf->SetUpdatedCellVisistor([this, t](int x, int y) { dirties[t].push_back({x, y}); });
  }
}

}  // namespace internal
}  // namespace qdpf
