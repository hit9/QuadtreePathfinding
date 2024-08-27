// Source Code: https://github.com/hit9/quadtree-pathfinding
// License: BSD. Chao Wang, hit9[At]icloud.com.

#include "pathfinder_flowfield.hpp"

#include <cassert>

namespace qdpf {
namespace internal {

void FlowFieldPathFinderImpl::Reset(const QuadtreeMap* m_, int x2_, int y2_,
                                    const Rectangle& dest_) {
  // resets the attributes.
  m = m_;
  x2 = x2_, y2 = y2_;
  t = m->PackXY(x2, y2);
  dest = dest_;  // copy updated

  tNode = m->FindNode(x2, y2);
  if (tNode == nullptr) return;

  // clears the old results.
  nodeFlowField.Clear();
  gateFlowField.Clear();
  finalFlowField.Clear();

  // Rebuild the tmp graph.
  // Add the target cell to the gate graph
  // TODO: Reset the helper

  // TODO: should add all cells in the tNode to the tmp graph...
}

}  // namespace internal
}  // namespace qdpf
