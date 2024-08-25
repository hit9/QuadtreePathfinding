// Source Code: https://github.com/hit9/quadtree-pathfinding
// License: BSD. Chao Wang, hit9[At]icloud.com.

#ifndef QDPF_INTERNAL_PATHFINDER_HELPER_HPP
#define QDPF_INTERNAL_PATHFINDER_HELPER_HPP

#include "graph.hpp"
#include "quadtree_map.hpp"

// PathFinderHelper
// ~~~~~~~~~~~~~~~
// Help util functions to implement path finders.

namespace qdpf {
namespace internal {

// PathFinderHelper is a mixin class to provide some util functions.
class PathFinderHelper {
 public:
  // Bresenham's line algorithm.
  // You can override it with a custom implementation.
  // Ref: https://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm
  // Ref: https://members.chello.at/easyfilter/bresenham.html
  virtual void ComputePathToNextRouteCell(int x1, int y1, int x2, int y2,
                                          CellCollector &collector) const;

 protected:
  const QuadtreeMapImpl *m = nullptr;
  // tmp gate graph is to store edges between start/target and other gate cells.
  SimpleUnorderedMapDirectedGraph<int> tmp;

  // Resets current working quadtree map.
  void Reset(const QuadtreeMapImpl *m);

  // BuildTmpGateGraph builds a temporary gate graph to store edges between start/target and
  // other gates in the same nodes.
  // This is a helper function.
  // Parameters:
  // 1. s and t are the ids of cell start and target.
  // 2. (x1,y1) and (x2,y2) are the positions of cell start and target.
  void BuildTmpGateGraph(int s, int t, int x1, int y1, int x2, int y2, QdNode *sNode,
                         QdNode *tNode);

  // ForEachNeighbourGateCellWithST iterates each neighbor gate cell connecting from given gate
  // cell u. What's the deference with the gate graph's ForEachNeighbours is: it will check both
  // the QuadtreeMap's gate cell graph and the temporary gate graph,
  // where stores the start, target informations.
  void ForEachNeighbourGateWithST(int u, NeighbourVertexVisitor<int> &visitor) const;

  // helper function to add a cell u to the given node in the temporary graph.
  void addCellToTmpGateGraph(int u, QdNode *node);
};

}  // namespace internal
}  // namespace qdpf

#endif
