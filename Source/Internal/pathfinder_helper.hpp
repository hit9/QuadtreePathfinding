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

// Collects the neighbor vertices from u.
template <typename Vertex>
using NeighboursCollector = std::function<void(Vertex u, NeighbourVertexVisitor<Vertex> &visitor)>;

// Filter a neighbor vertex, returns true for cared neighbor.
template <typename Vertex>
using NeighbourFilterTester = std::function<bool(Vertex)>;

// PathFinderHelper is a mixin class to provide some util functions.
class PathFinderHelper {
 protected:
  // Current working on map.
  const QuadtreeMap *m = nullptr;
  // tmp gate graph is to store edges between start/target and other gate cells.
  SimpleUnorderedMapDirectedGraph<int> tmp;

  // Resets current working quadtree map.
  void Reset(const QuadtreeMap *m);

  // ForEachNeighbourGateCellWithST iterates each neighbor gate cell connecting from given gate
  // cell u. What's the deference with the gate graph's ForEachNeighbours is: it will check both
  // the QuadtreeMap's gate cell graph and the temporary gate graph,
  // where stores the start, target informations.
  void ForEachNeighbourGateWithST(int u, NeighbourVertexVisitor<int> &visitor) const;

  // Helper function to add a cell u to the given node on the temporary graph.
  // it establishes bidirectional edges between u and existing gate cells inside the given node.
  void AddCellToNodeOnTmpGraph(int u, QdNode *node);

  // Helper function to connect cell u and v with bidirectional edges on the temporary graph.
  void ConnectCellsOnTmpGraph(int u, int v);
};

}  // namespace internal
}  // namespace qdpf

#endif
