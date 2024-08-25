// Source Code: https://github.com/hit9/quadtree-pathfinding
// License: BSD. Chao Wang, hit9[At]icloud.com.

#ifndef QDPF_INTERNAL_GRAPH_HPP
#define QDPF_INTERNAL_GRAPH_HPP

#include <functional>  // for std::function
#include <unordered_map>
#include <unordered_set>

// Graph
// ~~~~~~
// Directed graph abstraction.

namespace qdpf {
namespace internal {

// NeighbourVertexVisitor is the type of the function that visit neighbor vertices of a given
// vertex in a directed graph.
template <typename Vertex>
using NeighbourVertexVisitor = std::function<void(Vertex v, int cost)>;

// It's a pure virtual class so that the subclasses should implement all the virtual methods.
template <typename Vertex>
class IDirectedGraph {
 public:
  // Initialize the graph, where the n is the total number of vertices.
  // This method should be called by a path finder.
  virtual void Init(int n) = 0;
  // Add an edge from vertex u to v with given cost.
  virtual void AddEdge(Vertex u, Vertex v, int cost) = 0;
  // Remove an edge from vertex u to v.
  virtual void RemoveEdge(Vertex u, Vertex v) = 0;
  // Clears all edges starting from vertex u.
  virtual void ClearEdgeFrom(Vertex u) = 0;
  // Clears all edges connectinig to vertex v.
  virtual void ClearEdgeTo(Vertex v) = 0;
  // Call given visitor function for each neighbor vertex v connecting from given vertex u.
  virtual void ForEachNeighbours(Vertex u, NeighbourVertexVisitor<Vertex> &visitor) const = 0;
  // Clears all edges.
  virtual void Clear() = 0;
};

// SimpleDirectedGraph is a simple implementation of IDirectedGraph, using integral vertex.
class SimpleDirectedGraph : public IDirectedGraph<int> {
 public:
  void Init(int n) override;
  void AddEdge(int u, int v, int cost) override;
  void RemoveEdge(int u, int v) override;
  void ClearEdgeFrom(int u) override;
  void ClearEdgeTo(int v) override;
  void ForEachNeighbours(int u, NeighbourVertexVisitor<int> &visitor) const override;
  void Clear() override;

 protected:
  // edges[from] => { to => cost }
  std::vector<std::unordered_map<int, int>> edges;
  // predecessors[to] => { from .. }
  std::vector<std::unordered_set<int>> predecessors;
};

// SimpleUnorderedMapDirectedGraph is a simple directed graph storing in an unordered_map.
// This uses less memory than SimpleDirectedGraph for sparse graph.
template <typename Vertex, typename VertexHasher = std::hash<Vertex>>
class SimpleUnorderedMapDirectedGraph : public IDirectedGraph<Vertex> {
 public:
  void Init(int n) override;
  void AddEdge(Vertex u, Vertex v, int cost) override;
  void RemoveEdge(Vertex u, Vertex v) override;
  void ClearEdgeFrom(Vertex u) override;
  void ClearEdgeTo(Vertex v) override;
  void ForEachNeighbours(Vertex u, NeighbourVertexVisitor<Vertex> &visitor) const override;
  void Clear() override;

 protected:
  using M = std::unordered_map<Vertex, int, VertexHasher>;
  using ST = std::unordered_set<Vertex, VertexHasher>;
  // edges[v] => { v =>  cost(u => v) }
  std::unordered_map<Vertex, M, VertexHasher> edges;
  // predecessors[to] => { from .. }
  std::unordered_map<Vertex, ST, VertexHasher> predecessors;
};

// ~~~~~~~~~~~ Implements  SimpleUnorderedMapDirectedGraph ~~~~~~~~~~~~~~

template <typename Vertex, typename VertexHasher>
void SimpleUnorderedMapDirectedGraph<Vertex, VertexHasher>::Init(int n) {}

template <typename Vertex, typename VertexHasher>
void SimpleUnorderedMapDirectedGraph<Vertex, VertexHasher>::AddEdge(Vertex u, Vertex v, int cost) {
  edges[u].insert({v, cost});
  predecessors[v].insert(u);
}

template <typename Vertex, typename VertexHasher>
void SimpleUnorderedMapDirectedGraph<Vertex, VertexHasher>::RemoveEdge(Vertex u, Vertex v) {
  // Remove from edges.
  auto it = edges.find(u);
  if (it != edges.end()) {
    // m is edges[u]
    auto &m = it->second;
    m.erase(v);
    if (m.empty()) edges.erase(it);
  }
  // Remove from predecessors.
  auto it1 = predecessors.find(v);
  if (it1 != predecessors.end()) {
    // st is predecessors[v]
    auto &st = it->second;
    st.erase(u);
    if (st.empty()) predecessors.erase(it1);
  }
}

template <typename Vertex, typename VertexHasher>
void SimpleUnorderedMapDirectedGraph<Vertex, VertexHasher>::ClearEdgeFrom(Vertex u) {
  auto it = edges.find(u);
  if (it == edges.end()) return;
  // m is edges[u]
  auto &m = it->second;
  // Remove (u => v) in predecessors in advance.
  for (auto [v, _] : m) {
    auto it1 = predecessors.find(v);
    if (it1 != predecessors.end()) {
      // st is predecessors[v]
      auto &st = it1->second;
      st.erase(u);
      if (st.empty()) predecessors.erase(it1);
    }
  }
  // Clear and remove edges from u.
  m.clear();
  edges.erase(it);
}

template <typename Vertex, typename VertexHasher>
void SimpleUnorderedMapDirectedGraph<Vertex, VertexHasher>::ClearEdgeTo(Vertex v) {
  auto it = predecessors.find(v);
  if (it == predecessors.end()) return;
  // st is predecessors[v]
  auto &st = it->second;
  // Remove (u => v) in edges in advance.
  for (auto u : st) {
    auto it1 = edges.find(u);
    if (it1 != edges.end()) {
      // m is edges[u]
      auto &m = it1->second;
      m.erase(v);
      if (m.empty()) edges.erase(it1);
    }
  }
  // Clear and remove predecessors to v.
  st.clear();
  predecessors.erase(it);
}

template <typename Vertex, typename VertexHasher>
void SimpleUnorderedMapDirectedGraph<Vertex, VertexHasher>::ForEachNeighbours(
    Vertex u, NeighbourVertexVisitor<Vertex> &visitor) const {
  auto it = edges.find(u);
  if (it == edges.end()) return;
  // m is edges[u]
  const auto &m = it->second;
  for (const auto [v, cost] : m) visitor(v, cost);
}

template <typename Vertex, typename VertexHasher>
void SimpleUnorderedMapDirectedGraph<Vertex, VertexHasher>::Clear() {
  edges.clear();
  predecessors.clear();
}

}  // namespace internal
}  // namespace qdpf
#endif
