// Source Code: https://github.com/hit9/quadtree-pathfinding
// License: BSD. Chao Wang, hit9[At]icloud.com.

#include "Graph.h"

namespace QDPF
{
	namespace Internal
	{

		//////////////////////////////////////
		/// SimpleDirectedGraph
		//////////////////////////////////////

		void SimpleDirectedGraph::Resize(int n)
		{
			edges.resize(n), predecessors.resize(n);
		}

		void SimpleDirectedGraph::Init() {}

		void SimpleDirectedGraph::AddEdge(int u, int v, float cost)
		{
			edges[u].insert({ v, cost });
			predecessors[v].insert(u);
		}

		void SimpleDirectedGraph::RemoveEdge(int u, int v)
		{
			edges[u].erase(v);
			predecessors[v].erase(u);
		}

		void SimpleDirectedGraph::ClearEdgeFrom(int u)
		{
			for (auto [v, _] : edges[u])
				predecessors[v].erase(u);
			edges[u].clear();
		}

		void SimpleDirectedGraph::ClearEdgeTo(int v)
		{
			for (auto u : predecessors[v])
				edges[u].erase(v);
			predecessors[v].clear();
		}

		void SimpleDirectedGraph::ForEachNeighbours(int u, NeighbourVertexVisitor<int>& visitor) const
		{
			for (const auto [v, cost] : edges[u])
				visitor(v, cost);
		}

		void SimpleDirectedGraph::Clear()
		{
			edges.clear();
			predecessors.clear();
		}

		void SimpleDirectedGraph::ForEachEdge(EdgeVisitor<int>& visitor) const
		{
			for (int u = 0; u < edges.size(); ++u)
			{
				for (auto [v, cost] : edges[u])
				{
					visitor(u, v, cost);
				}
			}
		}

	} // namespace Internal
} // namespace QDPF
