// Source Code: https://github.com/hit9/quadtree-pathfinding
// License: BSD. Chao Wang, hit9[At]icloud.com.

// 2D grid map in format of a graph.
// **NOTE**: This file is NOT required to use quadtree-pathfinding.
// It's for debuging and testing purpose.

#ifndef QDPF_NAIVE_MAP_HPP
#define QDPF_NAIVE_MAP_HPP

#include "../Internal/Base.h"
#include "../Internal/Graph.h"
#include "../Internal/QuadtreeMap.h"

namespace QDPF
{
	namespace Naive
	{

		using Internal::Cell;
		using Internal::DistanceCalculator;
		using Internal::ObstacleChecker;
		using Internal::PairHasher;
		using NaiveGridGraph = Internal::SimpleDirectedGraph;

		class NaiveGridMap
		{
		public:
			NaiveGridMap(int w, int h, ObstacleChecker isObstacle, DistanceCalculator distance);

			int W() const { return w; }
			int H() const { return h; }

			bool IsObstacle(int x, int y) const;
			bool IsObstacle(const Cell& c) const;

			int Distance(int x1, int y1, int x2, int y2) const;
			int Distance(const Cell& c1, const Cell& c2) const;
			int Distance(int u, int v) const;

			const NaiveGridGraph& GetGraph() const { return g; }

			int	 PackXY(int x, int y) const;
			Cell UnpackXY(int v) const;

			// Build on a an existing 2D grid map.
			void Build();
			void Update(int x, int y, int maxd = 8);

		private:
			const int	   w, h, s;
			NaiveGridGraph g;

			ObstacleChecker	   isObstacle;
			DistanceCalculator distance;

			// directions[i] => {dx, dy, cost}
			int directions[8][3];
		};

	} // namespace Naive
} // namespace QDPF

#endif
