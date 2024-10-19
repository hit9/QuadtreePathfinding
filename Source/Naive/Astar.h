// Source Code: https://github.com/hit9/quadtree-pathfinding
// License: BSD. Chao Wang, hit9[At]icloud.com.

// Naive A* path finding implementation on 2D grid map.
// **NOTE**: This file is NOT required to use quadtree-pathfinding.
// It's for debuging and testing purpose.

#ifndef QDPF_NAIVE_ASTAR_HPP
#define QDPF_NAIVE_ASTAR_HPP

#include <functional>

#include "../Internal/Base.h"
#include "../Internal/PathfinderAstar.h"
#include "GridMap.h"

namespace QDPF
{

	namespace Naive
	{

		using Internal::AstarResult;
		using Internal::Cell;

		using Path = std::vector<Cell>;
		using PathCollector = std::function<void(int x, int y, float cost)>;

		class NaiveAStarPathFinder
		{
		public:
			// Find a shortest path from (x1,y1) to (x2,y2) on given map m.
			// Returns { Ok } on success.
			// Returns { Unreachable } on failure.
			AstarResult Compute(const NaiveGridMap* m, int x1, int y1, int x2, int y2, PathCollector& collector);
			AstarResult Compute(const NaiveGridMap* m, int x1, int y1, int x2, int y2, Path& path);
		};

	} // namespace Naive

} // namespace QDPF
#endif
