// Source Code: https://github.com/hit9/quadtree-pathfinding
// License: BSD. Chao Wang, hit9[At]icloud.com.

#include "Astar.h"

#include <cassert>

#include "../Internal/PathfinderAstar.h"
#include "../Internal/Base.h"

namespace QDPF
{
	namespace Naive
	{

		using Internal::AStar;
		using Internal::AstarResult;
		using Internal::ErrorCode;
		using Internal::inf;
		using Internal::NeighbourVertexVisitor;

		AstarResult NaiveAStarPathFinder::Compute(const NaiveGridMap* m, int x1, int y1, int x2, int y2,
			PathCollector& collector)
		{
			assert(m != nullptr);

			if (m->IsObstacle(x1, y1))
				return { ErrorCode::Unreachable, 0 };
			if (m->IsObstacle(x2, y2))
				return { ErrorCode::Unreachable, 0 };

			int s = m->PackXY(x1, y1);
			int t = m->PackXY(x2, y2);

			using A = AStar<int, inf>;

			A astar;

			A::PathCollector collector1 = [&collector, m](int v, float cost) {
				auto [x, y] = m->UnpackXY(v);
				collector(x, y, cost);
			};
			A::Distance distance = [m](int u, int v) -> float { return m->Distance(u, v); };

			A::NeighboursCollectorT neighboursCollector = [m](int u, NeighbourVertexVisitor<int>& visitor) {
				return m->GetGraph().ForEachNeighbours(u, visitor);
			};

			A::NeighbourFilterTesterT neighbourTester = nullptr;

			return astar.Compute(s, t, collector1, distance, neighboursCollector, neighbourTester);
		}

		AstarResult NaiveAStarPathFinder::Compute(const NaiveGridMap* m, int x1, int y1, int x2, int y2,
			Path& path)
		{
			PathCollector collector = [&path](int x, int y, float cost) { path.push_back({ x, y }); };
			return Compute(m, x1, y1, x2, y2, collector);
		}

	} // namespace Naive
} // namespace QDPF
