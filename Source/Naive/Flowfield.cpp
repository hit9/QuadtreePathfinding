// Source Code: https://github.com/hit9/quadtree-pathfinding
// License: BSD. Chao Wang, hit9[At]icloud.com.

#include "Flowfield.h"

#include <cassert>

#include "../Internal/Base.h"
#include "../Internal/PathfinderFlowfield.h"

namespace qdpf
{
	namespace Naive
	{

		using Internal::FlowFieldAlgorithm;
		using Internal::inf;
		using Internal::IsInsideRectangle;
		using Internal::NeighbourVertexVisitor;
		using Internal::PackedCellFlowField;

		int NaiveFlowFieldPathFinder::Compute(const NaiveGridMap* m, int x2, int y2,
			const Rectangle& qrange, FinalFlowField& field)
		{
			assert(m != nullptr);

			if (m->IsObstacle(x2, y2))
				return -1;
			if (!(qrange.x1 <= qrange.x2 && qrange.y1 <= qrange.y2))
				return -1;

			int t = m->PackXY(x2, y2);

			using FFA = FlowFieldAlgorithm<int, inf>;
			FFA ffa;

			FFA::HeuristicFunction heuristic = [x2, y2, m](int v) {
				auto [x, y] = m->UnpackXY(v);
				return m->Distance(x, y, x2, y2);
			};

			FFA::NeighboursCollectorT neighboursCollector = [m](int							 u,
																NeighbourVertexVisitor<int>& visitor) {
				return m->GetGraph().ForEachNeighbours(u, visitor);
			};

			// how many grids should check.
			int n = (qrange.x2 - qrange.x1 + 1) * (qrange.y2 - qrange.y1 + 1);
			// how many grids already checked.
			int numChecked = 0;

			// We check if there's n cells are checked bu ffa.
			// actually we should count the non-obstacle cells, but we may need to count the obstacles inside
			// qrange, which may be slow, so we may stop later, just using n instead of nNonObstacles.
			FFA::StopAfterFunction stopf = [n, &numChecked, &qrange, m](int u) {
				auto [x, y] = m->UnpackXY(u);
				if (IsInsideRectangle(x, y, qrange))
					++numChecked;
				return numChecked >= n;
			};

			FFA::NeighbourFilterTesterT neighbourTester = nullptr;

			PackedCellFlowField pfield;

			ffa.Compute(t, pfield, heuristic, neighboursCollector, neighbourTester, stopf);

			// unpack to field.
			for (auto [u, p] : pfield.GetUnderlyingMap())
			{
				auto& [v, cost] = p;
				auto [x, y] = m->UnpackXY(u);
				auto [x1, y1] = m->UnpackXY(v);
				field[{ x, y }] = { { x1, y1 }, cost };
			}

			return 0;
		}

	} // namespace Naive
} // namespace qdpf
