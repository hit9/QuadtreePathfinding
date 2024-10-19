// Source Code: https://github.com/hit9/quadtree-pathfinding
// License: BSD. Chao Wang, hit9[At]icloud.com.

// Naive flow field path finding implementation on 2D grid map.
// **NOTE**: This file is NOT required to use quadtree-pathfinding.
// It's for debuging and testing purpose.

#ifndef QDPF_NAIVE_FLOWFIELD_HPP
#define QDPF_NAIVE_FLOWFIELD_HPP

#include "../Internal/Base.h"
#include "../Internal/PathfinderFlowfield.h"
#include "GridMap.h"

namespace QDPF
{
	namespace Naive
	{

		using Internal::FinalFlowField;
		using Internal::FlowFieldResult;
		using Internal::Rectangle;

		class NaiveFlowFieldPathFinder
		{
		public:
			// Computes a flowfield inside given rectangle range.
			// Returns { Unreachable } if unreachable.
			// Returns { Ok } if success.
			FlowFieldResult Compute(const NaiveGridMap* m, int x2, int y2, const Rectangle& qrange,
				FinalFlowField& field);
		};

	} // namespace Naive
} // namespace QDPF

#endif
