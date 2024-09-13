// Source Code: https://github.com/hit9/quadtree-pathfinding
// License: BSD. Chao Wang, hit9[At]icloud.com.

// Naive flow field path finding implementation on 2D grid map.
// **NOTE**: This file is NOT required to use quadtree-pathfinding.
// It's for debuging and testing purpose.

#ifndef QDPF_NAIVE_FLOWFIELD_HPP
#define QDPF_NAIVE_FLOWFIELD_HPP

#include "../Internal/base.hpp"
#include "../Internal/pathfinder_flowfield.hpp"
#include "grid_map.hpp"

namespace qdpf
{
	namespace naive
	{

		using internal::FinalFlowField;
		using internal::Rectangle;

		class NaiveFlowFieldPathFinder
		{
		public:
			// Computes a flowfield inside given rectangle range.
			// Returns -1 if unreachable.
			int Compute(const NaiveGridMap* m, int x2, int y2, const Rectangle& qrange,
				FinalFlowField& field);
		};

	} // namespace naive
} // namespace qdpf

#endif