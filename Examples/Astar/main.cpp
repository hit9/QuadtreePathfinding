#include <iostream>
#include <utility>
#include <vector>

#include "QDPF.h"

const int N = 8;

enum Terrain
{
	Land = 0b001,	  // 1
	Water = 0b010,	  // 2
	Building = 0b100, // 4
};

int grid[N][N] = {
	// 8x8
	// clang-format off
    // ----------------------> x
    {1, 1, 1, 1, 1, 1, 1, 1},
    {4, 4, 4, 4, 4, 1, 1, 1},
    {1, 2, 2, 2, 4, 2, 1, 1},
    {1, 2, 2, 2, 4, 2, 1, 1},
    {1, 2, 2, 2, 2, 1, 1, 1},
    {1, 1, 1, 2, 1, 2, 2, 2},
    {1, 1, 1, 1, 1, 1, 1, 1},
    {1, 1, 1, 1, 1, 1, 1, 1},
	// clang-format on
};

int main(void)
{
	int w = 8, h = 8;

	// Setup a QuadtreeMapX.
	// (Note that the y is the row, x is the column)
	QDPF::TerrainTypesChecker  terrainChecker = [](int x, int y) { return grid[y][x]; };
	auto					   distance = QDPF::EuclideanDistance<1.f>;
	QDPF::QuadtreeMapXSettings settings{
		{ 1, Terrain::Land },				   // e.g. soldiers
		{ 2, Terrain::Land },				   // e.g. tanks
		{ 1, Terrain::Land | Terrain::Water }, // e.g. seals
		{ 2, Terrain::Water },				   // e.g. boats
	};
	QDPF::QuadtreeMapX mx(w, h, distance, terrainChecker, settings);
	mx.Build();

	// Setup an A* path finder.
	QDPF::AStarPathFinder pf(mx);

	// Change terrain.
	grid[6][6] = Terrain::Water;
	mx.Update(6, 6);
	mx.Compute();

	// Resets the path finder.
	// Find path from (0,0) to (7,7), agent size is 1, we can only walk on { Land }.
	if (0 != pf.Reset(0, 0, 7, 7, 1, Terrain::Land))
	{
		std::cout << "reset failed!" << std::endl;
		return -1;
	}

	// ComputeNodeRoutes is much faster than ComputeGateRoutes to test whether the target is
	// reachable.

	std::cout << "node route path:" << std::endl;
	QDPF::NodePath nodePath;

	if (pf.ComputeNodeRoutes(nodePath).code != QDPF::ErrorCode::Ok)
	{
		std::cout << "unreachable!" << std::endl;
		return -1;
	}
	for (auto [node, cost] : nodePath)
	{
		std::cout << node->x1 << "," << node->y1 << " " << node->x2 << "," << node->y2
				  << " cost: " << cost << std::endl;
	}

	// Compute gate cell routes.
	std::cout << "collect route gate cell path..." << std::endl;
	QDPF::GatePath routes;

	// Using the computed nodePath will make the ComputeGateRoutes running much faster,
	// but less optimal.
	auto result = pf.ComputeGateRoutes(routes, nodePath);

	std::cout << "cost: " << result.cost << std::endl;

	for (auto [x, y, cost] : routes)
		std::cout << x << "," << y << " cost: " << cost << std::endl;

	std::cout << "collect detailed path..." << std::endl;

	// Fill the detailed path (straight lines).
	std::vector<std::pair<int, int>> path;
	QDPF::CellCollector				 collector1 = [&path](int x, int y) {
		 if (path.size())
		 {
			 // skip duplicates end of previous route.
			 auto [x2, y2] = path.back();
			 if ((x2 == x && y2 == y))
				 return;
		 }
		 path.push_back({ x, y });
	};
	auto [x, y, _] = routes[0];
	for (int i = 1; i < routes.size(); i++)
	{
		auto [x2, y2, _] = routes[i];
		QDPF::ComputeStraightLine(x, y, x2, y2, collector1);
		x = x2, y = y2;
	}

	// Print the path.
	for (auto [x, y] : path)
		std::cout << x << "," << y << std::endl;
	return 0;
}
