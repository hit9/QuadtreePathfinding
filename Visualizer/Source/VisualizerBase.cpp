#include <spdlog/spdlog.h>

#include <chrono>

#include "QDPF.h"
#include "Visualizer.h"

using QDPF::Internal::IsInsideRectangle;

// ~~~~~~~~~~ Agent ~~~~~~~~~~

void Agent::Reset()
{
	size = 1;
	capability = Terrain::Land;
}

// ~~~~~~~~~~ Map ~~~~~~~~~~

Map::Map(int w, int h, int gridSize, int step)
	: w(w), h(h), gridSize(gridSize), step(step)
{
	// Inits the map, by default:
	// 1. the center are water.
	// 2. there's 2 walls.
	// 3. other cells are land.
	memset(grids, 0, sizeof grids);
	memset(changes, 0, sizeof changes);

	QDPF::Rectangle center{ 2 * w / 7, 2 * h / 7, w * 5 / 7, h * 5 / 7 };
	QDPF::Rectangle wall1{ center.x2 + 4, center.y1, center.x2 + 4, center.y2 + 4 };
	QDPF::Rectangle wall2{ center.x1, center.y2 + 4, center.x2 + 4, center.y2 + 4 };
	QDPF::Rectangle wall3{ center.x1 - 4, center.y1 - 4, center.x1 - 4, center.y2 };
	QDPF::Rectangle wall4{ center.x1 - 4, center.y1 - 4, center.x2, center.y1 - 4 };

	for (int y = 0; y < h; ++y)
	{
		for (int x = 0; x < w; ++x)
		{
			if (IsInsideRectangle(x, y, center))
				grids[y][x] = Terrain::Water;
			else if (IsInsideRectangle(x, y, wall1))
				grids[y][x] = Terrain::Building;
			else if (IsInsideRectangle(x, y, wall2))
				grids[y][x] = Terrain::Building;
			else if (IsInsideRectangle(x, y, wall3))
				grids[y][x] = Terrain::Building;
			else if (IsInsideRectangle(x, y, wall4))
				grids[y][x] = Terrain::Building;
			else
				grids[y][x] = Terrain::Land;
		}
	}
}

Map::~Map()
{
	delete qmx;
	qmx = nullptr;
	delete naiveMap;
	;
}

void Map::Build()
{
	// Build QuadtreeMapX.
	auto					   stepf = (step == -1) ? [](int z) -> int { return z / 8 + 1; } : nullptr;
	QDPF::QuadtreeMapXSettings settings{
		{ 1, Terrain::Land },
		{ 2, Terrain::Land },
		{ 3, Terrain::Land },
		{ 1, Terrain::Water },
		{ 2, Terrain::Water },
		{ 3, Terrain::Water },
		{ 1, Terrain::Land | Terrain::Water },
		{ 2, Terrain::Land | Terrain::Water },
		{ 3, Terrain::Land | Terrain::Water },
	};
	auto distance = QDPF::EuclideanDistance<1.f>;
	auto terrianChecker = [this](int x, int y) { return grids[y][x]; };
	auto clearanceFieldKind = (options.clearanceFieldFlag == 0)
		? QDPF::ClearanceFieldKind::TrueClearanceField
		: QDPF::ClearanceFieldKind::BrushfireClearanceField;
	qmx = new QDPF::QuadtreeMapX(w, h, distance, terrianChecker, settings, step, stepf, -1, -1,
		clearanceFieldKind);
	qmx->Build();
	spdlog::info("Build quadtree maps done");

	// Build naive map.
	auto isObstacle = [this](int x, int y) { return grids[y][x] != Terrain::Land; };
	naiveMap = new QDPF::Naive::NaiveGridMap(w, h, isObstacle, distance);
	naiveMap->Build();
	spdlog::info("Build naive map done");
}

void Map::WantChangeTerrain(const Cell& cell, Terrain to)
{
	auto [x, y] = cell;
	changes[y][x] = to;
}

void Map::ApplyChangeTerrain(const std::vector<Cell>& cells)
{
	for (auto [x, y] : cells)
	{
		grids[y][x] = changes[y][x];
		changes[y][x] = 0;
		qmx->Update(x, y);
		naiveMap->Update(x, y);
	}
	qmx->Compute();
}

void Map::ClearAllTerrains()
{
	for (int y = 0; y < h; ++y)
	{
		for (int x = 0; x < w; ++x)
		{
			grids[y][x] = Terrain::Land;
			qmx->Update(x, y);
			naiveMap->Update(x, y);
		}
	}
	qmx->Compute();
}

// ~~~~~~~~~~ AStarContext ~~~~~~~~~~

void NaiveAStarContext::Reset()
{
	timeCost = std::chrono::microseconds(0);
	path.clear();
}

void AStarContext::InitPf(QDPF::QuadtreeMapX* qmx)
{
	pf = new QDPF::AStarPathFinder(*qmx);
}

AStarContext::~AStarContext()
{
	delete pf;
	pf = nullptr;
}

void AStarContext::ClearResults()
{
	nodePath.clear(), gatePath.clear(), finalPath.clear();
}

void AStarContext::Reset()
{
	ClearResults();
	x1 = y1 = x2 = y2 = 0;
	isPfReset = false;
	timeCost = std::chrono::microseconds(0);
}

int AStarContext::ResetPf(int agentSize, int capabilities)
{
	if (isPfReset)
		return 0;
	auto startAt = std::chrono::high_resolution_clock::now();
	auto ret = pf->Reset(x1, y1, x2, y2, agentSize, capabilities);
	isPfReset = true;
	auto endAt = std::chrono::high_resolution_clock::now();
	timeCost += std::chrono::duration_cast<std::chrono::microseconds>(endAt - startAt);
	return ret;
}

// ~~~~~~~~~~ FlowFieldContext ~~~~~~~~~~

void NaiveFlowFieldContext::Reset()
{
	timeCost = std::chrono::microseconds(0);
	finalFlowField.Clear();
	testPaths.clear();
}

FlowFieldContext::~FlowFieldContext()
{
	delete pf;
	pf = nullptr;
}

void FlowFieldContext::InitPf(QDPF::QuadtreeMapX* qmx)
{
	pf = new QDPF::FlowFieldPathFinder(*qmx);
}

int FlowFieldContext::ResetPf(int agentSize, int capabilities)
{
	if (isPfReset)
		return 0;
	auto startAt = std::chrono::high_resolution_clock::now();
	auto ret = pf->Reset(x2, y2, qrange, agentSize, capabilities);
	isPfReset = true;
	auto endAt = std::chrono::high_resolution_clock::now();
	timeCost += std::chrono::duration_cast<std::chrono::microseconds>(endAt - startAt);
	return ret;
}

void FlowFieldContext::ClearResults()
{
	nodeFlowField.Clear(), gateFlowField.Clear(), finalFlowField.Clear();
	testPaths.clear();
}

void FlowFieldContext::Reset()
{
	ClearResults();
	x2 = y2 = 0;
	qrange = { 0, 0, 0, 0 };
	isPfReset = false;
	timeCost = std::chrono::microseconds(0);
}

// ~~~~~~~~~~ Camera ~~~~~~~~~~

Camera::Camera(int w, int h, int mpw, int mph)
	: w(w), h(h), mpw(mpw), mph(mph) {}

void Camera::MoveUp(int k)
{
	dy -= k;
}
void Camera::MoveDown(int k)
{
	dy += k;
}
void Camera::MoveLeft(int k)
{
	dx -= k;
}
void Camera::MoveRight(int k)
{
	dx += k;
}
void Camera::MoveToLeftMost()
{
	x = 0;
}
void Camera::MoveToRightMost()
{
	x = mpw - w - 1;
}

void Camera::Update()
{
	if (dx == 0 && dy == 0)
		return;
	x += dx;
	y += dy;
	// bounds check.
	x = std::max(0, x);
	y = std::max(0, y);
	x = std::min(x, mpw - w - 1);
	y = std::min(y, mph - h - 1);
	// clears the delta changes.
	dx = dy = 0;
}
