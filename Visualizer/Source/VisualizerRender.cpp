#include "QDPF.h"
#include "Visualizer.h"

using QDPF::Internal::IsInsideRectangle;

void Visualizer::RenderWorld()
{
	RenderHighlightedNodes();
	RenderGates();
	RenderGateGraph();
	RenderNodeGraph();
	RenderGrids();
	RenderQuadtreeNodes();
	RenderPathfindingDispatch();
}

void Visualizer::RenderGrids()
{
	// (i,j) is the cell's position.
	// (x,y) is the grid in pixel (in SDL coordinates).
	for (int i = 0, y = 0; i < map.h; ++i, y += map.gridSize)
	{
		for (int j = 0, x = 0; j < map.w; ++j, x += map.gridSize)
		{
			SDL_Rect grid = { x, y, map.gridSize, map.gridSize };
			SDL_Rect gridInner = { grid.x + 1, grid.y + 1, grid.w - 2, grid.h - 2 };
			RenderDrawRect(grid, LightGray);
			if (hideTerrainRenderings)
				continue;
			// Its terrain value or changing to value
			auto v = map.grids[i][j];
			if (map.changes[i][j] != 0)
				v = map.changes[i][j];
			switch (v)
			{
				case Terrain::Building:
					RenderFillRect(gridInner, Red);
					break;
				case Terrain::Water:
					RenderFillRect(gridInner, Blue);
					break;
			}
		}
	}
}

// render quadtree nodes of current map.
void Visualizer::RenderQuadtreeNodes()
{
	if (hideNodeBorders)
		return;
	auto mp = GetCurrentQuadtreeMapByAgent();
	if (mp != nullptr)
	{
		QDPF::Internal::QdNodeVisitor c1 = [this](const QDPF::Internal::QdNode* node) {
			int x = node->x1 * map.gridSize;
			int y = node->y1 * map.gridSize;
			int w = (node->x2 - node->x1 + 1) * map.gridSize;
			int h = (node->y2 - node->y1 + 1) * map.gridSize;
			// Outer liner rectangle (border width 2)
			SDL_Rect rect1 = { x, y, w, h };
			SDL_Rect rect2 = { x + 1, y + 1, w - 2, h - 2 };
			RenderDrawRect(rect1, Black);
			RenderDrawRect(rect2, Black);
		};
		mp->Nodes(c1);
	}
}

void Visualizer::RenderGates()
{
	if (hideGateCellHighlights)
		return;
	auto mp = GetCurrentQuadtreeMapByAgent();
	if (mp != nullptr)
	{
		// Gates.
		QDPF::Internal::GateVisitor visitor = [this, &mp](const QDPF::Internal::Gate* gate) {
			auto [x1, y1] = mp->UnpackXY(gate->a);
			int		 x3 = x1 * map.gridSize + 1;
			int		 y3 = y1 * map.gridSize + 1;
			SDL_Rect rect = { x3, y3, map.gridSize - 2, map.gridSize - 2 };
			RenderFillRect(rect, LightPurple);
		};

		mp->Gates(visitor);
	}
}

void Visualizer::RenderGateGraph()
{
	if (!showGateGraph)
		return;

	auto mp = GetCurrentQuadtreeMapByAgent();
	if (mp != nullptr)
	{
		auto&							 graph = mp->GetGateGraph();
		QDPF::Internal::EdgeVisitor<int> visitor = [this, mp](int u, int v, int cost) {
			auto [x1, y1] = mp->UnpackXY(u);
			auto [x2, y2] = mp->UnpackXY(v);
			RenderDrawLineBetweenCells(x1, y1, x2, y2, Blue);
		};
		graph.ForEachEdge(visitor);
	}
}

void Visualizer::RenderNodeGraph()
{
	if (!showNodeGraph)
		return;

	auto mp = GetCurrentQuadtreeMapByAgent();
	if (mp != nullptr)
	{
		auto&												 graph = mp->GetNodeGraph();
		QDPF::Internal::EdgeVisitor<QDPF::Internal::QdNode*> visitor =
			[this, mp](QDPF::Internal::QdNode* u, QDPF::Internal::QdNode* v, int cost) {
				auto x1 = (u->x1 + u->x2) / 2, y1 = (u->y1 + u->y2) / 2;
				auto x2 = (v->x1 + v->x2) / 2, y2 = (v->y1 + v->y2) / 2;
				RenderDrawLineBetweenCells(x1, y1, x2, y2, Red);
			};
		graph.ForEachEdge(visitor);
	}
}

void Visualizer::RenderPathfindingDispatch()
{
	switch (pathfinderFlag)
	{
		case PathFinderFlag::AStar:
			RenderPathfindingAStar();
			return;
		case PathFinderFlag::FlowField:
			RenderPathfindingFlowField();
			return;
	}
}

void Visualizer::RenderHighlightedNodes()
{
	// render highlighted nodes with background at first.
	switch (state)
	{
		case State::AStarNodePathComputed:
			[[fallthrough]];
		case State::AStarGatePathComputed:
			[[fallthrough]];
		case State::AStarFinalPathComputed:
			RenderHighlightedNodesAstar();
			break;
		case State::FlowFieldNodeLevelComputed:
			RenderHighlightedNodesFlowField();
			[[fallthrough]];
		case State::FlowFieldGateLevelComputed:
			[[fallthrough]];
		case State::FlowFieldFinalLevelComputed:
			RenderHighlightedNodesFlowField();
			break;
		default:
			break; // avoiding warning
	}
}

void Visualizer::RenderHighlightedNodesAstar()
{
	for (auto [node, cost] : astar.nodePath)
	{
		int		 x1 = node->x1, y1 = node->y1, x2 = node->x2, y2 = node->y2;
		int		 w = (x2 - x1 + 1) * map.gridSize;
		int		 h = (y2 - y1 + 1) * map.gridSize;
		int		 x = x1 * map.gridSize;
		int		 y = y1 * map.gridSize;
		SDL_Rect rect = { x, y, w, h };
		SDL_Rect inner = { x + 1, y + 1, w - 2, h - 2 };
		RenderFillRect(inner, LightOrange);
	}
}

void Visualizer::RenderHighlightedNodesFlowField()
{
	for (auto& [node, p] : flowfield.nodeFlowField.GetUnderlyingMap())
	{
		auto [next, cost] = p;
		// highlight node with background orange
		int		 x1 = node->x1, y1 = node->y1, x2 = node->x2, y2 = node->y2;
		int		 w = (x2 - x1 + 1) * map.gridSize;
		int		 h = (y2 - y1 + 1) * map.gridSize;
		int		 x = x1 * map.gridSize;
		int		 y = y1 * map.gridSize;
		SDL_Rect rect = { x, y, w, h };
		SDL_Rect inner = { x + 1, y + 1, w - 2, h - 2 };
		RenderFillRect(inner, LightOrange);
	}
}

void Visualizer::RenderPathfindingAStar()
{
	int agentSizeInPixels = agent.size * map.gridSize;

	switch (state)
	{
		case State::AStarWaitTarget:
			RenderFillCell(astar.x1, astar.y1, Green); // start
			break;
		case State::AStarWaitCompution:
			[[fallthrough]];
		case State::AStarNodePathComputed:
			RenderFillCell(astar.x1, astar.y1, Green); // start
			RenderFillCell(astar.x2, astar.y2, Green); // target
			// NOTE: nodepath already render in renderHighlightedNodesAstar.
			break;
		case State::AStarGatePathComputed:
			// draw gate route cells.
			// start and target are included in gate path.
			for (const auto [x, y, _] : astar.gatePath)
			{
				RenderFillCell(x, y, Green);
			}
			break;
		case State::AStarFinalPathComputed: //
			for (const auto [x, y] : astar.finalPath)
			{
				RenderFillAgent(x, y);
			}

			RenderPathfindingAStarNaive();
			break;
		default:
			return; // do nothing
	}
}

void Visualizer::RenderPathfindingAStarNaive()
{
	if (astarNaive.path.empty())
		return;
	for (const auto [x, y] : astarNaive.path)
	{
		RenderFillAgent(x, y, DarkGreen);
	}
}

void Visualizer::RenderPathfindingFlowField()
{
	auto drawQrangeRectangle = [this]() {
		// render the qrange rect
		SDL_Rect qrange{
			flowfield.qrange.x1 * map.gridSize,
			flowfield.qrange.y1 * map.gridSize,
			(flowfield.qrange.x2 - flowfield.qrange.x1 + 1) * map.gridSize,
			(flowfield.qrange.y2 - flowfield.qrange.y1 + 1) * map.gridSize,
		};
		SDL_Rect inner{ qrange.x + 1, qrange.y + 1, qrange.w - 2, qrange.h - 2 };
		RenderDrawRect(qrange, Green);
		RenderDrawRect(inner, Green);
	};

	switch (state)
	{
		case State::FlowFieldWaitQrangeRightBottom:
			RenderFillCell(flowfield.qrange.x1, flowfield.qrange.y1, Green); // qrange.left-top
			return;
		case State::FlowFieldWaitTarget:
			drawQrangeRectangle();
			RenderFillCell(flowfield.qrange.x1, flowfield.qrange.y1, Green); // qrange.left-top
			RenderFillCell(flowfield.qrange.x2, flowfield.qrange.y2, Green); // qrange.right-bottom
			return;
		case State::FlowFieldWaitCompution:
			[[fallthrough]];
		case State::FlowFieldNodeLevelComputed:
			// NOTE the highlighting of the node field are already done in
			// renderHighlightedNodesFlowField
			drawQrangeRectangle();
			RenderFillCell(flowfield.qrange.x1, flowfield.qrange.y1, Green); // qrange.left-top
			RenderFillCell(flowfield.qrange.x2, flowfield.qrange.y2, Green); // qrange.right-bottom
			RenderFillCell(flowfield.x2, flowfield.y2, Green);				 // target
			return;
		case State::FlowFieldGateLevelComputed:
			drawQrangeRectangle();
			RenderPathFindingFlowFieldGateField();
			RenderPathFindingFlowFieldGateNextsLines();
			return;
		case State::FlowFieldFinalLevelComputed:
			drawQrangeRectangle();
			RenderPathFindingFlowFieldGateNextsLines();
			RenderFillCell(flowfield.x2, flowfield.y2, Green); // target
			RenderPathFindingFlowFieldFinalField();
			return;
		default:

			return; // do nothing
	}
}

void Visualizer::RenderPathFindingFlowFieldGateField()
{
	// draw gate cells on the flowfield.
	for (auto& [gate, p] : flowfield.gateFlowField.GetUnderlyingMap())
	{
		auto [next, cost] = p;
		auto [x, y] = gate;
		auto [xNext, yNext] = next;
		RenderFillCell(x, y, Green);
	}
}

void Visualizer::RenderPathFindingFlowFieldGateNextsLines()
{
	if (showNaiveFlowFieldResults)
		return;
	if (!renderFlowFieldGateNextLines)
		return;
	for (auto& [gate, p] : flowfield.gateFlowField.GetUnderlyingMap())
	{
		auto [next, cost] = p;
		auto [x, y] = gate;
		auto [xNext, yNext] = next;
		RenderDrawLineBetweenCells(x, y, xNext, yNext, Black);
	}
}

void Visualizer::RenderPathFindingFlowFieldFinalField()
{
	const auto& testPaths =
		showNaiveFlowFieldResults ? flowfieldNaive.testPaths : flowfield.testPaths;

	if (testPaths.size())
	{
		for (const auto& testPath : testPaths)
		{
			for (auto [x, y] : testPath)
			{
				RenderFillAgent(x, y);
			}
		}
	}

	const auto& finalFlowField =
		showNaiveFlowFieldResults ? flowfieldNaive.finalFlowField : flowfield.finalFlowField;

	for (auto& [u, p] : finalFlowField.GetUnderlyingMap())
	{
		auto [v, cost] = p;
		auto [x, y] = u;
		auto [x1, y1] = v;
		if (u == v)
			continue;
		if (IsInsideRectangle(x, y, flowfield.qrange))
		{
			int dx = (x1 - x), dy = (y1 - y);
			int d = (dy + 1) * 3 + (dx + 1);
			if (!(d >= 0 && d <= 8 && d != 4))
				continue;
			int k = ARROWS_DIRECTIONS[d];
			int w = arrows.w[k], h = arrows.h, offset = arrows.offset[k];
			w = std::min(w, map.gridSize);
			h = std::min(h, map.gridSize);
			SDL_Rect dst = { x * map.gridSize + std::max(0, map.gridSize / 2 - w / 2),
				y * map.gridSize + std::max(0, map.gridSize / 2 - h / 2), w, h };
			SDL_Rect src = { offset, 0, w, h };
			RenderCopy(arrows.texture, src, dst);
		}
	}
}
