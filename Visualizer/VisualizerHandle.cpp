#include <fmt/format.h>
#include <imgui.h>
#include <imgui_impl_sdl2.h>
#include <imgui_impl_sdlrenderer2.h>
#include <spdlog/spdlog.h>

#include <chrono>
#include <string_view>
#include <vector>

#include "QDPF.h"
#include "Visualizer.h"

using qdpf::Internal::IsInsideRectangle;

std::string StateToString(State state)
{
	switch (state)
	{
		case State::Idle:
			return "Idle";
		case State::DrawingBuildings:
			return "Changing Terrain (Building <=> Land)";
		case State::DrawingWaters:
			return "Changing Terrain (Water <=> Land)";
		case State::AStarWaitStart:
			return "(A*) Waiting Input Start Cell";
		case State::AStarWaitTarget:
			return "(A*) Waiting Input Target Cell";
		case State::AStarWaitCompution:
			return "(A*) Waiting Compution";
		case State::AStarNodePathComputed:
			return "(A*) Node Path Computed";
		case State::AStarGatePathComputed:
			return "(A*) Gate Path Computed";
		case State::AStarFinalPathComputed:
			return "(A*) Final Path Computed";
		case State::FlowFieldWaitQrangeLeftTop:
			return "(FlowField) Wait Input Query Range (left-top)";
		case State::FlowFieldWaitQrangeRightBottom:
			return "(FlowField) Wait Input Query Range (right-bottom)";
		case State::FlowFieldWaitTarget:
			return "(FlowField) Wait Input Target";
		case State::FlowFieldWaitCompution:
			return "(FlowField) Wait Compution";
		case State::FlowFieldNodeLevelComputed:
			return "(FlowField) Node Flow Field Computed";
		case State::FlowFieldGateLevelComputed:
			return "(FlowField) Gate Flow Field Computed";
		case State::FlowFieldFinalLevelComputed:
			return "(FlowField) Final Flow Field Computed";
		default:
			return "Unknown";
	}
}

void Visualizer::Start()
{
	auto& io = ImGui::GetIO();
	while (!stop)
	{
		// quit on -1
		handleInputs();
		if (stop)
			break;

		// update camera.
		if (camera != nullptr)
			camera->Update();

		handleLogics();

		// starting a new imgui frame
		ImGui_ImplSDLRenderer2_NewFrame();
		ImGui_ImplSDL2_NewFrame();
		ImGui::NewFrame();

		// render imgui panel to imgui's buffer.
		renderImguiPanel();
		ImGui::Render();

		// clears SDL buffer (white background)
		SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
		SDL_RenderClear(renderer);

		// render the world
		renderWorld();

		// pass the imgui's render buffer to SDL's renderer.
		SDL_RenderSetScale(renderer, io.DisplayFramebufferScale.x, io.DisplayFramebufferScale.y);
		ImGui_ImplSDLRenderer2_RenderDrawData(ImGui::GetDrawData(), renderer);

		// present sdl buffer.
		SDL_RenderPresent(renderer);
		SDL_Delay(50); // ms
	}
}

// reset path finder compution results.
void Visualizer::reset()
{
	state = State::Idle;
	astar.Reset();
	astarNaive.Reset();
	flowfield.Reset();
	flowfieldNaive.Reset();
	changeTo = Terrain::Building;
	changingTerrainCells.clear();
	showClearAllTerrainsConfirm = false;
	setMessageHint("Reset done!", ImGreen);
}

void Visualizer::setMessageHint(std::string_view message, const ImVec4& color)
{
	messageHint = message;
	messageHintColor = color;
}

void Visualizer::handleStartDrawBuildings()
{
	reset();
	state = State::DrawingBuildings;
	changeTo = Terrain::Building;
	setMessageHint("click or drag mouse to draw buildings!", ImGreen);
}

void Visualizer::handleStartDrawWater()
{
	reset();
	state = State::DrawingWaters;
	changeTo = Terrain::Water;
	setMessageHint("click or drag mouse to draw water!", ImGreen);
}

void Visualizer::pushTerrainChanges(const Cell& cell)
{
	auto [x, y] = cell;
	// invert between land and changeTo.
	auto to = (map.grids[y][x] == Terrain::Land) ? changeTo : Terrain::Land;
	if (IsInsideRectangle(x, y, 0, 0, map.w - 1, map.h - 1) && !map.changes[y][x])
	{
		changingTerrainCells.push_back(cell);
		map.WantChangeTerrain(cell, to);
	}
}

void Visualizer::applyTerrainChanges()
{
	if (changingTerrainCells.empty())
		return;
	map.ApplyChangeTerrain(changingTerrainCells);
	changingTerrainCells.clear();
}

void Visualizer::handleSwitchPathFinderHandler(PathFinderFlag to)
{
	if (to == pathfinderFlag)
		return;
	reset();
	pathfinderFlag = to;
	setMessageHint("Old states reset done and pathfinder switched.", ImGreen);
}

void Visualizer::handleChangeAgentSize(int to)
{
	if (agent.size == to)
		return;
	reset();
	auto oldAgentSize = agent.size;
	agent.size = to;
	if (getCurrentQuadtreeMapByAgent() == nullptr)
	{
		setMessageHint("Failed to change agent size, stay unchanged!", ImRed);
		agent.size = oldAgentSize;
	}
	else
		setMessageHint("Agent size changed (current quadtree map switched)!", ImGreen);
}

void Visualizer::handleChangeAgentCompability(int to)
{
	if (agent.capability == to)
		return;
	reset();
	auto oldCapability = agent.capability;
	agent.capability = to;
	if (getCurrentQuadtreeMapByAgent() == nullptr)
	{
		setMessageHint("Failed to change agent capability, stay unchanged!", ImRed);
		agent.size = oldCapability;
	}
	else
		setMessageHint("Agent capability changed (current quadtree map switched)!", ImGreen);
}

void Visualizer::handleClearAllTerrains()
{
	reset();
	map.ClearAllTerrains();
	setMessageHint("Map is cleared.", ImGreen);
}

// Returns the pointer the internal quadtree map of which current agent is using.
const qdpf::Internal::QuadtreeMap* Visualizer::getCurrentQuadtreeMapByAgent() const
{
	return map.qmx->Get(agent.size, agent.capability);
}

// returns cell at the position for (x,y) in the camera.
Cell Visualizer::getCellAtPixelPosition(int x, int y) const
{
	return { (x + camera->x) / map.gridSize, (y + camera->y) / map.gridSize };
}

void Visualizer::handleAstarInputBegin()
{
	if (pathfinderFlag != PathFinderFlag::AStar)
	{
		pathfinderFlag = PathFinderFlag::AStar;
	}
	if (state != State::Idle)
		reset();
	state = State::AStarWaitStart;
	setMessageHint("A*: waiting to click a start cell", ImGreen);
}

void Visualizer::computeAstarNodePath()
{
	if (state != State::AStarWaitCompution)
	{
		setMessageHint("invalid state", ImRed);
		return;
	}
	if (0 != astar.ResetPf(agent.size, agent.capability))
	{
		setMessageHint("internal error: astar reset failure", ImRed);
		return;
	}

	if (astar.nodePath.size())
		astar.nodePath.clear();

	std::chrono::high_resolution_clock::time_point startAt, endAt;

	startAt = std::chrono::high_resolution_clock::now();
	int cost = astar.pf->ComputeNodeRoutes(astar.nodePath);
	endAt = std::chrono::high_resolution_clock::now();

	state = State::AStarNodePathComputed;
	if (cost == -1)
	{
		setMessageHint("A*: unreachable!", ImRed);
		return;
	}

	auto timeCost = std::chrono::duration_cast<std::chrono::microseconds>(endAt - startAt);
	astar.timeCost += timeCost;

	setMessageHint(fmt::format("A*: Node path computed! cost {}us (cumulative {}us); Next we can "
							   "click button < Compute Gate Path >.",
					   timeCost.count(), astar.timeCost.count()),
		ImGreen);
}

void Visualizer::computeAstarGatePath()
{
	if (state != State::AStarWaitCompution && state != State::AStarNodePathComputed)
	{
		setMessageHint("invalid state", ImRed);
		return;
	}
	if (0 != astar.ResetPf(agent.size, agent.capability))
	{
		setMessageHint("internal error: astar reset failure", ImRed);
		return;
	}

	if (astar.gatePath.size())
		astar.gatePath.clear();

	std::chrono::high_resolution_clock::time_point startAt, endAt;

	startAt = std::chrono::high_resolution_clock::now();
	int cost = astar.pf->ComputeGateRoutes(astar.gatePath, astar.nodePath);
	endAt = std::chrono::high_resolution_clock::now();

	state = State::AStarGatePathComputed;
	if (cost == -1)
	{
		setMessageHint("A*: unreachable!", ImRed);
		return;
	}
	auto timeCost = std::chrono::duration_cast<std::chrono::microseconds>(endAt - startAt);
	astar.timeCost += timeCost;

	setMessageHint(fmt::format("A*: Gate path computed! useNodePath: {}  cost {}us (cumulative "
							   "{}us); Next we can click button "
							   "< Compute Final Path >.",
					   astar.nodePath.size() > 0, timeCost.count(), astar.timeCost.count()),
		ImGreen);
}

void Visualizer::computeAstarFinalPath()
{
	if (state != State::AStarGatePathComputed)
	{
		setMessageHint("invalid state", ImRed);
		return;
	}
	if (astar.gatePath.empty())
	{
		setMessageHint("A*: empty gate route cells!", ImRed);
		return;
	}
	if (astar.finalPath.size())
		astar.finalPath.clear();

	std::chrono::high_resolution_clock::time_point startAt, endAt;

	qdpf::CellCollector collector = [this](int x, int y) {
		if (astar.finalPath.size())
		{
			auto [x2, y2] = astar.finalPath.back();
			if ((x2 == x && y2 == y))
				return;
		}
		astar.finalPath.push_back({ x, y });
	};

	startAt = std::chrono::high_resolution_clock::now();

	auto [x, y, _] = astar.gatePath[0];
	for (int i = 1; i < astar.gatePath.size(); i++)
	{
		auto [x2, y2, _] = astar.gatePath[i];
		qdpf::ComputeStraightLine(x, y, x2, y2, collector);
		x = x2, y = y2;
	}
	state = State::AStarFinalPathComputed;
	endAt = std::chrono::high_resolution_clock::now();

	auto timeCost = std::chrono::duration_cast<std::chrono::microseconds>(endAt - startAt);
	astar.timeCost += timeCost;

	setMessageHint(fmt::format("A*: final path computed! cost {}us (cumulative {}us); Click button "
							   "< Reset > to clear these results.",
					   timeCost.count(), astar.timeCost.count()),
		ImGreen);
}

void Visualizer::computeAStarNaive()
{
	if (state != State::AStarFinalPathComputed)
	{
		setMessageHint("invalid state; please compute astar on quadtree maps first.", ImRed);
		return;
	}

	if (agent.size != COST_UNIT || agent.capability != Terrain::Land)
	{
		setMessageHint("NaiveAstar works only for agent {1x1,Land}", ImRed);
		return;
	}

	if (astarNaive.path.size())
		astarNaive.path.clear();

	std::chrono::high_resolution_clock::time_point startAt, endAt;

	startAt = std::chrono::high_resolution_clock::now();
	auto ret =
		astarNaive.pf.Compute(map.naiveMap, astar.x1, astar.y1, astar.x2, astar.y2, astarNaive.path);
	endAt = std::chrono::high_resolution_clock::now();
	if (-1 == ret)
	{
		setMessageHint("Naive A*: failed!", ImRed);
		return;
	}

	auto timeCost = std::chrono::duration_cast<std::chrono::microseconds>(endAt - startAt);

	astarNaive.timeCost = std::chrono::microseconds(0);
	astarNaive.timeCost += timeCost;

	setMessageHint(fmt::format("NaiveAstar: path computed ,cost {}us", timeCost.count()), ImGreen);
}

void Visualizer::computeFlowFieldNaive()
{
	if (state != State::FlowFieldFinalLevelComputed)
	{
		setMessageHint("invalid state; please compute final flow field on quadtree maps first.",
			ImRed);
		return;
	}

	if (agent.size != COST_UNIT || agent.capability != Terrain::Land)
	{
		setMessageHint("NaiveFlowField works only for agent {1x1,Land}", ImRed);
		return;
	}

	if (flowfieldNaive.finalFlowField.Size())
		flowfieldNaive.finalFlowField.Clear();

	std::chrono::high_resolution_clock::time_point startAt, endAt;

	startAt = std::chrono::high_resolution_clock::now();

	auto ret = flowfieldNaive.pf.Compute(map.naiveMap, flowfield.x2, flowfield.y2, flowfield.qrange,
		flowfieldNaive.finalFlowField);
	if (-1 == ret)
	{
		setMessageHint("Naive FlowField: failed!", ImRed);
		return;
	}

	endAt = std::chrono::high_resolution_clock::now();

	auto timeCost = std::chrono::duration_cast<std::chrono::microseconds>(endAt - startAt);
	flowfieldNaive.timeCost = std::chrono::microseconds(0);
	flowfieldNaive.timeCost += timeCost;

	setMessageHint(fmt::format("NaiveFlowField: path computed ,cost {}us", timeCost.count()),
		ImGreen);
}

void Visualizer::handleFlowFieldInputQueryRangeBegin()
{
	if (pathfinderFlag != PathFinderFlag::FlowField)
	{
		pathfinderFlag = PathFinderFlag::FlowField;
	}
	if (state != State::Idle)
		reset();
	state = State::FlowFieldWaitQrangeLeftTop;
	setMessageHint("FlowField: waiting to click a left-top cell", ImGreen);
}

void Visualizer::computeNodeFlowField()
{
	if (state != State::FlowFieldWaitCompution)
	{
		setMessageHint("invalid state", ImRed);
		return;
	}
	if (0 != flowfield.ResetPf(agent.size, agent.capability))
	{
		setMessageHint("internal error: flowfield reset failure", ImRed);
		return;
	}

	if (flowfield.nodeFlowField.Size())
		flowfield.nodeFlowField.Clear();

	std::chrono::high_resolution_clock::time_point startAt, endAt;

	startAt = std::chrono::high_resolution_clock::now();
	int ret = flowfield.pf->ComputeNodeFlowField(flowfield.nodeFlowField);
	endAt = std::chrono::high_resolution_clock::now();

	state = State::FlowFieldNodeLevelComputed;
	if (ret == -1)
	{
		setMessageHint("FlowField: unreachable!", ImRed);
		return;
	}

	auto timeCost = std::chrono::duration_cast<std::chrono::microseconds>(endAt - startAt);
	flowfield.timeCost += timeCost;

	setMessageHint(fmt::format("FlowField: Node flowfield computed! cost {}us (cumulative {}us); "
							   "Next we can click button < "
							   "Compute Gate Flow Field  >.",
					   timeCost.count(), flowfield.timeCost.count()),
		ImGreen);
}

void Visualizer::computeGateFlowField()
{
	if (state != State::FlowFieldWaitCompution && state != State::FlowFieldNodeLevelComputed)
	{
		setMessageHint("invalid state", ImRed);
		return;
	}

	if (0 != flowfield.ResetPf(agent.size, agent.capability))
	{
		setMessageHint("internal error: flowfield reset failure", ImRed);
		return;
	}

	std::chrono::high_resolution_clock::time_point startAt, endAt;

	if (flowfield.gateFlowField.Size())
		flowfield.gateFlowField.Clear();

	startAt = std::chrono::high_resolution_clock::now();
	int ret = flowfield.pf->ComputeGateFlowField(flowfield.gateFlowField, flowfield.nodeFlowField);
	endAt = std::chrono::high_resolution_clock::now();

	state = State::FlowFieldGateLevelComputed;

	if (-1 == ret)
	{
		setMessageHint("FlowField: unreachable!", ImRed);
		return;
	}

	auto timeCost = std::chrono::duration_cast<std::chrono::microseconds>(endAt - startAt);
	flowfield.timeCost += timeCost;

	setMessageHint(
		fmt::format("Flowfield:: Gate flow field computed! useNodeFlowField: {}  cost {}us "
					"(cumulative {}us); Next "
					"we can click button "
					"< Compute Final Flow Field >.",
			flowfield.nodeFlowField.Size() > 0, timeCost.count(),
			flowfield.timeCost.count()),
		ImGreen);
}

void Visualizer::computeFinalFlowField()
{
	if (state != State::FlowFieldGateLevelComputed)
	{
		setMessageHint("invalid state", ImRed);
		return;
	}

	std::chrono::high_resolution_clock::time_point startAt, endAt;

	if (flowfield.finalFlowField.Size())
		flowfield.finalFlowField.Clear();

	startAt = std::chrono::high_resolution_clock::now();
	int ret = flowfield.pf->ComputeFinalFlowField(flowfield.finalFlowField, flowfield.gateFlowField);
	endAt = std::chrono::high_resolution_clock::now();
	state = State::FlowFieldFinalLevelComputed;

	if (-1 == ret)
	{
		setMessageHint("FlowField: unreachable!", ImRed);
		return;
	}

	auto timeCost = std::chrono::duration_cast<std::chrono::microseconds>(endAt - startAt);
	flowfield.timeCost += timeCost;

	setMessageHint(
		fmt::format("Flowfield:: Final flow field computed!  cost {}us  (cumulative {}us)",
			timeCost.count(), flowfield.timeCost.count()),
		ImGreen);
}

void Visualizer::handleLogics()
{
	switch (state)
	{
		case State::FlowFieldFinalLevelComputed:
			handlePlayFlowFieldTestPath();
			break;
		default:
			break;
	}
}

void Visualizer::handlePlayFlowFieldTestPath()
{
	// (x2,y2) is the target
	int x2 = flowfield.x2, y2 = flowfield.y2;

	auto& testPaths = showNaiveFlowFieldResults ? flowfieldNaive.testPaths : flowfield.testPaths;
	auto& finalFlowField =
		showNaiveFlowFieldResults ? flowfieldNaive.finalFlowField : flowfield.finalFlowField;

	for (auto& p : testPaths)
	{
		if (p.empty())
			continue;

		// get current position
		auto [x3, y3] = p.back();

		if (x3 == x2 && y3 == y2)
		{
			// arrived the target.
			// back to start;
			auto [x1, y1] = p[0];
			p.clear();
			p.push_back({ x1, y1 });
			return;
		}

		// Is inside the rect?
		if (IsInsideRectangle(x3, y3, flowfield.qrange))
		{
			// get next from the final flow field.
			const auto& next = finalFlowField.Next({ x3, y3 });
			p.push_back(next);
		}
	}
}
