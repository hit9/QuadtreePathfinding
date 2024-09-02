#include <fmt/format.h>
#include <imgui.h>
#include <imgui_impl_sdl2.h>
#include <imgui_impl_sdlrenderer2.h>

#include <algorithm>
#include <chrono>
#include <string_view>
#include <unordered_map>
#include <vector>

#include "visualizer.hpp"

std::string StateToString(State state) {
  switch (state) {
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

void Visualizer::Start() {
  auto& io = ImGui::GetIO();
  while (!stop) {
    // quit on -1
    handleInputs();
    if (stop) break;

    // update camera.
    if (camera != nullptr) camera->Update();

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
    SDL_Delay(50);  // ms
  }
}

// reset path finder compution results.
void Visualizer::reset() {
  state = State::Idle;
  astar.Reset();
  flowfield.Reset();
  changeTo = Terrain::Building;
  changingTerrainCells.clear();
  showClearAllTerrainsConfirm = false;
  setMessageHint("Reset done!", ImGreen);
}

void Visualizer::setMessageHint(std::string_view message, const ImVec4& color) {
  messageHint = message;
  messageHintColor = color;
}

void Visualizer::handleStartDrawBuildings() {
  reset();
  state = State::DrawingBuildings;
  changeTo = Terrain::Building;
  setMessageHint("click or drag mouse to draw buildings!", ImGreen);
}

void Visualizer::handleStartDrawWater() {
  reset();
  state = State::DrawingWaters;
  changeTo = Terrain::Water;
  setMessageHint("click or drag mouse to draw water!", ImGreen);
}

void Visualizer::pushTerrainChanges(const Cell& cell) {
  auto [x, y] = cell;
  // invert between land and changeTo.
  auto to = (map.grids[x][y] == Terrain::Land) ? changeTo : Terrain::Land;
  if (x >= 0 && x < map.h && y >= 0 && y < map.w && !map.changes[x][y]) {
    changingTerrainCells.push_back(cell);
    map.WantChangeTerrain(cell, to);
  }
}

void Visualizer::applyTerrainChanges() {
  if (changingTerrainCells.empty()) return;
  map.ApplyChangeTerrain(changingTerrainCells);
  changingTerrainCells.clear();
}

void Visualizer::handleSwitchPathFinderHandler(PathFinderFlag to) {
  if (to == pathfinderFlag) return;
  reset();
  pathfinderFlag = to;
  setMessageHint("Old states reset done and pathfinder switched.", ImGreen);
}

void Visualizer::handleChangeAgentSize(int to) {
  if (agent.size == to) return;
  reset();
  auto oldAgentSize = agent.size;
  agent.size = to;
  if (getCurrentQuadtreeMapByAgent() == nullptr) {
    setMessageHint("Failed to change agent size, stay unchanged!", ImRed);
    agent.size = oldAgentSize;
  } else
    setMessageHint("Agent size changed (current quadtree map switched)!", ImGreen);
}

void Visualizer::handleChangeAgentCompability(int to) {
  if (agent.capability == to) return;
  reset();
  auto oldCapability = agent.capability;
  agent.capability = to;
  if (getCurrentQuadtreeMapByAgent() == nullptr) {
    setMessageHint("Failed to change agent capability, stay unchanged!", ImRed);
    agent.size = oldCapability;
  } else
    setMessageHint("Agent capability changed (current quadtree map switched)!", ImGreen);
}

void Visualizer::handleClearAllTerrains() {
  reset();
  map.ClearAllTerrains();
  setMessageHint("Map is cleared.", ImGreen);
}

// Returns the pointer the internal quadtree map of which current agent is using.
const qdpf::internal::QuadtreeMap* Visualizer::getCurrentQuadtreeMapByAgent() const {
  return map.qmx->Get(agent.size, agent.capability);
}

// returns cell at the position for (x,y) in the camera.
Cell Visualizer::getCellAtPixelPosition(int x, int y) const {
  return {(y + camera->y) / map.gridSize, (x + camera->x) / map.gridSize};
}

void Visualizer::handleAstarInputBegin() {
  if (pathfinderFlag != PathFinderFlag::AStar) {
    pathfinderFlag = PathFinderFlag::AStar;
  }
  if (state != State::Idle) reset();
  state = State::AStarWaitStart;
  setMessageHint("A*: waiting to click a start cell", ImGreen);
}

void Visualizer::computeAstarNodePath() {
  if (state != State::AStarWaitCompution) {
    setMessageHint("invalid state", ImRed);
    return;
  }
  if (0 != astar.ResetPf(agent.size, agent.capability)) {
    setMessageHint("internal error: astar reset failure", ImRed);
    return;
  }
  std::chrono::high_resolution_clock::time_point startAt, endAt;

  startAt = std::chrono::high_resolution_clock::now();
  int cost = astar.pf->ComputeNodeRoutes();
  endAt = std::chrono::high_resolution_clock::now();

  state = State::AStarNodePathComputed;
  if (cost == -1) {
    setMessageHint("A*: unreachable!", ImRed);
    return;
  }

  if (astar.nodePath.size()) astar.nodePath.clear();
  qdpf::NodeVisitor visitor = [this](const qdpf::QdNode* node) { astar.nodePath.push_back(node); };
  if (astar.pf->NodePathSize()) astar.pf->VisitComputedNodeRoutes(visitor);

  setMessageHint(
      fmt::format(
          "A*: Node path computed! cost {}us ; Next we can click button < Compute Gate Path >.",
          std::chrono::duration_cast<std::chrono::microseconds>(endAt - startAt).count()),
      ImGreen);
}

void Visualizer::computeAstarGatePath() {
  if (state != State::AStarWaitCompution && state != State::AStarNodePathComputed) {
    setMessageHint("invalid state", ImRed);
    return;
  }
  if (0 != astar.ResetPf(agent.size, agent.capability)) {
    setMessageHint("internal error: astar reset failure", ImRed);
    return;
  }

  std::chrono::high_resolution_clock::time_point startAt, endAt;

  if (astar.gatePath.size()) astar.gatePath.clear();
  bool useNodePath = astar.pf->NodePathSize() > 0;
  qdpf::CellCollector collector = [this](int x, int y) { astar.gatePath.push_back({x, y}); };

  startAt = std::chrono::high_resolution_clock::now();
  int cost = astar.pf->ComputeGateRoutes(collector, useNodePath);
  endAt = std::chrono::high_resolution_clock::now();

  state = State::AStarGatePathComputed;
  if (cost == -1) {
    setMessageHint("A*: unreachable!", ImRed);
    return;
  }

  setMessageHint(
      fmt::format("A*: Gate path computed! useNodePath: {}  cost {}us ; Next we can click button "
                  "< Compute Final Path >.",
                  useNodePath,
                  std::chrono::duration_cast<std::chrono::microseconds>(endAt - startAt).count()),
      ImGreen);
}

void Visualizer::computeAstarFinalPath() {
  if (state != State::AStarGatePathComputed) {
    setMessageHint("invalid state", ImRed);
    return;
  }
  if (astar.gatePath.empty()) {
    setMessageHint("A*: empty gate route cells!", ImRed);
    return;
  }
  if (astar.finalPath.size()) astar.finalPath.clear();

  std::chrono::high_resolution_clock::time_point startAt, endAt;

  qdpf::CellCollector collector = [this](int x, int y) {
    if (astar.finalPath.size()) {
      auto [x2, y2] = astar.finalPath.back();
      if ((x2 == x && y2 == y)) return;
    }
    astar.finalPath.push_back({x, y});
  };

  startAt = std::chrono::high_resolution_clock::now();

  auto [x, y] = astar.gatePath[0];
  for (int i = 1; i < astar.gatePath.size(); i++) {
    auto [x2, y2] = astar.gatePath[i];
    qdpf::ComputeStraightLine(x, y, x2, y2, collector);
    x = x2, y = y2;
  }
  state = State::AStarFinalPathComputed;
  endAt = std::chrono::high_resolution_clock::now();

  setMessageHint(
      fmt::format(
          "A*: final path computed! cost {}us ; Click button < Reset > to clear these results.",
          std::chrono::duration_cast<std::chrono::microseconds>(endAt - startAt).count()),
      ImGreen);
}

void Visualizer::handleFlowFieldInputQueryRangeBegin() {
  if (pathfinderFlag != PathFinderFlag::FlowField) {
    pathfinderFlag = PathFinderFlag::FlowField;
  }
  if (state != State::Idle) reset();
  state = State::FlowFieldWaitQrangeLeftTop;
  setMessageHint("FlowField: waiting to click a left-top cell", ImGreen);
}

void Visualizer::computeNodeFlowField() {
  if (state != State::FlowFieldWaitCompution) {
    setMessageHint("invalid state", ImRed);
    return;
  }
  if (0 != flowfield.ResetPf(agent.size, agent.capability)) {
    setMessageHint("internal error: flowfield reset failure", ImRed);
    return;
  }

  std::chrono::high_resolution_clock::time_point startAt, endAt;

  startAt = std::chrono::high_resolution_clock::now();
  int ret = flowfield.pf->ComputeNodeFlowField();
  endAt = std::chrono::high_resolution_clock::now();

  state = State::FlowFieldNodeLevelComputed;
  if (ret == -1) {
    setMessageHint("FlowField: unreachable!", ImRed);
    return;
  }

  if (flowfield.nodeFlowField.size()) flowfield.nodeFlowField.clear();

  qdpf::NodeFlowFieldVisitor visitor = [this](const qdpf::QdNode* node,
                                              const qdpf::QdNode* nextNode, int cost) {
    if (node != nullptr) flowfield.nodeFlowField.push_back({node, nextNode, cost});
  };
  flowfield.pf->VisitComputedNodeFlowField(visitor);

  // sort to draw from left-top to right-bottom.
  auto cmp = [](const FlowFieldItem<const qdpf::QdNode*>& a,
                const FlowFieldItem<const qdpf::QdNode*>& b) {
    return ((a.current->x1) < (b.current->x1)) || ((a.current->y1) < (b.current->y1));
  };
  std::stable_sort(flowfield.nodeFlowField.begin(), flowfield.nodeFlowField.end(), cmp);

  setMessageHint(
      fmt::format("FlowField: Node flowfield computed! cost {}us ; Next we can click button < "
                  "Compute Gate Flow Field  >.",
                  std::chrono::duration_cast<std::chrono::microseconds>(endAt - startAt).count()),
      ImGreen);
}

void Visualizer::computeGateFlowField() {
  if (state != State::FlowFieldWaitCompution && state != State::FlowFieldNodeLevelComputed) {
    setMessageHint("invalid state", ImRed);
    return;
  }

  if (0 != flowfield.ResetPf(agent.size, agent.capability)) {
    setMessageHint("internal error: flowfield reset failure", ImRed);
    return;
  }

  std::chrono::high_resolution_clock::time_point startAt, endAt;

  if (flowfield.gateFlowField.size()) flowfield.gateFlowField.clear();
  bool useNodeFlowField = flowfield.nodeFlowField.size() > 0;

  startAt = std::chrono::high_resolution_clock::now();
  int ret = flowfield.pf->ComputeGateFlowField(useNodeFlowField);
  endAt = std::chrono::high_resolution_clock::now();

  state = State::FlowFieldGateLevelComputed;

  if (-1 == ret) {
    setMessageHint("FlowField: unreachable!", ImRed);
    return;
  }

  qdpf::CellFlowFieldVisitor visitor = [this](int x, int y, int xNext, int yNext, int cost) {
    flowfield.gateFlowField.push_back({{x, y}, {xNext, yNext}, cost});
  };
  flowfield.pf->VisitComputedGateFlowField(visitor);

  // sort to draw from left-top to right-bottom
  std::stable_sort(flowfield.gateFlowField.begin(), flowfield.gateFlowField.end(),
                   [](const FlowFieldItem<Cell>& a, const FlowFieldItem<Cell>& b) {
                     return a.current < b.current;
                   });

  setMessageHint(
      fmt::format("Flowfield:: Gate flow field computed! useNodeFlowField: {}  cost {}us ; Next "
                  "we can click button "
                  "< Compute Final Flow Field >.",
                  useNodeFlowField,
                  std::chrono::duration_cast<std::chrono::microseconds>(endAt - startAt).count()),
      ImGreen);
}

void Visualizer::computeFinalFlowField() {
  if (state != State::FlowFieldGateLevelComputed) {
    setMessageHint("invalid state", ImRed);
    return;
  }

  std::chrono::high_resolution_clock::time_point startAt, endAt;

  if (flowfield.finalFlowField.size()) flowfield.finalFlowField.clear();
  if (flowfield.finalFlowNextMap.size()) flowfield.finalFlowNextMap.clear();

  startAt = std::chrono::high_resolution_clock::now();
  int ret = flowfield.pf->ComputeFinalFlowFieldInQueryRange();
  endAt = std::chrono::high_resolution_clock::now();
  state = State::FlowFieldFinalLevelComputed;

  if (-1 == ret) {
    setMessageHint("FlowField: unreachable!", ImRed);
    return;
  }

  qdpf::CellFlowFieldVisitor visitor = [this](int x, int y, int xNext, int yNext, int cost) {
    flowfield.finalFlowField.push_back({{x, y}, {xNext, yNext}, cost});
    flowfield.finalFlowNextMap[{x, y}] = {xNext, yNext};
  };

  flowfield.pf->VisitComputedCellFlowFieldInQueryRange(visitor);

  std::stable_sort(flowfield.finalFlowField.begin(), flowfield.finalFlowField.end(),
                   [](const FlowFieldItem<Cell>& a, const FlowFieldItem<Cell>& b) {
                     return a.current < b.current;
                   });

  setMessageHint(
      fmt::format("Flowfield:: Final flow field computed!  cost {}us  ",
                  std::chrono::duration_cast<std::chrono::microseconds>(endAt - startAt).count()),
      ImGreen);
}

void Visualizer::handleLogics() {
  switch (state) {
    case State::FlowFieldFinalLevelComputed:
      handlePlayFlowFieldTestPath();
      break;
    default:
      break;
  }
}

void Visualizer::handlePlayFlowFieldTestPath() {
  // (x1,y1) is the start
  int x1 = flowfield.x1, y1 = flowfield.y1;
  if (x1 == -1 || y1 == -1) return;

  // (x2,y2) is thet target
  int x2 = flowfield.x2, y2 = flowfield.y2;

  auto& p = flowfield.testPath;

  if (p.empty()) {
    p.push_back({x1, y1});
    return;
  }

  // get current position
  auto [x3, y3] = p.back();

  if (x3 == x2 && y3 == y2) {
    // arrived the target.
    // back to start;
    p.clear();
    p.push_back({x1, y1});
    return;
  }

  // Is inside the rect?
  if ((x3 >= flowfield.qrange.x1 && x3 <= flowfield.qrange.x2 && y3 >= flowfield.qrange.y1 &&
       y3 <= flowfield.qrange.y2)) {
    // get next from the final flow field.
    p.push_back(flowfield.finalFlowNextMap[{x3, y3}]);
  }
}
