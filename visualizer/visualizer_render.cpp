#include "qdpf.hpp"
#include "visualizer.hpp"

using qdpf::internal::IsInsideRectangle;

void Visualizer::renderWorld() {
  renderHighlightedNodes();
  renderGates();
  renderGateGraph();
  renderNodeGraph();
  renderGrids();
  renderQuadtreeNodes();
  renderPathfindingDispatch();
}

void Visualizer::renderGrids() {
  // (i,j) is the cell's position.
  // (x,y) is the grid in pixel (in SDL coordinates).
  for (int i = 0, y = 0; i < map.h; ++i, y += map.gridSize) {
    for (int j = 0, x = 0; j < map.w; ++j, x += map.gridSize) {
      SDL_Rect grid = {x, y, map.gridSize, map.gridSize};
      SDL_Rect gridInner = {grid.x + 1, grid.y + 1, grid.w - 2, grid.h - 2};
      renderDrawRect(grid, LightGray);
      if (hideTerrainRenderings) continue;
      // Its terrain value or changing to value
      auto v = map.grids[i][j];
      if (map.changes[i][j] != 0) v = map.changes[i][j];
      switch (v) {
        case Terrain::Building:
          renderFillRect(gridInner, Red);
          break;
        case Terrain::Water:
          renderFillRect(gridInner, Blue);
          break;
      }
    }
  }
}

// render quadtree nodes of current map.
void Visualizer::renderQuadtreeNodes() {
  if (hideNodeBorders) return;
  auto mp = getCurrentQuadtreeMapByAgent();
  if (mp != nullptr) {
    qdpf::internal::QdNodeVisitor c1 = [this](const qdpf::internal::QdNode* node) {
      int x = node->y1 * map.gridSize;
      int y = node->x1 * map.gridSize;
      int w = (node->y2 - node->y1 + 1) * map.gridSize;
      int h = (node->x2 - node->x1 + 1) * map.gridSize;
      // Outer liner rectangle (border width 2)
      SDL_Rect rect1 = {x, y, w, h};
      SDL_Rect rect2 = {x + 1, y + 1, w - 2, h - 2};
      renderDrawRect(rect1, Black);
      renderDrawRect(rect2, Black);
    };
    mp->Nodes(c1);
  }
}

void Visualizer::renderGates() {
  if (hideGateCellHighlights) return;
  auto mp = getCurrentQuadtreeMapByAgent();
  if (mp != nullptr) {
    // Gates.
    qdpf::internal::GateVisitor visitor = [this, &mp](const qdpf::internal::Gate* gate) {
      auto [x1, y1] = mp->UnpackXY(gate->a);
      int x3 = y1 * map.gridSize + 1;
      int y3 = x1 * map.gridSize + 1;
      SDL_Rect rect = {x3, y3, map.gridSize - 2, map.gridSize - 2};
      renderFillRect(rect, LightPurple);
    };

    mp->Gates(visitor);
  }
}

void Visualizer::renderGateGraph() {
  if (!showGateGraph) return;

  auto mp = getCurrentQuadtreeMapByAgent();
  if (mp != nullptr) {
    auto& graph = mp->GetGateGraph();
    qdpf::internal::EdgeVisitor<int> visitor = [this, mp](int u, int v, int cost) {
      auto [x1, y1] = mp->UnpackXY(u);
      auto [x2, y2] = mp->UnpackXY(v);
      renderDrawLineBetweenCells(x1, y1, x2, y2, Blue);
    };
    graph.ForEachEdge(visitor);
  }
}

void Visualizer::renderNodeGraph() {
  if (!showNodeGraph) return;

  auto mp = getCurrentQuadtreeMapByAgent();
  if (mp != nullptr) {
    auto& graph = mp->GetNodeGraph();
    qdpf::internal::EdgeVisitor<qdpf::internal::QdNode*> visitor =
        [this, mp](qdpf::internal::QdNode* u, qdpf::internal::QdNode* v, int cost) {
          auto x1 = (u->x1 + u->x2) / 2, y1 = (u->y1 + u->y2) / 2;
          auto x2 = (v->x1 + v->x2) / 2, y2 = (v->y1 + v->y2) / 2;
          renderDrawLineBetweenCells(x1, y1, x2, y2, Red);
        };
    graph.ForEachEdge(visitor);
  }
}

void Visualizer::renderPathfindingDispatch() {
  switch (pathfinderFlag) {
    case PathFinderFlag::AStar:
      renderPathfindingAStar();
      return;
    case PathFinderFlag::FlowField:
      renderPathfindingFlowField();
      return;
  }
}

void Visualizer::renderHighlightedNodes() {
  // render highlighted nodes with background at first.
  switch (state) {
    case State::AStarNodePathComputed:
      [[fallthrough]];
    case State::AStarGatePathComputed:
      [[fallthrough]];
    case State::AStarFinalPathComputed:
      renderHighlightedNodesAstar();
      break;
    case State::FlowFieldNodeLevelComputed:
      renderHighlightedNodesFlowField();
      [[fallthrough]];
    case State::FlowFieldGateLevelComputed:
      [[fallthrough]];
    case State::FlowFieldFinalLevelComputed:
      renderHighlightedNodesFlowField();
      break;
    default:
      break;  // avoiding warning
  }
}

void Visualizer::renderHighlightedNodesAstar() {
  for (auto [node, cost] : astar.nodePath) {
    int x1 = node->x1, y1 = node->y1, x2 = node->x2, y2 = node->y2;
    int h = (x2 - x1 + 1) * map.gridSize;
    int w = (y2 - y1 + 1) * map.gridSize;
    int x = y1 * map.gridSize;
    int y = x1 * map.gridSize;
    SDL_Rect rect = {x, y, w, h};
    SDL_Rect inner = {x + 1, y + 1, w - 2, h - 2};
    renderFillRect(inner, LightOrange);
  }
}

void Visualizer::renderHighlightedNodesFlowField() {
  for (auto& [node, p] : flowfield.nodeFlowField.GetUnderlyingMap()) {
    auto [next, cost] = p;
    // highlight node with background orange
    int x1 = node->x1, y1 = node->y1, x2 = node->x2, y2 = node->y2;
    int h = (x2 - x1 + 1) * map.gridSize;
    int w = (y2 - y1 + 1) * map.gridSize;
    int x = y1 * map.gridSize;
    int y = x1 * map.gridSize;
    SDL_Rect rect = {x, y, w, h};
    SDL_Rect inner = {x + 1, y + 1, w - 2, h - 2};
    renderFillRect(inner, LightOrange);
  }
}

void Visualizer::renderPathfindingAStar() {
  int agentSizeInPixels = agent.size * map.gridSize / COST_UNIT;

  switch (state) {
    case State::AStarWaitTarget:
      renderFillCell(astar.x1, astar.y1, Green);  // start
      break;
    case State::AStarWaitCompution:
      [[fallthrough]];
    case State::AStarNodePathComputed:
      renderFillCell(astar.x1, astar.y1, Green);  // start
      renderFillCell(astar.x2, astar.y2, Green);  // target
      // NOTE: nodepath already render in renderHighlightedNodesAstar.
      break;
    case State::AStarGatePathComputed:
      // draw gate route cells.
      // start and target are included in gate path.
      for (const auto [x, y, _] : astar.gatePath) {
        renderFillCell(x, y, Green);
      }
      break;
    case State::AStarFinalPathComputed:  //
      for (const auto [x, y] : astar.finalPath) {
        renderFillAgent(x, y);
      }

      renderPathfindingAStarNaive();
      break;
    default:
      return;  // do nothing
  }
}

void Visualizer::renderPathfindingAStarNaive() {
  if (astarNaive.path.empty()) return;
  for (const auto [x, y] : astarNaive.path) {
    renderFillAgent(x, y, DarkGreen);
  }
}

void Visualizer::renderPathfindingFlowField() {
  auto drawQrangeRectangle = [this]() {
    // render the qrange rect
    SDL_Rect qrange{
        flowfield.qrange.y1 * map.gridSize,
        flowfield.qrange.x1 * map.gridSize,
        (flowfield.qrange.y2 - flowfield.qrange.y1 + 1) * map.gridSize,
        (flowfield.qrange.x2 - flowfield.qrange.x1 + 1) * map.gridSize,
    };
    SDL_Rect inner{qrange.x + 1, qrange.y + 1, qrange.w - 2, qrange.h - 2};
    renderDrawRect(qrange, Green);
    renderDrawRect(inner, Green);
  };

  switch (state) {
    case State::FlowFieldWaitQrangeRightBottom:
      renderFillCell(flowfield.qrange.x1, flowfield.qrange.y1, Green);  // qrange.left-top
      return;
    case State::FlowFieldWaitTarget:
      drawQrangeRectangle();
      renderFillCell(flowfield.qrange.x1, flowfield.qrange.y1, Green);  // qrange.left-top
      renderFillCell(flowfield.qrange.x2, flowfield.qrange.y2, Green);  // qrange.right-bottom
      return;
    case State::FlowFieldWaitCompution:
      [[fallthrough]];
    case State::FlowFieldNodeLevelComputed:
      // NOTE the highlighting of the node field are already done in
      // renderHighlightedNodesFlowField
      drawQrangeRectangle();
      renderFillCell(flowfield.qrange.x1, flowfield.qrange.y1, Green);  // qrange.left-top
      renderFillCell(flowfield.qrange.x2, flowfield.qrange.y2, Green);  // qrange.right-bottom
      renderFillCell(flowfield.x2, flowfield.y2, Green);                // target
      return;
    case State::FlowFieldGateLevelComputed:
      drawQrangeRectangle();
      renderPathFindingFlowFieldGateField();
      renderPathFindingFlowFieldGateNextsLines();
      return;
    case State::FlowFieldFinalLevelComputed:
      drawQrangeRectangle();
      renderPathFindingFlowFieldGateNextsLines();
      renderFillCell(flowfield.x2, flowfield.y2, Green);  // target
      renderPathFindingFlowFieldFinalField();
      return;
    default:

      return;  // do nothing
  }
}

void Visualizer::renderPathFindingFlowFieldGateField() {
  // draw gate cells on the flowfield.
  for (auto& [gate, p] : flowfield.gateFlowField.GetUnderlyingMap()) {
    auto [next, cost] = p;
    auto [x, y] = gate;
    auto [xNext, yNext] = next;
    renderFillCell(x, y, Green);
  }
}

void Visualizer::renderPathFindingFlowFieldGateNextsLines() {
  if (showNaiveFlowFieldResults) return;
  if (!renderFlowFieldGateNextLines) return;
  for (auto& [gate, p] : flowfield.gateFlowField.GetUnderlyingMap()) {
    auto [next, cost] = p;
    auto [x, y] = gate;
    auto [xNext, yNext] = next;
    renderDrawLineBetweenCells(x, y, xNext, yNext, Black);
  }
}

void Visualizer::renderPathFindingFlowFieldFinalField() {
  const auto& testPaths =
      showNaiveFlowFieldResults ? flowfieldNaive.testPaths : flowfield.testPaths;

  if (testPaths.size()) {
    for (const auto& testPath : testPaths) {
      for (auto [x, y] : testPath) {
        renderFillAgent(x, y);
      }
    }
  }

  const auto& finalFlowField =
      showNaiveFlowFieldResults ? flowfieldNaive.finalFlowField : flowfield.finalFlowField;

  for (auto& [u, p] : finalFlowField.GetUnderlyingMap()) {
    auto [v, cost] = p;
    auto [x, y] = u;
    auto [x1, y1] = v;
    if (u == v) continue;
    if (IsInsideRectangle(x, y, flowfield.qrange)) {
      int dx = (x1 - x), dy = (y1 - y);
      int d = (dx + 1) * 3 + (dy + 1);
      if (!(d >= 0 && d <= 8 && d != 4)) continue;
      int k = ARROWS_DIRECTIONS[d];
      int w = arrows.w[k], h = arrows.h, offset = arrows.offset[k];
      w = std::min(w, map.gridSize);
      h = std::min(h, map.gridSize);
      SDL_Rect dst = {y * map.gridSize + std::max(0, map.gridSize / 2 - w / 2),
                      x * map.gridSize + std::max(0, map.gridSize / 2 - h / 2), w, h};
      SDL_Rect src = {offset, 0, w, h};
      renderCopy(arrows.texture, src, dst);
    }
  }
}
