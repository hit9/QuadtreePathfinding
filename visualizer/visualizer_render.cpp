#include "qdpf.hpp"
#include "visualizer.hpp"

using qdpf::internal::IsInsideRectangle;

void Visualizer::renderWorld() {
  renderHighlightedNodes();
  renderGates();
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
  for (auto node : astar.nodePath) {
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
  for (auto [node, nextNode, cost] : flowfield.nodeFlowField) {
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
      for (const auto [x, y] : astar.gatePath) {
        renderFillCell(x, y, Green);
      }
      break;
    case State::AStarFinalPathComputed:  //
      for (const auto [x, y] : astar.finalPath) {
        renderFillAgent(x, y);
      }
      break;
    default:
      return;  // do nothing
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
  for (int i = 0; i < flowfield.gateFlowField.size(); ++i) {
    auto& [gate, next, cost] = flowfield.gateFlowField[i];
    auto [x, y] = gate;
    auto [xNext, yNext] = next;
    renderFillCell(x, y, Green);
  }
}

void Visualizer::renderPathFindingFlowFieldGateNextsLines() {
  if (!renderFlowFieldGateNextLines) return;
  for (int i = 0; i < flowfield.gateFlowField.size(); ++i) {
    auto& [gate, next, cost] = flowfield.gateFlowField[i];
    auto [x, y] = gate;
    auto [xNext, yNext] = next;
    renderDrawLineBetweenCells(x, y, xNext, yNext, Black);
  }
}

void Visualizer::renderPathFindingFlowFieldFinalField() {
  if (flowfield.testPaths.size()) {
    for (const auto& testPath : flowfield.testPaths) {
      for (auto [x, y] : testPath) {
        renderFillAgent(x, y);
      }
    }
  }
  for (int i = 0; i < flowfield.finalFlowField.size(); ++i) {
    auto& [u, v, cost] = flowfield.finalFlowField[i];
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

void Visualizer::renderDrawRect(const SDL_Rect& rect, const SDL_Color& color) {
  SDL_Rect overlap;
  if (cropRectByCamera(rect, overlap)) {
    SDL_SetRenderDrawColor(renderer, color.r, color.g, color.b, color.a);
    SDL_RenderDrawRect(renderer, &overlap);
  }
}

void Visualizer::renderFillRect(const SDL_Rect& rect, const SDL_Color& color) {
  SDL_Rect overlap;
  if (cropRectByCamera(rect, overlap)) {
    SDL_SetRenderDrawColor(renderer, color.r, color.g, color.b, color.a);
    SDL_RenderFillRect(renderer, &overlap);
  }
}

void Visualizer::renderDrawLine(int x1, int y1, int x2, int y2, const SDL_Color& color) {
  // TODO: how to crop? May we need the Cohen-Sutherland..
  x1 -= camera->x;
  x2 -= camera->x;
  y1 -= camera->y;
  y2 -= camera->y;
  SDL_SetRenderDrawColor(renderer, color.r, color.g, color.b, color.a);
  SDL_RenderDrawLine(renderer, x1, y1, x2, y2);
}

void Visualizer::renderDrawLineBetweenCells(int x1, int y1, int x2, int y2,
                                            const SDL_Color& color) {
  auto px1 = y1 * map.gridSize;
  auto py1 = x1 * map.gridSize;
  auto px2 = y2 * map.gridSize;
  auto py2 = x2 * map.gridSize;
  auto centerx1 = px1 + map.gridSize / 2;
  auto centery1 = py1 + map.gridSize / 2;
  auto centerx2 = px2 + map.gridSize / 2;
  auto centery2 = py2 + map.gridSize / 2;
  renderDrawLine(centerx1, centery1, centerx2, centery2, color);
}

void Visualizer::renderCopy(SDL_Texture* texture, const SDL_Rect& src, const SDL_Rect& dst) {
  SDL_Rect dstOverlap;
  if (!cropRectByCamera(dst, dstOverlap, false)) return;
  // find the crop region in src
  SDL_Rect srcOverlap = {src.x + dstOverlap.x - dst.x, src.y + dstOverlap.y - dst.y, dstOverlap.w,
                         dstOverlap.h};
  dstOverlap.x -= camera->x;
  dstOverlap.y -= camera->y;
  SDL_RenderCopy(renderer, texture, &srcOverlap, &dstOverlap);
}

void Visualizer::renderFillCell(int x, int y, const SDL_Color& color) {
  SDL_Rect cell{y * map.gridSize + 1, x * map.gridSize + 1, map.gridSize - 2, map.gridSize - 2};
  renderFillRect(cell, color);
}

void Visualizer::renderFillAgent(int x, int y) {
  int agentSizeInPixels = agent.size * map.gridSize / COST_UNIT;
  SDL_Rect outer{y * map.gridSize, x * map.gridSize, agentSizeInPixels, agentSizeInPixels};
  SDL_Rect inner{outer.x + 1, outer.y + 1, outer.w - 2, outer.h - 2};
  renderDrawRect(outer, Black);
  renderFillRect(inner, Green);
}

// Crop given rect by camera, and converts to the coordinates relative to the camera's left-top
// corner. The results are stored into overlap. Returns false if no overlaping.
bool Visualizer::cropRectByCamera(const SDL_Rect& rect, SDL_Rect& overlap,
                                  bool marginToCameraCoordinates) {
  qdpf::Rectangle c{camera->x, camera->y, camera->x + camera->w - 1, camera->y + camera->h - 1};
  qdpf::Rectangle a{rect.x, rect.y, rect.x + rect.w - 1, rect.y + rect.h - 1};
  qdpf::Rectangle d;
  auto b = qdpf::internal::GetOverlap(c, a, d);
  if (!b) return false;
  overlap.x = d.x1, overlap.y = d.y1;
  overlap.w = d.x2 - d.x1 + 1, overlap.h = d.y2 - d.y1 + 1;
  if (marginToCameraCoordinates) {
    overlap.x -= camera->x;
    overlap.y -= camera->y;
  }
  return true;
}
