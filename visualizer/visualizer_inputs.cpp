#include <imgui.h>
#include <imgui_impl_sdl2.h>
#include <imgui_impl_sdlrenderer2.h>

#include "visualizer.hpp"

void Visualizer::handleInputs() {
  SDL_Event e;
  auto& io = ImGui::GetIO();
  while (SDL_PollEvent(&e)) {
    ImGui_ImplSDL2_ProcessEvent(&e);
    if (e.type == SDL_QUIT) {
      stop = true;
      return;
    }

    // isMouseDown;
    if (e.type == SDL_MOUSEBUTTONDOWN)
      isMouseDown = true;
    else if (e.type == SDL_MOUSEBUTTONUP)
      isMouseDown = false;

    // shortcuts (in front of imgui)
    handleInputsShortcuts(e);
    handleInputsForCrameMovementsByKeyBoard(e);

    // ImGui didn't handle this event, turn it to SDL.
    if (!io.WantCaptureMouse && !io.WantCaptureKeyboard) {
      handleInputsForCrameMovementsByMouse(e);
      handleInputsDispatchByState(e);
    }
  }
}

// returns -1 for stop the whole window.
void Visualizer::handleInputsShortcuts(SDL_Event& e) {
  // shortcuts
  if (e.type == SDL_KEYDOWN) {
    switch (e.key.keysym.sym) {
        // Ctrl-C
      case SDLK_c:
        if (SDL_GetModState() & KMOD_CTRL) stop = true;
        break;
        // ESC
      case SDLK_ESCAPE:
        reset();
        break;
      case SDLK_b:
        handleStartDrawBuildings();
        break;
      case SDLK_w:
        handleStartDrawWater();
        break;
    }
  }
}

void Visualizer::handleInputsForCrameMovementsByMouse(SDL_Event& e) {
  switch (e.type) {
    case SDL_MOUSEWHEEL:
      if (e.wheel.x > 0) camera->MoveLeft();
      if (e.wheel.x < 0) camera->MoveRight();
      if (e.wheel.y > 0) camera->MoveUp();
      if (e.wheel.y < 0) camera->MoveDown();
      break;
  }
}

void Visualizer::handleInputsForCrameMovementsByKeyBoard(SDL_Event& e) {
  switch (e.type) {
    case SDL_KEYDOWN:
      if (e.key.keysym.sym == SDLK_UP || e.key.keysym.sym == SDLK_k) camera->MoveUp();
      if (e.key.keysym.sym == SDLK_DOWN || e.key.keysym.sym == SDLK_j) camera->MoveDown();
      if (e.key.keysym.sym == SDLK_LEFT || e.key.keysym.sym == SDLK_h) camera->MoveLeft();
      if (e.key.keysym.sym == SDLK_RIGHT || e.key.keysym.sym == SDLK_l) camera->MoveRight();
      break;
  }
}

void Visualizer::handleInputsDispatchByState(SDL_Event& e) {
  switch (state) {
    case State::DrawingBuildings:
      [[fallthrough]];
    case State::DrawingWaters:
      handleInputsChangeTerrains(e);
      break;
    case State::AStarWaitStart:
      handleInputsAStarSetStart(e);
      break;
    case State::AStarWaitTarget:
      handleInputsAStarSetTarget(e);
      break;
    case State::FlowFieldWaitQrangeLeftTop:
      handleInputsFlowFieldSetQrangeLeftTop(e);
      break;
    case State::FlowFieldWaitQrangeRightBottom:
      handleInputsFlowFieldSetQrangeRightBottom(e);
      break;
    case State::FlowFieldWaitTarget:
      handleInputsFlowFieldSetTarget(e);
      break;
    case State::FlowFieldFinalLevelComputed:
      // reset test path's star:
      handleInputsFlowFieldSetTestStart(e);
      break;
    default:
      break;  // avoid warning.
  }
}

void Visualizer::handleInputsChangeTerrains(SDL_Event& e) {
  switch (e.type) {
    case SDL_MOUSEBUTTONUP:
      applyTerrainChanges();
      break;
    case SDL_MOUSEMOTION:
      if (isMouseDown) pushTerrainChanges(getCellAtPixelPosition(e.button.x, e.button.y));
      break;
    case SDL_MOUSEBUTTONDOWN:
      pushTerrainChanges(getCellAtPixelPosition(e.button.x, e.button.y));
      break;
  }
}

void Visualizer::handleInputsAStarSetStart(SDL_Event& e) {
  if (e.type == SDL_MOUSEBUTTONDOWN) {
    auto cell = getCellAtPixelPosition(e.button.x, e.button.y);
    astar.x1 = cell.first;
    astar.y1 = cell.second;
    state = State::AStarWaitTarget;
    setMessageHint("A*: waiting to click a target cell", ImGreen);
  }
}

void Visualizer::handleInputsAStarSetTarget(SDL_Event& e) {
  if (e.type == SDL_MOUSEBUTTONDOWN) {
    auto cell = getCellAtPixelPosition(e.button.x, e.button.y);
    astar.x2 = cell.first;
    astar.y2 = cell.second;
    state = State::AStarWaitCompution;
    setMessageHint("A*: waiting to click a compution button", ImGreen);
  }
}

void Visualizer::handleInputsFlowFieldSetQrangeLeftTop(SDL_Event& e) {
  if (e.type == SDL_MOUSEBUTTONDOWN) {
    auto cell = getCellAtPixelPosition(e.button.x, e.button.y);
    flowfield.qrange.x1 = cell.first;
    flowfield.qrange.y1 = cell.second;
    state = State::FlowFieldWaitQrangeRightBottom;
    setMessageHint("FlowField: waiting to click a right-bottom cell", ImGreen);
  }
}
void Visualizer::handleInputsFlowFieldSetQrangeRightBottom(SDL_Event& e) {
  if (e.type == SDL_MOUSEBUTTONDOWN) {
    auto cell = getCellAtPixelPosition(e.button.x, e.button.y);
    flowfield.qrange.x2 = cell.first;
    flowfield.qrange.y2 = cell.second;
    state = State::FlowFieldWaitTarget;
    setMessageHint("FlowField: waiting to click a target cell", ImGreen);
  }
}

void Visualizer::handleInputsFlowFieldSetTarget(SDL_Event& e) {
  if (e.type == SDL_MOUSEBUTTONDOWN) {
    auto cell = getCellAtPixelPosition(e.button.x, e.button.y);
    flowfield.x2 = cell.first;
    flowfield.y2 = cell.second;
    state = State::FlowFieldWaitCompution;
    setMessageHint("FlowField: waiting to click a compution button", ImGreen);
  }
}

void Visualizer::handleInputsFlowFieldSetTestStart(SDL_Event& e) {
  if (e.type == SDL_MOUSEBUTTONDOWN) {
    auto cell = getCellAtPixelPosition(e.button.x, e.button.y);
    auto [x, y] = cell;
    if (flowfield.finalFlowNextMap.find({x, y}) == flowfield.finalFlowNextMap.end()) {
      setMessageHint("FlowField: test path start invalid", ImRed);
      flowfield.x1 = -1;
      flowfield.y1 = -1;
      return;
    }
    flowfield.x1 = x;
    flowfield.y1 = y;
    flowfield.testPath.clear();
    setMessageHint("FlowField: playing a test path ...", ImGreen);
  }
}