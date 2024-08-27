#include <SDL2/SDL.h>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <argparse/argparse.hpp>
#include <chrono>
#include <cstring>
#include <string>
#include <utility>
#include <vector>

#include "qdpf.hpp"

// pixels per grid side.
int GRID_SIZE = 16;

enum Terrain {
  Land = 0b001,
  Water = 0b010,
  Building = 0b100,
};

// Max value of w and h.
const int N = 5000;

// the value of GRIDS[x][y] is a terrain type integer.
int GRIDS[N][N];

// TO_BECOME_TERRAIN[x][y] records the terrain value that cell (x,y) is going to change to.
// 0 for no changes.
int TO_BECOME_TERRAIN[N][N];

using Cell = std::pair<int, int>;  // {x,y}

// the terrain capability for current agent.
int Capability = Terrain::Land;

struct Options {
  int w = 10, h = 10, step = -1;
  int maxNodeWidth = -1, maxNodeHeight = -1;
  int createWallsOnInit = 0;
  bool useNodePath = false;
  int agentSize = 10;
  bool canWalkOnWater = false;
};

// Parse options from command line.
// Returns 0 on success.
int ParseOptionsFromCommandline(int argc, char* argv[], Options& options);

class Visualizer {
 public:
  Visualizer(qdpf::QuadtreeMapX& mx, qdpf::AStarPathFinder* pf, Options& options);
  int Init();
  void Start();
  void Destroy();

 private:
  qdpf::QuadtreeMapX& mx;
  qdpf::AStarPathFinder* pf;
  Options& options;
  SDL_Window* window;
  SDL_Renderer* renderer;

  // the weight and height of the window.
  int windowW, windowH;
  // width and height in pixels of the map.
  int mapW, mapH;
  // camera's left-up position, the camera is the window
  int cameraX = 0;
  int cameraY = 0;
  // total scrolls in x and y during a frame.
  int scrollDx = 0;
  int scrollDy = 0;

  // start(x1,y1) and target (x2,y2)
  int x1, y1, x2, y2;

  // state
  // 0  wait for start cell input
  // 1  wait for target cell input, and then calculates the routes.
  // 2  routes calculated, and wait for path calculation
  // 3. path calculated.
  int state = 0;

  // current routes and path
  std::vector<Cell> routes, path;

  // cells to change terrains
  std::vector<Cell> toChangeTerrains;
  // Is mouse left button down?
  bool isMouseDown = false;
  // terrain types to add when mouse down.
  int terrainTypeToChange = 0;

  void draw();
  void handleCameraMovements();
  bool isInsideCamera(int x, int y);
  bool isOverlapsCamera(const SDL_Rect& rect);
  void updateRectRelativeToCamera(SDL_Rect& rect);
  std::pair<int, int> getCellAtMousePosition(SDL_Event& e) const;
  int handleInputs();
  void handleChangeTerrains();
  void recordCellsToChangeTerrains(int x, int y, int to = 0);
  void calculateRoutes();
  void calculatePath();
  void reset();
  void createsWallsOnInit();
};

int main(int argc, char* argv[]) {
  // Parse arguments.
  Options options;
  if (ParseOptionsFromCommandline(argc, argv, options) != 0) return -1;

  // Inits the map, by default, all cells are land.
  memset(TO_BECOME_TERRAIN, 0, sizeof TO_BECOME_TERRAIN);
  for (int x = 0; x < options.h; ++x)
    for (int y = 0; y < options.w; ++y) GRIDS[x][y] = Terrain::Land;

  // Quadtree map.
  auto distance = qdpf::EuclideanDistance<10>;
  qdpf::TerrainTypesChecker terrainChecker = [](int x, int y) { return GRIDS[x][y]; };
  auto stepf = options.step != -1 ? nullptr : [](int z) -> int { return z / 8 + 1; };

  if (options.canWalkOnWater) Capability |= Terrain::Water;

  qdpf::QuadtreeMapXSettings settings{
      {options.agentSize, Capability},
  };

  qdpf::QuadtreeMapX mx(options.w, options.h, distance, terrainChecker, settings, options.step,
                        stepf, options.maxNodeWidth, options.maxNodeHeight);
  // Path finder.
  qdpf::AStarPathFinder pf(mx);
  // Visualizer.
  Visualizer visualizer(mx, &pf, options);
  if (visualizer.Init() != 0) return -1;
  visualizer.Start();
  visualizer.Destroy();
  return 0;
}

int ParseOptionsFromCommandline(int argc, char* argv[], Options& options) {
  argparse::ArgumentParser program("quadtree-pathfinding-visualizer");
  program.add_argument("-w", "--width")
      .help("width of grid map")
      .default_value(8)
      .store_into(options.w);
  program.add_argument("-h", "--height")
      .help("height of grid map")
      .default_value(8)
      .store_into(options.h);
  program.add_argument("-s", "--step")
      .help("step to pick gates. we use a stepf by default")
      .default_value(-1)
      .store_into(options.step);
  program.add_argument("-wn", "--max-node-width")
      .help("max node width")
      .default_value(-1)
      .store_into(options.maxNodeWidth);
  program.add_argument("-hn", "--max-node-height")
      .help("max node height")
      .default_value(-1)
      .store_into(options.maxNodeHeight);
  program.add_argument("-cw", "--create-walls")
      .help("number of walls to create on init")
      .default_value(0)
      .store_into(options.createWallsOnInit);
  program.add_argument("-gs", "--grid-cell-size-in-pixels")
      .help("grid cell size in pixels")
      .default_value(16)
      .store_into(GRID_SIZE);
  program.add_argument("-u", "--use-node-path")
      .help("use node path to filter less gate cells for computing route cells.")
      .default_value(false)
      .store_into(options.useNodePath);
  program.add_argument("-a", "--agent-size")
      .help("the size of path finding agent")
      .default_value(10)
      .store_into(options.agentSize);
  program.add_argument("-wa", "--can-walk-on-water")
      .help("enable the agent to walk on water.")
      .default_value(false)
      .store_into(options.canWalkOnWater);
  try {
    program.parse_args(argc, argv);
  } catch (const std::exception& e) {
    spdlog::error(e.what());
    return 1;
  }
  if (options.w > N || options.h > N) {
    spdlog::error("w or h is too large");
    return 2;
  }
  return 0;
}

Visualizer::Visualizer(qdpf::QuadtreeMapX& mx, qdpf::AStarPathFinder* pf, Options& options)
    : mx(mx), pf(pf), options(options), mapW(options.w * GRID_SIZE), mapH(options.h * GRID_SIZE) {}

int Visualizer::Init() {
  // Init SDL
  if (SDL_Init(SDL_INIT_EVERYTHING) != 0) {
    spdlog::error("SDL init error: {}", SDL_GetError());
    return -1;
  }
  // Get display bounds
  SDL_Rect displayBounds;
  if (SDL_GetDisplayBounds(0, &displayBounds) != 0) {
    printf("Failed to get display bounds: %s\n", SDL_GetError());
    return 1;
  }

  windowW = std::min(mapW, displayBounds.w);
  windowH = std::min(mapH, displayBounds.h);

  // Creates window.
  window = SDL_CreateWindow("quadtree-pathfinding-visualizer", SDL_WINDOWPOS_CENTERED,
                            SDL_WINDOWPOS_CENTERED, windowW, windowH, SDL_WINDOW_SHOWN);
  if (window == nullptr) {
    spdlog::error("Create window error: {}", SDL_GetError());
    SDL_Quit();
    return -3;
  }
  // Creates renderer.
  renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
  if (renderer == nullptr) {
    spdlog::error("Create renderer error: {}", SDL_GetError());
    SDL_DestroyWindow(window);
    SDL_Quit();
    return -1;
  }
  // Build the quadtree map.
  spdlog::info("Visualizer init done");
  if (options.createWallsOnInit > 0) createsWallsOnInit();

  mx.Build();
  spdlog::info("QuadtreeMapX build done");

  return 0;
}

void Visualizer::Destroy() {
  SDL_DestroyRenderer(renderer);
  SDL_DestroyWindow(window);
  SDL_Quit();
}

void Visualizer::Start() {
  while (true) {
    // quit on -1
    if (handleInputs() == -1) break;
    // camera
    handleCameraMovements();
    // Background: white
    SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
    SDL_RenderClear(renderer);
    draw();
    SDL_RenderPresent(renderer);

    SDL_Delay(10);  // 10ms
  }
}

// helper function to get mouse position.
std::pair<int, int> Visualizer::getCellAtMousePosition(SDL_Event& e) const {
  return {(e.button.y + cameraY) / GRID_SIZE, (e.button.x + cameraX) / GRID_SIZE};
}

int Visualizer::handleInputs() {
  SDL_Event e;
  scrollDx = 0, scrollDy = 0;
  while (SDL_PollEvent(&e)) {
    switch (e.type) {
      case SDL_QUIT:  // quit
        return -1;
      case SDL_KEYDOWN:
        if (e.key.keysym.sym == SDLK_c && SDL_GetModState() & KMOD_CTRL) {  // Ctrl-C
          spdlog::info("Ctrl-C : quit...");
          return -1;
        }
        if (e.key.keysym.sym == SDLK_ESCAPE) {
          // TODO clear queries
          reset();
          return 0;
        }
        if (e.key.keysym.sym == SDLK_w) {
          terrainTypeToChange = Terrain::Water;
          spdlog::info("sets terrain type to draw to water");
        }
        if (e.key.keysym.sym == SDLK_UP || e.key.keysym.sym == SDLK_k) scrollDy -= 50;
        if (e.key.keysym.sym == SDLK_DOWN || e.key.keysym.sym == SDLK_j) scrollDy += 50;
        if (e.key.keysym.sym == SDLK_LEFT || e.key.keysym.sym == SDLK_h) scrollDx -= 50;
        if (e.key.keysym.sym == SDLK_RIGHT || e.key.keysym.sym == SDLK_l) scrollDx += 50;
        break;
      case SDL_MOUSEWHEEL:
        if (e.wheel.x > 0) scrollDx -= 50;  // left
        if (e.wheel.x < 0) scrollDx += 50;  // right
        if (e.wheel.y > 0) scrollDy -= 50;  // up
        if (e.wheel.y < 0) scrollDy += 50;  // down
        break;
      case SDL_MOUSEBUTTONUP:
        if (e.button.button == SDL_BUTTON_LEFT) {
          if (isMouseDown) {
            auto [x, y] = getCellAtMousePosition(e);
            recordCellsToChangeTerrains(x, y);
            handleChangeTerrains();
            isMouseDown = false;
            terrainTypeToChange = 0;
          }
        }
        break;
      case SDL_MOUSEMOTION:
        if (isMouseDown) {
          recordCellsToChangeTerrains(e.button.y / GRID_SIZE, e.button.x / GRID_SIZE);
        }
        break;
      case SDL_MOUSEBUTTONDOWN:
        if (e.button.button == SDL_BUTTON_LEFT) {  // Change terrains
          isMouseDown = true;
          auto [x, y] = getCellAtMousePosition(e);
          recordCellsToChangeTerrains(x, y);
        } else if (e.button.button == SDL_BUTTON_RIGHT) {  // Set start/target
          auto [x, y] = getCellAtMousePosition(e);
          if (state == 0) {  // waiting start point
            reset();
            x1 = x, y1 = y;
            spdlog::info("start cell is set to {},{}, please right click a target cell..", x1, y1);
            state = 1;
          } else if (state == 1) {  // waiting target point
            x2 = x, y2 = y;
            spdlog::info("target cell is set to {},{}, will calculate the routes", x2, y2);
            calculateRoutes();
            state = 2;
          } else if (state == 2) {  // routes calculation done.
            calculatePath();
            state = 3;
            spdlog::info("right click anywhere again to clear this result");
          } else if (state == 3) {
            reset();
          }
        }
        break;
    }
  }
  return 0;
}

void Visualizer::reset() {
  state = 0;
  terrainTypeToChange = 0;
  path.clear();
  routes.clear();
  if (-1 == pf->Reset(0, 0, 0, 0, options.agentSize, Capability)) {
    spdlog::info("reset failed !");
  } else
    spdlog::info("reset");
}

void Visualizer::createsWallsOnInit() {
  spdlog::info("Creates walls in the middle on init");

  float z = (float)options.w / (options.createWallsOnInit + 1);
  int w = options.h / 4;
  int x1 = 0, x2 = options.h - w;
  for (int k = 1; k <= options.createWallsOnInit; k++) {
    int y = std::min(static_cast<int>(z * k), options.w - 1);
    for (int x = x1; x < x2; x++) {
      GRIDS[x][y] = Terrain::Building;
    }
    if (x1 == 0)
      x1 += w, x2 += w;
    else {
      x1 = 0, x2 -= w;
    }
  }
}

void Visualizer::recordCellsToChangeTerrains(int x, int y, int to) {
  // TODO: invert land and building.
  if (terrainTypeToChange == 0) terrainTypeToChange = Terrain::Building;
  if (to == 0) to = (GRIDS[x][y] == Terrain::Land) ? terrainTypeToChange : Terrain::Land;
  if (x >= 0 && x < options.h && y >= 0 && y < options.w && !TO_BECOME_TERRAIN[x][y]) {
    TO_BECOME_TERRAIN[x][y] = to;
    toChangeTerrains.push_back({x, y});
  }
}

void Visualizer::handleChangeTerrains() {
  for (auto [x, y] : toChangeTerrains) {
    GRIDS[x][y] = TO_BECOME_TERRAIN[x][y];
    mx.Update(x, y);
    spdlog::info("{},{} changed terrain to {}", x, y, GRIDS[x][y]);
  }
  mx.Compute();
  memset(TO_BECOME_TERRAIN, 0, sizeof TO_BECOME_TERRAIN);
  toChangeTerrains.clear();
}

void Visualizer::calculateRoutes() {
  std::chrono::high_resolution_clock::time_point startAt, endAt;
  if (-1 == pf->Reset(x1, y1, x2, y2, options.agentSize, Capability)) {
    spdlog::error("reset path finder failed!");
    return;
  }
  // calculate node route path.
  startAt = std::chrono::high_resolution_clock::now();
  qdpf::CellCollector c = [this](int x, int y) { routes.push_back({x, y}); };
  int ret = pf->ComputeNodeRoutes();
  endAt = std::chrono::high_resolution_clock::now();
  if (-1 == ret) {
    spdlog::info("can't find path (node path not determined)");
    return;
  }
  spdlog::info("node routes calculated, cost {}us",
               std::chrono::duration_cast<std::chrono::microseconds>(endAt - startAt).count());
  // calculate gate route path.
  startAt = std::chrono::high_resolution_clock::now();
  int cost = pf->ComputeGateRoutes(c, options.useNodePath);
  endAt = std::chrono::high_resolution_clock::now();
  std::string more = "";
  if (options.useNodePath)
    more = "(using node path)";
  else
    more = "(not using node path)";
  spdlog::info(
      "routes calculated, cost={} cost {}us {}. please right click anywhere to show full path",
      cost, std::chrono::duration_cast<std::chrono::microseconds>(endAt - startAt).count(), more);
}

void Visualizer::calculatePath() {
  std::chrono::high_resolution_clock::time_point startAt, endAt;
  startAt = std::chrono::high_resolution_clock::now();
  qdpf::CellCollector c = [this](int x, int y) {
    if (path.size()) {
      auto [x2, y2] = path.back();
      if ((x2 == x && y2 == y)) return;
    }
    path.push_back({x, y});
  };
  if (routes.empty()) {
    spdlog::info("routes empty!");
  } else {
    auto [x, y] = routes[0];
    for (int i = 1; i < routes.size(); i++) {
      auto [x2, y2] = routes[i];
      pf->ComputePathToNextRouteCell(x, y, x2, y2, c);
      x = x2, y = y2;
    }
    endAt = std::chrono::high_resolution_clock::now();
    spdlog::info("path calculated, cost {}us",
                 std::chrono::duration_cast<std::chrono::microseconds>(endAt - startAt).count());
  }
}

bool Visualizer::isInsideCamera(int x, int y) {
  return (x >= 0 && x <= windowW && y >= 0 && y <= windowH);
}

bool Visualizer::isOverlapsCamera(const SDL_Rect& rect) {
  return isInsideCamera(rect.x, rect.y) || isInsideCamera(rect.x + rect.w, rect.y + rect.h);
}

void Visualizer::handleCameraMovements() {
  if (scrollDx == 0 && scrollDy == 0) return;
  cameraX += scrollDx;
  cameraY += scrollDy;
  // bounds check.
  cameraX = std::max(0, cameraX);
  cameraY = std::max(0, cameraY);
  cameraX = std::min(cameraX, mapW - windowW);
  cameraY = std::min(cameraY, mapH - windowH);
  // clear after the scroll applied.
  scrollDx = 0;
  scrollDy = 0;
}

void Visualizer::updateRectRelativeToCamera(SDL_Rect& rect) {
  rect.x -= cameraX;
  rect.y -= cameraY;
}

// 1. Draw grid border and terrains.
// 2. Draw gates.
// 3. Draw quadtree leaf node borders.
// 4. Show Path and start/target
// 5. Show cells to invert on mouse down.
void Visualizer::draw() {
  // highlights the route nodes.

  qdpf::NodeVisitor visitor = [this](const qdpf::QdNode* node) {
    int x1 = node->x1, y1 = node->y1, x2 = node->x2, y2 = node->y2;
    int h = (x2 - x1) * GRID_SIZE - 2;
    int w = (y2 - y1) * GRID_SIZE - 2;
    int x = y1 * GRID_SIZE + 1;
    int y = x1 * GRID_SIZE + 1;
    SDL_Rect rect = {x, y, w, h};
    updateRectRelativeToCamera(rect);
    if (isOverlapsCamera(rect)) {
      SDL_SetRenderDrawColor(renderer, 255, 200, 128, 255);  // light orange
      SDL_RenderFillRect(renderer, &rect);
    }
  };
  if (pf->NodePathSize()) pf->VisitComputedNodeRoutes(visitor);

  //  current working map.
  auto mp = mx.Get(options.agentSize, Capability);
  if (mp != nullptr) {
    // Gates.
    qdpf::internal::GateVisitor c2 = [this, &mp](const qdpf::internal::Gate* gate) {
      auto [x1, y1] = mp->UnpackXY(gate->a);
      int x3 = y1 * GRID_SIZE + 1;
      int y3 = x1 * GRID_SIZE + 1;
      SDL_Rect rect = {x3, y3, GRID_SIZE - 2, GRID_SIZE - 2};
      updateRectRelativeToCamera(rect);
      if (isOverlapsCamera(rect)) {
        SDL_SetRenderDrawColor(renderer, 230, 190, 230, 255);  // light purple
        SDL_RenderFillRect(renderer, &rect);
      }
    };

    mp->Gates(c2);
  }

  // Grids and terrains.
  // (i,j) is the cell's position.
  // (x,y) is the grid in pixel (in SDL coordinates).
  for (int i = 0, y = 0; i < options.h; i++, y += GRID_SIZE) {
    for (int j = 0, x = 0; j < options.w; j++, x += GRID_SIZE) {
      SDL_Rect rect = {x, y, GRID_SIZE, GRID_SIZE};
      updateRectRelativeToCamera(rect);
      if (isOverlapsCamera(rect)) {
        SDL_SetRenderDrawColor(renderer, 180, 180, 180, 255);  // light gray
        SDL_RenderDrawRect(renderer, &rect);
      }
      SDL_Rect inner = {x + 1, y + 1, GRID_SIZE - 2, GRID_SIZE - 2};
      updateRectRelativeToCamera(inner);
      if (isOverlapsCamera(inner)) {
        if (GRIDS[i][j] == Terrain::Building) {              // Buildings
          SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255);  // red
          SDL_RenderFillRect(renderer, &inner);
        } else if (GRIDS[i][j] == Terrain::Water) {          // Water
          SDL_SetRenderDrawColor(renderer, 0, 0, 255, 255);  // blue
          SDL_RenderFillRect(renderer, &inner);
        }
      }
    }
  }

  if (mp != nullptr) {
    // Quadtree nodes borders.
    qdpf::internal::QdNodeVisitor c1 = [this](const qdpf::internal::QdNode* node) {
      int x = node->y1 * GRID_SIZE;
      int y = node->x1 * GRID_SIZE;
      int w = (node->y2 - node->y1 + 1) * GRID_SIZE;
      int h = (node->x2 - node->x1 + 1) * GRID_SIZE;
      // Outer liner rectangle (border width 2)
      SDL_Rect rect1 = {x, y, w, h};
      SDL_Rect rect2 = {x + 1, y + 1, w - 2, h - 2};
      updateRectRelativeToCamera(rect1);
      updateRectRelativeToCamera(rect2);
      SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);  // black
      if (isOverlapsCamera(rect1)) SDL_RenderDrawRect(renderer, &rect1);
      if (isOverlapsCamera(rect2)) SDL_RenderDrawRect(renderer, &rect2);
    };
    mp->Nodes(c1);
  }

  // Routes and path.
  auto drawRouteCell = [this](int x, int y) {
    SDL_Rect tmp = {y * GRID_SIZE, x * GRID_SIZE, GRID_SIZE, GRID_SIZE};
    SDL_Rect inner = {tmp.x + 1, tmp.y + 1, tmp.w - 2, tmp.h - 2};
    updateRectRelativeToCamera(inner);
    if (isOverlapsCamera(inner)) {
      SDL_SetRenderDrawColor(renderer, 0, 180, 0, 255);  // green
      SDL_RenderFillRect(renderer, &inner);
    }
  };

  auto drawPathCell = [this](int x, int y) {
    // draw the pathfinding agent.
    auto agentSize = (options.agentSize / 10) * GRID_SIZE;
    SDL_Rect agent{y * GRID_SIZE, x * GRID_SIZE, agentSize, agentSize};
    SDL_Rect agentInner{agent.x + 1, agent.y + 1, agentSize - 2, agentSize - 2};

    updateRectRelativeToCamera(agent);
    if (isOverlapsCamera(agent)) {
      SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);  // black
      SDL_RenderFillRect(renderer, &agent);
    }

    updateRectRelativeToCamera(agentInner);
    if (isOverlapsCamera(agentInner)) {
      SDL_SetRenderDrawColor(renderer, 0, 180, 0, 255);  // green
      SDL_RenderFillRect(renderer, &agentInner);
    }
  };

  switch (state) {
    case 1:  // start is set.
      drawRouteCell(x1, y1);
      break;
    case 2:  // start,target are set, routes are calculated.
      for (auto [x, y] : routes) drawRouteCell(x, y);
      break;
    case 3:  // path cells are calculated.
      for (auto [x, y] : path) drawPathCell(x, y);
      break;
  }

  // Draw cells to invert.
  for (auto [x, y] : toChangeTerrains) {
    SDL_Rect inner = {y * GRID_SIZE + 1, x * GRID_SIZE + 1, GRID_SIZE - 2, GRID_SIZE - 2};
    updateRectRelativeToCamera(inner);
    if (isOverlapsCamera(inner)) {
      if (terrainTypeToChange == Terrain::Water) {         // Water.
        SDL_SetRenderDrawColor(renderer, 0, 0, 255, 255);  // red
      } else {                                             // Wall (Building)
        SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255);  // red
      }
      SDL_RenderFillRect(renderer, &inner);
    }
  }
}
