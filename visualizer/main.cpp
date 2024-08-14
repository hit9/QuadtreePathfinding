#include <SDL2/SDL.h>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <argparse/argparse.hpp>
#include <chrono>
#include <cstring>
#include <string>
#include <utility>
#include <vector>

#include "quadtree_astar.hpp"

// pixels per grid side.
int GRID_SIZE = 16;

// Max value of w and h.
const int N = 1000;

int GRIDS[N][N];

// The cells will being invert their values.
int TO_INVERT_OBSTACLES[N][N];

using Cell = std::pair<int, int>;  // {x,y}

struct Options {
  int w = 10, h = 10, step = 1;
  int maxNodeWidth = -1, maxNodeHeight = -1;
  int createWallsOnInit = 0;
};

// Parse options from command line.
// Returns 0 on success.
int ParseOptionsFromCommandline(int argc, char* argv[], Options& options);

class Visualizer {
 public:
  Visualizer(quadtree_astar::QuadtreeMap& mp, quadtree_astar::IPathFinder* pf, Options& options);
  int Init();
  void Start();
  void Destroy();

 private:
  quadtree_astar::QuadtreeMap& mp;
  quadtree_astar::IPathFinder* pf;
  Options& options;
  SDL_Window* window;
  SDL_Renderer* renderer;
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

  // to invert obstacles
  std::vector<Cell> toInverts;
  // Is mouse left button down?
  bool isMouseDown = false;

  void draw();
  int handleInputs();
  void handleInvertObstacles();
  void recordToInvertObstacelCell(int x, int y);
  void calculateRoutes();
  void calculatePath();
  void reset();
  void createsWallsOnInit();
};

int main(int argc, char* argv[]) {
  memset(GRIDS, 0, sizeof GRIDS);
  // Parse arguments.
  Options options;
  if (ParseOptionsFromCommandline(argc, argv, options) != 0) return -1;
  // Quadtree map.
  quadtree_astar::QuadtreeMap mp(
      options.w, options.h, [](int x, int y) -> bool { return GRIDS[x][y]; },
      quadtree_astar::EuclideanDistance<10>, options.step, options.maxNodeWidth,
      options.maxNodeHeight);
  // Path finder.
  quadtree_astar::AStarPathFinder pf(mp);
  // Visualizer.
  Visualizer visualizer(mp, &pf, options);
  if (visualizer.Init() != 0) return -1;
  visualizer.Start();
  visualizer.Destroy();
  return 0;
}

int ParseOptionsFromCommandline(int argc, char* argv[], Options& options) {
  argparse::ArgumentParser program("quadtree-astar-visualizer");
  program.add_argument("-w", "--width")
      .help("width of grid map")
      .default_value(8)
      .store_into(options.w);
  program.add_argument("-h", "--height")
      .help("height of grid map")
      .default_value(8)
      .store_into(options.h);
  program.add_argument("-s", "--step")
      .help("step to pick gates")
      .default_value(3)
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

Visualizer::Visualizer(quadtree_astar::QuadtreeMap& mp, quadtree_astar::IPathFinder* pf,
                       Options& options)
    : mp(mp), pf(pf), options(options) {}

int Visualizer::Init() {
  // Init SDL
  if (SDL_Init(SDL_INIT_EVERYTHING) != 0) {
    spdlog::error("SDL init error: {}", SDL_GetError());
    return -1;
  }
  // Creates window.
  int window_w = options.w * GRID_SIZE;
  int window_h = options.h * GRID_SIZE;
  window = SDL_CreateWindow("quadtree-astar visualizer", SDL_WINDOWPOS_CENTERED,
                            SDL_WINDOWPOS_CENTERED, window_w, window_h, SDL_WINDOW_SHOWN);
  if (window == nullptr) {
    spdlog::error("Create window error: {}", SDL_GetError());
    SDL_Quit();
    return -3;
  }
  // Creates renderer.
  renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_SOFTWARE);
  if (renderer == nullptr) {
    spdlog::error("Create renderer error: {}", SDL_GetError());
    SDL_DestroyWindow(window);
    SDL_Quit();
    return -1;
  }
  // Build the quadtree map.
  spdlog::info("Visualizer init done");
  mp.RegisterGraph(pf->GetGraph());
  mp.Build();
  spdlog::info("quadtree-astar path finder build done");

  if (options.createWallsOnInit > 0) createsWallsOnInit();
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

    // Background: white
    SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
    SDL_RenderClear(renderer);
    draw();
    SDL_RenderPresent(renderer);

    SDL_Delay(10);  // 10ms
  }
}

// helper function to get mouse position.
std::pair<int, int> GetMousePosition(SDL_Event& e) {
  return {e.button.y / GRID_SIZE, e.button.x / GRID_SIZE};
}

int Visualizer::handleInputs() {
  SDL_Event e;
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
        break;
      case SDL_MOUSEBUTTONUP:
        if (e.button.button == SDL_BUTTON_LEFT) {
          if (isMouseDown) {
            auto [x, y] = GetMousePosition(e);
            recordToInvertObstacelCell(x, y);
            handleInvertObstacles();
            isMouseDown = false;
          }
        }
        break;
      case SDL_MOUSEMOTION:
        if (isMouseDown) {
          recordToInvertObstacelCell(e.button.y / GRID_SIZE, e.button.x / GRID_SIZE);
        }
        break;
      case SDL_MOUSEBUTTONDOWN:
        if (e.button.button == SDL_BUTTON_LEFT) {  // Invert obstacles.
          isMouseDown = true;
          auto [x, y] = GetMousePosition(e);
          recordToInvertObstacelCell(x, y);
        } else if (e.button.button == SDL_BUTTON_RIGHT) {  // Set start/target
          auto [x, y] = GetMousePosition(e);
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
  path.clear();
  routes.clear();
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
      GRIDS[x][y] ^= 1;
      mp.Update(x, y);
    }
    if (x1 == 0)
      x1 += w, x2 += w;
    else {
      x1 = 0, x2 -= w;
    }
  }
}

void Visualizer::recordToInvertObstacelCell(int x, int y) {
  if (x >= 0 && x < options.h && y >= 0 && y < options.w && !TO_INVERT_OBSTACLES[x][y]) {
    TO_INVERT_OBSTACLES[x][y] = 1;
    toInverts.push_back({x, y});
  }
}

void Visualizer::handleInvertObstacles() {
  for (auto [x, y] : toInverts) {
    GRIDS[x][y] ^= 1;
    std::string op = GRIDS[x][y] ? "added" : "removed";
    mp.Update(x, y);
    spdlog::info("{} obstacle {},{}", op, x, y);
  }
  memset(TO_INVERT_OBSTACLES, 0, sizeof TO_INVERT_OBSTACLES);
  toInverts.clear();
}

void Visualizer::calculateRoutes() {
  std::chrono::high_resolution_clock::time_point startAt, endAt;

  startAt = std::chrono::high_resolution_clock::now();
  quadtree_astar::CellCollector c = [this](int x, int y) { routes.push_back({x, y}); };
  pf->ComputeRoutes(x1, y1, x2, y2, c);
  endAt = std::chrono::high_resolution_clock::now();
  spdlog::info("routes calculated, cost {}us. please right click anywhere to show full path",
               std::chrono::duration_cast<std::chrono::microseconds>(endAt - startAt).count());
}

void Visualizer::calculatePath() {
  std::chrono::high_resolution_clock::time_point startAt, endAt;
  startAt = std::chrono::high_resolution_clock::now();
  quadtree_astar::CellCollector c = [this](int x, int y) {
    if (path.size()) {
      auto [x2, y2] = path.back();
      if ((x1 == x && y1 == y)) return;
    }
    path.push_back({x, y});
  };
  if (routes.empty()) {
    spdlog::info("routes empty!");
  } else {
    auto [x, y] = routes[0];
    for (int i = 1; i < routes.size(); i++) {
      auto [x2, y2] = routes[i];
      pf->ComputePathToNextRoute(x, y, x2, y2, c);
      x = x2, y = y2;
    }
    endAt = std::chrono::high_resolution_clock::now();
    spdlog::info("path calculated, cost {}us",
                 std::chrono::duration_cast<std::chrono::microseconds>(endAt - startAt).count());
  }
}

// 1. Draw grid border and obstacles.
// 2. Draw gates.
// 3. Draw quadtree leaf node borders.
// 4. Show Path and start/target
// 5. Show cells to invert on mouse down.
void Visualizer::draw() {
  // Grids and obstacles.
  for (int i = 0, y = 0; i < options.h; i++, y += GRID_SIZE) {
    for (int j = 0, x = 0; j < options.w; j++, x += GRID_SIZE) {
      SDL_Rect rect = {x, y, GRID_SIZE, GRID_SIZE};
      SDL_SetRenderDrawColor(renderer, 180, 180, 180, 255);  // light gray
      SDL_RenderDrawRect(renderer, &rect);
      if (GRIDS[i][j]) {
        SDL_Rect inner = {x + 1, y + 1, GRID_SIZE - 2, GRID_SIZE - 2};
        SDL_SetRenderDrawColor(renderer, 64, 64, 64, 255);  // gray
        SDL_RenderFillRect(renderer, &inner);
      }
    }
  }
  // Gates.
  quadtree_astar::CellCollector c2 = [this](int x, int y) {
    SDL_Rect rect = {y * GRID_SIZE + 1, x * GRID_SIZE + 1, GRID_SIZE - 2, GRID_SIZE - 2};
    SDL_SetRenderDrawColor(renderer, 173, 216, 230, 255);  // light blue
    SDL_RenderFillRect(renderer, &rect);
  };

  mp.Gates(c2);
  // Quadtree nodes borders.
  quadtree_astar::QdNodeCollector c1 = [this](const quadtree_astar::QdNode* node) {
    int x = node->y1 * GRID_SIZE;
    int y = node->x1 * GRID_SIZE;
    int w = (node->y2 - node->y1 + 1) * GRID_SIZE;
    int h = (node->x2 - node->x1 + 1) * GRID_SIZE;
    // Outer liner rectangle (border width 2)
    SDL_Rect rect1 = {x, y, w, h};
    SDL_Rect rect2 = {x + 1, y + 1, w - 2, h - 2};
    SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);  // black
    SDL_RenderDrawRect(renderer, &rect1);
    SDL_RenderDrawRect(renderer, &rect2);
  };
  mp.Nodes(c1);

  // Routes and path.
  auto drawRouteCell = [this](int x, int y) {
    SDL_Rect rect = {y * GRID_SIZE, x * GRID_SIZE, GRID_SIZE, GRID_SIZE};
    SDL_Rect inner = {rect.x + 1, rect.y + 1, rect.w - 2, rect.h - 2};
    SDL_SetRenderDrawColor(renderer, 0, 0, 255, 255);  // blue
    SDL_RenderFillRect(renderer, &inner);
  };
  auto drawPathCell = [this](int x, int y) {
    SDL_Rect rect = {y * GRID_SIZE, x * GRID_SIZE, GRID_SIZE, GRID_SIZE};
    SDL_Rect inner = {rect.x + 1, rect.y + 1, rect.w - 2, rect.h - 2};
    SDL_SetRenderDrawColor(renderer, 0, 0, 255, 255);  // blue
    SDL_RenderFillRect(renderer, &inner);
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
  for (auto [x, y] : toInverts) {
    SDL_Rect inner = {y * GRID_SIZE + 1, x * GRID_SIZE + 1, GRID_SIZE - 2, GRID_SIZE - 2};
    SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255);  // red
    SDL_RenderFillRect(renderer, &inner);
  }
}
