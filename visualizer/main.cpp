#include <SDL2/SDL.h>
#include <fmt/format.h>
#include <imgui.h>
#include <imgui_impl_sdl2.h>
#include <imgui_impl_sdlrenderer2.h>
#include <spdlog/spdlog.h>

#include <argparse/argparse.hpp>
#include <chrono>
#include <qdpf.hpp>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

enum Terrain {
  Land = 0b001,
  Water = 0b010,
  Building = 0b100,
};

// Max value of w and h.
const int N = 800;

const int COST_UNIT = 10;

using Cell = std::pair<int, int>;  // {x,y}

struct CommandlineOptions {
  int w = 100, h = 60;
  // number of pixels per grid side.
  int gridSize = 18;
  // directory to fonts.
  std::string fontsPath;
};

// global options.
CommandlineOptions options;

struct Agent {
  int size = 10;
  // the terrain capability for current agent.
  int capability = Terrain::Land;

  void Reset();
};

struct Map {
  // qmx is never changed once set.
  qdpf::QuadtreeMapX* qmx = nullptr;
  // width and height (in cell/grids)
  const int w = 32, h = 32;
  // grid size in pixels
  const int gridSize = 18;
  // the value of GRIDS[x][y] is a terrain type integer.
  int grids[N][N];
  // CHANGES[x][y] stores the terrain value that cell (x,y) is going to change to.
  // 0 for no changes.
  int changes[N][N];
  // step to pick gate cells.
  int step = -1;
  // use the step function.
  bool useStepFunction = true;

  Map(int w, int h, int gridSize, int step = -1, bool useStepFunction = false);
  ~Map();
  void Reset();
  void BuildMapX();  // call only once.
  // Change Terrain
  void WantChangeTerrain(const Cell& cell, Terrain to);
  void ApplyChangeTerrain(const std::vector<Cell>& cells);
  void ClearAllTerrains();
};

struct AStarContext {
  // never change once set.
  qdpf::AStarPathFinder* pf = nullptr;
  bool isPfReset = false;
  // start cell
  int x1 = 0, y1 = 0;
  // target cell
  int x2 = 0, y2 = 0;
  // ~~~~~~ results ~~~~~~
  std::vector<const qdpf::QdNode*> nodePath;
  std::vector<Cell> gatePath;
  std::vector<Cell> finalPath;

  ~AStarContext();
  void InitPf(qdpf::QuadtreeMapX* qmx);
  void ClearResults();
  void Reset();
  int ResetPf(int agentSize, int capabilities);
};

template <typename V>
using FlowField = std::unordered_map<V, std::pair<V, int>>;  // V => {Next, cost}

struct FlowFieldContext {
  qdpf::FlowFieldPathFinder* pf = nullptr;
  // target cell.
  int x2 = 0, y2 = 0;
  // dest rectangle.
  qdpf::Rectangle dest;
  // optional start cell.
  int x1 = -1, y1 = -1;
  // ~~~~~~ results ~~~~~~
  FlowField<qdpf::QdNode*> nodeFlowField;
  FlowField<int> gateFlowField;
  FlowField<int> finalFlowField;

  ~FlowFieldContext();
  void InitPf(qdpf::QuadtreeMapX* qmx);
  int ResetPf(int agentSize, int capabilities);
  void ClearResults();
  void Reset();
};

// SDL coordinates, in pixels
struct Camera {
  // size of the camera window in pixels.
  const int w, h;
  // size of the whole  map in pixel.
  const int mpw, mph;

  // left corner (x,y), in pixel-level, on the whole map.
  int x = 0, y = 0;
  // changes.
  int dx = 0, dy = 0;

  Camera(int w, int h, int mpw, int mph);

  // Movements
  void MoveUp();
  void MoveDown();
  void MoveLeft();
  void MoveRight();

  // Update should be called after lots of MoveXX calls
  void Update();
};

// Interaction states.
enum class State {
  // Idle => AStarWaitStart | FlowFieldWaitDestLeftTop
  Idle = 0,

  DrawingBuildings = 1,
  DrawingWaters = 2,

  // ~~~~ AStar ~~~~
  AStarWaitStart = 21,          // => AStarWaitTarget
  AStarWaitTarget = 22,         // => AStarWaitCompution
  AStarWaitCompution = 23,      // => AStarNodePathComputed  | AStarGatePathComputed
  AStarNodePathComputed = 25,   // => AStarGatePathComputed
  AStarGatePathComputed = 26,   // => AStarFinalPathComputed
  AStarFinalPathComputed = 27,  // => Idle | AStarWaitStart

  // ~~~~ FlowField ~~~~
  FlowFieldWaitDestLeftTop = 31,      // => FlowFieldWaitDestRightBottom
  FlowFieldWaitDestRightBottom = 32,  // => FlowFieldWaitTarget
  FlowFieldWaitTarget = 33,           // => FlowFieldNodeLevelComputed
  FlowFieldNodeLevelComputed = 37,    // => FlowFieldGateLevelComputed
  FlowFieldGateLevelComputed = 38,    // => FlowFieldFinalLevelComputed
  FlowFieldFinalLevelComputed = 39,   // => Idle | FlowFieldWaitDestRightBottom
};

std::string StateToString(State state);

enum class PathFinderFlag {
  AStar = 0,
  FlowField = 1,
};

const SDL_Color LightGray{180, 180, 180, 255};
const SDL_Color Red{255, 0, 0, 255};
const SDL_Color Blue{0, 0, 255, 255};
const SDL_Color Black{0, 0, 0, 255};
const SDL_Color Green{0, 180, 0, 255};
const SDL_Color LightOrange{255, 200, 128, 255};
const SDL_Color Orange{255, 165, 0, 255};
const SDL_Color LightPurple{230, 190, 230, 255};

const ImVec4 ImRed(0.7f, 0.2f, 0.2f, 1.0f);
const ImVec4 ImWhite(1.0f, 1.0f, 1.0f, 1.0f);
const ImVec4 ImGreen(0.0f, 1.0f, 0.0f, 1.0f);
const ImVec4 ImBlue(0.2f, 0.2f, 0.7f, 1.0f);

class Visualizer {
 public:
  // w, h: the size of the grid map.
  // gridSize: the size of one grid.
  Visualizer(int w, int h, int gridSize);
  int Init();
  void Start();
  void Destroy();
  ~Visualizer();

 private:
  State state = State::Idle;
  SDL_Window* window = nullptr;
  SDL_Renderer* renderer = nullptr;
  Camera* camera = nullptr;

  Agent agent;

  Map map;

  AStarContext astar;
  FlowFieldContext flowfield;

  PathFinderFlag pathfinderFlag = PathFinderFlag::AStar;

  bool isMouseDown = false;

  std::string messageHint = "";
  ImVec4 messageHintColor = ImWhite;

  bool stop = false;

  // ~~~~~ changing terrains ~~~~~~
  Terrain changeTo = Terrain::Building;
  std::vector<Cell> changingTerrainCells;

  // ~~~~~~ ui states ~~~~~~~
  bool showClearAllTerrainsConfirm = false;

  // ~~~~~~ imgui ~~~~~~~
  ImFont* largeFont;

  void reset();
  int initSDL();
  int initImgui();

  // ~~~~~~ render the world ~~~~~~~
  void renderWorld();
  void renderGrids();
  void renderQuadtreeNodes();
  void renderGates();
  void renderHighlightedNodes();
  void renderHighlightedNodesAstar();
  void renderPathfindingDispatch();
  void renderPathfindingAStar();
  void renderPathfindingFlowField();
  void renderDrawRect(const SDL_Rect& rect, const SDL_Color& color);
  void renderFillRect(const SDL_Rect& rect, const SDL_Color& color);
  void setMessageHint(std::string_view message, const ImVec4& color);
  // ~~~~~~ render the panel ~~~~~~~
  void renderImguiPanel();
  void renderImguiPanelSectionCommon();
  void renderImguiPanelSectionPathFinding();
  void renderImguiPanelSectionPathFindingAStar();
  void renderImguiPanelSectionPathFindingFlowField();
  void renderImguiPanelSectionAgent();
  // ~~~~ camera ~~~~~
  bool cropRectByCamera(const SDL_Rect& rect, SDL_Rect& overlap);
  // ~~~~ astar ~~~~~~~
  void computeAstarNodePath();
  void computeAstarGatePath();
  void computeAstarFinalPath();
  void handleAstarInputBegin();
  // ~~~~~ flowfield ~~~~~~
  void computeNodeFlowField();
  void computeGateFlowField();
  void computeFinalFlowField();
  // ~~~~ interaction ~~~~~~~
  void handleInputs();
  void handleInputsShortcuts(SDL_Event& e);
  void handleInputsForCrameMovementsByKeyBoard(SDL_Event& e);
  void handleInputsForCrameMovementsByMouse(SDL_Event& e);
  void handleInputsDispatchByState(SDL_Event& e);
  void handleInputsChangeTerrains(SDL_Event& e);
  void handleInputsAStarSetStart(SDL_Event& e);
  void handleInputsAStarSetTarget(SDL_Event& e);
  // ~~~~~~~~ handlers ~~~~~~~~~~~
  void handleStartDrawBuildings();
  void handleStartDrawWater();
  void pushTerrainChanges(const Cell&);
  void applyTerrainChanges();
  void handleSwitchPathFinderHandler(PathFinderFlag to);
  void handleChangeAgentSize(int to);
  void handleChangeAgentCompability(int to);
  void handleClearAllTerrains();
  // ~~~~~~ util ~~~~~~~
  std::pair<int, int> getCellAtPixelPosition(int x, int y) const;
  const qdpf::internal::QuadtreeMap* getCurrentQuadtreeMapByAgent() const;
};

// Parse options from command line.
// Returns 0 on success.
int ParseCommandlineOptions(int argc, char* argv[]);

int main(int argc, char* argv[]) {
  if (0 != ParseCommandlineOptions(argc, argv)) return -1;
  Visualizer visualizer(options.w, options.h, options.gridSize);
  if (0 != visualizer.Init()) return -1;
  visualizer.Start();
  visualizer.Destroy();
  return 0;
}

// ~~~~~~~~~~~~~~~~~~~~ Implements ~~~~~~~~~~~~~~~~~~~~~~

void Agent::Reset() {
  size = 10;
  capability = Terrain::Land;
}

Map::Map(int w, int h, int gridSize, int step, bool useStepFunction)
    : w(w), h(h), gridSize(gridSize), step(step), useStepFunction(useStepFunction) {
  Reset();
}

Map::~Map() {
  delete qmx;
  qmx = nullptr;
}

void Map::Reset() {
  // Inits the map, by default:
  // 1. the center are water.
  // 2. there's 2 walls.
  // 3. other cells are land.
  memset(grids, 0, sizeof grids);
  memset(changes, 0, sizeof changes);

  qdpf::Rectangle center{2 * h / 5, 2 * w / 5, h * 3 / 5, w * 3 / 5};
  qdpf::Rectangle wall1{center.x2 + 4, center.y1, center.x2 + 4, center.y2 + 4};
  qdpf::Rectangle wall2{center.x1, center.y2 + 4, center.x2 + 4, center.y2 + 4};

  for (int x = 0; x < h; ++x) {
    for (int y = 0; y < w; ++y) {
      if (x >= center.x1 && x <= center.x2 && y >= center.y1 && y <= center.y2)
        grids[x][y] = Terrain::Water;
      else if (x >= wall1.x1 && x <= wall1.x2 && y >= wall1.y1 && y <= wall1.y2)
        grids[x][y] = Terrain::Building;
      else if (x >= wall2.x1 && x <= wall2.x2 && y >= wall2.y1 && y <= wall2.y2)
        grids[x][y] = Terrain::Building;
      else
        grids[x][y] = Terrain::Land;
    }
  }
  step = -1;
  useStepFunction = true;
}

void Map::BuildMapX() {
  auto stepf = (step == -1) ? [](int z) -> int { return z / 8 + 1; } : nullptr;
  qdpf::QuadtreeMapXSettings settings{
      {COST_UNIT, Terrain::Land},
      {2 * COST_UNIT, Terrain::Land},
      {3 * COST_UNIT, Terrain::Land},
      {COST_UNIT, Terrain::Water},
      {2 * COST_UNIT, Terrain::Water},
      {3 * COST_UNIT, Terrain::Water},
      {COST_UNIT, Terrain::Land | Terrain::Water},
      {2 * COST_UNIT, Terrain::Land | Terrain::Water},
      {3 * COST_UNIT, Terrain::Land | Terrain::Water},
  };
  qmx = new qdpf::QuadtreeMapX(
      w, h, qdpf::EuclideanDistance<COST_UNIT>, [this](int x, int y) { return grids[x][y]; },
      settings, step, stepf);
  qmx->Build();
}

void Map::WantChangeTerrain(const Cell& cell, Terrain to) {
  auto [x, y] = cell;
  changes[x][y] = to;
}

void Map::ApplyChangeTerrain(const std::vector<Cell>& cells) {
  for (auto [x, y] : cells) {
    grids[x][y] = changes[x][y];
    changes[x][y] = 0;
    qmx->Update(x, y);
  }
  qmx->Compute();
}

void Map::ClearAllTerrains() {
  for (int x = 0; x < h; ++x) {
    for (int y = 0; y < w; ++y) {
      grids[x][y] = Terrain::Land;
      qmx->Update(x, y);
    }
  }
  qmx->Compute();
}

void AStarContext::InitPf(qdpf::QuadtreeMapX* qmx) { pf = new qdpf::AStarPathFinder(*qmx); }

AStarContext::~AStarContext() {
  delete pf;
  pf = nullptr;
}

void AStarContext::ClearResults() { nodePath.clear(), gatePath.clear(), finalPath.clear(); }

void AStarContext::Reset() {
  ClearResults();
  x1 = y1 = x2 = y2 = 0;
  isPfReset = false;
}

int AStarContext::ResetPf(int agentSize, int capabilities) {
  if (isPfReset) return 0;
  auto ret = pf->Reset(x1, y1, x2, y2, agentSize, capabilities);
  isPfReset = true;
  return ret;
}

FlowFieldContext::~FlowFieldContext() {
  delete pf;
  pf = nullptr;
}

void FlowFieldContext::InitPf(qdpf::QuadtreeMapX* qmx) {
  pf = new qdpf::FlowFieldPathFinder(*qmx);
}

int FlowFieldContext::ResetPf(int agentSize, int capabilities) {
  return pf->Reset(x2, y2, dest, agentSize, capabilities);
}

void FlowFieldContext::ClearResults() {
  nodeFlowField.clear(), gateFlowField.clear(), finalFlowField.clear();
}

void FlowFieldContext::Reset() {
  ClearResults();
  x2 = y2 = 0;
  x1 = y1 = -1;
  dest = {0, 0, 0, 0};
}

Camera::Camera(int w, int h, int mpw, int mph) : w(w), h(h), mpw(mpw), mph(mph) {}

void Camera::MoveUp() { dy -= 50; }
void Camera::MoveDown() { dy += 50; }
void Camera::MoveLeft() { dx -= 50; }
void Camera::MoveRight() { dx += 50; }

void Camera::Update() {
  if (dx == 0 && dy == 0) return;
  x += dx;
  y += dy;
  // bounds check.
  x = std::max(0, x);
  y = std::max(0, y);
  x = std::min(x, mpw - w);
  y = std::min(y, mph - h);
  // clears the delta changes.
  dx = dy = 0;
}

Visualizer::Visualizer(int w, int h, int gridSize) : map(Map(w, h, gridSize)) {}

Visualizer::~Visualizer() {
  delete camera;
  camera = nullptr;
}

int Visualizer::Init() {
  if (0 != initSDL()) return -1;
  if (0 != initImgui()) return -1;
  spdlog::info("Visualizer Init done.");

  // Build the map.
  map.BuildMapX();
  // Build the pfs;
  astar.InitPf(map.qmx);
  flowfield.InitPf(map.qmx);
  return 0;
}

int Visualizer::initSDL() {
  // Init SDL.
  if (SDL_Init(SDL_INIT_EVERYTHING) != 0) {
    spdlog::error("Error: {}", SDL_GetError());
    return -1;
  }
  // Get display bounds
  SDL_Rect displayBounds;
  if (SDL_GetDisplayBounds(0, &displayBounds) != 0) {
    printf("Failed to get display bounds: %s\n", SDL_GetError());
    return 1;
  }

  // Creates window and camera.
  int mpw = map.w * map.gridSize, mph = map.h * map.gridSize;
  int w = std::min(mpw, displayBounds.w);
  int h = std::min(mph, displayBounds.h);
  window =
      SDL_CreateWindow("quadtree-pathfinding-visualizer", SDL_WINDOWPOS_CENTERED,
                       SDL_WINDOWPOS_CENTERED, w, h, SDL_WINDOW_SHOWN | SDL_WINDOW_ALLOW_HIGHDPI);
  if (window == nullptr) {
    spdlog::error("Create window error: {}", SDL_GetError());
    SDL_Quit();
    return -3;
  }
  camera = new Camera(w, h, mpw, mph);

  // Creates renderer.
  renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
  if (renderer == nullptr) {
    spdlog::error("Create renderer error: {}", SDL_GetError());
    SDL_DestroyWindow(window);
    SDL_Quit();
    return -1;
  }
  return 0;
}

int Visualizer::initImgui() {
  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  auto& io = ImGui::GetIO();
  io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;  // Enable Keyboard Controls
  io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;   // Enable Gamepad Controls
  io.Fonts->AddFontFromFileTTF((options.fontsPath + "/Roboto-Medium.ttf").c_str(), 18);
  largeFont = io.Fonts->AddFontFromFileTTF((options.fontsPath + "/Roboto-Medium.ttf").c_str(), 24);
  // Setup Dear ImGui style
  ImGui::StyleColorsDark();
  // ImGui::StyleColorsLight();
  // Setup Platform/Renderer backends
  ImGui_ImplSDL2_InitForSDLRenderer(window, renderer);
  ImGui_ImplSDLRenderer2_Init(renderer);
  return 0;
}

void Visualizer::Destroy() {
  // deinit ImGui.
  ImGui_ImplSDLRenderer2_Shutdown();
  ImGui_ImplSDL2_Shutdown();
  ImGui::DestroyContext();
  // deinit SDL.
  if (renderer) SDL_DestroyRenderer(renderer);
  if (window) SDL_DestroyWindow(window);
  SDL_Quit();
}

void Visualizer::Start() {
  auto& io = ImGui::GetIO();
  while (!stop) {
    // quit on -1
    handleInputs();
    if (stop) break;
    // update camera.
    if (camera != nullptr) camera->Update();
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

    // ImGui didn't handle this mouse event, turn it to SDL.
    if (!io.WantCaptureMouse) {
      handleInputsForCrameMovementsByMouse(e);
      handleInputsDispatchByState(e);
    }

    // ImGui didn't handle this keyboard event, turn it to SDL.
    if (!io.WantCaptureKeyboard) {
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

void Visualizer::renderImguiPanel() {
  ImGui::Begin("Quadtree Pathfinder Visualizer");

  // Status
  ImGui::PushStyleColor(ImGuiCol_Text, ImWhite);
  ImGui::PushStyleColor(ImGuiCol_HeaderHovered, ImBlue);
  ImGui::PushFont(largeFont);
  ImGui::Selectable(fmt::format("Status: {}", StateToString(state)).c_str(), true);
  ImGui::PopFont();
  ImGui::PopStyleColor(2);

  ImGui::Spacing();

  // Hint
  ImGui::PushStyleColor(ImGuiCol_Text, messageHintColor);
  ImGui::TextWrapped("HINT: %s", messageHint.c_str());
  ImGui::PopStyleColor();
  ImGui::Spacing();

  // Sections.
  renderImguiPanelSectionCommon();
  ImGui::Spacing();
  renderImguiPanelSectionAgent();
  ImGui::Spacing();
  renderImguiPanelSectionPathFinding();

  ImGui::End();
}

void Visualizer::renderImguiPanelSectionCommon() {
  ImGui::SeparatorText("Common");

  // Reset.
  if (ImGui::Button("Reset < ESC >")) reset();
  if (ImGui::IsItemHovered()) {
    ImGui::BeginTooltip();
    ImGui::Text("Clear the compution results and back to idle. (key: ESC)");
    ImGui::EndTooltip();
  }
  ImGui::SameLine();

  // Button: Buildings
  ImGui::PushStyleColor(ImGuiCol_Button, ImRed);
  if (ImGui::Button("Building < B > ")) {
    handleStartDrawBuildings();
  }
  ImGui::PopStyleColor(3);  // rollback color
  if (ImGui::IsItemHovered()) {
    ImGui::BeginTooltip();
    ImGui::Text("Add or Remove building");
    ImGui::EndTooltip();
  }

  ImGui::SameLine();

  // Button: Water
  ImGui::PushStyleColor(ImGuiCol_Button, ImBlue);
  if (ImGui::Button("Water < W >")) {
    handleStartDrawWater();
  }
  ImGui::PopStyleColor(3);  // rollback color
  if (ImGui::IsItemHovered()) {
    ImGui::BeginTooltip();
    ImGui::Text("Add or Remove water");
    ImGui::EndTooltip();
  }

  ImGui::SameLine();

  // Button clear terrains;
  if (ImGui::Button("Clear All Building and Water")) {
    showClearAllTerrainsConfirm = true;
    ImGui::OpenPopup("ClearAllTerrainsConfirmationDialog");
  }
  if (ImGui::IsItemHovered()) {
    ImGui::BeginTooltip();
    ImGui::Text("Clear all building and water on the map");
    ImGui::EndTooltip();
  }
  if (showClearAllTerrainsConfirm) {
    if (ImGui::BeginPopupModal("ClearAllTerrainsConfirmationDialog", &showClearAllTerrainsConfirm,
                               ImGuiWindowFlags_AlwaysAutoResize)) {
      ImGui::Text("Are you sure to clear all terrains to land ?");
      ImGui::Separator();
      if (ImGui::Button("Yes")) {
        handleClearAllTerrains();
        showClearAllTerrainsConfirm = false;
        ImGui::CloseCurrentPopup();
      }
      ImGui::SetItemDefaultFocus();
      ImGui::SameLine();
      if (ImGui::Button("No")) {
        showClearAllTerrainsConfirm = false;
        ImGui::CloseCurrentPopup();
      }

      ImGui::EndPopup();
    }
  }
}

void Visualizer::renderImguiPanelSectionAgent() {
  ImGui::SeparatorText("Agent");

  ImGui::Text("Size: ");
  ImGui::SameLine();
  if (ImGui::RadioButton("10x10", agent.size == 1 * COST_UNIT)) handleChangeAgentSize(COST_UNIT);
  ImGui::SameLine();
  if (ImGui::RadioButton("20x20", agent.size == 2 * COST_UNIT))
    handleChangeAgentSize(2 * COST_UNIT);
  ImGui::SameLine();
  if (ImGui::RadioButton("30x30", agent.size == 3 * COST_UNIT))
    handleChangeAgentSize(3 * COST_UNIT);

  ImGui::Text("Capability: ");
  ImGui::SameLine();
  if (ImGui::RadioButton("Land", agent.capability == Terrain::Land))
    handleChangeAgentCompability(Terrain::Land);
  ImGui::SameLine();
  if (ImGui::RadioButton("Water", agent.capability == Terrain::Water))
    handleChangeAgentCompability(Terrain::Water);
  ImGui::SameLine();
  if (ImGui::RadioButton("Land | Water", agent.capability == (Terrain::Land | Terrain::Water)))
    handleChangeAgentCompability(Terrain::Land | Terrain::Water);
}

void Visualizer::renderImguiPanelSectionPathFinding() {
  ImGui::SeparatorText("Pathfinding");

  if (ImGui::RadioButton("A*", pathfinderFlag == PathFinderFlag::AStar))
    handleSwitchPathFinderHandler(PathFinderFlag::AStar);
  ImGui::SameLine();
  if (ImGui::RadioButton("FlowField", pathfinderFlag == PathFinderFlag::FlowField))
    handleSwitchPathFinderHandler(PathFinderFlag::FlowField);

  switch (pathfinderFlag) {
    case PathFinderFlag::AStar:
      return renderImguiPanelSectionPathFindingAStar();
    case PathFinderFlag::FlowField:
      return renderImguiPanelSectionPathFindingFlowField();
  }
}

void Visualizer::renderImguiPanelSectionPathFindingAStar() {
  if (ImGui::Button("Set Start and Target")) {
    handleAstarInputBegin();
  }
  ImGui::SameLine();
  if (ImGui::Button("Compute Node Path")) {
    computeAstarNodePath();
  }
  ImGui::SameLine();
  if (ImGui::Button("Compute Gate Path")) {
    computeAstarGatePath();
  }
  ImGui::SameLine();
  if (ImGui::Button("Compute Final Path")) {
    computeAstarFinalPath();
  }
}

void Visualizer::renderImguiPanelSectionPathFindingFlowField() {
  // TODO
  if (ImGui::Button("Set Start Rectangle  < S >")) {
    // TODO Drag a rectangle
  }
  if (ImGui::Button("Set Target  < T >")) {
    // TODO
  }
  if (ImGui::Button("Compute Node FlowField")) {
    // TODO
  }
  ImGui::SameLine();
  if (ImGui::Button("Compute Gate FlowField")) {
    // TODO
  }
  ImGui::SameLine();
  if (ImGui::Button("Compute Final FlowField")) {
    // TODO
  }
}

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
    default:
      break;  // avoiding warning
  }
  // TODO:  flow field
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

void Visualizer::renderPathfindingAStar() {
  auto drawCell = [this](int x, int y) {
    SDL_Rect cell{y * map.gridSize + 1, x * map.gridSize + 1, map.gridSize - 2, map.gridSize - 2};
    renderFillRect(cell, Green);
  };

  int agentSizeInPixels = agent.size * map.gridSize / COST_UNIT;

  auto drawAgent = [this, agentSizeInPixels](int x, int y) {
    SDL_Rect outer{y * map.gridSize, x * map.gridSize, agentSizeInPixels, agentSizeInPixels};
    SDL_Rect inner{outer.x + 1, outer.y + 1, outer.w - 2, outer.h - 2};
    renderDrawRect(outer, Black);
    renderFillRect(inner, Green);
  };

  switch (state) {
    case State::AStarWaitTarget:
      drawCell(astar.x1, astar.y1);  // start
      break;
    case State::AStarWaitCompution:
      [[fallthrough]];
    case State::AStarNodePathComputed:
      drawCell(astar.x1, astar.y1);  // start
      drawCell(astar.x2, astar.y2);  // target
      // NOTE: nodepath already render in renderHighlightedNodesAstar.
      break;
    case State::AStarGatePathComputed:
      // draw gate route cells.
      // start and target are included in gate path.
      for (const auto [x, y] : astar.gatePath) {
        drawCell(x, y);
      }
      break;
    case State::AStarFinalPathComputed:  //
      for (const auto [x, y] : astar.finalPath) {
        drawAgent(x, y);
      }
      break;
    default:
      return;  // do nothing
  }
}

void Visualizer::renderPathfindingFlowField() {
  // TODO
  switch (state) {
    default:
      return;  // do nothing
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

// Crop given rect by camera, and converts to the coordinates relative to the camera's left-top
// corner. The results are stored into overlap. Returns false if no overlaping.
bool Visualizer::cropRectByCamera(const SDL_Rect& rect, SDL_Rect& overlap) {
  qdpf::Rectangle c{camera->x, camera->y, camera->x + camera->w - 1, camera->y + camera->h - 1};
  qdpf::Rectangle a{rect.x, rect.y, rect.x + rect.w - 1, rect.y + rect.h - 1};
  qdpf::Rectangle d;
  auto b = qdpf::internal::GetOverlap(c, a, d);
  if (!b) return false;
  overlap.x = d.x1, overlap.y = d.y1;
  overlap.w = d.x2 - d.x1 + 1, overlap.h = d.y2 - d.y1 + 1;
  overlap.x -= camera->x;
  overlap.y -= camera->y;
  return true;
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
  state = State::DrawingBuildings;
  changeTo = Terrain::Building;
  setMessageHint("click or drag mouse to draw buildings!", ImGreen);
}

void Visualizer::handleStartDrawWater() {
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
  if (state != State::Idle) reset();
  state = State::AStarWaitStart;
  setMessageHint("A*: waiting to click a start cell", ImGreen);
}

void Visualizer::computeAstarNodePath() {
  if (state != State::AStarWaitCompution && state != State::AStarNodePathComputed) {
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
  state = State::AStarNodePathComputed;
  if (cost == -1) {
    setMessageHint("A*: unreachable!", ImRed);
    return;
  }

  if (astar.nodePath.size()) astar.nodePath.clear();
  qdpf::NodeVisitor visitor = [this](const qdpf::QdNode* node) { astar.nodePath.push_back(node); };
  if (astar.pf->NodePathSize()) astar.pf->VisitComputedNodeRoutes(visitor);

  endAt = std::chrono::high_resolution_clock::now();

  setMessageHint(
      fmt::format(
          "A*: Node path computed! cost {}us ; Next we can click button < Compute Gate Path >.",
          std::chrono::duration_cast<std::chrono::microseconds>(endAt - startAt).count()),
      ImGreen);
}

void Visualizer::computeAstarGatePath() {
  if (state != State::AStarWaitCompution && state != State::AStarNodePathComputed &&
      state != State::AStarGatePathComputed) {
    setMessageHint("invalid state", ImRed);
    return;
  }
  if (0 != astar.ResetPf(agent.size, agent.capability)) {
    setMessageHint("internal error: astar reset failure", ImRed);
    return;
  }

  std::chrono::high_resolution_clock::time_point startAt, endAt;
  startAt = std::chrono::high_resolution_clock::now();

  if (astar.gatePath.size()) astar.gatePath.clear();
  bool useNodePath = astar.pf->NodePathSize() > 0;
  qdpf::CellCollector collector = [this](int x, int y) { astar.gatePath.push_back({x, y}); };
  int cost = astar.pf->ComputeGateRoutes(collector, useNodePath);
  state = State::AStarGatePathComputed;
  endAt = std::chrono::high_resolution_clock::now();
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
  if (state != State::AStarGatePathComputed && state != State::AStarNodePathComputed) {
    setMessageHint("invalid state", ImRed);
    return;
  }
  if (astar.gatePath.empty()) {
    setMessageHint("A*: empty gate route cells!", ImRed);
    return;
  }
  std::chrono::high_resolution_clock::time_point startAt, endAt;
  startAt = std::chrono::high_resolution_clock::now();
  if (astar.finalPath.size()) astar.finalPath.clear();
  qdpf::CellCollector collector = [this](int x, int y) {
    if (astar.finalPath.size()) {
      auto [x2, y2] = astar.finalPath.back();
      if ((x2 == x && y2 == y)) return;
    }
    astar.finalPath.push_back({x, y});
  };
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
    case State::FlowFieldWaitDestLeftTop:
      return "(FlowField) Wait Input Dest Rectangle (left-top)";
    case State::FlowFieldWaitDestRightBottom:
      return "(FlowField) Wait Input Dest Rectangle (right-bottom)";
    case State::FlowFieldWaitTarget:
      return "(FlowField) Wait Input Target";
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

int ParseCommandlineOptions(int argc, char* argv[]) {
  argparse::ArgumentParser program("quadtree-pathfinding-visualizer");
  program.add_argument("-w", "--width")
      .help("width of grid map")
      .default_value(60)
      .store_into(options.w);
  program.add_argument("-h", "--height")
      .help("height of grid map")
      .default_value(40)
      .store_into(options.h);
  program.add_argument("-gs", "--grid-cell-size-in-pixels")
      .help("grid cell size in pixels")
      .default_value(18)
      .store_into(options.gridSize);
  program.add_argument("-fonts", "--fonts-dir-path")
      .help("the relative fonts dir path")
      .default_value(std::string("./visualizer/fonts"))
      .store_into(options.fontsPath);
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
