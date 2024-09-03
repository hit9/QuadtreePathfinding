#ifndef QUADTREE_PATHFINDING_VISUALIZER_HPP
#define QUADTREE_PATHFINDING_VISUALIZER_HPP

#include <SDL2/SDL.h>
#include <SDL2/SDL_render.h>
#include <SDL_ttf.h>
#include <imgui.h>

#include <string>
#include <utility>

#include "qdpf.hpp"

enum Terrain {
  Land = 0b001,
  Water = 0b010,
  Building = 0b100,
};

// Max value of w and h.
const int N = 800;

// Cost unit between neighour cells.
// better to set a value > 10.
const int COST_UNIT = 100;

// {x,y}
using Cell = std::pair<int, int>;

// SDL Colors

const SDL_Color LightGray{180, 180, 180, 255};
const SDL_Color Red{255, 0, 0, 255};
const SDL_Color Blue{0, 0, 255, 255};
const SDL_Color Black{0, 0, 0, 255};
const SDL_Color Green{0, 180, 0, 255};
const SDL_Color LightOrange{255, 200, 128, 255};
const SDL_Color Orange{255, 165, 0, 255};
const SDL_Color LightPurple{230, 190, 230, 255};

// IMGUI Colors

const ImVec4 ImRed(0.7f, 0.2f, 0.2f, 1.0f);
const ImVec4 ImWhite(1.0f, 1.0f, 1.0f, 1.0f);
const ImVec4 ImGreen(0.0f, 1.0f, 0.0f, 1.0f);
const ImVec4 ImBlue(0.2f, 0.2f, 0.7f, 1.0f);

struct CommandlineOptions {
  int w = 100, h = 70;
  // number of pixels per grid side.
  int gridSize = 18;
  // directory to fonts.
  std::string fontsPath;
  // step to pick gates, -1 for using stepf.
  int step = -1;
};

// global options.
extern CommandlineOptions options;

struct Agent {
  int size = COST_UNIT;
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
  // the value of grids[x][y] is a terrain type integer.
  int grids[N][N];
  // CHANGES[x][y] stores the terrain value that cell (x,y) is going to change to.
  // 0 for no changes.
  int changes[N][N];
  // step to pick gate cells.
  int step = -1;

  Map(int w, int h, int gridSize, int step = -1);
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
struct FlowFieldItem {
  V current;
  V next;
  int cost;
};

struct FlowFieldContext {
  qdpf::FlowFieldPathFinder* pf = nullptr;

  // target cell.
  int x2 = 0, y2 = 0;
  // query rectangle.
  qdpf::Rectangle qrange;

  // ~~~~~~ results ~~~~~~
  std::vector<FlowFieldItem<const qdpf::QdNode*>> nodeFlowField;
  std::vector<FlowFieldItem<Cell>> gateFlowField;
  std::vector<FlowFieldItem<Cell>> finalFlowField;
  std::unordered_map<Cell, Cell, qdpf::internal::PairHasher<int, int>> finalFlowNextMap;

  bool isPfReset = false;

  // ~~~~~ optional test path ~~~~~~
  std::vector<std::vector<Cell>> testPaths;

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

struct ArrowFont {
  TTF_Font* font = nullptr;
  SDL_Texture* texture;
  // width and offset of each arrwo
  int w[8];
  int offset[8];
  // font height
  int h;
};

// char in arrows font.
// 0 A →
// 1 B ←
// 2 C ↑
// 3 D ↓
// 4 E ↖
// 5 F ↙
// 6 G ↗
// 7 H ↘
const char ARROWS_CHAR[9] = "ABCDEFGH";

// (dx+1)*3+(dy+1) => index in ARROWS_CHAR
const int ARROWS_DIRECTIONS[9] = {
    4,   // 0  ↖,
    2,   // 1   ↑
    6,   // 2   ↗
    1,   // 3   ←
    -1,  // NA
    0,   // 5   →
    5,   // 6   ↙
    3,   // 7   ↓
    7,   // 8   ↘,
};

// Interaction states.
enum class State {
  // Idle => AStarWaitStart | FlowFieldWaitQrangeLeftTop
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
  FlowFieldWaitQrangeLeftTop = 31,      // => FlowFieldWaitQrangeRightBottom
  FlowFieldWaitQrangeRightBottom = 32,  // => FlowFieldWaitTarget
  FlowFieldWaitTarget = 33,             // => FlowFieldWaitCompution
  FlowFieldWaitCompution = 34,       // => FlowFieldNodeLevelComputed | FlowFieldGateLevelComputed
  FlowFieldNodeLevelComputed = 37,   // => FlowFieldGateLevelComputed
  FlowFieldGateLevelComputed = 38,   // => FlowFieldFinalLevelComputed
  FlowFieldFinalLevelComputed = 39,  // => Idle | FlowFieldWaitQrangeRightBottom
};

std::string StateToString(State state);

enum class PathFinderFlag {
  AStar = 0,
  FlowField = 1,
};

class Visualizer {
 public:
  // w, h: the size of the grid map.
  // gridSize: the size of one grid.
  Visualizer(int w, int h, int gridSize, int step = -1);
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
  bool renderFlowFieldGateNextLines = false;
  bool hideTerrainRenderings = false;
  bool showGateGraph = false;

  // ~~~~~~ imgui ~~~~~~~
  ImFont* largeFont;

  // ~~~~~~ misc ~~~~~~~
  ArrowFont arrows;

  void reset();
  int initSDL();
  int initImgui();
  int initArrowsFont();
  void destroyArrowsFont();
  void destroyImgui();
  void destroySDL();

  // ~~~~~~ render the world ~~~~~~~
  void renderWorld();
  void renderGrids();
  void renderQuadtreeNodes();
  void renderGates();
  void renderGateGraph();
  void renderHighlightedNodes();
  void renderHighlightedNodesAstar();
  void renderHighlightedNodesFlowField();
  void renderPathfindingDispatch();
  void renderPathfindingAStar();
  void renderPathfindingFlowField();
  void renderPathFindingFlowFieldGateField();
  void renderPathFindingFlowFieldFinalField();
  void renderPathFindingFlowFieldGateNextsLines();
  void renderDrawRect(const SDL_Rect& rect, const SDL_Color& color);
  void renderFillRect(const SDL_Rect& rect, const SDL_Color& color);
  void renderDrawLine(int x1, int y1, int x2, int y2, const SDL_Color& color);
  void renderDrawLineBetweenCells(int x1, int y1, int x2, int y2, const SDL_Color& color);
  void renderCopy(SDL_Texture* texture, const SDL_Rect& src, const SDL_Rect& dst);
  void renderFillCell(int x, int y, const SDL_Color& color);
  void renderFillAgent(int x, int y);
  void setMessageHint(std::string_view message, const ImVec4& color);

  // ~~~~~~ render the panel ~~~~~~~
  void renderImguiPanel();
  void renderImguiPanelSectionCommon();
  void renderImguiPanelSectionPathFinding();
  void renderImguiPanelSectionPathFindingAStar();
  void renderImguiPanelSectionPathFindingFlowField();
  void renderImguiPanelSectionAgent();

  // ~~~~ camera ~~~~~
  bool cropRectByCamera(const SDL_Rect& rect, SDL_Rect& overlap,
                        bool marginToCameraCoordinates = true);

  // ~~~~ astar ~~~~~~~
  void computeAstarNodePath();
  void computeAstarGatePath();
  void computeAstarFinalPath();
  void handleAstarInputBegin();
  void handleFlowFieldInputQueryRangeBegin();

  // ~~~~~ flowfield ~~~~~~
  void computeNodeFlowField();
  void computeGateFlowField();
  void computeFinalFlowField();
  void handlePlayFlowFieldTestPath();

  // ~~~~ interaction ~~~~~~~
  void handleInputs();
  void handleInputsShortcuts(SDL_Event& e);
  void handleInputsForCrameMovementsByKeyBoard(SDL_Event& e);
  void handleInputsForCrameMovementsByMouse(SDL_Event& e);
  void handleInputsDispatchByState(SDL_Event& e);
  void handleInputsChangeTerrains(SDL_Event& e);
  void handleInputsAStarSetStart(SDL_Event& e);
  void handleInputsAStarSetTarget(SDL_Event& e);
  void handleInputsFlowFieldSetQrangeLeftTop(SDL_Event& e);
  void handleInputsFlowFieldSetQrangeRightBottom(SDL_Event& e);
  void handleInputsFlowFieldSetTarget(SDL_Event& e);
  void handleInputsFlowFieldSetTestStart(SDL_Event& e);

  // ~~~~~~~~ handlers ~~~~~~~~~~~
  void handleLogics();
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

#endif
