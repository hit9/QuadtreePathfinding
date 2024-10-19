#ifndef QUADTREE_PATHFINDING_VISUALIZER_HPP
#define QUADTREE_PATHFINDING_VISUALIZER_HPP

#include <SDL2/SDL.h>
#include <SDL2/SDL_render.h>
#include <SDL_ttf.h>
#include <imgui.h>

#include <string>
#include <utility>

#include "Naive/Astar.h"
#include "Naive/Flowfield.h"
#include "Naive/GridMap.h"
#include "QDPF.h"

enum Terrain
{
	Land = 0b001,
	Water = 0b010,
	Building = 0b100,
};

// Max value of w and h.
const int N = 800;

// {x,y}
using Cell = std::pair<int, int>;

// SDL Colors

const SDL_Color LightGray{ 180, 180, 180, 255 };
const SDL_Color Red{ 255, 0, 0, 255 };
const SDL_Color Blue{ 0, 0, 255, 255 };
const SDL_Color Black{ 0, 0, 0, 255 };
const SDL_Color Green{ 0, 180, 0, 255 };
const SDL_Color DarkGreen{ 0, 100, 0, 255 };
const SDL_Color LightOrange{ 255, 200, 128, 255 };
const SDL_Color Orange{ 255, 165, 0, 255 };
const SDL_Color LightPurple{ 230, 190, 230, 255 };

// IMGUI Colors

const ImVec4 ImRed(0.7f, 0.2f, 0.2f, 1.0f);
const ImVec4 ImWhite(1.0f, 1.0f, 1.0f, 1.0f);
const ImVec4 ImGreen(0.0f, 1.0f, 0.0f, 1.0f);
const ImVec4 ImBlue(0.2f, 0.2f, 0.7f, 1.0f);

struct CommandlineOptions
{
	int w = 100, h = 70;
	// number of pixels per grid side.
	int gridSize = 18;
	// directory to fonts.
	std::string fontsPath;
	// step to pick gates, -1 for using stepf.
	int step = -1;
	// clearance field:
	// 0 for TrueClearanceField (the default)
	// 1 for BrushfireClearanceField
	int clearanceFieldFlag = 0;
};

// global options.
extern CommandlineOptions options;

struct Agent
{
	int size = 1;
	// the terrain capability for current agent.
	int capability = Terrain::Land;

	void Reset();
};

struct Map
{
	// qmx is never changed once set.
	QDPF::QuadtreeMapX* qmx = nullptr;
	// width and height (in cell/grids)
	const int w = 32, h = 32;
	// grid size in pixels
	const int gridSize = 18;
	// the value of grids[y][x] is a terrain type integer.
	int grids[N][N];
	// CHANGES[y][x] stores the terrain value that cell (x,y) is going to change to.
	// 0 for no changes.
	int changes[N][N];
	// step to pick gate cells.
	int step = -1;

	// naive map for comparasion
	QDPF::Naive::NaiveGridMap* naiveMap = nullptr;

	Map(int w, int h, int gridSize, int step = -1);
	~Map();
	void Build(); // call only once.
	// Change Terrain
	void WantChangeTerrain(const Cell& cell, Terrain to);
	void ApplyChangeTerrain(const std::vector<Cell>& cells);
	void ClearAllTerrains();
};

struct NaiveAStarContext
{
	QDPF::Naive::NaiveAStarPathFinder pf;
	// timecost of naive astar in us.
	std::chrono::microseconds timeCost = std::chrono::microseconds(0);
	// results of naive astar.
	std::vector<Cell> path;

	void Reset();
};

struct AStarContext
{
	// never change once set.
	QDPF::AStarPathFinder* pf = nullptr;
	bool				   isPfReset = false;

	// start cell
	int x1 = 0, y1 = 0;
	// target cell
	int x2 = 0, y2 = 0;

	// Cumulative call time
	std::chrono::microseconds timeCost = std::chrono::microseconds(0);

	// ~~~~~~ results ~~~~~~
	QDPF::NodePath	  nodePath;
	QDPF::GatePath	  gatePath;
	std::vector<Cell> finalPath;

	~AStarContext();
	void InitPf(QDPF::QuadtreeMapX* qmx);
	void ClearResults();
	void Reset();
	int	 ResetPf(int agentSize, int capabilities);
};

template <typename V>
struct FlowFieldItem
{
	V	current;
	V	next;
	int cost;
};

struct NaiveFlowFieldContext
{
	QDPF::Naive::NaiveFlowFieldPathFinder pf;
	// timecost of naive flowfield in us.
	std::chrono::microseconds timeCost = std::chrono::microseconds(0);
	// results of naive flowfield
	QDPF::FinalFlowField finalFlowField;

	// ~~~~~ optional test path ~~~~~~
	std::vector<std::vector<Cell>> testPaths;

	void Reset();
};

struct FlowFieldContext
{
	QDPF::FlowFieldPathFinder* pf = nullptr;

	// target cell.
	int x2 = 0, y2 = 0;
	// query rectangle.
	QDPF::Rectangle qrange;

	// Cumulative call time
	std::chrono::microseconds timeCost = std::chrono::microseconds(0);

	// ~~~~~~ results ~~~~~~
	QDPF::NodeFlowField	 nodeFlowField;
	QDPF::GateFlowField	 gateFlowField;
	QDPF::FinalFlowField finalFlowField;

	bool isPfReset = false;

	// ~~~~~ optional test path ~~~~~~
	std::vector<std::vector<Cell>> testPaths;

	~FlowFieldContext();
	void InitPf(QDPF::QuadtreeMapX* qmx);
	int	 ResetPf(int agentSize, int capabilities);
	void ClearResults();
	void Reset();
};

// SDL coordinates, in pixels
struct Camera
{
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
	void MoveUp(int k = 50);
	void MoveDown(int k = 50);
	void MoveLeft(int k = 50);
	void MoveRight(int k = 50);
	void MoveToLeftMost();
	void MoveToRightMost();

	// Update should be called after lots of MoveXX calls
	void Update();
};

struct ArrowFont
{
	TTF_Font*	 font = nullptr;
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

// (dy+1)*3+(dx+1) => index in ARROWS_CHAR
const int ARROWS_DIRECTIONS[9] = {
	4,	// 0  ↖,
	2,	// 1   ↑
	6,	// 2   ↗
	1,	// 3   ←
	-1, // NA
	0,	// 5   →
	5,	// 6   ↙
	3,	// 7   ↓
	7,	// 8   ↘,
};

// Interaction states.
enum class State
{
	// Idle => AStarWaitStart | FlowFieldWaitQrangeLeftTop
	Idle = 0,

	DrawingBuildings = 1,
	DrawingWaters = 2,

	// ~~~~ AStar ~~~~
	AStarWaitStart = 21,		 // => AStarWaitTarget
	AStarWaitTarget = 22,		 // => AStarWaitCompution
	AStarWaitCompution = 23,	 // => AStarNodePathComputed  | AStarGatePathComputed
	AStarNodePathComputed = 25,	 // => AStarGatePathComputed
	AStarGatePathComputed = 26,	 // => AStarFinalPathComputed
	AStarFinalPathComputed = 27, // => Idle | AStarWaitStart

	// ~~~~ FlowField ~~~~
	FlowFieldWaitQrangeLeftTop = 31,	 // => FlowFieldWaitQrangeRightBottom
	FlowFieldWaitQrangeRightBottom = 32, // => FlowFieldWaitTarget
	FlowFieldWaitTarget = 33,			 // => FlowFieldWaitCompution
	FlowFieldWaitCompution = 34,		 // => FlowFieldNodeLevelComputed | FlowFieldGateLevelComputed
	FlowFieldNodeLevelComputed = 37,	 // => FlowFieldGateLevelComputed
	FlowFieldGateLevelComputed = 38,	 // => FlowFieldFinalLevelComputed
	FlowFieldFinalLevelComputed = 39,	 // => Idle | FlowFieldWaitQrangeRightBottom
};

std::string StateToString(State state);

enum class PathFinderFlag
{
	AStar = 0,
	FlowField = 1,
};

// Compare to Naive Astar and FlowField.
using NaiveGraph =
	QDPF::Internal::SimpleUnorderedMapDirectedGraph<Cell, QDPF::Internal::PairHasher<int, int>>;

class Visualizer
{
public:
	// w, h: the size of the grid map.
	// gridSize: the size of one grid.
	Visualizer(int w, int h, int gridSize, int step = -1);
	int	 Init();
	void Start();
	void Destroy();
	~Visualizer();

private:
	State state = State::Idle;

	SDL_Window*	  window = nullptr;
	SDL_Renderer* renderer = nullptr;
	Camera*		  camera = nullptr;

	Agent agent;

	Map map;

	AStarContext	 astar;
	FlowFieldContext flowfield;

	NaiveAStarContext	  astarNaive;
	NaiveFlowFieldContext flowfieldNaive;

	PathFinderFlag pathfinderFlag = PathFinderFlag::AStar;

	bool isMouseDown = false;

	std::string messageHint = "";
	ImVec4		messageHintColor = ImWhite;

	bool stop = false;

	// ~~~~~ changing terrains ~~~~~~
	Terrain			  changeTo = Terrain::Building;
	std::vector<Cell> changingTerrainCells;

	// ~~~~~~ ui states ~~~~~~~
	bool showClearAllTerrainsConfirm = false;
	bool renderFlowFieldGateNextLines = false;
	bool hideTerrainRenderings = false;
	bool showGateGraph = false;
	bool showNodeGraph = false;
	bool hideNodeBorders = false;
	bool hideGateCellHighlights = false;
	bool showNaiveFlowFieldResults = false;

	// ~~~~~~ imgui ~~~~~~~
	ImFont* largeFont;

	// ~~~~~~ misc ~~~~~~~
	ArrowFont arrows;

	void Reset();
	int	 InitSDL();
	int	 InitImgui();
	int	 InitArrowsFont();
	void DestroyArrowsFont();
	void DestroyImgui();
	void DestroySDL();

	// ~~~~~~ render the world ~~~~~~~
	void RenderWorld();
	void RenderGrids();
	void RenderQuadtreeNodes();
	void RenderGates();
	void RenderGateGraph();
	void RenderNodeGraph();
	void RenderHighlightedNodes();
	void RenderHighlightedNodesAstar();
	void RenderHighlightedNodesFlowField();
	void RenderPathfindingDispatch();
	void RenderPathfindingAStar();
	void RenderPathfindingAStarNaive();
	void RenderPathfindingFlowField();
	void RenderPathFindingFlowFieldGateField();
	void RenderPathFindingFlowFieldFinalField();
	void RenderPathFindingFlowFieldGateNextsLines();
	void RenderDrawRect(const SDL_Rect& rect, const SDL_Color& color);
	void RenderFillRect(const SDL_Rect& rect, const SDL_Color& color);
	void RenderDrawLine(int x1, int y1, int x2, int y2, const SDL_Color& color);
	void RenderDrawLineBetweenCells(int x1, int y1, int x2, int y2, const SDL_Color& color);
	void RenderCopy(SDL_Texture* texture, const SDL_Rect& src, const SDL_Rect& dst);
	void RenderFillCell(int x, int y, const SDL_Color& color);
	void RenderFillAgent(int x, int y);
	void RenderFillAgent(int x, int y, const SDL_Color& color);
	void SetMessageHint(std::string_view message, const ImVec4& color);

	// ~~~~~~ render the panel ~~~~~~~
	void RenderImguiPanel();
	void RenderImguiPanelSectionCommon();
	void RenderImguiPanelSectionPathFinding();
	void RenderImguiPanelSectionPathFindingAStar();
	void RenderImguiPanelSectionPathFindingFlowField();
	void RenderImguiPanelSectionAgent();

	// ~~~~ camera ~~~~~
	bool CropRectByCamera(const SDL_Rect& rect, SDL_Rect& overlap,
		bool marginToCameraCoordinates = true);

	// ~~~~ astar ~~~~~~~
	void ComputeAstarNodePath();
	void ComputeAstarGatePath();
	void ComputeAstarFinalPath();
	void HandleAstarInputBegin();
	void HandleFlowFieldInputQueryRangeBegin();
	void ComputeAStarNaive();
	void ComputeFlowFieldNaive();

	// ~~~~~ flowfield ~~~~~~
	void ComputeNodeFlowField();
	void ComputeGateFlowField();
	void ComputeFinalFlowField();
	void HandlePlayFlowFieldTestPath();

	// ~~~~ interaction ~~~~~~~
	void HandleInputs();
	void HandleInputsShortcuts(SDL_Event& e);
	void HandleInputsForCrameMovementsByKeyBoard(SDL_Event& e);
	void HandleInputsForCrameMovementsByMouse(SDL_Event& e);
	void HandleInputsDispatchByState(SDL_Event& e);
	void HandleInputsChangeTerrains(SDL_Event& e);
	void HandleInputsAStarSetStart(SDL_Event& e);
	void HandleInputsAStarSetTarget(SDL_Event& e);
	void HandleInputsFlowFieldSetQrangeLeftTop(SDL_Event& e);
	void HandleInputsFlowFieldSetQrangeRightBottom(SDL_Event& e);
	void HandleInputsFlowFieldSetTarget(SDL_Event& e);
	void HandleInputsFlowFieldSetTestStart(SDL_Event& e);

	// ~~~~~~~~ handlers ~~~~~~~~~~~
	void HandleLogics();
	void HandleStartDrawBuildings();
	void HandleStartDrawWater();
	void PushTerrainChanges(const Cell&);
	void ApplyTerrainChanges();
	void HandleSwitchPathFinderHandler(PathFinderFlag to);
	void HandleChangeAgentSize(int to);
	void HandleChangeAgentCompability(int to);
	void HandleClearAllTerrains();

	// ~~~~~~ util ~~~~~~~
	std::pair<int, int>				   GetCellAtPixelPosition(int x, int y) const;
	const QDPF::Internal::QuadtreeMap* GetCurrentQuadtreeMapByAgent() const;
};

#endif
