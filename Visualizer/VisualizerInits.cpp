#include <SDL2/SDL.h>
#include <SDL2/SDL_render.h>
#include <SDL_ttf.h>
#include <imgui.h>
#include <imgui_impl_sdl2.h>
#include <imgui_impl_sdlrenderer2.h>
#include <spdlog/spdlog.h>

#include "Visualizer.h"

Visualizer::Visualizer(int w, int h, int gridSize, int step)
	: map(Map(w, h, gridSize, step)) {}

Visualizer::~Visualizer()
{
	delete camera;
	camera = nullptr;
}

int Visualizer::Init()
{
	if (0 != InitSDL())
		return -1;
	if (0 != InitImgui())
	{
		DestroySDL();
		return -1;
	}
	if (0 != InitArrowsFont())
	{
		DestroyImgui();
		DestroySDL();
		return -1;
	}
	spdlog::info("Visualizer Init done.");

	spdlog::info("Building map (may take some time...)");
	// Build the map.
	map.Build();
	// Build the pfs;
	astar.InitPf(map.qmx);
	flowfield.InitPf(map.qmx);
	spdlog::info("Visualizer's Init done");
	return 0;
}

int Visualizer::InitSDL()
{
	// Init SDL.
	if (SDL_Init(SDL_INIT_EVERYTHING) != 0)
	{
		spdlog::error("Error: {}", SDL_GetError());
		return -1;
	}

	// Init ttf font
	if (TTF_Init() == -1)
	{
		spdlog::error("SDL_ttf Error: {}", SDL_GetError());
		SDL_Quit();
		return -1;
	}

	// Get display bounds
	SDL_Rect displayBounds;
	if (SDL_GetDisplayBounds(0, &displayBounds) != 0)
	{
		spdlog::error("Failed to get display bounds: {}", SDL_GetError());
		TTF_Quit();
		SDL_Quit();
		return 1;
	}

	// Creates window and camera.
	int mpw = map.w * map.gridSize, mph = map.h * map.gridSize;
	int w = std::min(mpw, displayBounds.w);
	int h = std::min(mph, displayBounds.h);
	window =
		SDL_CreateWindow("quadtree-pathfinding-visualizer", SDL_WINDOWPOS_CENTERED,
			SDL_WINDOWPOS_CENTERED, w, h, SDL_WINDOW_SHOWN | SDL_WINDOW_ALLOW_HIGHDPI);
	if (window == nullptr)
	{
		spdlog::error("Create window error: {}", SDL_GetError());
		TTF_Quit();
		SDL_Quit();
		return -3;
	}
	camera = new Camera(w, h, mpw, mph);

	// Creates renderer.
	renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
	if (renderer == nullptr)
	{
		spdlog::error("Create renderer error: {}", SDL_GetError());
		SDL_DestroyWindow(window);
		TTF_Quit();
		SDL_Quit();
		return -1;
	}
	return 0;
}

int Visualizer::InitImgui()
{
	IMGUI_CHECKVERSION();
	ImGui::CreateContext();
	auto& io = ImGui::GetIO();
	io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard; // Enable Keyboard Controls
	io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;  // Enable Gamepad Controls
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

int Visualizer::InitArrowsFont()
{
	// Open font arrows.
	std::string arrowsFontPath = options.fontsPath + "/Arrows.ttf";
	arrows.font = TTF_OpenFont(arrowsFontPath.c_str(), 18);
	if (arrows.font == nullptr)
	{
		spdlog::error("Cannot open Arrows.ttf: {} {}", arrowsFontPath, SDL_GetError());
		return -1;
	}

	// Creates surface for arrows font.
	SDL_Surface* ts = TTF_RenderUTF8_Solid(arrows.font, ARROWS_CHAR, { 0, 0, 0, 255 });
	if (!ts)
	{
		spdlog::error("SDL_Surface: {}", TTF_GetError());
		TTF_CloseFont(arrows.font);
		return -1;
	}

	// Create arrows font texture.
	arrows.texture = SDL_CreateTextureFromSurface(renderer, ts);
	if (arrows.texture == nullptr)
	{
		spdlog::error("Create arrows font texture: {}", SDL_GetError());
		SDL_FreeSurface(ts);
		TTF_CloseFont(arrows.font);
		return -1;
	}

	// Computes the font height, weight and offset for each char.
	int offset = 0;
	for (int i = 0; i < 8; i++)
	{
		int minx, maxx, miny, maxy, advance;
		TTF_GlyphMetrics(arrows.font, ARROWS_CHAR[i], &minx, &maxx, &miny, &maxy, &advance);
		arrows.w[i] = advance;
		arrows.offset[i] = offset;
		offset += advance;
	}
	arrows.h = TTF_FontHeight(arrows.font);

	SDL_FreeSurface(ts);
	return 0;
}

void Visualizer::Destroy()
{
	DestroyArrowsFont();
	DestroyImgui();
	DestroySDL();
}

void Visualizer::DestroyArrowsFont()
{
	if (arrows.font != nullptr)
	{
		TTF_CloseFont(arrows.font);
		arrows.font = nullptr;
	}
	if (arrows.texture != nullptr)
	{
		SDL_DestroyTexture(arrows.texture);
		arrows.texture = nullptr;
	}
}

void Visualizer::DestroySDL()
{
	// deinit SDL.
	if (renderer)
		SDL_DestroyRenderer(renderer);
	if (window)
		SDL_DestroyWindow(window);
	if (arrows.font != nullptr)
		TTF_CloseFont(arrows.font);
	TTF_Quit();
	SDL_Quit();
}

void Visualizer::DestroyImgui()
{
	// deinit ImGui.
	ImGui_ImplSDLRenderer2_Shutdown();
	ImGui_ImplSDL2_Shutdown();
	ImGui::DestroyContext();
}
