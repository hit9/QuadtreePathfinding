#include <imgui.h>
#include <imgui_impl_sdl2.h>
#include <imgui_impl_sdlrenderer2.h>

#include "Visualizer.h"

void Visualizer::HandleInputs()
{
	SDL_Event e;
	auto&	  io = ImGui::GetIO();
	while (SDL_PollEvent(&e))
	{
		ImGui_ImplSDL2_ProcessEvent(&e);
		if (e.type == SDL_QUIT)
		{
			stop = true;
			return;
		}

		// isMouseDown;
		if (e.type == SDL_MOUSEBUTTONDOWN)
			isMouseDown = true;
		else if (e.type == SDL_MOUSEBUTTONUP)
			isMouseDown = false;

		// shortcuts (in front of imgui)
		HandleInputsShortcuts(e);
		HandleInputsForCrameMovementsByKeyBoard(e);

		// ImGui didn't Handle this event, turn it to SDL.
		if (!io.WantCaptureMouse && !io.WantCaptureKeyboard)
		{
			HandleInputsForCrameMovementsByMouse(e);
			HandleInputsDispatchByState(e);
		}
	}
}

// returns -1 for stop the whole window.
void Visualizer::HandleInputsShortcuts(SDL_Event& e)
{
	// shortcuts
	if (e.type == SDL_KEYDOWN)
	{
		switch (e.key.keysym.sym)
		{
				// Ctrl-C
			case SDLK_c:
				if (SDL_GetModState() & KMOD_CTRL)
					stop = true;
				break;
				// ESC
			case SDLK_ESCAPE:
				Reset();
				break;
			case SDLK_b:
				HandleStartDrawBuildings();
				break;
			case SDLK_w:
				HandleStartDrawWater();
				break;
		}
	}
}

void Visualizer::HandleInputsForCrameMovementsByMouse(SDL_Event& e)
{
	switch (e.type)
	{
		case SDL_MOUSEWHEEL:
			if (e.wheel.x > 0)
				camera->MoveLeft();
			if (e.wheel.x < 0)
				camera->MoveRight();
			if (e.wheel.y > 0)
				camera->MoveUp();
			if (e.wheel.y < 0)
				camera->MoveDown();
			break;
	}
}

void Visualizer::HandleInputsForCrameMovementsByKeyBoard(SDL_Event& e)
{
	switch (e.type)
	{
		case SDL_KEYDOWN:
			if (e.key.keysym.sym == SDLK_UP || e.key.keysym.sym == SDLK_k)
				camera->MoveUp(); // k
			if (e.key.keysym.sym == SDLK_DOWN || e.key.keysym.sym == SDLK_j)
				camera->MoveDown(); // j
			if (e.key.keysym.sym == SDLK_LEFT || e.key.keysym.sym == SDLK_h)
				camera->MoveLeft(); // h
			if (e.key.keysym.sym == SDLK_RIGHT || e.key.keysym.sym == SDLK_l)
				camera->MoveRight(); // l
			if (e.key.keysym.sym == SDLK_0)
				camera->MoveToLeftMost(); // 0
			if (e.key.keysym.sym == SDLK_DOLLAR || (e.key.keysym.sym == SDLK_4 && SDL_GetModState() & KMOD_SHIFT))
				camera->MoveToRightMost(); // $
			if (e.key.keysym.sym == SDLK_d && SDL_GetModState() & KMOD_CTRL)
				camera->MoveDown(800); // Ctrl-D
			if (e.key.keysym.sym == SDLK_u && SDL_GetModState() & KMOD_CTRL)
				camera->MoveUp(800); // Ctrl-U
			break;
	}
}

void Visualizer::HandleInputsDispatchByState(SDL_Event& e)
{
	switch (state)
	{
		case State::DrawingBuildings:
			[[fallthrough]];
		case State::DrawingWaters:
			HandleInputsChangeTerrains(e);
			break;
		case State::AStarWaitStart:
			HandleInputsAStarSetStart(e);
			break;
		case State::AStarWaitTarget:
			HandleInputsAStarSetTarget(e);
			break;
		case State::FlowFieldWaitQrangeLeftTop:
			HandleInputsFlowFieldSetQrangeLeftTop(e);
			break;
		case State::FlowFieldWaitQrangeRightBottom:
			HandleInputsFlowFieldSetQrangeRightBottom(e);
			break;
		case State::FlowFieldWaitTarget:
			HandleInputsFlowFieldSetTarget(e);
			break;
		case State::FlowFieldFinalLevelComputed:
			// reset test path's star:
			HandleInputsFlowFieldSetTestStart(e);
			break;
		default:
			break; // avoid warning.
	}
}

void Visualizer::HandleInputsChangeTerrains(SDL_Event& e)
{
	switch (e.type)
	{
		case SDL_MOUSEBUTTONUP:
			ApplyTerrainChanges();
			break;
		case SDL_MOUSEMOTION:
			if (isMouseDown)
				PushTerrainChanges(GetCellAtPixelPosition(e.button.x, e.button.y));
			break;
		case SDL_MOUSEBUTTONDOWN:
			PushTerrainChanges(GetCellAtPixelPosition(e.button.x, e.button.y));
			break;
	}
}

void Visualizer::HandleInputsAStarSetStart(SDL_Event& e)
{
	if (e.type == SDL_MOUSEBUTTONDOWN)
	{
		auto cell = GetCellAtPixelPosition(e.button.x, e.button.y);
		astar.x1 = cell.first;
		astar.y1 = cell.second;
		state = State::AStarWaitTarget;
		SetMessageHint("A*: waiting to click a target cell", ImGreen);
	}
}

void Visualizer::HandleInputsAStarSetTarget(SDL_Event& e)
{
	if (e.type == SDL_MOUSEBUTTONDOWN)
	{
		auto cell = GetCellAtPixelPosition(e.button.x, e.button.y);
		astar.x2 = cell.first;
		astar.y2 = cell.second;
		state = State::AStarWaitCompution;
		SetMessageHint("A*: waiting to click a compution button", ImGreen);
	}
}

void Visualizer::HandleInputsFlowFieldSetQrangeLeftTop(SDL_Event& e)
{
	if (e.type == SDL_MOUSEBUTTONDOWN)
	{
		auto cell = GetCellAtPixelPosition(e.button.x, e.button.y);
		flowfield.Reset();
		flowfieldNaive.Reset();
		flowfield.qrange.x1 = cell.first;
		flowfield.qrange.y1 = cell.second;
		state = State::FlowFieldWaitQrangeRightBottom;
		SetMessageHint("FlowField: waiting to click a right-bottom cell", ImGreen);
	}
}
void Visualizer::HandleInputsFlowFieldSetQrangeRightBottom(SDL_Event& e)
{
	if (e.type == SDL_MOUSEBUTTONDOWN)
	{
		auto cell = GetCellAtPixelPosition(e.button.x, e.button.y);
		flowfield.qrange.x2 = cell.first;
		flowfield.qrange.y2 = cell.second;
		state = State::FlowFieldWaitTarget;
		SetMessageHint("FlowField: waiting to click a target cell", ImGreen);
	}
}

void Visualizer::HandleInputsFlowFieldSetTarget(SDL_Event& e)
{
	if (e.type == SDL_MOUSEBUTTONDOWN)
	{
		auto cell = GetCellAtPixelPosition(e.button.x, e.button.y);
		flowfield.x2 = cell.first;
		flowfield.y2 = cell.second;
		state = State::FlowFieldWaitCompution;
		SetMessageHint("FlowField: waiting to click a compution button", ImGreen);
	}
}

void Visualizer::HandleInputsFlowFieldSetTestStart(SDL_Event& e)
{
	if (e.type == SDL_MOUSEBUTTONDOWN)
	{
		auto cell = GetCellAtPixelPosition(e.button.x, e.button.y);
		if (!flowfield.finalFlowField.Exist(cell))
		{
			SetMessageHint("FlowField: test path start invalid", ImRed);
			return;
		}
		auto& testPaths = showNaiveFlowFieldResults ? flowfieldNaive.testPaths : flowfield.testPaths;
		testPaths.resize(flowfield.testPaths.size() + 1);
		testPaths.back().push_back(cell);
		SetMessageHint("FlowField: playing test path ...", ImGreen);
	}
}
