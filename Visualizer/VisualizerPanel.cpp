#include <fmt/format.h>

#include "Visualizer.h"

void Visualizer::RenderImguiPanel()
{
	// window's background
	ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.0f, 0.0f, 0.0f, 0.7f));

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
	RenderImguiPanelSectionCommon();
	ImGui::Spacing();
	RenderImguiPanelSectionAgent();
	ImGui::Spacing();
	RenderImguiPanelSectionPathFinding();

	ImGui::End();

	ImGui::PopStyleColor();
}

void Visualizer::RenderImguiPanelSectionCommon()
{
	ImGui::SeparatorText("Common");

	// Reset.
	if (ImGui::Button("Reset < ESC >"))
		Reset();
	if (ImGui::IsItemHovered())
	{
		ImGui::BeginTooltip();
		ImGui::Text("Clear the compution results and back to idle. (key: ESC)");
		ImGui::EndTooltip();
	}
	ImGui::SameLine();

	// Button: Buildings
	ImGui::PushStyleColor(ImGuiCol_Button, ImRed);
	if (ImGui::Button("Building < B > "))
	{
		HandleStartDrawBuildings();
	}
	ImGui::PopStyleColor(3); // rollback color
	if (ImGui::IsItemHovered())
	{
		ImGui::BeginTooltip();
		ImGui::Text("Add or Remove building");
		ImGui::EndTooltip();
	}

	ImGui::SameLine();

	// Button: Water
	ImGui::PushStyleColor(ImGuiCol_Button, ImBlue);
	if (ImGui::Button("Water < W >"))
	{
		HandleStartDrawWater();
	}
	ImGui::PopStyleColor(3); // rollback color
	if (ImGui::IsItemHovered())
	{
		ImGui::BeginTooltip();
		ImGui::Text("Add or Remove water");
		ImGui::EndTooltip();
	}

	ImGui::SameLine();

	// Button clear terrains;
	if (ImGui::Button("Clear All Building and Water"))
	{
		showClearAllTerrainsConfirm = true;
		ImGui::OpenPopup("ClearAllTerrainsConfirmationDialog");
	}
	if (ImGui::IsItemHovered())
	{
		ImGui::BeginTooltip();
		ImGui::Text("Clear all building and water on the map");
		ImGui::EndTooltip();
	}
	if (showClearAllTerrainsConfirm)
	{
		if (ImGui::BeginPopupModal("ClearAllTerrainsConfirmationDialog", &showClearAllTerrainsConfirm,
				ImGuiWindowFlags_AlwaysAutoResize))
		{
			ImGui::Text("Are you sure to clear all terrains to land ?");
			ImGui::Separator();
			if (ImGui::Button("Yes"))
			{
				HandleClearAllTerrains();
				showClearAllTerrainsConfirm = false;
				ImGui::CloseCurrentPopup();
			}
			ImGui::SetItemDefaultFocus();
			ImGui::SameLine();
			if (ImGui::Button("No"))
			{
				showClearAllTerrainsConfirm = false;
				ImGui::CloseCurrentPopup();
			}

			ImGui::EndPopup();
		}
	}

	// Debug

	if (!hideTerrainRenderings)
	{
		if (ImGui::Button("Hide Terrains"))
		{
			hideTerrainRenderings = true;
		}
	}
	else
	{
		if (ImGui::Button("Show Terrains"))
		{
			hideTerrainRenderings = false;
		}
	}

	ImGui::SameLine();

	if (!showGateGraph)
	{
		if (ImGui::Button("Show Gate Graph (May Slow!)"))
		{
			showGateGraph = true;
		}
	}
	else
	{
		if (ImGui::Button("Hide Gate Graph"))
		{
			showGateGraph = false;
		}
	}

	ImGui::SameLine();

	if (!showNodeGraph)
	{
		if (ImGui::Button("Show Node Graph (May Slow!)"))
		{
			showNodeGraph = true;
		}
	}
	else
	{
		if (ImGui::Button("Hide Node Graph"))
		{
			showNodeGraph = false;
		}
	}

	ImGui::SameLine();

	if (hideNodeBorders)
	{
		if (ImGui::Button("Show Node Borders"))
		{
			hideNodeBorders = false;
		}
	}
	else
	{
		if (ImGui::Button("Hide Node Borders"))
		{
			hideNodeBorders = true;
		}
	}

	ImGui::SameLine();

	if (hideGateCellHighlights)
	{
		if (ImGui::Button("Show Gates"))
		{
			hideGateCellHighlights = false;
		}
	}
	else
	{
		if (ImGui::Button("Hide Gates"))
		{
			hideGateCellHighlights = true;
		}
	}
}

void Visualizer::RenderImguiPanelSectionAgent()
{
	ImGui::SeparatorText("Agent");

	ImGui::Text("Size: ");
	ImGui::SameLine();
	if (ImGui::RadioButton("1x1", agent.size == 1 * COST_UNIT))
		HandleChangeAgentSize(COST_UNIT);
	ImGui::SameLine();
	if (ImGui::RadioButton("2x2", agent.size == 2 * COST_UNIT))
		HandleChangeAgentSize(2 * COST_UNIT);
	ImGui::SameLine();
	if (ImGui::RadioButton("3x3", agent.size == 3 * COST_UNIT))
		HandleChangeAgentSize(3 * COST_UNIT);

	ImGui::Text("Capability: ");
	ImGui::SameLine();
	if (ImGui::RadioButton("Land", agent.capability == Terrain::Land))
		HandleChangeAgentCompability(Terrain::Land);
	ImGui::SameLine();
	if (ImGui::RadioButton("Water", agent.capability == Terrain::Water))
		HandleChangeAgentCompability(Terrain::Water);
	ImGui::SameLine();
	if (ImGui::RadioButton("Land | Water", agent.capability == (Terrain::Land | Terrain::Water)))
		HandleChangeAgentCompability(Terrain::Land | Terrain::Water);
}

void Visualizer::RenderImguiPanelSectionPathFinding()
{
	ImGui::SeparatorText("Pathfinding");

	if (ImGui::RadioButton("A*", pathfinderFlag == PathFinderFlag::AStar))
		HandleSwitchPathFinderHandler(PathFinderFlag::AStar);
	ImGui::SameLine();
	if (ImGui::RadioButton("FlowField", pathfinderFlag == PathFinderFlag::FlowField))
		HandleSwitchPathFinderHandler(PathFinderFlag::FlowField);

	switch (pathfinderFlag)
	{
		case PathFinderFlag::AStar:
			return RenderImguiPanelSectionPathFindingAStar();
		case PathFinderFlag::FlowField:
			return RenderImguiPanelSectionPathFindingFlowField();
	}
}

void Visualizer::RenderImguiPanelSectionPathFindingAStar()
{
	ImGui::Text("Compution:");

	if (ImGui::Button("Set Start and Target"))
	{
		HandleAstarInputBegin();
	}
	ImGui::SameLine();
	if (ImGui::Button("Compute Node Path"))
	{
		ComputeAstarNodePath();
	}
	ImGui::SameLine();
	if (ImGui::Button("Compute Gate Path"))
	{
		ComputeAstarGatePath();
	}
	ImGui::SameLine();
	if (ImGui::Button("Compute Final Path"))
	{
		ComputeAstarFinalPath();
	}

	if (state == State::AStarFinalPathComputed)
	{
		ImGui::Text("Compare:");
		if (ImGui::Button("Compare with Naive A*"))
		{
			ComputeAStarNaive();
		}
		if (astarNaive.path.size() && astar.finalPath.size())
		{
			ImGui::Text("A* on quadtree vs Naive A* time costs: %lldus vs %lldus",
				astar.timeCost.count(), astarNaive.timeCost.count());
		}
	}
}

void Visualizer::RenderImguiPanelSectionPathFindingFlowField()
{
	ImGui::Text("Compution:");

	if (ImGui::Button("Set Start Rectangle and Target"))
	{
		HandleFlowFieldInputQueryRangeBegin();
	}

	if (ImGui::IsItemHovered())
	{
		ImGui::BeginTooltip();
		ImGui::Text(
			"Click the left-top and right-bottom corners of the start rectangle, and then click a "
			"target");
		ImGui::EndTooltip();
	}

	ImGui::SameLine();

	if (ImGui::Button("Compute Node FlowField"))
	{
		ComputeNodeFlowField();
	}

	ImGui::SameLine();

	if (ImGui::Button("Compute Gate FlowField"))
	{
		ComputeGateFlowField();
	}

	if (ImGui::Button("Compute Final FlowField"))
	{
		ComputeFinalFlowField();
	}

	ImGui::Text("Debug:");

	if (ImGui::Button("Clear Tests Path"))
	{
		flowfield.testPaths.clear();
		flowfieldNaive.testPaths.clear();
	}

	ImGui::SameLine();

	if (!renderFlowFieldGateNextLines)
	{
		if (ImGui::Button("Show Gate Connection to Next"))
		{
			renderFlowFieldGateNextLines = true;
		}
	}
	else
	{
		if (ImGui::Button("Hide Gate Connection to Next"))
		{
			renderFlowFieldGateNextLines = false;
		}
	}

	if (state == State::FlowFieldFinalLevelComputed)
	{
		ImGui::Text("Compare:");
		if (ImGui::Button("Compare with Naive FlowField"))
		{
			ComputeFlowFieldNaive();
		}
		if (flowfieldNaive.finalFlowField.Size() && flowfield.finalFlowField.Size())
		{
			ImGui::Text("FlowField on quadtree vs Naive FlowField time costs: %lldus vs %lldus",
				flowfield.timeCost.count(), flowfieldNaive.timeCost.count());
		}

		if (!showNaiveFlowFieldResults)
		{
			if (ImGui::Button("Show Naive FlowField Results"))
			{
				showNaiveFlowFieldResults = true;
			}
			ImGui::SameLine();
			ImGui::Text("( Showing flowfield on Quadtrees results )");
		}
		else
		{
			if (ImGui::Button("Show FlowField on Quadtrees Results"))
			{
				showNaiveFlowFieldResults = false;
			}
			ImGui::SameLine();
			ImGui::Text("( Showing Naive FlowField Results) ");
		}
	}
}
