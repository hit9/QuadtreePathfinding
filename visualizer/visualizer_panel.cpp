#include <fmt/format.h>

#include "visualizer.hpp"

void Visualizer::renderImguiPanel() {
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
  renderImguiPanelSectionCommon();
  ImGui::Spacing();
  renderImguiPanelSectionAgent();
  ImGui::Spacing();
  renderImguiPanelSectionPathFinding();

  ImGui::End();

  ImGui::PopStyleColor();
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
  if (ImGui::RadioButton("1x1", agent.size == 1 * COST_UNIT)) handleChangeAgentSize(COST_UNIT);
  ImGui::SameLine();
  if (ImGui::RadioButton("2x2", agent.size == 2 * COST_UNIT)) handleChangeAgentSize(2 * COST_UNIT);
  ImGui::SameLine();
  if (ImGui::RadioButton("3x3", agent.size == 3 * COST_UNIT)) handleChangeAgentSize(3 * COST_UNIT);

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
  if (ImGui::Button("Set Start Rectangle and Target")) {
    handleFlowFieldInputQueryRangeBegin();
  }

  if (ImGui::IsItemHovered()) {
    ImGui::BeginTooltip();
    ImGui::Text(
        "Click the left-top and right-bottom corners of the start rectangle, and then click a "
        "target");
    ImGui::EndTooltip();
  }

  ImGui::SameLine();

  if (ImGui::Button("Compute Node FlowField")) {
    computeNodeFlowField();
  }

  ImGui::SameLine();

  if (ImGui::Button("Compute Gate FlowField")) {
    computeGateFlowField();
  }

  if (ImGui::Button("Compute Final FlowField")) {
    computeFinalFlowField();
  }

  ImGui::SameLine();

  if (ImGui::Button("Clear Tests Path")) {
    flowfield.testPaths.clear();
  }
}
