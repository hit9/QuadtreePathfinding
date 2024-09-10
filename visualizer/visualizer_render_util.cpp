#include "imgui.h"
#include "visualizer.hpp"

class CohenSutherland {
 public:
  bool Clip(int& x0, int& y0, int& x1, int& y1, int xmin, int ymin, int xmax, int ymax);

 private:
  int computeOutCode(int x, int y, int xmin, int ymin, int xmax, int ymax);
};

// Visualizer utils

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

void Visualizer::renderDrawLine(int x1, int y1, int x2, int y2, const SDL_Color& color) {
  // TODO: how to crop? May we need the Cohen-Sutherland..
  CohenSutherland clip;
  clip.Clip(x1, y1, x2, y2, camera->x, camera->y, camera->x + camera->w - 1,
            camera->y + camera->h - 1);
  x1 -= camera->x;
  x2 -= camera->x;
  y1 -= camera->y;
  y2 -= camera->y;
  SDL_SetRenderDrawColor(renderer, color.r, color.g, color.b, color.a);
  SDL_RenderDrawLine(renderer, x1, y1, x2, y2);
}

void Visualizer::renderDrawLineBetweenCells(int x1, int y1, int x2, int y2,
                                            const SDL_Color& color) {
  auto px1 = x1 * map.gridSize;
  auto py1 = y1 * map.gridSize;
  auto px2 = x2 * map.gridSize;
  auto py2 = y2 * map.gridSize;
  auto centerx1 = px1 + map.gridSize / 2;
  auto centery1 = py1 + map.gridSize / 2;
  auto centerx2 = px2 + map.gridSize / 2;
  auto centery2 = py2 + map.gridSize / 2;
  renderDrawLine(centerx1, centery1, centerx2, centery2, color);
}

void Visualizer::renderCopy(SDL_Texture* texture, const SDL_Rect& src, const SDL_Rect& dst) {
  SDL_Rect dstOverlap;
  if (!cropRectByCamera(dst, dstOverlap, false)) return;
  // find the crop region in src
  SDL_Rect srcOverlap = {src.x + dstOverlap.x - dst.x, src.y + dstOverlap.y - dst.y, dstOverlap.w,
                         dstOverlap.h};
  dstOverlap.x -= camera->x;
  dstOverlap.y -= camera->y;
  SDL_RenderCopy(renderer, texture, &srcOverlap, &dstOverlap);
}

void Visualizer::renderFillCell(int x, int y, const SDL_Color& color) {
  SDL_Rect cell{x * map.gridSize + 1, y * map.gridSize + 1, map.gridSize - 2, map.gridSize - 2};
  renderFillRect(cell, color);
}

void Visualizer::renderFillAgent(int x, int y, const SDL_Color& color) {
  int agentSizeInPixels = agent.size * map.gridSize / COST_UNIT;

  int px = x * map.gridSize, py = y * map.gridSize;

  SDL_Rect outer;

  if (options.clearanceFieldFlag == 0) {
    // TrueClearanceField: let's use the left corner as the position of the agent.
    outer = {px, py, agentSizeInPixels, agentSizeInPixels};
  } else {
    // BrushfireClearanceField: let's use the center as the position of the agent.
    outer = {px - map.gridSize / 2 + 1, py - map.gridSize / 2 + 1, agentSizeInPixels,
             agentSizeInPixels};
  }

  SDL_Rect inner{outer.x + 1, outer.y + 1, outer.w - 2, outer.h - 2};
  renderDrawRect(outer, Black);
  renderFillRect(inner, color);
}

void Visualizer::renderFillAgent(int x, int y) { renderFillAgent(x, y, Green); }

// Crop given rect by camera, and converts to the coordinates relative to the camera's left-top
// corner. The results are stored into overlap. Returns false if no overlaping.
bool Visualizer::cropRectByCamera(const SDL_Rect& rect, SDL_Rect& overlap,
                                  bool marginToCameraCoordinates) {
  qdpf::Rectangle c{camera->x, camera->y, camera->x + camera->w - 1, camera->y + camera->h - 1};
  qdpf::Rectangle a{rect.x, rect.y, rect.x + rect.w - 1, rect.y + rect.h - 1};
  qdpf::Rectangle d;
  auto b = qdpf::internal::GetOverlap(c, a, d);
  if (!b) return false;
  overlap.x = d.x1, overlap.y = d.y1;
  overlap.w = d.x2 - d.x1 + 1, overlap.h = d.y2 - d.y1 + 1;
  if (marginToCameraCoordinates) {
    overlap.x -= camera->x;
    overlap.y -= camera->y;
  }
  return true;
}

// Cohen-Sutherland

const int CohenSutherlandClip_Outcode_Inside = 0;
const int CohenSutherlandClip_Outcode_Left = 0b0001;
const int CohenSutherlandClip_Outcode_Right = 0b0010;
const int CohenSutherlandClip_Outcode_Bottom = 0b0100;
const int CohenSutherlandClip_Outcode_Top = 0b1000;

int CohenSutherland::computeOutCode(int x, int y, int xmin, int ymin, int xmax, int ymax) {
  int code = CohenSutherlandClip_Outcode_Inside;
  if (x < xmin)
    code |= CohenSutherlandClip_Outcode_Left;
  else if (x > xmax)
    code |= CohenSutherlandClip_Outcode_Right;
  if (y < ymin)
    code |= CohenSutherlandClip_Outcode_Bottom;
  else if (y > ymax)
    code |= CohenSutherlandClip_Outcode_Top;
  return code;
}

// Cohen-Sutherland camera clip
bool CohenSutherland::Clip(int& x0, int& y0, int& x1, int& y1, int xmin, int ymin, int xmax,
                           int ymax) {
  int outcode0 = computeOutCode(x0, y0, xmin, ymin, xmax, ymax);
  int outcode1 = computeOutCode(x1, y1, xmin, ymin, xmax, ymax);
  bool accept = false;

  while (true) {
    if (!(outcode0 | outcode1)) {
      accept = true;
      break;
    } else if (outcode0 & outcode1) {
      break;
    } else {
      int x, y;
      int outcodeOut = outcode0 ? outcode0 : outcode1;

      if (outcodeOut & CohenSutherlandClip_Outcode_Top) {
        x = x0 + (x1 - x0) * (ymax - y0) / (y1 - y0);
        y = ymax;
      } else if (outcodeOut & CohenSutherlandClip_Outcode_Bottom) {
        x = x0 + (x1 - x0) * (ymin - y0) / (y1 - y0);
        y = ymin;
      } else if (outcodeOut & CohenSutherlandClip_Outcode_Right) {
        y = y0 + (y1 - y0) * (xmax - x0) / (x1 - x0);
        x = xmax;
      } else if (outcodeOut & CohenSutherlandClip_Outcode_Left) {
        y = y0 + (y1 - y0) * (xmin - x0) / (x1 - x0);
        x = xmin;
      }

      if (outcodeOut == outcode0) {
        x0 = x;
        y0 = y;
        outcode0 = computeOutCode(x0, y0, xmin, ymin, xmax, ymax);
      } else {
        x1 = x;
        y1 = y;
        outcode1 = computeOutCode(x1, y1, xmin, ymin, xmax, ymax);
      }
    }
  }
  return accept;
}

// Cohen-Sutherland Implementation
