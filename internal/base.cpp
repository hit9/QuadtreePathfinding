// Source Code: https://github.com/hit9/quadtree-pathfinding
// License: BSD. Chao Wang, hit9[At]icloud.com.

#include "base.hpp"

#include <algorithm>
#include <cmath>

namespace qdpf {
namespace internal {

int CountBits(unsigned int n) {
  int count = 0;
  while (n) {
    n &= (n - 1);
    count++;
  }
  return count;
}

void ComputeStraightLine(int x1, int y1, int x2, int y2, CellCollector &collector, int limit) {
  int dx = abs(x2 - x1), sx = x1 < x2 ? 1 : -1;
  int dy = -abs(y2 - y1), sy = y1 < y2 ? 1 : -1;
  int err = dx + dy, e2;
  int steps = 0;
  while (true) {
    collector(x1, y1);
    if (++steps >= limit) break;
    e2 = 2 * err;
    if (e2 >= dy) {
      if (x1 == x2) break;
      err += dy;
      x1 += sx;
    }
    if (e2 <= dx) {
      if (y1 == y2) break;
      err += dx;
      y1 += sy;
    }
  }
}

// https://writings.sh/post/aabb
bool IsOverlap(const Rectangle &a, const Rectangle &b) {
  return a.x1 <= b.x2 && a.x2 >= b.x1 && a.y1 <= b.y2 && a.y2 >= b.y1;
}

bool GetOverlap(const Rectangle &a, const Rectangle &b, Rectangle &c) {
  if (!IsOverlap(a, b)) return false;
  c.x1 = std::max(a.x1, b.x1);
  c.y1 = std::max(a.y1, b.y1);
  c.x2 = std::min(a.x2, b.x2);
  c.y2 = std::min(a.y2, b.y2);
  return true;
}

}  // namespace internal
}  // namespace qdpf
