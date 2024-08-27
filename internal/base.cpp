// Source Code: https://github.com/hit9/quadtree-pathfinding
// License: BSD. Chao Wang, hit9[At]icloud.com.

#include "base.hpp"

#include <cmath>

namespace qdpf {
namespace internal {

int countBits(unsigned int n) {
  int count = 0;
  while (n) {
    n &= (n - 1);
    count++;
  }
  return count;
}

void ComputeStraightLine(int x1, int y1, int x2, int y2, CellCollector &collector) {
  int dx = abs(x2 - x1), sx = x1 < x2 ? 1 : -1;
  int dy = -abs(y2 - y1), sy = y1 < y2 ? 1 : -1;
  int err = dx + dy, e2;
  while (true) {
    collector(x1, y1);
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

}  // namespace internal
}  // namespace qdpf
