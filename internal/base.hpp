// Source Code: https://github.com/hit9/quadtree-pathfinding
// License: BSD. Chao Wang, hit9[At]icloud.com.

#ifndef QDPF_INTERNAL_BASE_HPP
#define QDPF_INTERNAL_BASE_HPP

#include <functional>

namespace qdpf {
namespace internal {

const int inf = 0x3f3f3f3f;

// CellCollector is the function to collect cells (x,y).
using CellCollector = std::function<void(int x, int y)>;

// Rectangle
struct Rectangle {
  // the (x1,y1) and (x2,y2) are the left-top and right-bottom corner cells.
  int x1, y1, x2, y2;
};

// Utils

// Returns the number of true bits in given unsigned number n.
int countBits(unsigned int n);

}  // namespace internal
}  // namespace qdpf

#endif
