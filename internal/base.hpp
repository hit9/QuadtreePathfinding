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

// Utils

// Returns the number of true bits in given unsigned number n.
int countBits(unsigned int n);

}  // namespace internal
}  // namespace qdpf

#endif
