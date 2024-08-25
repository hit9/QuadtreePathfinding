// Source Code: https://github.com/hit9/quadtree-pathfinding
// License: BSD. Chao Wang, hit9[At]icloud.com.

#include "base.hpp"

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

}  // namespace internal
}  // namespace qdpf
