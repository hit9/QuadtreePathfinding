// Source Code: https://github.com/hit9/quadtree-pathfinding
// License: BSD. Chao Wang, hit9[At]icloud.com.

#ifndef QDPF_INTERNAL_BASE_HPP
#define QDPF_INTERNAL_BASE_HPP

#include <functional>
#include <unordered_map>
#include <vector>

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

// ~~~~~~~~~~~~  Utils ~~~~~~~~~~~~~~~

// Returns the number of true bits in given unsigned number n.
int CountBits(unsigned int n);

// Bresenham's line algorithm.
// You can override it with a custom implementation.
// Ref: https://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm
// Ref: https://members.chello.at/easyfilter/bresenham.html
void ComputeStraightLine(int x1, int y1, int x2, int y2, CellCollector &collector);

// AABB overlap testing.
// Checks if rectangle a and b overlaps.
bool IsOverlap(const Rectangle &a, const Rectangle &b);

// Gets overlap of a and b into c.
// Returns true if the overlap exist.
bool GetOverlap(const Rectangle &a, const Rectangle &b, Rectangle &c);

// ~~~~~~~~~~~ Util Containers ~~~~~~~~~~~~

// A simple simple unordered_map with default value.
template <typename K, typename V, V DefaultValue>
class DefaultedUnorderedMap {
 public:
  void Resize(std::size_t _ignoredn) {}  // ignore
  V &operator[](K k) { return m.try_emplace(k, defaultValue).first->second; }
  const V &operator[](K k) const {
    auto it = m.find(k);
    if (it == m.end()) return defaultValue;
    return it->second;
  }
  void Clear() { m.clear(); }

 private:
  V defaultValue = DefaultValue;
  std::unordered_map<K, V> m;
};

template <typename K, int DefaultValue>
using DefaultedUnorderedMapInt = DefaultedUnorderedMap<K, int, DefaultValue>;

template <typename K, bool DefaultValue>
using DefaultedUnorderedMapBool = DefaultedUnorderedMap<K, bool, DefaultValue>;

// KVContainer on vector (faster but more memory occuption).
template <typename V, V DefaultValue>
class DefaultedVector {
 public:
  void Resize(std::size_t n) { vec.resize(n, defaultValue); }
  V &operator[](int k) { return vec[k]; }
  const V &operator[](int k) const { return vec[k]; }
  void Clear() { vec.clear(); }

 private:
  V defaultValue = DefaultValue;
  std::vector<V> vec;
};

template <int DefaultValue>
using DefaultedVectorInt = DefaultedVector<int, DefaultValue>;

// avoid using std::vector<bool>
template <bool DefaultValue>
using DefaultedVectorBool = DefaultedVector<unsigned char, DefaultValue>;

}  // namespace internal
}  // namespace qdpf

#endif
