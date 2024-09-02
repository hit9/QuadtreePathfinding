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
// the parameter limit is to limit the steps: -1 for no limitation.
// e.g. the first step is always (x1,y1), to obtain the next cell, pass limit = 2.
void ComputeStraightLine(int x1, int y1, int x2, int y2, CellCollector &collector, int limit = -1);

// Is (x,y) is inside rectangle rect?
bool IsInsideRectangle(int x, int y, const Rectangle &rect);

// Is (x,y) is inside rectangle (x1,y1),(x2,y2)?
bool IsInsideRectangle(int x, int y, int x1, int y1, int x2, int y2);

// AABB overlap testing.
// Checks if rectangle a and b overlaps.
bool IsOverlap(const Rectangle &a, const Rectangle &b);

// Gets overlap of a and b into c.
// Returns true if the overlap exist.
bool GetOverlap(const Rectangle &a, const Rectangle &b, Rectangle &c);

// Combine hash a and b into one via FNV hash.
std::size_t HashCombine(std::size_t a, std::size_t b);

// PairHasher is the hashing implementation for std::pair<A, B> using the FNV hash.
template <typename A, typename B>
class PairHasher {
 public:
  std::size_t operator()(const std::pair<A, B> &x) const {
    auto a = std::hash<A>{}(x.first);
    auto b = std::hash<B>{}(x.second);
    return HashCombine(a, b);
  }
};

// ~~~~~~~~~~~ Util Containers ~~~~~~~~~~~~

// A simple simple unordered_map with default value.
template <typename K, typename V, V DefaultValue, typename Hasher = std::hash<K>>
class DefaultedUnorderedMap {
 public:
  using UnderlyingUnorderedMap = std::unordered_map<K, V, Hasher>;

  // empty Resize. ignore
  void Resize(std::size_t _ignoredn) {}
  // Is k exist in this map?
  bool Exist(K k) const { return m.find(k) != m.end(); }
  // Returns a mutable reference to the value of given key.
  // Inserts a default value if not exist.
  V &operator[](K k) { return m.try_emplace(k, defaultValue).first->second; }
  // Returns a const reference to the value of given key.
  // Returns a reference to the defaultValue if not exist.
  const V &operator[](K k) const {
    auto it = m.find(k);
    if (it == m.end()) return defaultValue;
    return it->second;
  }
  // Clears all items in the map.
  void Clear() { m.clear(); }
  // Returns a const reference to underlying map.
  const UnderlyingUnorderedMap &GetUnderlyingUnorderedMap() const { return m; }
  // Returns the size of the map.
  std::size_t Size() const { return m.size(); }

 private:
  V defaultValue = DefaultValue;
  UnderlyingUnorderedMap m;
};

template <typename K, int DefaultValue, typename Hasher = std::hash<K>>
using DefaultedUnorderedMapInt = DefaultedUnorderedMap<K, int, DefaultValue, Hasher>;

template <typename K, bool DefaultValue, typename Hasher = std::hash<K>>
using DefaultedUnorderedMapBool = DefaultedUnorderedMap<K, bool, DefaultValue, Hasher>;

// Nested unordered_map with default value.
template <typename K1, typename K2, typename V, V DefaultValue>
class NestedDefaultedUnorderedMap {
 public:
  // Inner level map.
  using InnerMap = DefaultedUnorderedMap<K2, V, DefaultValue>;
  // An empty inner level map.
  static InnerMap EmptyInnerMap;
  // Returns a mutable reference to the inner map for given key.
  // Inserts one if not exist.
  InnerMap &operator[](K1 k) { return m.try_emplace(k).first->second; }
  // Returns a const reference to the inner map for given key.
  // Returns the reference to EmptyInnerMap if not exist.
  const InnerMap &operator[](K1 k) const {
    auto it = m.find(k);
    if (it == m.end()) return EmptyInnerMap;
    return it->second;
  }
  // Clears all inner map.
  void Clear() { m.clear(); }

 private:
  V defaultValue = DefaultValue;
  std::unordered_map<K1, InnerMap> m;
};

// KVContainer on vector (faster but more memory occuption).
template <typename V, V DefaultValue>
class DefaultedVector {
 public:
  // Resize the vector to size n with defaultValues.
  void Resize(std::size_t n) { vec.resize(n, defaultValue); }
  // Returns a mutable reference to the kth item.
  V &operator[](int k) { return vec[k]; }
  // Returns a const reference to the kth item.
  const V &operator[](int k) const { return vec[k]; }
  // Clears all the items
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
