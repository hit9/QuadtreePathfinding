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

//////////////////////
/// Utils
/////////////////////

// Returns the number of true bits in given unsigned number n.
int countBits(unsigned int n);

// KVContainer is an internal abstraction for KV containers with default value support.
template <typename K, typename V, V DefaultValue>
class KVContainer {
 public:
  // resize the container's size to n.
  virtual void Resize(std::size_t n);
  // set k by reference
  virtual V &operator[](K k);
  // get value, returns default value by default.
  virtual const V &operator[](K k) const;
  // clears the container.
  virtual void Clear();
};

// KVContainer on unordered_map.
template <typename K, typename V, V DefaultValue>
class DefaultedUnorderedMap : KVContainer<K, V, DefaultValue> {
 public:
  void Resize(std::size_t _ignoredn) override {}  // ignore
  V &operator[](K k) override;
  const V &operator[](K k) const override;
  void Clear() override { m.clear(); }

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
class DefaultedVector : KVContainer<int, V, DefaultValue> {
 public:
  void Resize(std::size_t n) override { vec.resize(n, defaultValue); }
  V &operator[](int k) override { return vec[k]; }
  const V &operator[](int k) const override { return vec[k]; }
  void Clear() override { vec.clear(); }

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
