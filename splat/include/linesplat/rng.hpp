// A small seeded random number generator that gives the same numbers with
// every compiler. (std::normal_distribution differs between libstdc++ and
// MSVC, which would make the synthetic dataset depend on the platform.)
#pragma once

#include <cstdint>

namespace linesplat {

class Rng {
 public:
  explicit Rng(uint64_t seed) : state_(seed) {}
  uint64_t next_u64();      // splitmix64
  double uniform();         // [0, 1)
  double uniform(double lo, double hi) { return lo + (hi - lo) * uniform(); }
  double normal();          // standard normal (Box-Muller)

 private:
  uint64_t state_;
  bool has_spare_ = false;
  double spare_ = 0.0;
};

}  // namespace linesplat
