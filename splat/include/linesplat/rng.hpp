// A small seeded random number generator that gives the same numbers with
// every compiler. (std::normal_distribution differs between libstdc++ and
// MSVC, which would make the synthetic dataset depend on the platform.)
#pragma once

#include <cstdint>

#include "linesplat/math.hpp"

namespace linesplat {

// Random numbers by address rather than in sequence: the same (key, i, k)
// always gives the same number, on the CPU or the GPU, in any order. The
// trainer draws the offsets of split Gaussians this way, so that densifying
// on the GPU, where every Gaussian is its own thread, gives the CPU's result.
LS_HD uint64_t mix64(uint64_t z) {  // splitmix64's finalizer
  z += 0x9E3779B97F4A7C15ull;
  z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
  z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
  return z ^ (z >> 31);
}

// Uniform in (0, 1].
LS_HD double hash_uniform(uint64_t key, uint64_t i, uint64_t k) {
  return double((mix64(key ^ mix64(i * 64 + k)) >> 11) + 1) * (1.0 / 9007199254740992.0);
}

// Standard normal (Box-Muller), number k of item i.
LS_HD double hash_normal(uint64_t key, uint64_t i, uint64_t k) {
  const double u1 = hash_uniform(key, i, 2 * k), u2 = hash_uniform(key, i, 2 * k + 1);
  return ls_sqrt(-2.0 * ls_log(u1)) * ls_cos(2.0 * kPi * u2);
}

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
