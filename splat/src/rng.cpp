#include "linesplat/rng.hpp"

#include <cmath>

namespace linesplat {

uint64_t Rng::next_u64() {
  uint64_t z = (state_ += 0x9E3779B97F4A7C15ull);
  z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
  z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
  return z ^ (z >> 31);
}

double Rng::uniform() { return double(next_u64() >> 11) * (1.0 / 9007199254740992.0); }

double Rng::normal() {
  if (has_spare_) {
    has_spare_ = false;
    return spare_;
  }
  const double kTwoPi = 6.283185307179586;
  double u1 = uniform();
  while (u1 <= 0.0) u1 = uniform();
  const double u2 = uniform();
  const double r = std::sqrt(-2.0 * std::log(u1));
  spare_ = r * std::sin(kTwoPi * u2);
  has_spare_ = true;
  return r * std::cos(kTwoPi * u2);
}

}  // namespace linesplat
