// The per-Gaussian rules of densification, shared by the CPU trainer
// (trainer.cpp) and the GPU one (src/cuda/optimizer.cu) so that both make the
// same decisions and draw the same split offsets. Keep this header C++14.
#pragma once

#include <cstdint>

#include "linesplat/math.hpp"
#include "linesplat/rng.hpp"

namespace linesplat {

// Thresholds of one densify() call, worked out once on the host so that the
// CPU and the GPU compare against the same floats.
struct DensifyThresholds {
  double grad = 0.0;        // mean screen gradient per pair that densifies
  double split_log = 0.0;   // log scale above which a Gaussian splits rather than clones
  float shrink = 0.0f;      // log(1.6): split halves are 1.6 times smaller
  float min_logit = 0.0f;   // opacity logit below which a Gaussian is pruned
  float max_log = 0.0f;     // log scale above which a Gaussian is pruned
  uint64_t key = 0;         // split offsets are hash_normal(key, Gaussian, 0..5)
};

LS_HD float max3(const float* v) { return ls_max(v[0], ls_max(v[1], v[2])); }

// Whether Gaussian i's screen gradient, averaged over its pairs, is large
// enough to densify it.
LS_HD bool densify_wanted(double screen_sum, int pairs, const DensifyThresholds& t) {
  return !(pairs == 0 || screen_sum / pairs < t.grad);
}

LS_HD bool densify_splits(const float* log_scale, const DensifyThresholds& t) { return max3(log_scale) > t.split_log; }

// Whether a Gaussian with this opacity logit and these log scales stays.
LS_HD bool survives(float logit, const float* log_scale, const DensifyThresholds& t) {
  return logit >= t.min_logit && max3(log_scale) <= t.max_log;
}

// Half j (0 or 1) of splitting Gaussian i: a point drawn from the Gaussian,
// 1.6 times smaller.
LS_HD void split_half(const float* mean, const float* log_scale, const float* quat, uint64_t i, int j,
                      const DensifyThresholds& t, float* out_mean, float* out_log_scale) {
  const Mat3<double> R = quat_to_mat<double>(double(quat[0]), double(quat[1]), double(quat[2]), double(quat[3]));
  const Vec3<double> z{ls_exp(double(log_scale[0])) * hash_normal(t.key, i, uint64_t(3 * j)),
                       ls_exp(double(log_scale[1])) * hash_normal(t.key, i, uint64_t(3 * j + 1)),
                       ls_exp(double(log_scale[2])) * hash_normal(t.key, i, uint64_t(3 * j + 2))};
  const Vec3<double> off = R * z;
  out_mean[0] = mean[0] + float(off.x);
  out_mean[1] = mean[1] + float(off.y);
  out_mean[2] = mean[2] + float(off.z);
  for (int k = 0; k < 3; ++k) out_log_scale[k] = log_scale[k] - t.shrink;
}

}  // namespace linesplat
