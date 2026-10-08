// Scenes and cameras shared by the tests.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

#include "linesplat/backward_cpu.hpp"
#include "linesplat/rng.hpp"
#include "linesplat/scan_model.hpp"
#include "linesplat/scene.hpp"
#include "linesplat/trainer.hpp"

namespace lsfix {

using namespace linesplat;

// n random Gaussians in a ball around `center`, with k random features.
inline GaussianScene random_scene(int n, int k, uint64_t seed, Vec3d center = Vec3d{0, 0, 0.15},
                                  double radius = 0.03, double min_scale = 3e-4, double max_scale = 3e-3) {
  Rng rng(seed);
  GaussianScene s;
  s.num_features = k;
  std::vector<float> f(static_cast<size_t>(k));
  for (int i = 0; i < n; ++i) {
    Vec3d p;
    do {
      p = Vec3d{rng.uniform(-1, 1), rng.uniform(-1, 1), rng.uniform(-1, 1)};
    } while (dot(p, p) > 1.0);
    p = center + radius * p;
    const float mean[3] = {float(p.x), float(p.y), float(p.z)};
    float sc[3];
    for (float& v : sc) v = float(min_scale * std::exp(rng.uniform() * std::log(max_scale / min_scale)));
    float q[4] = {float(rng.normal()), float(rng.normal()), float(rng.normal()), float(rng.normal())};
    for (float& v : f) v = float(rng.uniform());
    s.add(mean, sc, q, float(rng.uniform(0.05, 0.99)), f.data());
  }
  s.basis.assign(size_t(k) * k, 0.0f);
  for (int i = 0; i < k; ++i) s.basis[size_t(i) * k + i] = 1.0f;
  s.background.assign(size_t(k), 0.0f);
  for (int i = 0; i < k; ++i) s.background[size_t(i)] = float(0.1 * i / std::max(1, k - 1));
  return s;
}

inline LineIntrinsics test_intrinsics(int width, double f) {
  LineIntrinsics in;
  in.width = width;
  in.f = f;
  in.cu = 0.5 * width;
  in.v_slit = 0.0;
  in.sigma_u = 0.5;
  in.sigma_v = 0.9;
  in.near_z = 0.01;
  return in;
}

// Cameras at the origin looking down +z, each turned a little about x (the
// scan direction) so the lines sweep across the scene.
inline std::vector<LineCameraT<double>> sweep_cameras(int lines, int width, double f, double half_angle,
                                                      double v_slit = 0.0) {
  std::vector<LineCameraT<double>> cams;
  LineIntrinsics in = test_intrinsics(width, f);
  in.v_slit = v_slit;
  for (int l = 0; l < lines; ++l) {
    const double a = lines == 1 ? 0.0 : -half_angle + 2.0 * half_angle * l / (lines - 1);
    Pose p;
    p.R = rotation_about(Vec3d{1, 0, 0}, a);
    cams.push_back(line_camera_from_pose(p, in));
  }
  return cams;
}

// A random gradient for scene s, as a backward pass would give: every
// array, the basis if learn_basis, screen gradients and pair counts (a
// quarter of the Gaussians drew nothing).
inline SceneGradT<float> random_gradient(const GaussianScene& s, uint64_t seed, bool learn_basis,
                                         double screen_scale = 1.0) {
  Rng rng(seed);
  SceneGradT<float> g;
  g.learn_basis = learn_basis;
  g.reset(s);
  auto fill = [&](std::vector<float>& v, double scale) {
    for (float& x : v) x = float(scale * rng.uniform(-1, 1));
  };
  fill(g.means, 1e-2);
  fill(g.log_scales, 1e-3);
  fill(g.rotations, 1e-3);
  fill(g.opacity_logits, 1e-3);
  fill(g.features, 1e-3);
  fill(g.background, 1e-4);
  fill(g.basis, 1e-3);
  for (int i = 0; i < s.size(); ++i) {
    g.pairs[size_t(i)] = rng.uniform() < 0.25 ? 0 : 1 + int(rng.uniform() * 3);
    g.screen_grad[size_t(i)] = g.pairs[size_t(i)] ? float(screen_scale * rng.uniform()) : 0.0f;
  }
  return g;
}

// The largest difference between two scenes' arrays, or infinity if their
// shapes differ; *differ counts the values that aren't equal.
inline double scene_diff(const GaussianScene& a, const GaussianScene& b, int* differ = nullptr) {
  if (a.num_features != b.num_features || a.size() != b.size() || a.basis.size() != b.basis.size())
    return HUGE_VAL;
  double worst = 0;
  int n = 0;
  auto cmp = [&](const std::vector<float>& x, const std::vector<float>& y) {
    for (size_t i = 0; i < x.size(); ++i) {
      const double d = std::fabs(double(x[i]) - double(y[i]));
      worst = std::max(worst, d);
      n += x[i] != y[i];
    }
  };
  cmp(a.means, b.means);
  cmp(a.log_scales, b.log_scales);
  cmp(a.rotations, b.rotations);
  cmp(a.opacity_logits, b.opacity_logits);
  cmp(a.features, b.features);
  cmp(a.background, b.background);
  cmp(a.basis, b.basis);
  if (differ) *differ = n;
  return worst;
}

// Six Gaussians that densification treats six ways, with the statistics
// and settings that make it so (see test_trainer.cpp):
//   0  small, large gradient           cloned
//   1  large, large gradient           split in two
//   2  small, small gradient           kept as it is
//   3  nearly transparent, large grad  cloned, then it and its clone pruned
//   4  oversized, no pairs             pruned
//   5  small, large gradient           kept: no room left after 0, 1 and 3
inline void densify_scenario(GaussianScene* s, SceneGradT<float>* g, DensifyParams* p) {
  *s = GaussianScene();
  s->num_features = 2;
  const double scale[6] = {5e-4, 2e-3, 5e-4, 5e-4, 2e-2, 5e-4};
  const double opacity[6] = {0.5, 0.6, 0.7, 0.003, 0.5, 0.8};
  const float screen[6] = {2.0f, 3.0f, 0.5f, 2.5f, 0.0f, 2.0f};
  const int pairs[6] = {1, 1, 1, 1, 0, 1};
  for (int i = 0; i < 6; ++i) {
    const float mean[3] = {0.01f * i, -0.02f, 0.15f};
    const float sc[3] = {float(scale[i]), float(0.8 * scale[i]), float(0.5 * scale[i])};
    const float q[4] = {1.0f, 0.1f * i, -0.2f, 0.05f};
    const float f[2] = {0.1f * i, 1.0f - 0.1f * i};
    s->add(mean, sc, q, float(opacity[i]), f);
  }
  s->set_identity_basis(2);
  s->background = {0.0f, 0.0f};
  g->learn_basis = false;
  g->reset(*s);
  for (int i = 0; i < 6; ++i) {
    g->screen_grad[size_t(i)] = screen[i];
    g->pairs[size_t(i)] = pairs[i];
  }
  p->grad = 1.0;
  p->split_scale = 1e-3;
  p->prune_opacity = 0.005;
  p->max_scale = 1e-2;
  p->max_gaussians = 9;  // room for three
  p->key = 1234;
}

template <typename T>
std::vector<LineCameraT<T>> cast_all(const std::vector<LineCameraT<double>>& c) {
  std::vector<LineCameraT<T>> o;
  for (const auto& x : c) o.push_back(cast_camera<T>(x));
  return o;
}

}  // namespace lsfix
