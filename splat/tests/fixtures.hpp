// Scenes and cameras shared by the tests.
#pragma once

#include <cmath>
#include <vector>

#include "linesplat/rng.hpp"
#include "linesplat/scan_model.hpp"
#include "linesplat/scene.hpp"

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

template <typename T>
std::vector<LineCameraT<T>> cast_all(const std::vector<LineCameraT<double>>& c) {
  std::vector<LineCameraT<T>> o;
  for (const auto& x : c) o.push_back(cast_camera<T>(x));
  return o;
}

}  // namespace lsfix
