// A hyperspectral Gaussian scene.
//
// Each Gaussian has the usual 3DGS shape parameters plus K spectral features
// in place of an RGB color. A shared basis matrix (bands x K) turns features
// into a spectrum: spectrum = basis * features. Because alpha compositing is
// linear in the color, the renderers composite the K features and apply the
// basis once per pixel afterwards, which costs K channels instead of one per
// band. With K = number of bands and an identity basis, the features are the
// spectrum itself (that's how the synthetic ground truth is stored).
#pragma once

#include <string>
#include <vector>

#include "linesplat/line_camera.hpp"

namespace linesplat {

struct GaussianScene {
  int num_features = 0;               // K
  std::vector<float> means;           // [N, 3] world position (m)
  std::vector<float> log_scales;      // [N, 3] log of the axis standard deviations (m)
  std::vector<float> rotations;       // [N, 4] quaternion (w, x, y, z), any length
  std::vector<float> opacity_logits;  // [N] opacity = sigmoid(logit)
  std::vector<float> features;        // [N, K]
  std::vector<float> basis;           // [bands, K] spectrum = basis * features
  std::vector<float> background;      // [K] features seen where no Gaussian covers a pixel

  int size() const { return int(opacity_logits.size()); }
  int num_bands() const { return num_features > 0 ? int(basis.size()) / num_features : 0; }

  // Appends one Gaussian. `scales` are standard deviations (m), not logs.
  void add(const float mean[3], const float scales[3], const float quat_wxyz[4], float opacity,
           const float* feats);
  // Sets the basis to the identity, so features are band values.
  void set_identity_basis(int bands);
  // Throws std::runtime_error if array sizes disagree.
  void validate() const;

  // A directory of .npy files (means.npy, log_scales.npy, rotations.npy,
  // opacity_logits.npy, features.npy, basis.npy, background.npy), readable
  // from Python with numpy.load.
  void save(const std::string& dir) const;
  static GaussianScene load(const std::string& dir);
};

// Per-Gaussian geometry in the form the projection needs. The GPU calls the
// same function on its own copy of the arrays.
template <typename T>
LS_HD GaussianGeomT<T> gaussian_geometry(const float* mean, const float* log_scale, const float* quat,
                                         float opacity_logit) {
  GaussianGeomT<T> g;
  g.mean = Vec3<T>{T(mean[0]), T(mean[1]), T(mean[2])};
  const Vec3<T> sc{ls_exp(T(log_scale[0])), ls_exp(T(log_scale[1])), ls_exp(T(log_scale[2]))};
  g.cov = covariance_from_scale_rot(sc, quat_to_mat(T(quat[0]), T(quat[1]), T(quat[2]), T(quat[3])));
  g.opacity = ls_sigmoid(T(opacity_logit));
  g.max_scale = ls_max(sc.x, ls_max(sc.y, sc.z));
  return g;
}

template <typename T>
GaussianGeomT<T> gaussian_geometry(const GaussianScene& s, int i) {
  return gaussian_geometry<T>(&s.means[3 * size_t(i)], &s.log_scales[3 * size_t(i)],
                              &s.rotations[4 * size_t(i)], s.opacity_logits[i]);
}

}  // namespace linesplat
