#include "linesplat/scene.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "linesplat/npy.hpp"
#include "linesplat/util.hpp"

namespace linesplat {

void GaussianScene::add(const float mean[3], const float scales[3], const float quat_wxyz[4], float opacity,
                        const float* feats) {
  for (int i = 0; i < 3; ++i) {
    means.push_back(mean[i]);
    log_scales.push_back(std::log(scales[i]));
  }
  for (int i = 0; i < 4; ++i) rotations.push_back(quat_wxyz[i]);
  const float o = std::min(std::max(opacity, 1e-6f), 1.0f - 1e-6f);
  opacity_logits.push_back(std::log(o / (1.0f - o)));
  features.insert(features.end(), feats, feats + num_features);
}

void GaussianScene::set_identity_basis(int bands) {
  if (bands != num_features) throw std::runtime_error("identity basis needs bands == num_features");
  basis.assign(size_t(bands) * bands, 0.0f);
  for (int b = 0; b < bands; ++b) basis[size_t(b) * bands + b] = 1.0f;
}

void GaussianScene::validate() const {
  const size_t n = size_t(size());
  if (num_features <= 0) throw std::runtime_error("scene has no features");
  if (means.size() != 3 * n || log_scales.size() != 3 * n || rotations.size() != 4 * n ||
      features.size() != n * num_features)
    throw std::runtime_error("scene arrays disagree on the number of Gaussians");
  if (basis.empty() || basis.size() % num_features != 0)
    throw std::runtime_error("scene basis must be [bands, num_features]");
  if (background.size() != size_t(num_features))
    throw std::runtime_error("scene background must have num_features entries");
}

void GaussianScene::save(const std::string& dir) const {
  validate();
  make_dirs(dir);
  const size_t n = size_t(size()), k = size_t(num_features);
  npy_save(join_path(dir, "means.npy"), means, {n, 3});
  npy_save(join_path(dir, "log_scales.npy"), log_scales, {n, 3});
  npy_save(join_path(dir, "rotations.npy"), rotations, {n, 4});
  npy_save(join_path(dir, "opacity_logits.npy"), opacity_logits, {n});
  npy_save(join_path(dir, "features.npy"), features, {n, k});
  npy_save(join_path(dir, "basis.npy"), basis, {basis.size() / k, k});
  npy_save(join_path(dir, "background.npy"), background, {k});
}

GaussianScene GaussianScene::load(const std::string& dir) {
  GaussianScene s;
  std::vector<size_t> shape;
  s.features = npy_load_f32(join_path(dir, "features.npy"), &shape);
  if (shape.size() != 2) throw std::runtime_error("features.npy must be [N, K]");
  s.num_features = int(shape[1]);
  s.means = npy_load_f32(join_path(dir, "means.npy"));
  s.log_scales = npy_load_f32(join_path(dir, "log_scales.npy"));
  s.rotations = npy_load_f32(join_path(dir, "rotations.npy"));
  s.opacity_logits = npy_load_f32(join_path(dir, "opacity_logits.npy"));
  s.basis = npy_load_f32(join_path(dir, "basis.npy"));
  s.background = npy_load_f32(join_path(dir, "background.npy"));
  s.validate();
  return s;
}

}  // namespace linesplat
