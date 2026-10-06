#include <algorithm>

#include "fixtures.hpp"
#include "linesplat/spectra.hpp"
#include "linesplat/synthetic.hpp"
#include "test.hpp"

using namespace linesplat;

namespace {
SyntheticOptions tiny_options() {
  SyntheticOptions o;
  o.width = 48;
  o.bands = 8;
  o.sweeps = 3;
  o.microsteps_per_line = 12;
  o.slit_samples = 3;
  o.spacing = 2.0e-3;
  return o;
}
}  // namespace

TEST(synthetic_dataset_has_the_promised_shape) {
  const SyntheticOptions o = tiny_options();
  const SyntheticData s = make_synthetic_dataset(o);
  const Dataset& d = s.dataset;
  // +-6 degrees of mirror in steps of 12 microsteps.
  const int per_sweep = int(std::floor(12.0 / (12 * 1.8 / 32) + 1e-9)) + 1;
  CHECK(d.num_lines() == o.sweeps * per_sweep);
  CHECK(d.num_sweeps() == o.sweeps);
  CHECK(d.width() == o.width && d.num_bands() == o.bands);
  CHECK(int(s.true_mirror_angle.size()) == d.num_lines());
  CHECK(s.clean_lines.size() == d.lines.size());
  d.validate();
  // The scan sees the scene, not the black background.
  double mean = 0;
  for (float v : s.clean_lines) mean += v;
  mean /= double(s.clean_lines.size());
  CHECK(mean > 0.2);
}

TEST(synthetic_poses_carry_the_requested_errors) {
  SyntheticOptions o = tiny_options();
  o.sweeps = 40;
  o.microsteps_per_line = 64;
  o.width = 16;
  o.bands = 2;
  o.slit_samples = 1;
  o.spacing = 4e-3;
  const SyntheticData s = make_synthetic_dataset(o);
  double t2 = 0, r2 = 0;
  for (int k = 0; k < o.sweeps; ++k) {
    const Pose d = s.true_head_pose[size_t(k)].inverse() * s.dataset.sweep_head_pose[size_t(k)];
    t2 += dot(d.t, d.t);
    const Vec3d w = rotation_log(d.R);
    r2 += dot(w, w);
  }
  // RMS per axis close to the sigmas (40 sweeps x 3 axes).
  CHECK_NEAR(std::sqrt(t2 / (3 * o.sweeps)), o.head_trans_sigma, 0.3 * o.head_trans_sigma);
  CHECK_NEAR(std::sqrt(r2 / (3 * o.sweeps)) * 180 / kPi, o.head_rot_sigma_deg, 0.3 * o.head_rot_sigma_deg);
  // Mirror: commanded angles are evenly spaced, true ones are off by a little.
  double worst = 0;
  for (int l = 0; l < s.dataset.num_lines(); ++l)
    worst = std::max(worst, std::fabs(s.true_mirror_angle[size_t(l)] - s.dataset.line_mirror_angle[size_t(l)]));
  CHECK(worst > 0.0 && worst < 6 * (o.mirror_offset_sigma_deg + o.mirror_jitter_sigma_deg) * kPi / 180);
}

TEST(synthetic_dataset_is_reproducible) {
  const SyntheticOptions o = tiny_options();
  const SyntheticData a = make_synthetic_dataset(o);
  const SyntheticData b = make_synthetic_dataset(o);
  CHECK(a.dataset.lines == b.dataset.lines);
  SyntheticOptions o2 = o;
  o2.seed = 2;
  CHECK(make_synthetic_dataset(o2).dataset.lines != a.dataset.lines);
}

TEST(hidden_word_shows_only_in_the_nir) {
  // The dye and the carbon look the same to the eye but not past 740 nm.
  const std::vector<double> wl = wavelength_grid(500, 950, 46);
  const auto dye = material_spectrum(Material::kIrBlackDye, wl);
  const auto carbon = material_spectrum(Material::kCarbonBlack, wl);
  const auto a = true_color(dye.data(), wl), b = true_color(carbon.data(), wl);
  for (int c = 0; c < 3; ++c) CHECK_NEAR(a[size_t(c)], b[size_t(c)], 0.01);
  const auto ca = color_infrared(dye.data(), wl), cb = color_infrared(carbon.data(), wl);
  CHECK(ca[0] > 0.6 && cb[0] < 0.1);
  // And the true color of white paper is white.
  const auto paper = material_spectrum(Material::kWhitePaper, wl);
  const auto w = true_color(paper.data(), wl);
  CHECK(w[0] > 0.8 && w[1] > 0.8 && w[2] > 0.8);
}
