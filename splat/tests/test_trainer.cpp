// Pose refinement maths and the trainer.
#include <algorithm>
#include <cmath>
#include <cstdio>

#include "fixtures.hpp"
#include "linesplat/backward_cpu.hpp"
#include "linesplat/render_cpu.hpp"
#include "linesplat/spectra.hpp"
#include "linesplat/synthetic.hpp"
#include "linesplat/trainer.hpp"
#include "test.hpp"

using namespace linesplat;

namespace {

constexpr double kDeg = kPi / 180.0;

// A head whose virtual camera (mirror at rest) sits at `eye` looking at the
// origin, like the synthetic scan plans its sweeps.
Pose head_looking_at_origin(const HeadModel& head, const Vec3d& eye, const Vec3d& up) {
  return look_at(eye, Vec3d{0, 0, 0}, up) * virtual_camera_in_head(head, 0.0).inverse();
}

std::vector<double> mirror_angles(int n, double half_deg) {
  std::vector<double> a;
  for (int i = 0; i < n; ++i) a.push_back((-half_deg + 2.0 * half_deg * i / std::max(1, n - 1)) * kDeg);
  return a;
}

}  // namespace

TEST(head_correction_grad_matches_finite_differences) {
  // Gaussians around the origin, seen by one sweep's lines through the CAD
  // head, and the loss sum(w * bands) with random w. The correction is made
  // in the head's frame, and in the camera's (as the trainer does).
  GaussianScene s = lsfix::random_scene(40, 3, 91, Vec3d{0, 0, 0}, 0.008, 4e-4, 2e-3);
  const HeadModel head = cad_head_model();
  const Pose recorded = head_looking_at_origin(head, Vec3d{0.02, -0.03, 0.145}, Vec3d{0, 0, 1});
  const std::vector<double> angles = mirror_angles(9, 2.0);
  LineIntrinsics in = lsfix::test_intrinsics(32, 400.0);
  in.v_slit = 0.7;  // off centre, so the mirror's flip of v_slit matters
  const double zeta0[6] = {4e-4, -2e-4, 3e-4, 0.3 * kDeg, -0.2 * kDeg, 0.25 * kDeg};
  Rng rng(92);
  const int W = in.width, B = 3;
  std::vector<double> w(angles.size() * W * B);
  for (double& v : w) v = rng.uniform(-1, 1);

  const Pose frames[2] = {Pose(), virtual_camera_in_head(head, 0.0)};
  for (int f = 0; f < 2; ++f) {
    const Pose& G = frames[f];
    auto cameras = [&](const double z[6]) {
      const Pose h = corrected_head(recorded, G, z);
      std::vector<LineCameraT<double>> cams;
      for (double a : angles) cams.push_back(make_line_camera_d(head, h, a, in));
      return cams;
    };
    auto F = [&](const double z[6]) {
      const LineImageT<double> img = features_to_bands(s, render_lines_cpu<double>(s, cameras(z)));
      double sum = 0;
      for (size_t i = 0; i < w.size(); ++i) sum += w[i] * img.values[i];
      return sum;
    };
    const std::vector<LineCameraT<double>> cams = cameras(zeta0);
    SceneGradT<double> g;
    std::vector<CameraGradT<double>> cg;
    render_backward_cpu<double>(
        s, cams,
        [&](int l, const double* b, double* gb) {
          double sum = 0;
          for (int i = 0; i < W * B; ++i) {
            gb[i] = w[size_t(l) * W * B + i];
            sum += gb[i] * b[i];
          }
          return sum;
        },
        &g, &cg);
    int drawn = 0;
    for (int p : g.pairs) drawn += p > 0;
    CHECK(drawn >= 10);

    double g_eta[6] = {0, 0, 0, 0, 0, 0}, ana[6];
    for (size_t l = 0; l < cams.size(); ++l)
      add_head_twist_grad(cams[l], virtual_camera_in_head(head, angles[l]), cg[l], g_eta);
    correction_grad(G, zeta0, g_eta, ana);
    double num[6];
    for (int k = 0; k < 6; ++k) {
      const double h = k < 3 ? 1e-7 : 1e-6;
      double zp[6], zm[6];
      std::copy(zeta0, zeta0 + 6, zp);
      std::copy(zeta0, zeta0 + 6, zm);
      zp[k] += h;
      zm[k] -= h;
      num[k] = (F(zp) - F(zm)) / (2 * h);
    }
    // Translation and rotation gradients have different units; compare each
    // group against its own size.
    for (int grp = 0; grp < 2; ++grp) {
      double scale = 0;
      for (int k = 3 * grp; k < 3 * grp + 3; ++k) scale = std::max(scale, std::fabs(num[k]));
      for (int k = 3 * grp; k < 3 * grp + 3; ++k) {
        std::printf("       %s %s[%d]  analytic %+.6e  numeric %+.6e\n", f ? "camera" : "head",
                    grp ? "phi" : "rho", k % 3, ana[k], num[k]);
        CHECK(std::fabs(ana[k] - num[k]) <= 1e-4 * scale);
      }
    }
  }
}

TEST(line_pose_error_ignores_rigid_motion_and_sees_a_tilt) {
  const HeadModel head = cad_head_model();
  const LineIntrinsics in = intrinsics_from_optics(128);
  const std::vector<double> angles = mirror_angles(40, 5.0);
  const Pose h0 = head_looking_at_origin(head, Vec3d{0.0, -0.01, 0.15}, Vec3d{0, 1, 0});
  const Pose h1 = head_looking_at_origin(head, Vec3d{0.08, 0.03, 0.12}, Vec3d{0, 0, 1});
  auto sweep = [&](const Pose& a, const Pose& b) {
    std::vector<LineCameraT<double>> cams;
    for (double x : angles) cams.push_back(make_line_camera_d(head, a, x, in));
    for (double x : angles) cams.push_back(make_line_camera_d(head, b, x, in));
    return cams;
  };
  const auto truth = sweep(h0, h1);

  // Everything moved rigidly: nothing to see.
  Pose X;
  X.R = rotation_exp(Vec3d{0.2, -0.3, 0.1});
  X.t = Vec3d{0.05, -0.02, 0.03};
  const PoseErrorPx e0 = line_pose_error_px(sweep(X * h0, X * h1), truth);
  std::printf("       rigid motion: rms %.2e px over %d points\n", e0.rms, e0.points);
  CHECK(e0.points == 2 * 40 * 5);
  CHECK(e0.rms < 1e-6);

  // One sweep turned by a pixel's angle about the slit: the views of the two
  // sweeps disagree by about a pixel, mostly across the slit. (The best
  // alignment splits the difference between the sweeps, which look from
  // different directions, so some of it shows along the slit.)
  const Pose h1t = perturb(h1, Vec3d{0, 0, 0}, Vec3d{1.0 / in.f, 0, 0});
  const PoseErrorPx e1 = line_pose_error_px(sweep(h0, h1t), truth);
  std::printf("       one sweep tilted 1 px: rms %.3f px (along %.3f, across %.3f), max %.3f\n", e1.rms, e1.rms_along,
              e1.rms_across, e1.max);
  CHECK(e1.rms > 0.3 && e1.rms < 1.0);
  CHECK(e1.rms_across > e1.rms_along);
}

TEST(training_fits_the_lines_and_improves_the_poses) {
  // A small synthetic scan: 4 sweeps of 27 lines, 32 px, 3 bands.
  SyntheticOptions so;
  so.width = 32;
  so.bands = 3;
  so.sweeps = 4;
  so.microsteps_per_line = 8;
  so.slit_samples = 3;
  const SyntheticData sd = make_synthetic_dataset(so);
  const Dataset& d = sd.dataset;
  std::vector<LineCameraT<double>> truth;
  for (int l = 0; l < d.num_lines(); ++l)
    truth.push_back(make_line_camera_d(d.head, sd.true_head_pose[size_t(d.line_sweep[size_t(l)])],
                                       sd.true_mirror_angle[size_t(l)], d.intrinsics));

  TrainOptions o;
  o.iterations = 200;
  o.batch_lines = 32;
  o.pose_from = 20;
  o.densify_from = 50;
  o.densify_every = 50;
  o.densify_until = 150;
  o.max_gaussians = 4000;
  Trainer t(d, init_on_plane(d, 3e-3), o);
  const int n0 = t.scene().size();
  const PoseErrorPx e0 = line_pose_error_px(t.cameras(), truth);
  double first = 0, last = 0;
  int added = 0, removed = 0;
  for (int i = 0; i < o.iterations; ++i) {
    const TrainStep st = t.step();
    if (i < 20) first += st.loss / 20;
    if (i >= o.iterations - 20) last += st.loss / 20;
    added += st.densified;
    removed += st.pruned;
  }
  const PoseErrorPx e1 = line_pose_error_px(t.cameras(), truth);
  std::printf("       loss %.4f -> %.4f, pose %.2f -> %.2f px, Gaussians %d -> %d (+%d -%d)\n", first, last, e0.rms,
              e1.rms, n0, t.scene().size(), added, removed);
  t.scene().validate();
  CHECK(last < 0.25 * first);
  CHECK(e1.rms < 0.8 * e0.rms);
  CHECK(added > 0);
  CHECK(t.scene().size() == n0 + added - removed);
  CHECK(t.scene().size() <= o.max_gaussians);
}

namespace {

// The spectra of a scene's Gaussians, [N, bands].
std::vector<double> spectra(const GaussianScene& s) {
  const int K = s.num_features, B = s.num_bands();
  std::vector<double> x(size_t(s.size()) * B, 0.0);
  for (int i = 0; i < s.size(); ++i)
    for (int b = 0; b < B; ++b)
      for (int c = 0; c < K; ++c) x[size_t(i) * B + b] += double(s.basis[size_t(b) * K + c]) * s.features[size_t(i) * K + c];
  return x;
}

BatchBackwardFn no_backward() {
  return [](const GaussianScene&, const std::vector<LineCamera>&, const std::vector<int>&, SceneGradT<float>*,
            std::vector<CameraGradT<float>>*) -> double { throw std::runtime_error("not used"); };
}

}  // namespace

TEST(reduce_features_keeps_the_spectra) {
  // The synthetic scene's 24-band spectra (an identity basis), through fewer
  // features.
  SyntheticOptions o;
  o.bands = 24;
  const GaussianScene s = make_synthetic_scene(o, wavelength_grid(o.wl_min_nm, o.wl_max_nm, o.bands));
  const std::vector<double> x = spectra(s);
  double prev = 0;  // fewer features can only fit worse
  for (int K : {24, 12, 8, 4, 2}) {
    const GaussianScene r = reduce_features(s, K);
    CHECK(r.num_features == K && r.num_bands() == 24 && r.size() == s.size());
    const std::vector<double> y = spectra(r);
    double se = 0;
    for (size_t i = 0; i < x.size(); ++i) se += (x[i] - y[i]) * (x[i] - y[i]);
    const double rms = std::sqrt(se / x.size());
    // The basis: orthogonal columns of squared length 24.
    double worst = 0;
    for (int a = 0; a < K; ++a)
      for (int b = 0; b < K; ++b) {
        double d = 0;
        for (int w = 0; w < 24; ++w) d += double(r.basis[size_t(w) * K + a]) * r.basis[size_t(w) * K + b];
        worst = std::max(worst, std::fabs(d - (a == b ? 24.0 : 0.0)));
      }
    std::printf("       K = %2d: spectra off by %.2e rms (reflectance), basis off orthogonal by %.1e\n", K, rms, worst);
    CHECK(worst < 1e-4);
    CHECK(rms >= prev - 1e-7);
    if (K == 24) CHECK(rms < 1e-5);
    if (K == 8) CHECK(rms < 0.01);
    prev = rms;
  }
}

TEST(densify_follows_the_rules) {
  GaussianScene s;
  SceneGradT<float> g;
  DensifyParams p;
  lsfix::densify_scenario(&s, &g, &p);
  HostSceneOptimizer opt(s, no_backward());
  // One Adam step first, so the kept Gaussians have moments to carry over.
  SceneGradT<float> ones = g;
  for (float& v : ones.means) v = 1.0f;
  opt.set_gradient(ones);
  SceneRates lr;
  lr.means = 1e-4;
  opt.adam(lr, 1);
  opt.set_gradient(g);
  opt.accumulate(1.0);
  const GaussianScene before = opt.scene();
  const DensifyCounts c = opt.densify(p);
  const GaussianScene& a = opt.scene();
  std::printf("       %d -> %d Gaussians, +%d -%d\n", before.size(), a.size(), c.added, c.removed);
  CHECK(c.added == 4);    // one clone each of 0 and 3, two halves of 1
  CHECK(c.removed == 4);  // 1 (split), 3 and its clone, 4
  CHECK(a.size() == 6);
  // The kept ones in order (0, 2, 5), then the new ones (0's clone, 1's halves).
  const int from[6] = {0, 2, 5, 0, 1, 1};
  const DensifyThresholds th = densify_thresholds(p);
  for (int k = 0; k < 6; ++k) {
    const int i = from[k];
    CHECK(a.opacity_logits[size_t(k)] == before.opacity_logits[size_t(i)]);
    CHECK(a.features[2 * size_t(k)] == before.features[2 * size_t(i)]);
    for (int d = 0; d < 4; ++d) CHECK(a.rotations[4 * size_t(k) + d] == before.rotations[4 * size_t(i) + d]);
    float mean[3], ls[3];
    if (k >= 4) {
      split_half(&before.means[3 * size_t(i)], &before.log_scales[3 * size_t(i)], &before.rotations[4 * size_t(i)],
                 uint64_t(i), k - 4, th, mean, ls);
    } else {
      std::copy(&before.means[3 * size_t(i)], &before.means[3 * size_t(i)] + 3, mean);
      std::copy(&before.log_scales[3 * size_t(i)], &before.log_scales[3 * size_t(i)] + 3, ls);
    }
    for (int d = 0; d < 3; ++d) {
      CHECK(a.means[3 * size_t(k) + d] == mean[d]);
      CHECK(a.log_scales[3 * size_t(k) + d] == ls[d]);
    }
  }
  // The halves are 1.6 times smaller and land within a few sigma of 1.
  CHECK_NEAR(std::exp(a.log_scales[12]), 2e-3 / 1.6, 1e-9);
  CHECK(std::fabs(a.means[12] - before.means[3]) < 4 * 2e-3);
  CHECK(a.means[12] != a.means[15]);
  // Kept Gaussians kept their moments: with a zero gradient they still move,
  // and the new ones, starting from zero, don't.
  SceneGradT<float> zero = g;
  zero.reset(a);
  opt.set_gradient(zero);
  const GaussianScene after_densify = a;
  opt.adam(lr, 2);
  for (int k = 0; k < 6; ++k) {
    const bool moved = opt.scene().means[3 * size_t(k)] != after_densify.means[3 * size_t(k)];
    CHECK(moved == (k < 3));
  }
}

TEST(training_with_a_learned_basis_fits_the_lines) {
  // The scan above with 12 bands, through 4 features and a learned basis.
  SyntheticOptions so;
  so.width = 32;
  so.bands = 12;
  so.sweeps = 4;
  so.microsteps_per_line = 8;
  so.slit_samples = 3;
  const SyntheticData sd = make_synthetic_dataset(so);
  const Dataset& d = sd.dataset;
  std::vector<LineCameraT<double>> truth;
  for (int l = 0; l < d.num_lines(); ++l)
    truth.push_back(make_line_camera_d(d.head, sd.true_head_pose[size_t(d.line_sweep[size_t(l)])],
                                       sd.true_mirror_angle[size_t(l)], d.intrinsics));
  TrainOptions o;
  o.iterations = 200;
  o.batch_lines = 32;
  o.pose_from = 20;
  o.densify_from = 50;
  o.densify_every = 50;
  o.densify_until = 150;
  o.max_gaussians = 4000;
  o.learn_basis = true;
  const GaussianScene init = reduce_features(init_on_plane(d, 3e-3), 4);
  Trainer t(d, init, o);
  const PoseErrorPx e0 = line_pose_error_px(t.cameras(), truth);
  double first = 0, last = 0;
  for (int i = 0; i < o.iterations; ++i) {
    const TrainStep st = t.step();
    if (i < 20) first += st.loss / 20;
    if (i >= o.iterations - 20) last += st.loss / 20;
  }
  const PoseErrorPx e1 = line_pose_error_px(t.cameras(), truth);
  double moved = 0;
  for (size_t j = 0; j < init.basis.size(); ++j)
    moved = std::max(moved, std::fabs(double(t.scene().basis[j]) - init.basis[j]));
  std::printf("       loss %.4f -> %.4f, pose %.2f -> %.2f px, %d Gaussians, basis moved by up to %.3f\n", first, last,
              e0.rms, e1.rms, t.scene().size(), moved);
  CHECK(t.scene().num_features == 4);
  CHECK(last < 0.25 * first);
  CHECK(e1.rms < 0.8 * e0.rms);
  CHECK(moved > 1e-3);
}
