// Pose refinement maths and the trainer.
#include <algorithm>
#include <cmath>
#include <cstdio>

#include "fixtures.hpp"
#include "linesplat/backward_cpu.hpp"
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
