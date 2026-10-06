#include <algorithm>

#include "fixtures.hpp"
#include "linesplat/preview.hpp"
#include "linesplat/render_cpu.hpp"
#include "test.hpp"

using namespace linesplat;

namespace {

GaussianScene one_gaussian(Vec3d p, double scale, float opacity, std::vector<float> feat) {
  GaussianScene s;
  s.num_features = int(feat.size());
  const float mean[3] = {float(p.x), float(p.y), float(p.z)};
  const float sc[3] = {float(scale), float(scale), float(scale)};
  const float q[4] = {1, 0, 0, 0};
  s.add(mean, sc, q, opacity, feat.data());
  s.set_identity_basis(s.num_features);
  s.background.assign(feat.size(), 0.0f);
  return s;
}

}  // namespace

TEST(single_gaussian_draws_its_profile) {
  GaussianScene s = one_gaussian(Vec3d{0.001, 0.0, 0.15}, 5e-4, 0.8f, {1.0f, 0.5f});
  s.background = {0.2f, 0.1f};
  const auto cams = lsfix::sweep_cameras(1, 64, 600.0, 0.0);
  const auto img = render_lines_cpu<double>(s, cams);
  const auto hits = project_scene<double>(s, cams[0]);
  CHECK(hits.size() == 1);
  const LineSplatT<double>& h = hits[0].first;
  for (int p = 0; p < 64; ++p) {
    const double a = (p >= h.p0 && p < h.p1) ? splat_alpha_at(h.u, h.inv_var, h.alpha, p) : 0.0;
    const double used = a >= 1.0 / 255.0 ? a : 0.0;
    CHECK_NEAR(img.at(0, p, 0), used * 1.0 + (1 - used) * double(0.2f), 1e-12);
    CHECK_NEAR(img.at(0, p, 1), used * 0.5 + (1 - used) * double(0.1f), 1e-12);
    CHECK_NEAR(img.transmittance[size_t(p)], 1 - used, 1e-12);
  }
  // Centred where the pinhole puts it.
  CHECK_NEAR(h.u, 600.0 * 0.001 / 0.15 + 32.0, 1e-6);  // the scene stores floats
}

TEST(front_gaussian_hides_the_back_one) {
  GaussianScene s;
  s.num_features = 1;
  const float q[4] = {1, 0, 0, 0}, sc[3] = {0.002f, 0.002f, 0.002f};
  const float back[3] = {0, 0, 0.20f}, front[3] = {0, 0, 0.15f};
  const float fb = 0.0f, ff = 1.0f;
  // Add the back one first: the renderer must sort by depth, not by index.
  s.add(back, sc, q, 0.99f, &fb);
  s.add(front, sc, q, 0.99f, &ff);
  s.set_identity_basis(1);
  s.background = {0.5f};
  const auto cams = lsfix::sweep_cameras(1, 32, 400.0, 0.0);
  const auto img = render_lines_cpu<double>(s, cams);
  const auto hits = project_scene<double>(s, cams[0]);
  CHECK(hits.size() == 2);
  CHECK(hits[0].second == 1 && hits[1].second == 0);  // front (index 1) first
  const auto& f = hits[0].first;
  const auto& b = hits[1].first;
  const double af = splat_alpha_at(f.u, f.inv_var, f.alpha, 16), ab = splat_alpha_at(b.u, b.inv_var, b.alpha, 16);
  CHECK(af > 0.9);
  CHECK_NEAR(img.at(0, 16, 0), af * 1.0 + (1 - af) * ab * 0.0 + (1 - af) * (1 - ab) * double(0.5f), 1e-12);
}

TEST(empty_line_shows_the_background) {
  GaussianScene s = one_gaussian(Vec3d{0.0, 0.05, 0.15}, 1e-4, 0.9f, {1.0f});  // far off the slit
  s.background = {0.25f};
  const auto img = render_lines_cpu<float>(s, lsfix::cast_all<float>(lsfix::sweep_cameras(3, 40, 500.0, 0.01)));
  for (float v : img.values) CHECK_NEAR(v, 0.25, 1e-7);
  for (float t : img.transmittance) CHECK(t == 1.0f);
}

TEST(float_and_double_renders_agree) {
  const GaussianScene s = lsfix::random_scene(3000, 4, 31);
  const auto cams_d = lsfix::sweep_cameras(40, 100, 700.0, 0.12);
  const auto a = render_lines_cpu<double>(s, cams_d);
  const auto b = render_lines_cpu<float>(s, lsfix::cast_all<float>(cams_d));
  double worst = 0, sum = 0;
  for (size_t i = 0; i < a.values.size(); ++i) {
    const double d = std::fabs(a.values[i] - double(b.values[i]));
    worst = std::max(worst, d);
    sum += d;
  }
  // Rounding can move a splat across a cutoff, which changes a pixel by at
  // most a few 1/255 steps; on average they must agree closely.
  CHECK(worst < 0.05);
  CHECK(sum / a.values.size() < 1e-5);
}

TEST(pinhole_rows_match_a_direct_2d_render) {
  // A full image built from line cameras (one per row) against a direct 2D
  // rasterization of the same Gaussians: per pixel, evaluate every 2D splat
  // with its inverse covariance and composite front to back.
  const GaussianScene s = lsfix::random_scene(300, 3, 41, Vec3d{0, 0, 0.15}, 0.02, 5e-4, 4e-3);
  const int Wd = 48, Hd = 40;
  const double f = 220.0;
  Pose cam_in_world = look_at(Vec3d{0.01, -0.01, 0.0}, Vec3d{0, 0, 0.15}, Vec3d{0, -1, 0});
  const std::vector<LineCamera> rows_f = pinhole_rows(cam_in_world, Wd, Hd, f);
  std::vector<LineCameraT<double>> rows;
  for (const auto& r : rows_f) {
    LineIntrinsics in;
    in.width = Wd;
    in.f = f;
    in.cu = 0.5 * Wd;
    in.v_slit = r.v_slit;
    in.sigma_u = in.sigma_v = std::sqrt(1.0 / 12.0);
    in.near_z = r.near_z;
    rows.push_back(line_camera_from_pose(cam_in_world, in));
  }
  const auto img = render_lines_cpu<double>(s, rows);

  const Pose w2c = cam_in_world.inverse();
  const double s2 = 1.0 / 12.0;
  double worst = 0.0;
  for (int r = 0; r < Hd; ++r) {
    const double v = rows[size_t(r)].v_slit;
    for (int px = 0; px < Wd; ++px) {
      const double u = px + 0.5;
      struct Hit { double depth, alpha; int i; };
      std::vector<Hit> hits;
      for (int i = 0; i < s.size(); ++i) {
        const auto g = gaussian_geometry<double>(s, i);
        const Vec3d pc = w2c.apply(g.mean);
        if (pc.z <= rows[0].near_z) continue;
        const double lim = 1.3 * 0.5 * Wd / f, limy = lim + std::fabs(v) / f;
        const double tx = std::clamp(pc.x / pc.z, -lim, lim), ty = std::clamp(pc.y / pc.z, -limy, limy);
        // T = J W, Sigma2D = T Sigma T^T
        const Mat3d Wm = w2c.R;
        const Vec3d t0 = (f / pc.z) * (Wm.row(0) - tx * Wm.row(2));
        const Vec3d t1 = (f / pc.z) * (Wm.row(1) - ty * Wm.row(2));
        const double a = quad(g.cov, t0, t0), b = quad(g.cov, t0, t1), c = quad(g.cov, t1, t1);
        const double A = a + s2, C = c + s2, det = A * C - b * b;
        const double du = u - (f * pc.x / pc.z + 0.5 * Wd), dv = v - f * pc.y / pc.z;
        const double m = (C * du * du - 2 * b * du * dv + A * dv * dv) / det;
        const double k = std::sqrt(std::max(0.0, a * c - b * b) / det);
        const double alpha_line = g.opacity * k * std::exp(-0.5 * dv * dv / C);
        if (alpha_line < 1.0 / 255.0) continue;
        // Same 3-sigma cut along the row as the line renderer.
        const double var_u = det / C, ustar = f * pc.x / pc.z + 0.5 * Wd + b / C * dv;
        const double n2 = std::min(9.0, 2.0 * std::log(255.0 * alpha_line));
        if ((u - ustar) * (u - ustar) > var_u * n2) continue;
        const double al = std::min(0.99, g.opacity * k * std::exp(-0.5 * m));
        if (al < 1.0 / 255.0) continue;
        hits.push_back({pc.z, al, i});
      }
      std::stable_sort(hits.begin(), hits.end(), [](const Hit& x, const Hit& y) { return x.depth < y.depth; });
      double T = 1.0;
      double out[3] = {0, 0, 0};
      for (const Hit& h : hits) {
        const double next = T * (1 - h.alpha);
        if (next < 1e-4) break;
        for (int c = 0; c < 3; ++c) out[c] += s.features[size_t(h.i) * 3 + c] * h.alpha * T;
        T = next;
      }
      for (int c = 0; c < 3; ++c) {
        out[c] += T * s.background[size_t(c)];
        worst = std::max(worst, std::fabs(out[c] - img.at(r, px, c)));
      }
    }
  }
  CHECK(worst < 1e-9);
}
