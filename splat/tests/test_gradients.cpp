// The backward pass against central finite differences, in double precision.
#include <algorithm>
#include <cstdio>
#include <functional>

#include "fixtures.hpp"
#include "linesplat/backward_cpu.hpp"
#include "linesplat/gradients.hpp"
#include "test.hpp"

using namespace linesplat;

namespace {

// Agreement of analytic and numeric gradients over one group of parameters.
// The renderer has hard cutoffs (alpha below 1/255 is dropped, splats end at
// 3 sigma), so a numeric derivative that straddles one is wrong; allow a few.
struct Agreement {
  int n = 0, bad = 0;
  double worst = 0.0;
  void add(double ana, double num, double scale, double tol = 1e-5) {
    const double err = std::fabs(ana - num) / (std::fabs(num) + scale);
    ++n;
    if (err > tol) ++bad;
    worst = std::max(worst, err);
  }
  void report(const char* what) const {
    std::printf("       %-16s %4d checked, %d off, worst %.1e\n", what, n, bad, worst);
  }
};

// Line cameras 15 cm from the origin looking at it, with random intrinsics
// (and random widths unless `width` is given).
std::vector<LineCameraT<double>> cameras_around_origin(int n, uint64_t seed, int width = 0) {
  Rng rng(seed);
  std::vector<LineCameraT<double>> cams;
  for (int i = 0; i < n; ++i) {
    const int w = 24 + int(rng.uniform() * 24);
    LineIntrinsics in = lsfix::test_intrinsics(width > 0 ? width : w, rng.uniform(300, 600));
    in.cu = in.width * rng.uniform(0.4, 0.6);
    in.v_slit = rng.uniform(-3, 3);
    in.sigma_u = rng.uniform(0.3, 0.8);
    in.sigma_v = rng.uniform(0.5, 2.0);
    const Vec3d eye{rng.uniform(-0.03, 0.03), rng.uniform(-0.03, 0.03), -0.15};
    const Vec3d target{rng.uniform(-0.004, 0.004), rng.uniform(-0.004, 0.004), 0.0};
    cams.push_back(line_camera_from_pose(look_at(eye, target, Vec3d{0, 1, 0}), in));
  }
  return cams;
}

}  // namespace

TEST(projection_backward_matches_finite_differences) {
  const GaussianScene scene = lsfix::random_scene(300, 1, 61, Vec3d{0, 0, 0}, 0.01, 2e-4, 2e-3);
  const auto cams = cameras_around_origin(12, 62);
  Rng rng(63);
  Agreement am, ac, ao, aR, at;
  for (const auto& cam : cams)
    for (int i = 0; i < scene.size(); ++i) {
      const GaussianGeomT<double> g = gaussian_geometry<double>(scene, i);
      LineSplatT<double> s;
      if (!project_to_line(cam, g, &s)) continue;
      const double wu = rng.uniform(-1, 1), wi = rng.uniform(-1, 1) / s.inv_var, wa = rng.uniform(-1, 1);
      ProjectionGradT<double> pg;
      project_to_line_backward(cam, g, wu, wi, wa, &pg);
      bool ok = true;
      auto F = [&](const LineCameraT<double>& c, const GaussianGeomT<double>& x) {
        LineSplatT<double> o{};
        ok = ok && project_to_line(c, x, &o);
        return wu * o.u + wi * o.inv_var + wa * o.alpha;
      };
      auto num = [&](double* v, double h, const LineCameraT<double>& c, GaussianGeomT<double>& x) {
        const double v0 = *v;
        *v = v0 + h;
        const double fp = F(c, x);
        *v = v0 - h;
        const double fm = F(c, x);
        *v = v0;
        return (fp - fm) / (2 * h);
      };
      GaussianGeomT<double> x = g;
      LineCameraT<double> c = cam;
      const double gm[3] = {pg.mean.x, pg.mean.y, pg.mean.z};
      double* pm[3] = {&x.mean.x, &x.mean.y, &x.mean.z};
      double nm[3];
      for (int k = 0; k < 3; ++k) nm[k] = num(pm[k], 1e-9, c, x);
      const double gc[6] = {pg.cov.xx, pg.cov.xy, pg.cov.xz, pg.cov.yy, pg.cov.yz, pg.cov.zz};
      double* pc[6] = {&x.cov.xx, &x.cov.xy, &x.cov.xz, &x.cov.yy, &x.cov.yz, &x.cov.zz};
      const double hc = 1e-6 * (g.cov.xx + g.cov.yy + g.cov.zz);
      double nc[6];
      for (int k = 0; k < 6; ++k) nc[k] = num(pc[k], hc, c, x);
      const double no = num(&x.opacity, 1e-7, c, x);
      double nR[9], nt[3];
      for (int k = 0; k < 9; ++k) nR[k] = num(&c.R[k], 1e-7, c, x);
      for (int k = 0; k < 3; ++k) nt[k] = num(&c.t[k], 1e-8, c, x);
      if (!ok) continue;  // visibility changed inside the step
      double sm = 0, sc = 0, sR = 0, st = 0;
      for (int k = 0; k < 3; ++k) sm = std::max(sm, std::fabs(nm[k]));
      for (int k = 0; k < 6; ++k) sc = std::max(sc, std::fabs(nc[k]));
      for (int k = 0; k < 9; ++k) sR = std::max(sR, std::fabs(nR[k]));
      for (int k = 0; k < 3; ++k) st = std::max(st, std::fabs(nt[k]));
      for (int k = 0; k < 3; ++k) am.add(gm[k], nm[k], 1e-4 * sm);
      for (int k = 0; k < 6; ++k) ac.add(gc[k], nc[k], 1e-4 * sc);
      ao.add(pg.opacity, no, 1e-6);
      for (int k = 0; k < 9; ++k) aR.add(pg.R[k], nR[k], 1e-4 * sR);
      for (int k = 0; k < 3; ++k) at.add(pg.t[k], nt[k], 1e-4 * st);
    }
  am.report("mean");
  ac.report("covariance");
  ao.report("opacity");
  aR.report("camera R");
  at.report("camera t");
  CHECK(am.n > 300);
  for (const Agreement* a : {&am, &ac, &ao, &aR, &at}) CHECK(a->bad <= a->n / 200);
}

TEST(geometry_backward_matches_finite_differences) {
  GaussianScene s = lsfix::random_scene(200, 1, 71, Vec3d{0, 0, 0}, 0.01, 2e-4, 2e-3);
  Rng rng(72);
  Agreement als, aq, al;
  for (int i = 0; i < s.size(); ++i) {
    double w[7];
    for (double& v : w) v = rng.uniform(-1, 1);
    auto F = [&]() {
      const GaussianGeomT<double> g = gaussian_geometry<double>(s, i);
      return (w[0] * g.cov.xx + w[1] * g.cov.xy + w[2] * g.cov.xz + w[3] * g.cov.yy + w[4] * g.cov.yz +
              w[5] * g.cov.zz) * 1e6 + w[6] * g.opacity;
    };
    double gls[3], gq[4], glog;
    const Sym3<double> gcov{w[0] * 1e6, w[1] * 1e6, w[2] * 1e6, w[3] * 1e6, w[4] * 1e6, w[5] * 1e6};
    gaussian_geometry_backward(&s.log_scales[3 * size_t(i)], &s.rotations[4 * size_t(i)], s.opacity_logits[size_t(i)],
                               gcov, w[6], gls, gq, &glog);
    // The parameters are floats: use the step they actually took.
    auto num = [&](float* v, float h) {
      const float v0 = *v;
      *v = v0 + h;
      const double xp = *v, fp = F();
      *v = v0 - h;
      const double xm = *v, fm = F();
      *v = v0;
      return (fp - fm) / (xp - xm);
    };
    for (int k = 0; k < 3; ++k) als.add(gls[k], num(&s.log_scales[3 * size_t(i) + k], 1e-3f), 1e-3);
    for (int k = 0; k < 4; ++k) aq.add(gq[k], num(&s.rotations[4 * size_t(i) + k], 1e-3f), 1e-3, 1e-4);
    al.add(glog, num(&s.opacity_logits[size_t(i)], 1e-3f), 1e-4);
  }
  als.report("log scales");
  aq.report("quaternion");
  al.report("opacity logit");
  CHECK(als.bad == 0 && aq.bad == 0 && al.bad == 0);
}

TEST(render_backward_matches_finite_differences) {
  // A few Gaussians in front of six line cameras, 3 features through a
  // random 4-band basis, and the loss sum(w * bands) with random w.
  GaussianScene s = lsfix::random_scene(24, 3, 81, Vec3d{0, 0, 0}, 0.006, 4e-4, 2e-3);
  Rng rng(82);
  for (float& o : s.opacity_logits) o = float(rng.uniform(-1.5, 1.0));
  s.basis.resize(4 * 3);
  for (float& b : s.basis) b = float(rng.uniform(-1, 1));
  std::vector<LineCameraT<double>> cams = cameras_around_origin(6, 83, 32);
  const int W = 32, B = 4;
  std::vector<double> w(cams.size() * W * B);
  for (double& v : w) v = rng.uniform(-1, 1);

  auto F = [&](const GaussianScene& sc, const std::vector<LineCameraT<double>>& cs) {
    const LineImageT<double> img = features_to_bands(sc, render_lines_cpu<double>(sc, cs));
    double sum = 0;
    for (size_t i = 0; i < w.size(); ++i) sum += w[i] * img.values[i];
    return sum;
  };
  SceneGradT<double> g;
  std::vector<CameraGradT<double>> cg;
  std::vector<double> bands;
  const double L = render_backward_cpu<double>(
      s, cams,
      [&](int l, const double* b, double* gb) {
        double sum = 0;
        for (int i = 0; i < W * B; ++i) {
          gb[i] = w[size_t(l) * W * B + i];
          sum += gb[i] * b[i];
        }
        return sum;
      },
      &g, &cg, &bands);
  CHECK_NEAR(L, F(s, cams), 1e-9);
  int drawn = 0, pairs = 0;
  for (int p : g.pairs) {
    drawn += p > 0;
    pairs += p;
  }
  std::printf("       %d of %d Gaussians drawn, in %d (line, Gaussian) pairs\n", drawn, s.size(), pairs);
  CHECK(drawn >= 10);

  auto num = [&](float* v, float h) {
    const float v0 = *v;
    *v = v0 + h;
    const double xp = *v, fp = F(s, cams);
    *v = v0 - h;
    const double xm = *v, fm = F(s, cams);
    *v = v0;
    return (fp - fm) / (xp - xm);
  };
  struct Group {
    const char* name;
    std::vector<float>* params;
    std::vector<double>* grads;
    float h;
  };
  std::vector<Group> groups = {{"means", &s.means, &g.means, 2e-8f},
                               {"log scales", &s.log_scales, &g.log_scales, 1e-5f},
                               {"rotations", &s.rotations, &g.rotations, 1e-5f},
                               {"opacity logits", &s.opacity_logits, &g.opacity_logits, 1e-5f},
                               {"features", &s.features, &g.features, 1e-4f},
                               {"background", &s.background, &g.background, 1e-4f}};
  for (const Group& gr : groups) {
    std::vector<double> n(gr.params->size());
    double scale = 0;
    for (size_t i = 0; i < n.size(); ++i) {
      n[i] = num(&(*gr.params)[i], gr.h);
      scale = std::max(scale, std::fabs(n[i]));
    }
    Agreement a;
    for (size_t i = 0; i < n.size(); ++i) a.add((*gr.grads)[i], n[i], 1e-5 * scale, 1e-4);
    a.report(gr.name);
    CHECK(a.bad <= std::max(1, a.n / 50));
  }

  // The cameras, through dL/dR and dL/dt.
  Agreement aR, at;
  for (size_t c = 0; c < cams.size(); ++c) {
    auto numc = [&](double* v, double h) {
      const double v0 = *v;
      *v = v0 + h;
      const double fp = F(s, cams);
      *v = v0 - h;
      const double fm = F(s, cams);
      *v = v0;
      return (fp - fm) / (2 * h);
    };
    double nR[9], nt[3], sR = 0, st = 0;
    for (int k = 0; k < 9; ++k) sR = std::max(sR, std::fabs(nR[k] = numc(&cams[c].R[k], 1e-7)));
    for (int k = 0; k < 3; ++k) st = std::max(st, std::fabs(nt[k] = numc(&cams[c].t[k], 1e-8)));
    for (int k = 0; k < 9; ++k) aR.add(cg[c].R[k], nR[k], 1e-5 * sR + 1e-9, 1e-4);
    for (int k = 0; k < 3; ++k) at.add(cg[c].t[k], nt[k], 1e-5 * st + 1e-9, 1e-4);
  }
  aR.report("camera R");
  at.report("camera t");
  CHECK(aR.bad <= 1 && at.bad <= 1);
}
