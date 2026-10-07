// The 1D line splat must be exactly the 2D EWA splat read off along the slit.
#include "fixtures.hpp"
#include "linesplat/render_cpu.hpp"
#include "test.hpp"

using namespace linesplat;

namespace {

// An independent 2D EWA projection with explicit matrices (3DGS's math).
struct Ewa2D {
  double mu_u, mu_v;
  double S[2][2];   // blurred covariance
  double det_raw;   // determinant before the blur
  double opacity;
};

Ewa2D ewa_2d(const LineCameraT<double>& cam, const GaussianGeomT<double>& g) {
  double W[3][3], p[3];
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) W[r][c] = cam.R[3 * r + c];
    p[r] = W[r][0] * g.mean.x + W[r][1] * g.mean.y + W[r][2] * g.mean.z + cam.t[r];
  }
  const double z = p[2];
  const double lim_x = 1.3 * std::max(cam.cu, cam.width - cam.cu) / cam.f;
  const double lim_y = lim_x + std::fabs(cam.v_slit) / cam.f;
  const double tx = std::min(std::max(p[0] / z, -lim_x), lim_x);
  const double ty = std::min(std::max(p[1] / z, -lim_y), lim_y);
  const double J[2][3] = {{cam.f / z, 0, -cam.f * tx / z}, {0, cam.f / z, -cam.f * ty / z}};
  const double Sg[3][3] = {{g.cov.xx, g.cov.xy, g.cov.xz}, {g.cov.xy, g.cov.yy, g.cov.yz}, {g.cov.xz, g.cov.yz, g.cov.zz}};
  double T[2][3] = {};
  for (int i = 0; i < 2; ++i)
    for (int j = 0; j < 3; ++j)
      for (int k = 0; k < 3; ++k) T[i][j] += J[i][k] * W[k][j];
  double S2[2][2] = {};
  for (int i = 0; i < 2; ++i)
    for (int j = 0; j < 2; ++j)
      for (int a = 0; a < 3; ++a)
        for (int b = 0; b < 3; ++b) S2[i][j] += T[i][a] * Sg[a][b] * T[j][b];
  Ewa2D e;
  e.mu_u = cam.f * p[0] / z + cam.cu;
  e.mu_v = cam.f * p[1] / z;
  e.det_raw = std::max(0.0, S2[0][0] * S2[1][1] - S2[0][1] * S2[1][0]);
  e.S[0][0] = S2[0][0] + cam.sigma_u * cam.sigma_u;
  e.S[1][1] = S2[1][1] + cam.sigma_v * cam.sigma_v;
  e.S[0][1] = e.S[1][0] = S2[0][1];
  e.opacity = g.opacity;
  return e;
}

// Alpha of the blurred 2D splat at image point (u, v), before clamping.
double ewa_alpha(const Ewa2D& e, double u, double v) {
  const double det = e.S[0][0] * e.S[1][1] - e.S[0][1] * e.S[1][0];
  const double du = u - e.mu_u, dv = v - e.mu_v;
  const double m = (e.S[1][1] * du * du - 2 * e.S[0][1] * du * dv + e.S[0][0] * dv * dv) / det;
  return e.opacity * std::sqrt(e.det_raw / det) * std::exp(-0.5 * m);
}

// Random cameras looking roughly at the fixture scene, with random slit offsets.
std::vector<LineCameraT<double>> random_cameras(int n, uint64_t seed) {
  Rng rng(seed);
  std::vector<LineCameraT<double>> cams;
  for (int i = 0; i < n; ++i) {
    LineIntrinsics in = lsfix::test_intrinsics(64 + int(rng.uniform() * 200), rng.uniform(300, 1200));
    in.cu = in.width * rng.uniform(0.3, 0.7);
    in.v_slit = rng.uniform(-20, 20);
    in.sigma_u = rng.uniform(0.2, 1.0);
    in.sigma_v = rng.uniform(0.2, 3.0);
    const Vec3d eye{rng.uniform(-0.05, 0.05), rng.uniform(-0.05, 0.05), rng.uniform(-0.05, 0.02)};
    const Vec3d target{rng.uniform(-0.01, 0.01), rng.uniform(-0.01, 0.01), 0.15};
    Pose p = look_at(eye, target, normalized(Vec3d{rng.normal(), rng.normal(), rng.normal()}));
    cams.push_back(line_camera_from_pose(p, in));
  }
  return cams;
}

}  // namespace

TEST(line_splat_is_the_2d_splat_on_the_slit) {
  const GaussianScene scene = lsfix::random_scene(400, 1, 11);
  const auto cams = random_cameras(30, 12);
  int visible = 0;
  for (const auto& cam : cams) {
    for (int i = 0; i < scene.size(); ++i) {
      const GaussianGeomT<double> g = gaussian_geometry<double>(scene, i);
      const Ewa2D e = ewa_2d(cam, g);
      LineSplatT<double> s{};  // {}: gcc 7 can't tell that s is only read when vis is true
      const bool vis = project_to_line(cam, g, &s);
      // Conditional width and centre of the 2D splat along v = v_slit.
      const double var_u = (e.S[0][0] * e.S[1][1] - e.S[0][1] * e.S[0][1]) / e.S[1][1];
      const double u_star = e.mu_u + e.S[0][1] / e.S[1][1] * (cam.v_slit - e.mu_v);
      for (int p = 0; p < cam.width; ++p) {
        const double a2d = std::min(0.99, ewa_alpha(e, p + 0.5, cam.v_slit));
        const double d = p + 0.5 - u_star;
        if (vis && p >= s.p0 && p < s.p1) {
          CHECK_NEAR(splat_alpha_at(s.u, s.inv_var, s.alpha, p), a2d, 1e-9 + 1e-9 * a2d);
        } else {
          // Left out only if it's faint or more than 3 sigma along the slit.
          CHECK(a2d < 1.0 / 255.0 + 1e-9 || d * d > 9.0 * var_u * (1 - 1e-9));
        }
      }
      visible += vis;
    }
  }
  CHECK(visible > 200);
}

TEST(point_lands_on_its_pinhole_pixel) {
  // A small Gaussian on the slit plane at camera coordinates (x, 0, z),
  // placed so it lands on the centre of pixel 120.
  LineIntrinsics in = lsfix::test_intrinsics(200, 800.0);
  const auto cam = line_camera_from_pose(Pose(), in);
  GaussianGeomT<double> g;
  g.mean = Vec3d{20.5 * 0.16 / 800.0, 0.0, 0.16};
  g.cov = Sym3<double>{1e-8, 0, 0, 1e-8, 0, 1e-8};  // 0.1 mm: half a pixel
  g.opacity = 0.9;
  g.max_scale = 1e-4;
  LineSplatT<double> s;
  CHECK(project_to_line(cam, g, &s));
  CHECK_NEAR(s.u, 120.5, 1e-9);
  CHECK_NEAR(s.depth, 0.16, 1e-12);
  CHECK(s.p0 <= 120 && s.p1 > 120);
  // A faint, tiny one between two pixel centres reaches neither: it's culled.
  g.mean = Vec3d{20.0 * 0.16 / 800.0, 0.0, 0.16};
  g.cov = Sym3<double>{1e-10, 0, 0, 1e-10, 0, 1e-10};
  g.opacity = 0.9;
  g.max_scale = 1e-5;
  CHECK(!project_to_line(cam, g, &s));
}

TEST(gaussian_off_the_slit_plane_is_culled) {
  LineIntrinsics in = lsfix::test_intrinsics(200, 800.0);
  const auto cam = line_camera_from_pose(Pose(), in);
  GaussianGeomT<double> g;
  g.cov = Sym3<double>{1e-6, 0, 0, 1e-6, 0, 1e-6};  // 1 mm sigma
  g.opacity = 0.9;
  g.max_scale = 1e-3;
  LineSplatT<double> s;
  g.mean = Vec3d{0.0, 0.002, 0.15};  // 2 sigma off the plane: still seen
  CHECK(project_to_line(cam, g, &s));
  g.mean = Vec3d{0.0, 0.006, 0.15};  // 6 sigma off: culled
  CHECK(!project_to_line(cam, g, &s));
  g.mean = Vec3d{0.0, 0.0, -0.15};   // behind the camera
  CHECK(!project_to_line(cam, g, &s));
}

TEST(blur_keeps_a_splats_total_alpha) {
  // Integrating a splat over every row v must give opacity * 2 pi sqrt(det)
  // of the unblurred 2D Gaussian, however wide the slit blur is. (Rows where
  // the splat is fainter than 1/255 are dropped, so only test splats bright
  // enough for that to cost under half a percent.)
  int tested = 0;
  for (double scale : {1e-4, 2e-4, 5e-4, 2e-3}) {
    for (double sigma_v : {0.3, 1.0, 2.5}) {
      LineIntrinsics in = lsfix::test_intrinsics(400, 900.0);
      in.sigma_v = sigma_v;
      GaussianGeomT<double> g;
      g.mean = Vec3d{0.001, 0.0005, 0.15};
      const Mat3d R = rotation_exp(Vec3d{0.4, 0.2, -0.3});
      g.cov = covariance_from_scale_rot(Vec3d{scale, 0.5 * scale, 2.0 * scale}, R);
      g.opacity = 0.9;
      g.max_scale = 2.0 * scale;
      const Ewa2D e = ewa_2d(line_camera_from_pose(Pose(), in), g);
      const double peak = g.opacity * std::sqrt(e.det_raw / (e.S[0][0] * e.S[1][1] - e.S[0][1] * e.S[0][1]));
      if (peak < 0.25) continue;
      ++tested;
      const double expected = g.opacity * 2.0 * kPi * std::sqrt(e.det_raw);
      const double sd = std::sqrt(e.S[1][1]);
      double total = 0.0;
      const int steps = 4000;
      for (int k = 0; k <= steps; ++k) {
        in.v_slit = e.mu_v + sd * (-8.0 + 16.0 * k / steps);
        const auto cam = line_camera_from_pose(Pose(), in);
        LineSplatT<double> s;
        if (!project_to_line(cam, g, &s)) continue;
        const double w = (k == 0 || k == steps) ? 0.5 : 1.0;
        total += w * s.alpha * std::sqrt(2.0 * kPi / s.inv_var) * (16.0 * sd / steps);
      }
      CHECK_NEAR(total / expected, 1.0, 0.006);
    }
  }
  CHECK(tested >= 6);
}
