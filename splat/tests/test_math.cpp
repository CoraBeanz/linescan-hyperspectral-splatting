#include "fixtures.hpp"
#include "test.hpp"

using namespace linesplat;

namespace {
double mat_diff(const Mat3d& a, const Mat3d& b) {
  double m = 0;
  for (int i = 0; i < 9; ++i) m = std::max(m, std::fabs(a.m[i] - b.m[i]));
  return m;
}
double det(const Mat3d& R) { return dot(R.col(0), cross(R.col(1), R.col(2))); }
}  // namespace

TEST(quaternion_gives_a_rotation) {
  Rng rng(3);
  for (int t = 0; t < 100; ++t) {
    const double q[4] = {rng.normal(), rng.normal(), rng.normal(), rng.normal()};
    const Mat3d R = quat_to_rotation(q);
    CHECK(mat_diff(R * transpose(R), mat3_identity<double>()) < 1e-12);
    CHECK_NEAR(det(R), 1.0, 1e-12);
    double back[4];
    rotation_to_quat(R, back);
    CHECK(mat_diff(quat_to_rotation(back), R) < 1e-12);
  }
}

TEST(quaternion_matches_axis_angle) {
  const Vec3d axis = normalized(Vec3d{1, 2, 3});
  const double a = 0.7;
  const double q[4] = {std::cos(a / 2), std::sin(a / 2) * axis.x, std::sin(a / 2) * axis.y, std::sin(a / 2) * axis.z};
  CHECK(mat_diff(quat_to_rotation(q), rotation_about(axis, a)) < 1e-12);
}

TEST(rotation_exp_log_round_trip) {
  Rng rng(5);
  for (int t = 0; t < 100; ++t) {
    const Vec3d w{rng.uniform(-2, 2), rng.uniform(-2, 2), rng.uniform(-2, 2)};
    if (norm(w) > 3.1) continue;
    const Vec3d back = rotation_log(rotation_exp(w));
    CHECK(norm(back - w) < 1e-9);
  }
  // Near and at 180 degrees.
  for (double th : {3.14159, 3.1415926, kPi}) {
    const Vec3d w = th * normalized(Vec3d{0.3, -0.5, 0.8});
    CHECK(mat_diff(rotation_exp(rotation_log(rotation_exp(w))), rotation_exp(w)) < 1e-9);
  }
}

TEST(pose_compose_and_invert) {
  Rng rng(7);
  for (int t = 0; t < 20; ++t) {
    Pose a, b;
    a.R = rotation_exp(Vec3d{rng.normal(), rng.normal(), rng.normal()});
    a.t = Vec3d{rng.normal(), rng.normal(), rng.normal()};
    b.R = rotation_exp(Vec3d{rng.normal(), rng.normal(), rng.normal()});
    b.t = Vec3d{rng.normal(), rng.normal(), rng.normal()};
    const Vec3d p{rng.normal(), rng.normal(), rng.normal()};
    CHECK(norm((a * b).apply(p) - a.apply(b.apply(p))) < 1e-12);
    CHECK(norm(a.inverse().apply(a.apply(p)) - p) < 1e-12);
    double v[7];
    a.to_array(v);
    const Pose c = Pose::from_array(v);
    CHECK(norm(c.apply(p) - a.apply(p)) < 1e-12);
  }
}

TEST(covariance_has_the_right_axes) {
  const Vec3d s{0.001, 0.002, 0.0005};
  const Mat3d R = rotation_exp(Vec3d{0.3, -0.2, 0.9});
  const Sym3<double> S = covariance_from_scale_rot(s, R);
  const double sv[3] = {s.x, s.y, s.z};
  for (int i = 0; i < 3; ++i) {
    const Vec3d a = R.col(i);
    // a^T S a = s_i^2 and S a is parallel to a.
    CHECK_NEAR(quad(S, a, a), sv[i] * sv[i], 1e-15);
    for (int j = 0; j < 3; ++j)
      if (j != i) CHECK_NEAR(quad(S, a, R.col(j)), 0.0, 1e-15);
  }
}
