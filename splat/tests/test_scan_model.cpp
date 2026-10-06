// The mirror model: the virtual camera must see exactly the reflected rays.
#include "fixtures.hpp"
#include "test.hpp"

using namespace linesplat;

TEST(virtual_camera_sees_the_reflected_rays) {
  const HeadModel head = cad_head_model();
  const LineIntrinsics in = intrinsics_from_optics(256);
  Rng rng(21);
  for (int t = 0; t < 200; ++t) {
    const double angle = rng.uniform(-0.2, 0.2);
    const double u = rng.uniform(0, in.width), v = rng.uniform(-5, 5);
    // The real camera's ray through image point (u, v), in the head frame.
    const Vec3d C = head.camera_in_head.t;
    const Vec3d d = normalized(head.camera_in_head.R * Vec3d{(u - in.cu) / in.f, v / in.f, 1.0});
    // Bounce it off the mirror.
    const Vec3d n = normalized(rotation_about(head.mirror.axis_dir, angle) * head.mirror.normal_at_zero);
    const Vec3d face = head.mirror.axis_point + head.mirror.face_offset * n;
    const Vec3d P = C + (dot(n, face - C) / dot(n, d)) * d;
    const Vec3d d2 = d - (2.0 * dot(n, d)) * n;
    // The virtual camera's ray through (u, -v) (its across-slit axis is flipped).
    double v_sign = 0;
    const Pose vc = virtual_camera_in_head(head, angle, &v_sign);
    CHECK(v_sign == -1.0);
    const Vec3d dv = normalized(vc.R * Vec3d{(u - in.cu) / in.f, v_sign * v / in.f, 1.0});
    CHECK(norm(dv - d2) < 1e-12);               // same direction
    CHECK(norm(cross(P - vc.t, dv)) < 1e-12);   // through the bounce point
    CHECK(dot(P - vc.t, dv) > 0.0);             // with the mirror in front of it
  }
}

TEST(view_turns_twice_the_mirror_angle) {
  const HeadModel head = cad_head_model();
  const Vec3d z0 = virtual_camera_in_head(head, 0.0).R.col(2);
  for (double a : {-0.1, -0.03, 0.02, 0.08}) {
    const Vec3d z = virtual_camera_in_head(head, a).R.col(2);
    CHECK_NEAR(std::acos(std::min(1.0, dot(z, z0))), 2.0 * std::fabs(a), 1e-9);
  }
}

TEST(cad_head_looks_out_of_its_window) {
  // At rest the view leaves along +Y, and the virtual camera sits 21 mm
  // behind the mirror centre (the mirror-to-objective distance). The mirror
  // centre is 7.1 mm out from the shaft at (y, z) = (-2.3, 24.1) mm.
  const Pose vc = virtual_camera_in_head(cad_head_model(), 0.0);
  const double e = 0.0071 * std::sqrt(0.5);
  CHECK(norm(vc.R.col(2) - Vec3d{0, 1, 0}) < 1e-9);
  CHECK(norm(vc.R.col(0) - Vec3d{1, 0, 0}) < 1e-9);
  CHECK(norm(vc.t - Vec3d{0.0, -0.0023 + e - 0.021, 0.0241 + e}) < 1e-9);
}

TEST(line_camera_inverts_its_pose) {
  const HeadModel head = cad_head_model();
  const LineIntrinsics in = intrinsics_from_optics(128);
  Pose head_in_world;
  head_in_world.R = rotation_exp(Vec3d{0.3, -1.2, 0.5});
  head_in_world.t = Vec3d{0.1, -0.05, 0.2};
  const auto cam = make_line_camera_d(head, head_in_world, 0.05, in);
  const Pose cw = head_in_world * virtual_camera_in_head(head, 0.05);
  // World point straight ahead of the virtual camera lands on the principal point.
  const Vec3d p = cw.apply(Vec3d{0, 0, 0.15});
  const double x = cam.R[0] * p.x + cam.R[1] * p.y + cam.R[2] * p.z + cam.t[0];
  const double y = cam.R[3] * p.x + cam.R[4] * p.y + cam.R[5] * p.z + cam.t[1];
  const double z = cam.R[6] * p.x + cam.R[7] * p.y + cam.R[8] * p.z + cam.t[2];
  CHECK_NEAR(x, 0.0, 1e-12);
  CHECK_NEAR(y, 0.0, 1e-12);
  CHECK_NEAR(z, 0.15, 1e-12);
}

TEST(intrinsics_follow_the_optics) {
  // 5 mm slit at 17.91 mm: 41.9 mm of scene at 150 mm (the Optiland model's number).
  const LineIntrinsics in = intrinsics_from_optics(256);
  const double line_mm = 2.0 * 150.0 * (0.5 * in.width / in.f);
  CHECK_NEAR(line_mm, 41.9, 0.1);
  // 50 um slit: 0.42 mm thick at 150 mm.
  CHECK_NEAR(150.0 * slit_width_px(256) / in.f, 0.42, 0.01);
}
