#include "linesplat/scan_model.hpp"

#include <algorithm>
#include <cmath>

namespace linesplat {

Pose Pose::inverse() const {
  Pose o;
  o.R = transpose(R);
  o.t = -(o.R * t);
  return o;
}

Pose Pose::operator*(const Pose& b) const {
  Pose o;
  o.R = R * b.R;
  o.t = R * b.t + t;
  return o;
}

Pose Pose::from_array(const double v[7]) {
  Pose p;
  p.t = Vec3d{v[0], v[1], v[2]};
  p.R = quat_to_rotation(v + 3);
  return p;
}

void Pose::to_array(double v[7]) const {
  v[0] = t.x;
  v[1] = t.y;
  v[2] = t.z;
  rotation_to_quat(R, v + 3);
}

Mat3d rotation_about(const Vec3d& axis, double angle) {
  const Vec3d k = normalized(axis);
  const double c = std::cos(angle), s = std::sin(angle), C = 1.0 - c;
  return Mat3d{{c + k.x * k.x * C, k.x * k.y * C - k.z * s, k.x * k.z * C + k.y * s,
                k.y * k.x * C + k.z * s, c + k.y * k.y * C, k.y * k.z * C - k.x * s,
                k.z * k.x * C - k.y * s, k.z * k.y * C + k.x * s, c + k.z * k.z * C}};
}

Mat3d rotation_exp(const Vec3d& w) {
  const double th = norm(w);
  if (th < 1e-12) {
    return Mat3d{{1.0, -w.z, w.y, w.z, 1.0, -w.x, -w.y, w.x, 1.0}};
  }
  return rotation_about((1.0 / th) * w, th);
}

Vec3d rotation_log(const Mat3d& R) {
  // sin and cos of the angle both come straight from R; recovering the angle
  // with acos and then taking its sine would lose precision near 180 degrees.
  const Vec3d v{R(2, 1) - R(1, 2), R(0, 2) - R(2, 0), R(1, 0) - R(0, 1)};  // 2 sin(th) * axis
  const double s = 0.5 * norm(v);
  const double c = 0.5 * (R(0, 0) + R(1, 1) + R(2, 2) - 1.0);
  const double th = std::atan2(s, c);
  if (th < 1e-6) return 0.5 * v;
  if (s < 1e-6) {
    // Within a hair of 180 degrees: take the axis from the largest diagonal
    // entry of R = 2 a a^T - I, and its sign from v.
    int i = 0;
    if (R(1, 1) > R(i, i)) i = 1;
    if (R(2, 2) > R(i, i)) i = 2;
    const int j = (i + 1) % 3, k = (i + 2) % 3;
    double a[3];
    a[i] = std::sqrt(std::max(0.0, 0.5 * (R(i, i) + 1.0)));
    // Off the diagonal use the symmetric part, where the small sin term cancels.
    a[j] = (R(j, i) + R(i, j)) / (4.0 * a[i]);
    a[k] = (R(k, i) + R(i, k)) / (4.0 * a[i]);
    Vec3d axis = normalized(Vec3d{a[0], a[1], a[2]});
    if (dot(axis, v) < 0.0) axis = -axis;
    return th * axis;
  }
  return (th / (2.0 * s)) * v;
}

Mat3d quat_to_rotation(const double q[4]) { return quat_to_mat(q[0], q[1], q[2], q[3]); }

void rotation_to_quat(const Mat3d& R, double q[4]) {
  // Shepperd's method: pivot on the largest of w, x, y, z.
  const double tr = R(0, 0) + R(1, 1) + R(2, 2);
  double w, x, y, z;
  if (tr > R(0, 0) && tr > R(1, 1) && tr > R(2, 2)) {
    const double s = 2.0 * std::sqrt(1.0 + tr);
    w = 0.25 * s;
    x = (R(2, 1) - R(1, 2)) / s;
    y = (R(0, 2) - R(2, 0)) / s;
    z = (R(1, 0) - R(0, 1)) / s;
  } else if (R(0, 0) > R(1, 1) && R(0, 0) > R(2, 2)) {
    const double s = 2.0 * std::sqrt(1.0 + R(0, 0) - R(1, 1) - R(2, 2));
    w = (R(2, 1) - R(1, 2)) / s;
    x = 0.25 * s;
    y = (R(0, 1) + R(1, 0)) / s;
    z = (R(0, 2) + R(2, 0)) / s;
  } else if (R(1, 1) > R(2, 2)) {
    const double s = 2.0 * std::sqrt(1.0 + R(1, 1) - R(0, 0) - R(2, 2));
    w = (R(0, 2) - R(2, 0)) / s;
    x = (R(0, 1) + R(1, 0)) / s;
    y = 0.25 * s;
    z = (R(1, 2) + R(2, 1)) / s;
  } else {
    const double s = 2.0 * std::sqrt(1.0 + R(2, 2) - R(0, 0) - R(1, 1));
    w = (R(1, 0) - R(0, 1)) / s;
    x = (R(0, 2) + R(2, 0)) / s;
    y = (R(1, 2) + R(2, 1)) / s;
    z = 0.25 * s;
  }
  const double sign = w < 0.0 ? -1.0 : 1.0;  // keep w >= 0
  const double n = sign / std::sqrt(w * w + x * x + y * y + z * z);
  q[0] = w * n;
  q[1] = x * n;
  q[2] = y * n;
  q[3] = z * n;
}

Pose perturb(const Pose& p, const Vec3d& rho, const Vec3d& phi) {
  Pose d;
  d.R = rotation_exp(phi);
  d.t = rho;
  return p * d;
}

Pose look_at(const Vec3d& eye, const Vec3d& target, const Vec3d& up) {
  const Vec3d z = normalized(target - eye);
  const Vec3d x = normalized(cross(z, up));
  Pose p;
  p.R = mat3_from_cols(x, cross(z, x), z);
  p.t = eye;
  return p;
}

double slit_width_px(int width, const SlitOptics& o) {
  return o.slit_width_mm / (o.slit_length_mm / width);
}

double optics_blur_px(int width, const SlitOptics& o) { return o.blur_mm / (o.slit_length_mm / width); }

LineIntrinsics intrinsics_from_optics(int width, const SlitOptics& o) {
  // Pixels are spread evenly along the slit's image, so one pixel covers
  // slit_length / width of the slit, and the pinhole's focal length is the
  // objective-to-slit distance in those pixels.
  const double pitch_mm = o.slit_length_mm / width;
  const double blur = optics_blur_px(width, o);
  const double slit = slit_width_px(width, o);
  LineIntrinsics in;
  in.width = width;
  in.f = o.slit_distance_mm / pitch_mm;
  in.cu = 0.5 * width;
  in.v_slit = 0.0;
  // A box of width w has the variance of a Gaussian with sigma = w / sqrt(12).
  in.sigma_u = std::sqrt(blur * blur + 1.0 / 12.0);
  in.sigma_v = std::sqrt(blur * blur + slit * slit / 12.0);
  in.near_z = 0.01;
  return in;
}

HeadModel cad_head_model() {
  // The head layout rows of cad/rig/params.py, with the Edmund 20 x 20 x 3 mm
  // mirror that PR #5 fits: the shaft at (y_shaft, z_shaft), the mirror face
  // mirror_e = 2.5 + 1.6 + mirror_t in front of it at 45 degrees, the optical
  // axis where that face is, and the objective mirror_to_obj above the mirror.
  const double mm = 1e-3, s = std::sqrt(0.5);
  const double y_shaft = -2.3 * mm, z_shaft = 24.1 * mm;
  const double mirror_e = (2.5 + 1.6 + 3.0) * mm, mirror_to_obj = 21.0 * mm;
  const double y_axis = y_shaft + mirror_e * s, z_mirror = z_shaft + mirror_e * s;
  HeadModel h;
  h.camera_in_head.R = mat3_from_cols(Vec3d{1, 0, 0}, Vec3d{0, -1, 0}, Vec3d{0, 0, -1});
  h.camera_in_head.t = Vec3d{0.0, y_axis, z_mirror + mirror_to_obj};
  h.mirror.axis_point = Vec3d{0.0, y_shaft, z_shaft};
  h.mirror.axis_dir = Vec3d{1.0, 0.0, 0.0};
  h.mirror.normal_at_zero = Vec3d{0.0, s, s};
  h.mirror.face_offset = mirror_e;
  return h;
}

Pose virtual_camera_in_head(const HeadModel& head, double mirror_angle, double* v_sign) {
  const ScanMirror& m = head.mirror;
  const Vec3d n = normalized(rotation_about(m.axis_dir, mirror_angle) * m.normal_at_zero);
  const Vec3d face = m.axis_point + m.face_offset * n;
  auto reflect_point = [&](const Vec3d& x) { return x - (2.0 * dot(n, x - face)) * n; };
  auto reflect_dir = [&](const Vec3d& d) { return d - (2.0 * dot(n, d)) * n; };

  const Pose& cam = head.camera_in_head;
  const Vec3d x = reflect_dir(cam.R.col(0));
  const Vec3d z = reflect_dir(cam.R.col(2));
  Pose v;
  v.R = mat3_from_cols(x, cross(z, x), z);  // y' = -reflect(y): proper rotation
  v.t = reflect_point(cam.t);
  if (v_sign) *v_sign = -1.0;
  return v;
}

LineCameraT<double> line_camera_from_pose(const Pose& camera_in_world, const LineIntrinsics& in) {
  const Pose w2c = camera_in_world.inverse();
  LineCameraT<double> c;
  for (int i = 0; i < 9; ++i) c.R[i] = w2c.R.m[i];
  c.t[0] = w2c.t.x;
  c.t[1] = w2c.t.y;
  c.t[2] = w2c.t.z;
  c.f = in.f;
  c.cu = in.cu;
  c.v_slit = in.v_slit;
  c.sigma_u = in.sigma_u;
  c.sigma_v = in.sigma_v;
  c.near_z = in.near_z;
  c.width = in.width;
  c.pad_ = 0;
  return c;
}

LineCameraT<double> make_line_camera_d(const HeadModel& head, const Pose& head_in_world, double mirror_angle,
                                       const LineIntrinsics& in) {
  double v_sign = 1.0;
  const Pose cam_in_world = head_in_world * virtual_camera_in_head(head, mirror_angle, &v_sign);
  LineCameraT<double> c = line_camera_from_pose(cam_in_world, in);
  c.v_slit = v_sign * in.v_slit;
  return c;
}

LineCamera make_line_camera(const HeadModel& head, const Pose& head_in_world, double mirror_angle,
                            const LineIntrinsics& in) {
  return cast_camera<float>(make_line_camera_d(head, head_in_world, mirror_angle, in));
}

}  // namespace linesplat
