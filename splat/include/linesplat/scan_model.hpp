// Where each scan line looks: the arm pose, the scan mirror and the optics.
//
// The head carries a fixed line camera (objective + slit) that looks into a
// scan mirror. Turning the mirror by an angle turns the view by twice that
// angle. The scene sees a reflected copy of the camera, the "virtual camera",
// and that is the camera we render through. For one scan line:
//
//   head_in_world   from the arm (forward kinematics or the AprilTag board),
//                   fixed for a whole sweep
//   mirror angle    from the stepper's step count, one value per line
//   head model      camera and mirror geometry in the head frame (the CAD)
//
// Splitting the pose this way matters for training: a line on its own says
// almost nothing about where it was across the slit, but one 6-DoF arm pose
// and one mirror offset per sweep are well constrained by all of its lines.
//
// Units: metres and radians. Poses map local coordinates into the parent
// frame: p_parent = R * p_local + t.
#pragma once

#include <string>

#include "linesplat/line_camera.hpp"
#include "linesplat/math.hpp"

namespace linesplat {

using Vec3d = Vec3<double>;
using Mat3d = Mat3<double>;

struct Pose {
  Mat3d R = mat3_identity<double>();
  Vec3d t{0.0, 0.0, 0.0};

  Vec3d apply(const Vec3d& p) const { return R * p + t; }
  Pose inverse() const;
  Pose operator*(const Pose& b) const;  // this after b: p -> this(b(p))

  // Layout used in every file: [tx, ty, tz, qw, qx, qy, qz].
  static Pose from_array(const double v[7]);
  void to_array(double v[7]) const;
};

// Rotation by `angle` (rad) about the unit vector `axis` (Rodrigues).
Mat3d rotation_about(const Vec3d& axis, double angle);
// Rotation of the rotation vector w (axis * angle).
Mat3d rotation_exp(const Vec3d& w);
// Rotation vector of R (inverse of rotation_exp).
Vec3d rotation_log(const Mat3d& R);
// Quaternion (w, x, y, z) <-> rotation matrix.
Mat3d quat_to_rotation(const double q_wxyz[4]);
void rotation_to_quat(const Mat3d& R, double q_wxyz[4]);
// p * [rotation_exp(phi), rho]: a small change expressed in p's own frame.
Pose perturb(const Pose& p, const Vec3d& rho, const Vec3d& phi);
// A camera -> world pose at `eye` looking at `target`, with the usual camera
// axes: x right, y down, z forward. `up` is the world's up direction.
Pose look_at(const Vec3d& eye, const Vec3d& target, const Vec3d& up);

// A plane mirror that turns about a fixed axis, in the head frame.
struct ScanMirror {
  Vec3d axis_point{0, 0, 0};      // a point on the rotation axis (m)
  Vec3d axis_dir{1, 0, 0};        // unit direction of the axis
  Vec3d normal_at_zero{0, 0, 1};  // unit normal of the reflecting face at angle 0
  double face_offset = 0.0;       // distance from the axis to the face, along the normal (m)
};

struct HeadModel {
  Pose camera_in_head;  // the real (unfolded) line camera: camera frame -> head frame
  ScanMirror mirror;
};

// The pinhole and blur parameters shared by every line (after calibration and
// binning along the slit). See LineCameraT for the coordinate conventions.
struct LineIntrinsics {
  int width = 256;       // pixels along the slit
  double f = 917.0;      // focal length (px)
  double cu = 128.0;     // principal point along the slit (px)
  double v_slit = 0.0;   // slit offset across the slit in the real camera (px)
  double sigma_u = 0.5;  // blur along the slit (px)
  double sigma_v = 0.85; // blur across the slit (px)
  double near_z = 0.01;  // near clip (m)
};

// The spectrograph numbers the intrinsics follow from (optics model, layout C).
struct SlitOptics {
  double slit_length_mm = 5.0;     // slit length
  double slit_width_mm = 0.05;     // slit width
  double slit_distance_mm = 17.91; // objective to slit when focused at 150 mm
  double blur_mm = 0.008;          // optics blur at the slit, 1 sigma (a guess until measured)
};

// Intrinsics for `width` pixels spread evenly along the slit.
LineIntrinsics intrinsics_from_optics(int width, const SlitOptics& o = SlitOptics());
// Slit width in pixels and the optics blur in pixels for those intrinsics.
double slit_width_px(int width, const SlitOptics& o = SlitOptics());
double optics_blur_px(int width, const SlitOptics& o = SlitOptics());

// The head as designed in cad/ (the head layout rows of cad/rig/params.py):
// the scan window faces +Y, X runs along the slit and the mirror shaft, and
// the objective looks back along -Z into a 45 degree mirror. Mirror angles use
// the convention of the ROS2 ScanLine message: radians, 0 at the 45 degree
// rest, right-handed about the head's +X (shaft) axis.
HeadModel cad_head_model();

// The virtual camera (camera frame -> head frame) at a mirror angle. A
// reflection flips handedness, so the returned frame has its y axis negated
// to stay a proper rotation; *v_sign (if given) is -1 to say the across-slit
// axis was flipped, and the slit offset must flip with it.
Pose virtual_camera_in_head(const HeadModel& head, double mirror_angle, double* v_sign = nullptr);

// The camera to render scan line `mirror_angle` of a sweep from `head_in_world`.
LineCamera make_line_camera(const HeadModel& head, const Pose& head_in_world, double mirror_angle,
                            const LineIntrinsics& in);
LineCameraT<double> make_line_camera_d(const HeadModel& head, const Pose& head_in_world,
                                       double mirror_angle, const LineIntrinsics& in);

// A line camera from a camera -> world pose with no mirror in between.
LineCameraT<double> line_camera_from_pose(const Pose& camera_in_world, const LineIntrinsics& in);

}  // namespace linesplat
