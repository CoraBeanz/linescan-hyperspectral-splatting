// The line camera and the projection of one 3D Gaussian onto one scan line.
//
// A pushbroom spectrograph sees one line of the scene per exposure: the image
// of its slit. We model each exposure as a pinhole camera that has a single
// row of pixels. Its image plane has two coordinates, both in pixels:
//
//   u  along the slit:  u = f * x / z + cu,   pixel p covers u in [p, p + 1)
//   v  across the slit: v = f * y / z,        the slit sits at v = v_slit
//
// (x, y, z) are camera coordinates: x along the slit, y across it, z forward.
// Every scan line gets its own camera pose, because the scan mirror and the
// arm move the view between lines.
//
// To draw a Gaussian on a line we do what 3DGS does for a pixel image: project
// the 3D Gaussian to a 2D Gaussian on the image plane (the EWA / local affine
// approximation), widen it by the footprint of a pixel (the slit width and the
// optics blur across the slit, the pixel pitch and the optics blur along it),
// then read it off along the row v = v_slit. A 2D Gaussian cut along a line is
// a 1D Gaussian, so each (line, Gaussian) pair becomes a 1D splat with a
// centre, a width and a peak opacity. Rasterizing the line is then a 1D version
// of the 3DGS tile rasterizer.
//
// This header is shared by the CPU reference renderer and the CUDA kernels.
#pragma once

#include "linesplat/math.hpp"

namespace linesplat {

// Thresholds every renderer must agree on (the same values as 3DGS).
constexpr float kMinAlpha = 1.0f / 255.0f;     // skip contributions below this
constexpr float kMaxAlpha = 0.99f;             // clamp a single splat's alpha
constexpr float kMinTransmittance = 1e-4f;     // stop once the pixel is this opaque
constexpr float kJacobianClamp = 1.3f;         // limit on |x/z| relative to the half field of view

template <typename T>
struct LineCameraT {
  T R[9];       // world -> camera rotation, row-major: x_cam = R * x_world + t
  T t[3];       // world -> camera translation (m)
  T f;          // focal length (px)
  T cu;         // principal point along the slit (px)
  T v_slit;     // where the slit sits across the image plane (px from the principal point)
  T sigma_u;    // Gaussian blur along the slit (px): pixel pitch + optics
  T sigma_v;    // Gaussian blur across the slit (px): slit width + optics (+ scan motion)
  T near_z;     // near clip (m)
  int width;    // pixels along the slit
  int pad_;
};
using LineCamera = LineCameraT<float>;

template <typename T, typename S>
LS_HD LineCameraT<T> cast_camera(const LineCameraT<S>& c) {
  LineCameraT<T> o;
  for (int i = 0; i < 9; ++i) o.R[i] = T(c.R[i]);
  for (int i = 0; i < 3; ++i) o.t[i] = T(c.t[i]);
  o.f = T(c.f); o.cu = T(c.cu); o.v_slit = T(c.v_slit);
  o.sigma_u = T(c.sigma_u); o.sigma_v = T(c.sigma_v); o.near_z = T(c.near_z);
  o.width = c.width; o.pad_ = 0;
  return o;
}

// One Gaussian as seen by one scan line.
template <typename T>
struct LineSplatT {
  T u;         // centre along the slit (px)
  T inv_var;   // 1 / variance along the slit (1/px^2)
  T alpha;     // peak opacity on the line (before the per-pixel kMaxAlpha clamp)
  T depth;     // camera z of the Gaussian's centre (m); lines composite front to back by it
  int p0, p1;  // pixels [p0, p1) whose centres lie inside the cutoff radius
};

// The quantities a Gaussian needs for projection, computed once per render.
template <typename T>
struct GaussianGeomT {
  Vec3<T> mean;     // world position (m)
  Sym3<T> cov;      // world covariance (m^2)
  T opacity;        // in [0, 1]
  T max_scale;      // largest axis standard deviation (m), for a cheap cull
};

// Projects one Gaussian onto one scan line. Returns false when the Gaussian
// can't touch any pixel of the line.
//
// The steps, with J the Jacobian of the projection at the Gaussian's centre:
//   [p q; q r] = J R Sigma R^T J^T + diag(sigma_u^2, sigma_v^2)
//   along-slit variance  var_u = p - q^2 / r      (conditional variance)
//   along-slit centre    u     = mu_u + (q / r)(v_slit - mu_v)
//   peak opacity         alpha = opacity * k * exp(-(v_slit - mu_v)^2 / (2 r))
// k = sqrt(det before blur / det after blur) keeps the blurred Gaussian's total
// alpha the same as the sharp one's (as in Mip-Splatting), so a Gaussian much
// thinner than the slit fades instead of growing to the slit's width at full
// opacity.
template <typename T>
LS_HD bool project_to_line(const LineCameraT<T>& cam, const GaussianGeomT<T>& g, LineSplatT<T>* out) {
  if (!(g.opacity >= T(kMinAlpha))) return false;
  const T* R = cam.R;
  const T xc = R[0] * g.mean.x + R[1] * g.mean.y + R[2] * g.mean.z + cam.t[0];
  const T yc = R[3] * g.mean.x + R[4] * g.mean.y + R[5] * g.mean.z + cam.t[1];
  const T zc = R[6] * g.mean.x + R[7] * g.mean.y + R[8] * g.mean.z + cam.t[2];
  if (!(zc > cam.near_z)) return false;

  const T inv_z = T(1) / zc;
  const T tx = xc * inv_z;
  const T ty = yc * inv_z;
  const T mu_u = cam.f * tx + cam.cu;
  const T mu_v = cam.f * ty;
  const T dv = cam.v_slit - mu_v;

  // Like 3DGS, evaluate the Jacobian at a clamped direction so that Gaussians
  // far outside the field of view don't produce huge footprints.
  const T half_u = ls_max(cam.cu, T(cam.width) - cam.cu) / cam.f;
  const T lim_x = T(kJacobianClamp) * half_u;
  const T lim_y = lim_x + ls_abs(cam.v_slit) / cam.f;
  const T txc = ls_clamp(tx, -lim_x, lim_x);
  const T tyc = ls_clamp(ty, -lim_y, lim_y);

  // Rows of J R: the image-plane directions pulled back into world space.
  const T a = cam.f * inv_z;
  const Vec3<T> r0{R[0], R[1], R[2]}, r1{R[3], R[4], R[5]}, r2{R[6], R[7], R[8]};
  const Vec3<T> j0 = a * (r0 - txc * r2);
  const Vec3<T> j1 = a * (r1 - tyc * r2);

  // Cheap cull: most Gaussians sit far from a given line's plane of view.
  // The variance across the slit is at most |j1|^2 max_scale^2 + sigma_v^2,
  // and alpha >= 1/255 needs dv^2 / r <= 2 ln 255 = 11.08.
  const T sv2 = cam.sigma_v * cam.sigma_v;
  const T r_bound = dot(j1, j1) * g.max_scale * g.max_scale + sv2;
  if (dv * dv > T(11.09) * r_bound) return false;

  const T p_raw = quad(g.cov, j0, j0);
  const T q = quad(g.cov, j0, j1);
  const T r_raw = quad(g.cov, j1, j1);
  const T p = p_raw + cam.sigma_u * cam.sigma_u;
  const T r = r_raw + sv2;
  const T det = p * r - q * q;
  if (!(det > T(0))) return false;
  const T det_raw = ls_max(p_raw * r_raw - q * q, T(0));
  const T k = ls_sqrt(det_raw / det);

  const T alpha = g.opacity * k * ls_exp(T(-0.5) * dv * dv / r);
  if (!(alpha >= T(kMinAlpha))) return false;

  const T var_u = det / r;
  const T u = mu_u + (q / r) * dv;
  // Cut off at 3 sigma, or sooner where the splat drops below 1/255.
  const T n2 = ls_min(T(9), T(2) * ls_log(T(255) * alpha));
  const T rad = ls_sqrt(var_u * n2);
  const T lo = u - rad - T(0.5);
  const T hi = u + rad - T(0.5);
  const T last = T(cam.width - 1);
  if (!(hi >= T(0)) || !(lo <= last)) return false;
  const int p0 = lo <= T(0) ? 0 : int(ls_ceil(lo));
  const int p1 = hi >= last ? cam.width : int(ls_floor(hi)) + 1;
  if (p0 >= p1) return false;

  out->u = u;
  out->inv_var = r / det;
  out->alpha = alpha;
  out->depth = zc;
  out->p0 = p0;
  out->p1 = p1;
  return true;
}

// Alpha of a splat at pixel p (the caller checks p0 <= p < p1). The GPU
// rasterizer writes out the same expression term by term.
template <typename T>
LS_HD T splat_alpha_at(T u, T inv_var, T alpha, int p) {
  const T d = T(p) + T(0.5) - u;
  return ls_min(T(kMaxAlpha), alpha * ls_exp(T(-0.5) * inv_var * d * d));
}

}  // namespace linesplat
