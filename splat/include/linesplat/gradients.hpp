// Backward passes of the projection (line_camera.hpp) and of the Gaussian
// parameterization (scene.hpp): how a change in a 1D line splat flows back to
// the Gaussian's mean, covariance and opacity, to the camera pose, and from
// the covariance and opacity to the stored log scales, quaternion and logit.
//
// Each function recomputes the forward intermediates it needs instead of
// storing them, so the GPU kernels can call the same code. Like the forward
// code, keep this header C++14.
#pragma once

#include "linesplat/line_camera.hpp"

namespace linesplat {

// Gradients from one (line, Gaussian) pair.
template <typename T>
struct ProjectionGradT {
  Vec3<T> mean;   // dL/d(world mean)
  Sym3<T> cov;    // dL/d(each stored covariance entry; xy etc. counted once)
  T opacity;      // dL/d(opacity)
  T R[9];         // dL/d(world -> camera rotation), row-major
  T t[3];         // dL/d(world -> camera translation)
  T mu_u, mu_v;   // dL/d(projected centre) in px, for densification
};

// Gradient of a quadratic form a^T S b with respect to S's stored entries.
template <typename T>
LS_HD Sym3<T> quad_grad(const Vec3<T>& a, const Vec3<T>& b) {
  return Sym3<T>{a.x * b.x, a.x * b.y + a.y * b.x, a.x * b.z + a.z * b.x,
                 a.y * b.y, a.y * b.z + a.z * b.y, a.z * b.z};
}

template <typename T>
LS_HD Sym3<T> sym_axpy(T s, const Sym3<T>& x, const Sym3<T>& y) {
  return Sym3<T>{y.xx + s * x.xx, y.xy + s * x.xy, y.xz + s * x.xz,
                 y.yy + s * x.yy, y.yz + s * x.yz, y.zz + s * x.zz};
}

// S v for a symmetric S.
template <typename T>
LS_HD Vec3<T> sym_mul(const Sym3<T>& S, const Vec3<T>& v) {
  return Vec3<T>{S.xx * v.x + S.xy * v.y + S.xz * v.z,
                 S.xy * v.x + S.yy * v.y + S.yz * v.z,
                 S.xz * v.x + S.yz * v.y + S.zz * v.z};
}

// Backward of project_to_line for a pair it returned true for. g_u, g_inv_var
// and g_alpha are dL/d(the splat's u, inv_var, alpha).
template <typename T>
LS_HD void project_to_line_backward(const LineCameraT<T>& cam, const GaussianGeomT<T>& g, T g_u, T g_inv_var,
                                    T g_alpha, ProjectionGradT<T>* out) {
  // Forward, as in project_to_line.
  const T* R = cam.R;
  const Vec3<T> r0{R[0], R[1], R[2]}, r1{R[3], R[4], R[5]}, r2{R[6], R[7], R[8]};
  const T xc = dot(r0, g.mean) + cam.t[0];
  const T yc = dot(r1, g.mean) + cam.t[1];
  const T zc = dot(r2, g.mean) + cam.t[2];
  const T inv_z = T(1) / zc;
  const T tx = xc * inv_z;
  const T ty = yc * inv_z;
  const T mu_v = cam.f * ty;
  const T dv = cam.v_slit - mu_v;
  const T half_u = ls_max(cam.cu, T(cam.width) - cam.cu) / cam.f;
  const T lim_x = T(kJacobianClamp) * half_u;
  const T lim_y = lim_x + ls_abs(cam.v_slit) / cam.f;
  const bool free_x = tx > -lim_x && tx < lim_x;
  const bool free_y = ty > -lim_y && ty < lim_y;
  const T txc = ls_clamp(tx, -lim_x, lim_x);
  const T tyc = ls_clamp(ty, -lim_y, lim_y);
  const T a = cam.f * inv_z;
  const Vec3<T> e0 = r0 - txc * r2, e1 = r1 - tyc * r2;
  const Vec3<T> j0 = a * e0, j1 = a * e1;
  const Vec3<T> Sj0 = sym_mul(g.cov, j0), Sj1 = sym_mul(g.cov, j1);
  const T p_raw = dot(j0, Sj0), q = dot(j0, Sj1), r_raw = dot(j1, Sj1);
  const T p = p_raw + cam.sigma_u * cam.sigma_u;
  const T r = r_raw + cam.sigma_v * cam.sigma_v;
  const T det = p * r - q * q;
  const T det_raw = p_raw * r_raw - q * q;  // > 0 for any visible pair
  const T k = ls_sqrt(det_raw / det);
  const T E = ls_exp(T(-0.5) * dv * dv / r);
  const T alpha = g.opacity * k * E;

  // u = mu_u + (q / r) dv
  T g_mu_u = g_u;
  T g_q = g_u * dv / r;
  T g_dv = g_u * q / r;
  T g_r = -g_u * q * dv / (r * r);
  // inv_var = r / det
  g_r += g_inv_var / det;
  T g_det = -g_inv_var * r / (det * det);
  // alpha = opacity k E, E = exp(-dv^2 / 2r)
  out->opacity = g_alpha * k * E;
  const T g_k = g_alpha * g.opacity * E;
  g_dv += -g_alpha * alpha * dv / r;
  g_r += g_alpha * alpha * T(0.5) * dv * dv / (r * r);
  // k = sqrt(det_raw / det)
  const T g_det_raw = g_k * k / (T(2) * det_raw);
  g_det += -g_k * k / (T(2) * det);
  // det = p r - q^2, det_raw = p_raw r_raw - q^2, p = p_raw + su^2, r = r_raw + sv^2
  const T g_p_raw = g_det * r + g_det_raw * r_raw;
  const T g_r_raw = g_r + g_det * p + g_det_raw * p_raw;
  g_q += T(-2) * q * (g_det + g_det_raw);
  // p_raw = j0' S j0, q = j0' S j1, r_raw = j1' S j1
  Sym3<T> gc = quad_grad(j0, j0);
  gc = Sym3<T>{g_p_raw * gc.xx, g_p_raw * gc.xy, g_p_raw * gc.xz, g_p_raw * gc.yy, g_p_raw * gc.yz,
               g_p_raw * gc.zz};
  gc = sym_axpy(g_q, quad_grad(j0, j1), gc);
  gc = sym_axpy(g_r_raw, quad_grad(j1, j1), gc);
  out->cov = gc;
  const Vec3<T> g_j0 = (T(2) * g_p_raw) * Sj0 + g_q * Sj1;
  const Vec3<T> g_j1 = g_q * Sj0 + (T(2) * g_r_raw) * Sj1;
  // j0 = a (r0 - txc r2), j1 = a (r1 - tyc r2), a = f / z
  const T g_a = dot(g_j0, e0) + dot(g_j1, e1);
  const T g_txc = -a * dot(g_j0, r2);
  const T g_tyc = -a * dot(g_j1, r2);
  Vec3<T> g_r0 = a * g_j0, g_r1 = a * g_j1;
  Vec3<T> g_r2 = -a * (txc * g_j0 + tyc * g_j1);
  T g_inv_z = cam.f * g_a;
  // mu_u = f tx + cu, mu_v = f ty, dv = v_slit - mu_v, and the clamps
  const T g_mu_v = -g_dv;
  const T g_tx = cam.f * g_mu_u + (free_x ? g_txc : T(0));
  const T g_ty = cam.f * g_mu_v + (free_y ? g_tyc : T(0));
  // tx = xc / z, ty = yc / z
  const T g_xc = g_tx * inv_z;
  const T g_yc = g_ty * inv_z;
  g_inv_z += g_tx * xc + g_ty * yc;
  const T g_zc = -g_inv_z * inv_z * inv_z;
  // (xc, yc, zc) = R mean + t
  out->mean = Vec3<T>{R[0] * g_xc + R[3] * g_yc + R[6] * g_zc, R[1] * g_xc + R[4] * g_yc + R[7] * g_zc,
                      R[2] * g_xc + R[5] * g_yc + R[8] * g_zc};
  g_r0 = g_r0 + g_xc * g.mean;
  g_r1 = g_r1 + g_yc * g.mean;
  g_r2 = g_r2 + g_zc * g.mean;
  out->R[0] = g_r0.x; out->R[1] = g_r0.y; out->R[2] = g_r0.z;
  out->R[3] = g_r1.x; out->R[4] = g_r1.y; out->R[5] = g_r1.z;
  out->R[6] = g_r2.x; out->R[7] = g_r2.y; out->R[8] = g_r2.z;
  out->t[0] = g_xc;
  out->t[1] = g_yc;
  out->t[2] = g_zc;
  out->mu_u = g_mu_u;
  out->mu_v = g_mu_v;
}

// Backward of splat_alpha_at: adds dL/d(u, inv_var, alpha) given dL/d(pixel
// alpha). The clamp at kMaxAlpha passes no gradient, as in 3DGS.
template <typename T>
LS_HD void splat_alpha_backward(T u, T inv_var, T alpha, int p, T g_a, T* g_u, T* g_inv_var, T* g_alpha) {
  const T d = T(p) + T(0.5) - u;
  const T G = ls_exp(T(-0.5) * inv_var * d * d);
  const T a = alpha * G;
  if (a >= T(kMaxAlpha)) return;
  *g_alpha += g_a * G;
  *g_u += g_a * a * inv_var * d;
  *g_inv_var += g_a * a * T(-0.5) * d * d;
}

// Backward of gaussian_geometry: from dL/d(covariance) and dL/d(opacity) to
// the stored log scales, quaternion (not necessarily unit) and opacity logit.
template <typename T>
LS_HD void gaussian_geometry_backward(const float* log_scale, const float* quat, float opacity_logit,
                                      const Sym3<T>& g_cov, T g_opacity, T g_log_scale[3], T g_quat[4],
                                      T* g_logit) {
  const T op = ls_sigmoid(T(opacity_logit));
  *g_logit = g_opacity * op * (T(1) - op);

  T qn[4] = {T(quat[0]), T(quat[1]), T(quat[2]), T(quat[3])};
  const T n = ls_sqrt(qn[0] * qn[0] + qn[1] * qn[1] + qn[2] * qn[2] + qn[3] * qn[3]);
  for (int i = 0; i < 4; ++i) g_quat[i] = T(0);
  const Mat3<T> Rm = quat_to_mat(qn[0], qn[1], qn[2], qn[3]);
  const T s2[3] = {ls_exp(T(2) * T(log_scale[0])), ls_exp(T(2) * T(log_scale[1])),
                   ls_exp(T(2) * T(log_scale[2]))};
  // cov = R diag(s^2) R^T with stored entries S_ij = sum_k R_ik R_jk s2_k.
  // D holds dL/dS_ij with the diagonal doubled, so dL/dR_ik = sum_j D_ij R_jk s2_k.
  const T D[9] = {T(2) * g_cov.xx, g_cov.xy, g_cov.xz, g_cov.xy, T(2) * g_cov.yy, g_cov.yz,
                  g_cov.xz, g_cov.yz, T(2) * g_cov.zz};
  T gR[9];
  for (int kk = 0; kk < 3; ++kk) {
    T g_s2 = T(0);
    for (int i = 0; i < 3; ++i) {
      T acc = T(0);
      for (int j = 0; j < 3; ++j) acc += D[3 * i + j] * Rm.m[3 * j + kk];
      gR[3 * i + kk] = acc * s2[kk];
      g_s2 += T(0.5) * acc * Rm.m[3 * i + kk];
    }
    g_log_scale[kk] = g_s2 * T(2) * s2[kk];
  }
  if (!(n > T(0))) return;
  const T w = qn[0] / n, x = qn[1] / n, y = qn[2] / n, z = qn[3] / n;
  // dR/dw, dR/dx, dR/dy, dR/dz of the unit quaternion's rotation matrix.
  const T gw = T(2) * (-z * gR[1] + y * gR[2] + z * gR[3] - x * gR[5] - y * gR[6] + x * gR[7]);
  const T gx = T(2) * (y * gR[1] + z * gR[2] + y * gR[3] - T(2) * x * gR[4] - w * gR[5] + z * gR[6] +
                       w * gR[7] - T(2) * x * gR[8]);
  const T gy = T(2) * (-T(2) * y * gR[0] + x * gR[1] + w * gR[2] + x * gR[3] + z * gR[5] - w * gR[6] +
                       z * gR[7] - T(2) * y * gR[8]);
  const T gz = T(2) * (-T(2) * z * gR[0] - w * gR[1] + x * gR[2] + w * gR[3] - T(2) * z * gR[4] +
                       y * gR[5] + x * gR[6] + y * gR[7]);
  // Through the normalization q / |q|.
  const T along = w * gw + x * gx + y * gy + z * gz;
  g_quat[0] = (gw - w * along) / n;
  g_quat[1] = (gx - x * along) / n;
  g_quat[2] = (gy - y * along) / n;
  g_quat[3] = (gz - z * along) / n;
}

}  // namespace linesplat
