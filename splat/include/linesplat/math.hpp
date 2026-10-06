// Small vector and matrix types shared by host and device code.
//
// The CUDA kernels and the CPU reference renderer run the same projection
// math (line_camera.hpp), so everything here is marked LS_HD and templated on
// the scalar type: float on the GPU, float or double on the CPU. Keep this
// header C++14, because the Jetson Nano's CUDA 10.2 compiler stops there.
#pragma once

#include <math.h>

#if defined(__CUDACC__)
#define LS_HD __host__ __device__ __forceinline__
#else
#define LS_HD inline
#endif

namespace linesplat {

constexpr double kPi = 3.14159265358979323846;

// Overloads instead of std:: calls: the C names work the same way in host
// and device code on every toolkit we target.
LS_HD float ls_sqrt(float x) { return sqrtf(x); }
LS_HD double ls_sqrt(double x) { return sqrt(x); }
LS_HD float ls_exp(float x) { return expf(x); }
LS_HD double ls_exp(double x) { return exp(x); }
LS_HD float ls_log(float x) { return logf(x); }
LS_HD double ls_log(double x) { return log(x); }
LS_HD float ls_floor(float x) { return floorf(x); }
LS_HD double ls_floor(double x) { return floor(x); }
LS_HD float ls_ceil(float x) { return ceilf(x); }
LS_HD double ls_ceil(double x) { return ceil(x); }
LS_HD float ls_abs(float x) { return fabsf(x); }
LS_HD double ls_abs(double x) { return fabs(x); }
template <typename T> LS_HD T ls_min(T a, T b) { return a < b ? a : b; }
template <typename T> LS_HD T ls_max(T a, T b) { return a > b ? a : b; }
template <typename T> LS_HD T ls_clamp(T x, T lo, T hi) { return ls_min(ls_max(x, lo), hi); }
template <typename T> LS_HD T ls_sigmoid(T x) { return T(1) / (T(1) + ls_exp(-x)); }

template <typename T>
struct Vec3 {
  T x, y, z;
};

template <typename T> LS_HD Vec3<T> operator+(const Vec3<T>& a, const Vec3<T>& b) {
  return Vec3<T>{a.x + b.x, a.y + b.y, a.z + b.z};
}
template <typename T> LS_HD Vec3<T> operator-(const Vec3<T>& a, const Vec3<T>& b) {
  return Vec3<T>{a.x - b.x, a.y - b.y, a.z - b.z};
}
template <typename T> LS_HD Vec3<T> operator-(const Vec3<T>& a) { return Vec3<T>{-a.x, -a.y, -a.z}; }
template <typename T> LS_HD Vec3<T> operator*(T s, const Vec3<T>& a) { return Vec3<T>{s * a.x, s * a.y, s * a.z}; }
template <typename T> LS_HD Vec3<T> operator*(const Vec3<T>& a, T s) { return s * a; }
template <typename T> LS_HD T dot(const Vec3<T>& a, const Vec3<T>& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
template <typename T> LS_HD Vec3<T> cross(const Vec3<T>& a, const Vec3<T>& b) {
  return Vec3<T>{a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
template <typename T> LS_HD T norm(const Vec3<T>& a) { return ls_sqrt(dot(a, a)); }
template <typename T> LS_HD Vec3<T> normalized(const Vec3<T>& a) { return (T(1) / norm(a)) * a; }

// 3x3 matrix, row-major.
template <typename T>
struct Mat3 {
  T m[9];
  LS_HD T operator()(int r, int c) const { return m[3 * r + c]; }
  LS_HD T& operator()(int r, int c) { return m[3 * r + c]; }
  LS_HD Vec3<T> row(int r) const { return Vec3<T>{m[3 * r], m[3 * r + 1], m[3 * r + 2]}; }
  LS_HD Vec3<T> col(int c) const { return Vec3<T>{m[c], m[3 + c], m[6 + c]}; }
};

template <typename T> LS_HD Mat3<T> mat3_identity() {
  return Mat3<T>{{T(1), T(0), T(0), T(0), T(1), T(0), T(0), T(0), T(1)}};
}
template <typename T> LS_HD Mat3<T> mat3_from_cols(const Vec3<T>& a, const Vec3<T>& b, const Vec3<T>& c) {
  return Mat3<T>{{a.x, b.x, c.x, a.y, b.y, c.y, a.z, b.z, c.z}};
}
template <typename T> LS_HD Vec3<T> operator*(const Mat3<T>& M, const Vec3<T>& v) {
  return Vec3<T>{M.m[0] * v.x + M.m[1] * v.y + M.m[2] * v.z,
                 M.m[3] * v.x + M.m[4] * v.y + M.m[5] * v.z,
                 M.m[6] * v.x + M.m[7] * v.y + M.m[8] * v.z};
}
template <typename T> LS_HD Mat3<T> operator*(const Mat3<T>& A, const Mat3<T>& B) {
  Mat3<T> C;
  for (int r = 0; r < 3; ++r)
    for (int c = 0; c < 3; ++c)
      C.m[3 * r + c] = A.m[3 * r] * B.m[c] + A.m[3 * r + 1] * B.m[3 + c] + A.m[3 * r + 2] * B.m[6 + c];
  return C;
}
template <typename T> LS_HD Mat3<T> transpose(const Mat3<T>& A) {
  return Mat3<T>{{A.m[0], A.m[3], A.m[6], A.m[1], A.m[4], A.m[7], A.m[2], A.m[5], A.m[8]}};
}

// Symmetric 3x3 matrix (a covariance), stored as its upper triangle.
template <typename T>
struct Sym3 {
  T xx, xy, xz, yy, yz, zz;
};

// a^T S b
template <typename T> LS_HD T quad(const Sym3<T>& S, const Vec3<T>& a, const Vec3<T>& b) {
  return a.x * (S.xx * b.x + S.xy * b.y + S.xz * b.z) +
         a.y * (S.xy * b.x + S.yy * b.y + S.yz * b.z) +
         a.z * (S.xz * b.x + S.yz * b.y + S.zz * b.z);
}

// Rotation matrix of the quaternion (w, x, y, z). The quaternion does not need
// to be unit length: like 3DGS, the optimizer works on an unnormalized one.
template <typename T> LS_HD Mat3<T> quat_to_mat(T w, T x, T y, T z) {
  const T n = ls_sqrt(w * w + x * x + y * y + z * z);
  const T inv = n > T(0) ? T(1) / n : T(0);
  w *= inv; x *= inv; y *= inv; z *= inv;
  if (n <= T(0)) w = T(1);
  return Mat3<T>{{T(1) - T(2) * (y * y + z * z), T(2) * (x * y - w * z), T(2) * (x * z + w * y),
                  T(2) * (x * y + w * z), T(1) - T(2) * (x * x + z * z), T(2) * (y * z - w * x),
                  T(2) * (x * z - w * y), T(2) * (y * z + w * x), T(1) - T(2) * (x * x + y * y)}};
}

// Covariance R diag(s^2) R^T of a Gaussian with axis scales s and rotation R.
template <typename T> LS_HD Sym3<T> covariance_from_scale_rot(const Vec3<T>& s, const Mat3<T>& R) {
  const T sx = s.x * s.x, sy = s.y * s.y, sz = s.z * s.z;
  Sym3<T> S;
  S.xx = R.m[0] * R.m[0] * sx + R.m[1] * R.m[1] * sy + R.m[2] * R.m[2] * sz;
  S.xy = R.m[0] * R.m[3] * sx + R.m[1] * R.m[4] * sy + R.m[2] * R.m[5] * sz;
  S.xz = R.m[0] * R.m[6] * sx + R.m[1] * R.m[7] * sy + R.m[2] * R.m[8] * sz;
  S.yy = R.m[3] * R.m[3] * sx + R.m[4] * R.m[4] * sy + R.m[5] * R.m[5] * sz;
  S.yz = R.m[3] * R.m[6] * sx + R.m[4] * R.m[7] * sy + R.m[5] * R.m[8] * sz;
  S.zz = R.m[6] * R.m[6] * sx + R.m[7] * R.m[7] * sy + R.m[8] * R.m[8] * sz;
  return S;
}

}  // namespace linesplat
