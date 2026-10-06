#include "linesplat/trainer.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <unordered_map>

#include "linesplat/util.hpp"

namespace linesplat {

namespace {

constexpr double kBeta1 = 0.9, kBeta2 = 0.999, kEps = 1e-15;

// One Adam step on every element of p, at step t (from 1).
void adam_step(std::vector<float>& p, const std::vector<float>& g, std::vector<float>& m, std::vector<float>& v,
               double lr, int t) {
  // Bias corrections folded into the step size and epsilon.
  const double c1 = 1.0 - std::pow(kBeta1, t), c2 = 1.0 - std::pow(kBeta2, t);
  const float step = float(lr * std::sqrt(c2) / c1), eps = float(kEps * std::sqrt(c2));
  const float b1 = float(kBeta1), b2 = float(kBeta2);
  const int n = int(p.size());
#pragma omp parallel for schedule(static)
  for (int i = 0; i < n; ++i) {
    const float gi = g[size_t(i)];
    float& mi = m[size_t(i)];
    float& vi = v[size_t(i)];
    mi = b1 * mi + (1.0f - b1) * gi;
    vi = b2 * vi + (1.0f - b2) * gi * gi;
    p[size_t(i)] -= step * mi / (std::sqrt(vi) + eps);
  }
}

Mat3d hat(const Vec3d& w) { return Mat3d{{0.0, -w.z, w.y, w.z, 0.0, -w.x, -w.y, w.x, 0.0}}; }

// Right Jacobian of SO(3): exp(phi + d) ~ exp(phi) exp(J_r(phi) d).
Mat3d right_jacobian(const Vec3d& phi) {
  const double th2 = dot(phi, phi), th = std::sqrt(th2);
  double a, b;  // J_r = I - a [phi]x + b [phi]x^2
  if (th < 1e-4) {
    a = 0.5 - th2 / 24.0;
    b = 1.0 / 6.0 - th2 / 120.0;
  } else {
    a = (1.0 - std::cos(th)) / th2;
    b = (th - std::sin(th)) / (th2 * th);
  }
  const Mat3d K = hat(phi), K2 = K * K;
  Mat3d J = mat3_identity<double>();
  for (int i = 0; i < 9; ++i) J.m[i] += -a * K.m[i] + b * K2.m[i];
  return J;
}

void shuffle(std::vector<int>& v, Rng& rng) {
  for (size_t i = v.size(); i > 1; --i) std::swap(v[i - 1], v[size_t(rng.next_u64() % i)]);
}

// Camera -> world for a line camera (which stores world -> camera).
Vec3d camera_to_world(const LineCameraT<double>& c, const Vec3d& x) {
  const Vec3d d{x.x - c.t[0], x.y - c.t[1], x.z - c.t[2]};
  return Vec3d{c.R[0] * d.x + c.R[3] * d.y + c.R[6] * d.z, c.R[1] * d.x + c.R[4] * d.y + c.R[7] * d.z,
               c.R[2] * d.x + c.R[5] * d.y + c.R[8] * d.z};
}

Vec3d world_to_camera(const LineCameraT<double>& c, const Vec3d& x) {
  return Vec3d{c.R[0] * x.x + c.R[1] * x.y + c.R[2] * x.z + c.t[0], c.R[3] * x.x + c.R[4] * x.y + c.R[5] * x.z + c.t[1],
               c.R[6] * x.x + c.R[7] * x.y + c.R[8] * x.z + c.t[2]};
}

// The camera-frame point where pixel p's ray (at the slit) meets the plane
// z = plane_z of the world, if it does so in front of the camera.
bool pixel_on_plane(const LineCameraT<double>& c, double p, double plane_z, Vec3d* x_cam) {
  const Vec3d dc{(p + 0.5 - c.cu) / c.f, c.v_slit / c.f, 1.0};
  const Vec3d o = camera_to_world(c, Vec3d{0, 0, 0});
  const Vec3d dw = camera_to_world(c, dc) - o;
  if (std::fabs(dw.z) < 1e-12) return false;
  const double s = (plane_z - o.z) / dw.z;  // depth along the camera's z, since dc.z = 1
  if (!(s > c.near_z)) return false;
  *x_cam = s * dc;
  return true;
}

// Eigen-decomposition of a symmetric 4x4 matrix by cyclic Jacobi rotations.
// On return A is diagonal (the eigenvalues) and V's columns the eigenvectors.
void jacobi_eigen4(double A[4][4], double V[4][4]) {
  for (int i = 0; i < 4; ++i)
    for (int j = 0; j < 4; ++j) V[i][j] = i == j ? 1.0 : 0.0;
  for (int sweep = 0; sweep < 60; ++sweep) {
    double off = 0.0, diag = 0.0;
    for (int p = 0; p < 4; ++p) {
      diag += A[p][p] * A[p][p];
      for (int q = p + 1; q < 4; ++q) off += A[p][q] * A[p][q];
    }
    if (off <= 1e-30 * diag || off == 0.0) break;
    for (int p = 0; p < 4; ++p)
      for (int q = p + 1; q < 4; ++q) {
        if (A[p][q] == 0.0) continue;
        const double theta = (A[q][q] - A[p][p]) / (2.0 * A[p][q]);
        const double t = (theta >= 0.0 ? 1.0 : -1.0) / (std::fabs(theta) + std::sqrt(theta * theta + 1.0));
        const double c = 1.0 / std::sqrt(t * t + 1.0), s = t * c;
        for (int k = 0; k < 4; ++k) {  // A J
          const double akp = A[k][p], akq = A[k][q];
          A[k][p] = c * akp - s * akq;
          A[k][q] = s * akp + c * akq;
        }
        for (int k = 0; k < 4; ++k) {  // J^T (A J)
          const double apk = A[p][k], aqk = A[q][k];
          A[p][k] = c * apk - s * aqk;
          A[q][k] = s * apk + c * aqk;
        }
        for (int k = 0; k < 4; ++k) {
          const double vkp = V[k][p], vkq = V[k][q];
          V[k][p] = c * vkp - s * vkq;
          V[k][q] = s * vkp + c * vkq;
        }
      }
  }
}

// The rigid transform X minimizing sum |X a_i - b_i|^2 (Horn's quaternion
// method), or with *scale (if given) the similarity s X.
Pose fit_rigid(const std::vector<Vec3d>& a, const std::vector<Vec3d>& b, double* scale = nullptr) {
  Vec3d ca{0, 0, 0}, cb{0, 0, 0};
  for (size_t i = 0; i < a.size(); ++i) {
    ca = ca + a[i];
    cb = cb + b[i];
  }
  ca = (1.0 / double(a.size())) * ca;
  cb = (1.0 / double(b.size())) * cb;
  double S[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
  for (size_t i = 0; i < a.size(); ++i) {
    const Vec3d x = a[i] - ca, y = b[i] - cb;
    const double xv[3] = {x.x, x.y, x.z}, yv[3] = {y.x, y.y, y.z};
    for (int r = 0; r < 3; ++r)
      for (int c = 0; c < 3; ++c) S[r][c] += xv[r] * yv[c];
  }
  const double xx = S[0][0], xy = S[0][1], xz = S[0][2], yx = S[1][0], yy = S[1][1], yz = S[1][2], zx = S[2][0],
               zy = S[2][1], zz = S[2][2];
  double N[4][4] = {{xx + yy + zz, yz - zy, zx - xz, xy - yx},
                    {yz - zy, xx - yy - zz, xy + yx, zx + xz},
                    {zx - xz, xy + yx, -xx + yy - zz, yz + zy},
                    {xy - yx, zx + xz, yz + zy, -xx - yy + zz}};
  double V[4][4];
  jacobi_eigen4(N, V);
  int best = 0;
  for (int i = 1; i < 4; ++i)
    if (N[i][i] > N[best][best]) best = i;
  const double q[4] = {V[0][best], V[1][best], V[2][best], V[3][best]};
  Pose X;
  X.R = quat_to_rotation(q);
  double sc = 1.0;
  if (scale) {
    double num = 0.0, den = 0.0;
    for (size_t i = 0; i < a.size(); ++i) {
      num += dot(b[i] - cb, X.R * (a[i] - ca));
      den += dot(a[i] - ca, a[i] - ca);
    }
    sc = *scale = num / den;
  }
  X.t = cb - sc * (X.R * ca);
  return X;
}

}  // namespace

Pose corrected_head(const Pose& recorded, const Pose& G, const double zeta[6]) {
  Pose E;
  E.R = rotation_exp(Vec3d{zeta[3], zeta[4], zeta[5]});
  E.t = Vec3d{zeta[0], zeta[1], zeta[2]};
  return recorded * G * E * G.inverse();
}

template <typename T>
void add_head_twist_grad(const LineCameraT<double>& cam, const Pose& virt, const CameraGradT<T>& g,
                         double g_eta[6]) {
  // The gradient for a small motion of the camera, exp(xi) applied after the
  // world -> camera transform: g_v = dL/dt, g_w = sum_k R_k x G_k + t x dL/dt
  // (R_k, G_k the k-th columns of R and dL/dR).
  const Vec3d gv{double(g.t[0]), double(g.t[1]), double(g.t[2])};
  Vec3d gw = cross(Vec3d{cam.t[0], cam.t[1], cam.t[2]}, gv);
  for (int k = 0; k < 3; ++k) {
    const Vec3d Rk{cam.R[k], cam.R[3 + k], cam.R[6 + k]};
    const Vec3d Gk{double(g.R[k]), double(g.R[3 + k]), double(g.R[6 + k])};
    gw = gw + cross(Rk, Gk);
  }
  // Moving the head by eta in its own frame (head * exp(eta)) moves the
  // camera by xi = -Ad(virt^-1) eta, so:
  const Vec3d a = virt.R * gv;
  const Vec3d g_v = -a;
  const Vec3d g_w = cross(a, virt.t) - virt.R * gw;
  g_eta[0] += g_v.x;
  g_eta[1] += g_v.y;
  g_eta[2] += g_v.z;
  g_eta[3] += g_w.x;
  g_eta[4] += g_w.y;
  g_eta[5] += g_w.z;
}

template void add_head_twist_grad(const LineCameraT<double>&, const Pose&, const CameraGradT<float>&, double[6]);
template void add_head_twist_grad(const LineCameraT<double>&, const Pose&, const CameraGradT<double>&, double[6]);

void correction_grad(const Pose& G, const double zeta[6], const double g_eta[6], double g_zeta[6]) {
  // A step d_zeta changes [exp(phi), rho] by [exp(J_r d_phi), exp(phi)^T d_rho]
  // on its right, which moves the head by eta = Ad(G) of that, so the
  // gradient goes back through Ad(G)^T, then exp(phi) and J_r^T.
  const Vec3d gv{g_eta[0], g_eta[1], g_eta[2]}, gw{g_eta[3], g_eta[4], g_eta[5]};
  const Mat3d Gt = transpose(G.R);
  const Vec3d xv = Gt * gv;
  const Vec3d xw = Gt * (gw + cross(gv, G.t));
  const Vec3d phi{zeta[3], zeta[4], zeta[5]};
  const Vec3d g_rho = rotation_exp(phi) * xv;
  const Vec3d g_phi = transpose(right_jacobian(phi)) * xw;
  g_zeta[0] = g_rho.x;
  g_zeta[1] = g_rho.y;
  g_zeta[2] = g_rho.z;
  g_zeta[3] = g_phi.x;
  g_zeta[4] = g_phi.y;
  g_zeta[5] = g_phi.z;
}

GaussianScene init_on_plane(const Dataset& d, double spacing, double plane_z) {
  d.validate();
  const int W = d.width(), B = d.num_bands();
  std::unordered_map<long long, int> cell_of;
  std::vector<long long> keys;
  std::vector<double> pos, spec;  // [cells, 2], [cells, B] sums
  std::vector<int> count;
  for (int l = 0; l < d.num_lines(); ++l) {
    const LineCameraT<double> c =
        make_line_camera_d(d.head, d.sweep_head_pose[size_t(d.line_sweep[size_t(l)])],
                           d.line_mirror_angle[size_t(l)], d.intrinsics);
    const float* line = d.line(l);
    for (int p = 0; p < W; ++p) {
      Vec3d xc;
      if (!pixel_on_plane(c, p, plane_z, &xc)) continue;
      const Vec3d x = camera_to_world(c, xc);
      if (std::fabs(x.x) > 1.0 || std::fabs(x.y) > 1.0) continue;
      const long long ix = (long long)std::floor(x.x / spacing), iy = (long long)std::floor(x.y / spacing);
      const long long key = (ix << 32) ^ (iy & 0xffffffffLL);
      auto it = cell_of.find(key);
      int k;
      if (it == cell_of.end()) {
        k = int(keys.size());
        cell_of.emplace(key, k);
        keys.push_back(key);
        pos.insert(pos.end(), {0.0, 0.0});
        spec.insert(spec.end(), size_t(B), 0.0);
        count.push_back(0);
      } else {
        k = it->second;
      }
      pos[2 * size_t(k)] += x.x;
      pos[2 * size_t(k) + 1] += x.y;
      for (int b = 0; b < B; ++b) spec[size_t(k) * B + b] += line[size_t(p) * B + b];
      ++count[size_t(k)];
    }
  }
  // Cells in a fixed order, so the scene doesn't depend on the hash table.
  std::vector<int> order(keys.size());
  for (size_t i = 0; i < order.size(); ++i) order[i] = int(i);
  std::sort(order.begin(), order.end(), [&](int a, int b) { return keys[size_t(a)] < keys[size_t(b)]; });

  GaussianScene s;
  s.num_features = B;
  std::vector<float> f(static_cast<size_t>(B));
  const float sc[3] = {float(0.5 * spacing), float(0.5 * spacing), float(0.5 * spacing)};
  const float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};
  for (int k : order) {
    const double n = count[size_t(k)];
    const float mean[3] = {float(pos[2 * size_t(k)] / n), float(pos[2 * size_t(k) + 1] / n), float(plane_z)};
    for (int b = 0; b < B; ++b) f[size_t(b)] = float(spec[size_t(k) * B + b] / n);
    s.add(mean, sc, q, 0.5f, f.data());
  }
  s.set_identity_basis(B);
  s.background.assign(size_t(B), 0.0f);
  return s;
}

BatchBackwardFn cpu_batch_backward(const Dataset& data) {
  return [&data](const GaussianScene& scene, const std::vector<LineCamera>& cams, const std::vector<int>& lines,
                 SceneGradT<float>* grad, std::vector<CameraGradT<float>>* cam_grad) {
    const int WB = data.width() * data.num_bands();
    const double inv = 1.0 / (double(std::max<size_t>(1, cams.size())) * WB);
    return render_backward_cpu<float>(
        scene, cams,
        [&](int i, const float* bands, float* g) {
          const float* meas = data.line(lines[size_t(i)]);
          double sum = 0.0;
          for (int k = 0; k < WB; ++k) {
            const double d = double(bands[k]) - double(meas[k]);
            g[k] = float(2.0 * d * inv);
            sum += d * d;
          }
          return sum * inv;
        },
        grad, cam_grad);
  };
}

Trainer::Trainer(const Dataset& data, GaussianScene init, const TrainOptions& opt, BatchBackwardFn backward)
    : data_(data),
      opt_(opt),
      backward_(backward ? std::move(backward) : cpu_batch_backward(data)),
      scene_(std::move(init)),
      rng_(opt.seed) {
  data_.validate();
  scene_.validate();
  if (scene_.num_bands() != data_.num_bands())
    throw std::runtime_error("Trainer: the scene has " + std::to_string(scene_.num_bands()) +
                             " bands and the dataset " + std::to_string(data_.num_bands()));
  if (opt_.iterations < 1 || opt_.batch_lines < 1 || opt_.densify_every < 1)
    throw std::runtime_error("Trainer: bad options");
  pose_frame_ = virtual_camera_in_head(data_.head, 0.0);
  const int L = data_.num_lines();
  virt_.resize(size_t(L));
  v_sign_.resize(size_t(L));
  for (int l = 0; l < L; ++l)
    virt_[size_t(l)] = virtual_camera_in_head(data_.head, data_.line_mirror_angle[size_t(l)], &v_sign_[size_t(l)]);
  poses_.resize(size_t(data_.num_sweeps()));
  const size_t N = size_t(scene_.size()), K = size_t(scene_.num_features);
  a_means_.resize(3 * N);
  a_scales_.resize(3 * N);
  a_rots_.resize(4 * N);
  a_opacity_.resize(N);
  a_features_.resize(N * K);
  a_background_.resize(K);
  screen_sum_.assign(N, 0.0);
  pair_count_.assign(N, 0);
  order_.resize(size_t(L));
  for (int l = 0; l < L; ++l) order_[size_t(l)] = l;
  shuffle(order_, rng_);
}

Pose Trainer::head_pose(int sweep) const {
  return corrected_head(data_.sweep_head_pose[size_t(sweep)], pose_frame_, poses_[size_t(sweep)].zeta);
}

LineCameraT<double> Trainer::camera(int line, const Pose& head) const {
  // As make_line_camera_d, with the line's virtual camera worked out once.
  LineCameraT<double> c = line_camera_from_pose(head * virt_[size_t(line)], data_.intrinsics);
  c.v_slit = v_sign_[size_t(line)] * data_.intrinsics.v_slit;
  return c;
}

std::vector<Pose> Trainer::head_poses() const {
  std::vector<Pose> out;
  for (int s = 0; s < data_.num_sweeps(); ++s) out.push_back(head_pose(s));
  return out;
}

std::vector<LineCameraT<double>> Trainer::cameras() const {
  const std::vector<Pose> heads = head_poses();
  std::vector<LineCameraT<double>> out;
  for (int l = 0; l < data_.num_lines(); ++l) out.push_back(camera(l, heads[size_t(data_.line_sweep[size_t(l)])]));
  return out;
}

double Trainer::decayed(double lr, double final_fraction) const {
  const double t = std::min(1.0, double(iter_) / double(opt_.iterations));
  return lr * std::pow(final_fraction, t);
}

TrainStep Trainer::step() {
  Timer timer;
  ++iter_;
  TrainStep st;
  st.iteration = iter_;

  // The next batch of lines, without repeats until every line has had a turn.
  const int n = std::min(opt_.batch_lines, data_.num_lines());
  std::vector<int> batch(static_cast<size_t>(n));
  for (int& l : batch) {
    if (next_ >= order_.size()) {
      shuffle(order_, rng_);
      next_ = 0;
    }
    l = order_[next_++];
  }
  const std::vector<Pose> heads = head_poses();
  std::vector<LineCameraT<double>> cams_d;
  std::vector<LineCamera> cams;
  for (int l : batch) {
    cams_d.push_back(camera(l, heads[size_t(data_.line_sweep[size_t(l)])]));
    cams.push_back(cast_camera<float>(cams_d.back()));
  }

  // Mean squared error over every pixel and band of the batch.
  const bool poses = opt_.refine_poses && iter_ >= opt_.pose_from;
  SceneGradT<float> grad;
  std::vector<CameraGradT<float>> cam_grad;
  st.loss = backward_(scene_, cams, batch, &grad, poses ? &cam_grad : nullptr);

  adam_scene(grad);
  if (poses) adam_poses(batch, cams_d, cam_grad);

  if (iter_ <= opt_.densify_until) {
    // Per pair, as if the loss were one line's (the batch averages n lines),
    // with the centre measured in line widths so that the threshold doesn't
    // depend on how many pixels the camera has.
    const double per_line = double(n) * data_.width();
    for (int i = 0; i < scene_.size(); ++i) {
      screen_sum_[size_t(i)] += double(grad.screen_grad[size_t(i)]) * per_line;
      pair_count_[size_t(i)] += grad.pairs[size_t(i)];
    }
    if (iter_ >= opt_.densify_from && iter_ % opt_.densify_every == 0) densify(&st);
  }
  st.gaussians = scene_.size();
  st.ms = timer.ms();
  return st;
}

void Trainer::adam_scene(const SceneGradT<float>& g) {
  adam_step(scene_.means, g.means, a_means_.m, a_means_.v, decayed(opt_.lr_means, 0.01), iter_);
  adam_step(scene_.log_scales, g.log_scales, a_scales_.m, a_scales_.v, opt_.lr_log_scales, iter_);
  adam_step(scene_.rotations, g.rotations, a_rots_.m, a_rots_.v, opt_.lr_rotations, iter_);
  adam_step(scene_.opacity_logits, g.opacity_logits, a_opacity_.m, a_opacity_.v, opt_.lr_opacity, iter_);
  adam_step(scene_.features, g.features, a_features_.m, a_features_.v, opt_.lr_features, iter_);
  adam_step(scene_.background, g.background, a_background_.m, a_background_.v, opt_.lr_background, iter_);
}

void Trainer::adam_poses(const std::vector<int>& lines, const std::vector<LineCameraT<double>>& cams,
                         const std::vector<CameraGradT<float>>& cam_grad) {
  const int S = data_.num_sweeps();
  std::vector<double> g_eta(6 * size_t(S), 0.0);
  std::vector<char> seen(size_t(S), 0);
  for (size_t i = 0; i < lines.size(); ++i) {
    const int s = data_.line_sweep[size_t(lines[i])];
    add_head_twist_grad(cams[i], virt_[size_t(lines[i])], cam_grad[i], &g_eta[6 * size_t(s)]);
    seen[size_t(s)] = 1;
  }
  std::vector<double> g(6 * size_t(S), 0.0);
  for (int s = 0; s < S; ++s)
    if (seen[size_t(s)]) correction_grad(pose_frame_, poses_[size_t(s)].zeta, &g_eta[6 * size_t(s)], &g[6 * size_t(s)]);
  const double lr_t = decayed(opt_.lr_pose_trans, 0.1), lr_r = decayed(opt_.lr_pose_rot, 0.1);
  for (int s = 0; s < S; ++s) {
    if (!seen[size_t(s)] || s == opt_.fixed_sweep) continue;
    PoseState& p = poses_[size_t(s)];
    ++p.steps;
    const double c1 = 1.0 - std::pow(kBeta1, p.steps), c2 = 1.0 - std::pow(kBeta2, p.steps);
    for (int k = 0; k < 6; ++k) {
      const double gk = g[6 * size_t(s) + size_t(k)];
      p.m[k] = kBeta1 * p.m[k] + (1.0 - kBeta1) * gk;
      p.v[k] = kBeta2 * p.v[k] + (1.0 - kBeta2) * gk * gk;
      p.zeta[k] -= (k < 3 ? lr_t : lr_r) * (p.m[k] / c1) / (std::sqrt(p.v[k] / c2) + kEps);
    }
  }
}

void Trainer::densify(TrainStep* st) {
  const int N = scene_.size(), K = scene_.num_features;
  struct Born {
    float mean[3], log_scale[3];
    int src;  // the Gaussian it copies everything else from
  };
  std::vector<Born> born;
  std::vector<char> keep(size_t(N), 1);
  int room = opt_.max_gaussians - N;
  const double split_log = std::log(opt_.split_scale), shrink = std::log(1.6);
  for (int i = 0; i < N && room > 0; ++i) {
    if (pair_count_[size_t(i)] == 0 || screen_sum_[size_t(i)] / pair_count_[size_t(i)] < opt_.densify_grad) continue;
    const float* ls = &scene_.log_scales[3 * size_t(i)];
    const float* m = &scene_.means[3 * size_t(i)];
    Born b;
    b.src = i;
    std::copy(m, m + 3, b.mean);
    std::copy(ls, ls + 3, b.log_scale);
    if (std::max(ls[0], std::max(ls[1], ls[2])) > split_log) {
      // Too big to sharpen by moving: replace it with two Gaussians drawn
      // from it, each 1.6 times smaller (as 3DGS does).
      const float* q = &scene_.rotations[4 * size_t(i)];
      const Mat3d R = quat_to_mat<double>(q[0], q[1], q[2], q[3]);
      for (int j = 0; j < 2; ++j) {
        const Vec3d z{std::exp(double(ls[0])) * rng_.normal(), std::exp(double(ls[1])) * rng_.normal(),
                      std::exp(double(ls[2])) * rng_.normal()};
        const Vec3d off = R * z;
        Born c = b;
        c.mean[0] += float(off.x);
        c.mean[1] += float(off.y);
        c.mean[2] += float(off.z);
        for (float& v : c.log_scale) v -= float(shrink);
        born.push_back(c);
      }
      keep[size_t(i)] = 0;
    } else {
      born.push_back(b);  // small: a copy, which the next steps pull apart
    }
    --room;
  }
  const int removed_by_split = int(std::count(keep.begin(), keep.end(), 0));

  // What survives: kept Gaussians, then the new ones, minus the nearly
  // transparent and the oversized.
  const float min_logit = float(std::log(opt_.prune_opacity / (1.0 - opt_.prune_opacity)));
  const float max_log = float(std::log(opt_.max_scale));
  struct Item {
    int old, born;  // one of them is -1
  };
  std::vector<Item> items;
  int pruned = 0;
  auto alive = [&](int src, const float* ls) {
    const bool ok = scene_.opacity_logits[size_t(src)] >= min_logit && std::max(ls[0], std::max(ls[1], ls[2])) <= max_log;
    pruned += !ok;
    return ok;
  };
  for (int i = 0; i < N; ++i)
    if (keep[size_t(i)] && alive(i, &scene_.log_scales[3 * size_t(i)])) items.push_back(Item{i, -1});
  for (int j = 0; j < int(born.size()); ++j)
    if (alive(born[size_t(j)].src, born[size_t(j)].log_scale)) items.push_back(Item{-1, j});

  // Rebuild every per-Gaussian array and its Adam state; new Gaussians start
  // with zero moments.
  auto rebuild = [&](std::vector<float>& arr, Adam& a, int dim, const float* (*born_src)(const Born&)) {
    std::vector<float> na(items.size() * dim), nm(items.size() * dim, 0.0f), nv(items.size() * dim, 0.0f);
    for (size_t k = 0; k < items.size(); ++k) {
      float* dst = &na[k * dim];
      if (items[k].old >= 0) {
        const size_t o = size_t(items[k].old) * dim;
        std::copy(&arr[o], &arr[o] + dim, dst);
        std::copy(&a.m[o], &a.m[o] + dim, &nm[k * dim]);
        std::copy(&a.v[o], &a.v[o] + dim, &nv[k * dim]);
      } else {
        const Born& b = born[size_t(items[k].born)];
        const float* src = born_src ? born_src(b) : &arr[size_t(b.src) * dim];
        std::copy(src, src + dim, dst);
      }
    }
    arr.swap(na);
    a.m.swap(nm);
    a.v.swap(nv);
  };
  rebuild(scene_.means, a_means_, 3, [](const Born& b) -> const float* { return b.mean; });
  rebuild(scene_.log_scales, a_scales_, 3, [](const Born& b) -> const float* { return b.log_scale; });
  rebuild(scene_.rotations, a_rots_, 4, nullptr);
  rebuild(scene_.opacity_logits, a_opacity_, 1, nullptr);
  rebuild(scene_.features, a_features_, K, nullptr);
  screen_sum_.assign(items.size(), 0.0);
  pair_count_.assign(items.size(), 0);

  st->densified = int(born.size());
  st->pruned = removed_by_split + pruned;
}

PoseErrorPx line_pose_error_px(const std::vector<LineCameraT<double>>& estimate,
                               const std::vector<LineCameraT<double>>& truth, double plane_z, bool with_scale) {
  if (estimate.size() != truth.size()) throw std::runtime_error("line_pose_error_px: different numbers of cameras");
  // Pairs of world points: where a true pixel ray meets the plane, and the
  // same camera-frame point placed by the estimate.
  std::vector<Vec3d> a, b;
  std::vector<int> line;
  std::vector<double> px;
  for (size_t l = 0; l < truth.size(); ++l) {
    const int W = truth[l].width;
    const int ps[5] = {0, W / 4, W / 2, (3 * W) / 4, W - 1};
    for (int j = 0; j < 5; ++j) {
      if (j > 0 && ps[j] == ps[j - 1]) continue;
      Vec3d x;
      if (!pixel_on_plane(truth[l], ps[j], plane_z, &x)) continue;
      a.push_back(camera_to_world(truth[l], x));
      b.push_back(camera_to_world(estimate[l], x));
      line.push_back(int(l));
      px.push_back(ps[j] + 0.5);
    }
  }
  PoseErrorPx e;
  e.points = int(a.size());
  if (a.size() < 3) return e;
  double scale = 1.0;
  const Pose X = fit_rigid(a, b, with_scale ? &scale : nullptr);
  e.scale = scale;
  double s2 = 0.0, su = 0.0, sv = 0.0;
  for (size_t i = 0; i < a.size(); ++i) {
    const LineCameraT<double>& c = estimate[size_t(line[i])];
    const Vec3d y = world_to_camera(c, scale * (X.R * a[i]) + X.t);
    const double du = c.f * y.x / y.z + c.cu - px[i];
    const double dv = c.f * y.y / y.z - c.v_slit;
    su += du * du;
    sv += dv * dv;
    s2 += du * du + dv * dv;
    e.max = std::max(e.max, std::sqrt(du * du + dv * dv));
  }
  const double n = double(a.size());
  e.rms = std::sqrt(s2 / n);
  e.rms_along = std::sqrt(su / n);
  e.rms_across = std::sqrt(sv / n);
  return e;
}

}  // namespace linesplat
