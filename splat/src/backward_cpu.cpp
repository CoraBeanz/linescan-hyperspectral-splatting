#include "linesplat/backward_cpu.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#ifdef _OPENMP
#include <omp.h>
#endif

namespace linesplat {

template <typename T>
void SceneGradT<T>::reset(const GaussianScene& s) {
  const size_t n = size_t(s.size()), k = size_t(s.num_features);
  means.assign(3 * n, T(0));
  log_scales.assign(3 * n, T(0));
  rotations.assign(4 * n, T(0));
  opacity_logits.assign(n, T(0));
  features.assign(n * k, T(0));
  background.assign(k, T(0));
  screen_grad.assign(n, T(0));
  pairs.assign(n, 0);
}

namespace {

// One thread's sums, before the covariance and opacity gradients are pushed
// through to the stored parameters.
template <typename T>
struct Partial {
  std::vector<T> mean, cov, opacity, features, background, screen;
  std::vector<int> pairs;
  double loss = 0.0;
  void init(size_t n, size_t k) {
    mean.assign(3 * n, T(0));
    cov.assign(6 * n, T(0));
    opacity.assign(n, T(0));
    features.assign(n * k, T(0));
    background.assign(k, T(0));
    screen.assign(n, T(0));
    pairs.assign(n, 0);
  }
};

int max_threads() {
#ifdef _OPENMP
  return omp_get_max_threads();
#else
  return 1;
#endif
}

int thread_id() {
#ifdef _OPENMP
  return omp_get_thread_num();
#else
  return 0;
#endif
}

}  // namespace

template <typename T>
double render_backward_cpu(const GaussianScene& scene, const std::vector<LineCameraT<T>>& cams,
                           const LineLossFn<T>& loss, SceneGradT<T>* grad, std::vector<CameraGradT<T>>* cam_grad,
                           std::vector<T>* bands_out) {
  scene.validate();
  const int L = int(cams.size()), N = scene.size(), K = scene.num_features, B = scene.num_bands();
  const int W = cams.empty() ? 0 : cams[0].width;
  for (const auto& c : cams)
    if (c.width != W) throw std::runtime_error("render_backward_cpu: all cameras need the same width");
  grad->reset(scene);
  if (cam_grad) cam_grad->assign(size_t(L), CameraGradT<T>{});
  if (bands_out) bands_out->assign(size_t(L) * W * B, T(0));

  std::vector<GaussianGeomT<T>> geom(static_cast<size_t>(N));
  std::vector<T> basis(scene.basis.begin(), scene.basis.end());
  std::vector<T> bg(scene.background.begin(), scene.background.end());

  // Each thread sums into its own copy of the gradients, so every thread
  // costs zeroing and adding up a copy the size of the scene, about as much
  // as projecting the scene for one line. The threads share that work out,
  // and each gets at least four lines so it stays a small part.
  const int threads = std::max(1, std::min(max_threads(), (L + 3) / 4));
  std::vector<Partial<T>> part(static_cast<size_t>(threads));

#pragma omp parallel num_threads(threads)
  {
    Partial<T>& A = part[size_t(thread_id())];
    A.init(size_t(N), size_t(K));

#pragma omp for schedule(static)
    for (int i = 0; i < N; ++i) geom[size_t(i)] = gaussian_geometry<T>(scene, i);

#pragma omp for schedule(dynamic, 2)
    for (int l = 0; l < L; ++l) {
      const LineCameraT<T>& cam = cams[size_t(l)];

      // Project and sort front to back (stable, like the renderers).
      std::vector<std::pair<LineSplatT<T>, int>> hits;
      LineSplatT<T> sp;
      for (int i = 0; i < N; ++i)
        if (project_to_line(cam, geom[size_t(i)], &sp)) hits.emplace_back(sp, i);
      std::stable_sort(hits.begin(), hits.end(),
                       [](const std::pair<LineSplatT<T>, int>& a, const std::pair<LineSplatT<T>, int>& b) {
                         return a.first.depth < b.first.depth;
                       });
      const int H = int(hits.size());
      std::vector<int> start(size_t(W) + 1, 0);
      for (const auto& h : hits)
        for (int p = h.first.p0; p < h.first.p1; ++p) ++start[size_t(p) + 1];
      for (int p = 0; p < W; ++p) start[size_t(p) + 1] += start[size_t(p)];
      std::vector<int> fill(start.begin(), start.end() - 1);
      std::vector<int> bucket(static_cast<size_t>(start[size_t(W)]));
      for (int h = 0; h < H; ++h)
        for (int p = hits[size_t(h)].first.p0; p < hits[size_t(h)].first.p1; ++p)
          bucket[size_t(fill[size_t(p)]++)] = h;

      // Forward: features per pixel, the final transmittance, and where each
      // pixel stopped in its list.
      std::vector<T> feat(size_t(W) * K), trans_final(static_cast<size_t>(W));
      std::vector<int> stop(static_cast<size_t>(W));
      for (int p = 0; p < W; ++p) {
        T* acc = &feat[size_t(p) * K];
        std::fill(acc, acc + K, T(0));
        T trans = T(1);
        int b = start[size_t(p)];
        for (; b < start[size_t(p) + 1]; ++b) {
          const auto& h = hits[size_t(bucket[size_t(b)])];
          const T a = splat_alpha_at(h.first.u, h.first.inv_var, h.first.alpha, p);
          if (a < T(kMinAlpha)) continue;
          const T next = trans * (T(1) - a);
          if (next < T(kMinTransmittance)) break;
          const float* f = &scene.features[size_t(h.second) * K];
          const T w = a * trans;
          for (int c = 0; c < K; ++c) acc[c] += T(f[c]) * w;
          trans = next;
        }
        for (int c = 0; c < K; ++c) acc[c] += trans * bg[size_t(c)];
        trans_final[size_t(p)] = trans;
        stop[size_t(p)] = b;
      }

      // Bands, the loss, and dL/dfeatures = basis^T dL/dbands.
      std::vector<T> bands(size_t(W) * B, T(0)), g_bands(size_t(W) * B, T(0)), g_feat(size_t(W) * K, T(0));
      for (int p = 0; p < W; ++p)
        for (int b = 0; b < B; ++b) {
          T s = T(0);
          for (int c = 0; c < K; ++c) s += basis[size_t(b) * K + c] * feat[size_t(p) * K + c];
          bands[size_t(p) * B + b] = s;
        }
      A.loss += loss(l, bands.data(), g_bands.data());
      if (bands_out) std::copy(bands.begin(), bands.end(), bands_out->begin() + std::ptrdiff_t(size_t(l) * W * B));
      for (int p = 0; p < W; ++p)
        for (int b = 0; b < B; ++b) {
          const T g = g_bands[size_t(p) * B + b];
          if (g == T(0)) continue;
          for (int c = 0; c < K; ++c) g_feat[size_t(p) * K + c] += basis[size_t(b) * K + c] * g;
        }

      // Backward per pixel, back to front.
      std::vector<T> g_u(size_t(H), T(0)), g_iv(size_t(H), T(0)), g_al(size_t(H), T(0));
      std::vector<char> drew(size_t(H), 0);
      std::vector<T> behind(static_cast<size_t>(K));
      for (int p = 0; p < W; ++p) {
        const T* gC = &g_feat[size_t(p) * K];
        T tr = trans_final[size_t(p)];
        for (int c = 0; c < K; ++c) {
          A.background[size_t(c)] += tr * gC[c];
          behind[size_t(c)] = bg[size_t(c)];
        }
        for (int b = stop[size_t(p)] - 1; b >= start[size_t(p)]; --b) {
          const int h = bucket[size_t(b)];
          const LineSplatT<T>& s = hits[size_t(h)].first;
          const int gid = hits[size_t(h)].second;
          const T a = splat_alpha_at(s.u, s.inv_var, s.alpha, p);
          if (a < T(kMinAlpha)) continue;
          const T t_i = tr / (T(1) - a);
          const float* f = &scene.features[size_t(gid) * K];
          T* gf = &A.features[size_t(gid) * K];
          T g_a = T(0);
          for (int c = 0; c < K; ++c) {
            gf[c] += a * t_i * gC[c];
            g_a += gC[c] * (T(f[c]) - behind[size_t(c)]);
            behind[size_t(c)] = a * T(f[c]) + (T(1) - a) * behind[size_t(c)];
          }
          g_a *= t_i;
          tr = t_i;
          splat_alpha_backward(s.u, s.inv_var, s.alpha, p, g_a, &g_u[size_t(h)], &g_iv[size_t(h)], &g_al[size_t(h)]);
          drew[size_t(h)] = 1;
        }
      }

      // Through the projection, into the Gaussians and this line's camera.
      CameraGradT<T> cg{};
      for (int h = 0; h < H; ++h) {
        if (!drew[size_t(h)]) continue;
        const int gid = hits[size_t(h)].second;
        ProjectionGradT<T> pg;
        project_to_line_backward(cam, geom[size_t(gid)], g_u[size_t(h)], g_iv[size_t(h)], g_al[size_t(h)], &pg);
        T* m = &A.mean[3 * size_t(gid)];
        m[0] += pg.mean.x;
        m[1] += pg.mean.y;
        m[2] += pg.mean.z;
        T* cv = &A.cov[6 * size_t(gid)];
        cv[0] += pg.cov.xx;
        cv[1] += pg.cov.xy;
        cv[2] += pg.cov.xz;
        cv[3] += pg.cov.yy;
        cv[4] += pg.cov.yz;
        cv[5] += pg.cov.zz;
        A.opacity[size_t(gid)] += pg.opacity;
        A.screen[size_t(gid)] += std::sqrt(pg.mu_u * pg.mu_u + pg.mu_v * pg.mu_v);
        A.pairs[size_t(gid)] += 1;
        for (int i = 0; i < 9; ++i) cg.R[i] += pg.R[i];
        for (int i = 0; i < 3; ++i) cg.t[i] += pg.t[i];
      }
      if (cam_grad) (*cam_grad)[size_t(l)] = cg;
    }

    // Sum the threads' copies, each thread taking a share of the Gaussians,
    // then go from covariance and opacity to the stored log scales,
    // quaternions and logits. A copy only has gradients for the Gaussians its
    // lines drew, which are the ones it counted pairs for.
#pragma omp for schedule(static)
    for (int i = 0; i < N; ++i) {
      T cov[6] = {T(0), T(0), T(0), T(0), T(0), T(0)}, opacity = T(0);
      for (const Partial<T>& P : part) {
        if (P.pairs.empty() || P.pairs[size_t(i)] == 0) continue;
        for (size_t j = 3 * size_t(i); j < 3 * size_t(i) + 3; ++j) grad->means[j] += P.mean[j];
        for (int j = 0; j < 6; ++j) cov[j] += P.cov[6 * size_t(i) + size_t(j)];
        opacity += P.opacity[size_t(i)];
        for (size_t j = size_t(i) * K; j < size_t(i + 1) * K; ++j) grad->features[j] += P.features[j];
        grad->screen_grad[size_t(i)] += P.screen[size_t(i)];
        grad->pairs[size_t(i)] += P.pairs[size_t(i)];
      }
      if (grad->pairs[size_t(i)] == 0) continue;
      const Sym3<T> gc{cov[0], cov[1], cov[2], cov[3], cov[4], cov[5]};
      gaussian_geometry_backward(&scene.log_scales[3 * size_t(i)], &scene.rotations[4 * size_t(i)],
                                 scene.opacity_logits[size_t(i)], gc, opacity, &grad->log_scales[3 * size_t(i)],
                                 &grad->rotations[4 * size_t(i)], &grad->opacity_logits[size_t(i)]);
    }
  }  // omp parallel

  double total = 0.0;
  for (const auto& A : part) {
    total += A.loss;
    for (size_t c = 0; c < A.background.size(); ++c) grad->background[c] += A.background[c];
  }
  return total;
}

template struct SceneGradT<float>;
template struct SceneGradT<double>;
template double render_backward_cpu(const GaussianScene&, const std::vector<LineCameraT<float>>&,
                                    const LineLossFn<float>&, SceneGradT<float>*, std::vector<CameraGradT<float>>*,
                                    std::vector<float>*);
template double render_backward_cpu(const GaussianScene&, const std::vector<LineCameraT<double>>&,
                                    const LineLossFn<double>&, SceneGradT<double>*,
                                    std::vector<CameraGradT<double>>*, std::vector<double>*);

}  // namespace linesplat
