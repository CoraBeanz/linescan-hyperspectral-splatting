// CudaSceneOptimizer: Adam and densification on the GPU, on the arrays the
// rasterizer already holds, so that a training step copies only cameras.
//
// It mirrors HostSceneOptimizer (trainer.cpp) operation for operation:
//   - Adam is one thread per value with the same float operations in the same
//     order, written with the _rn intrinsics so the compiler can't fuse a
//     multiply and an add into one FMA (which rounds once instead of twice);
//     a step gives the CPU's result to the last bit.
//   - Densification is the CPU's loop made parallel. The CPU walks the
//     Gaussians in order, densifies the first `room` that qualify, and lists
//     the kept ones and then the new ones; here prefix sums over per-Gaussian
//     flags give every Gaussian its rank among those that qualify and the
//     slots its survivors go to, so the new arrays come out in the same order.
//     Split offsets come from hash_normal, which both sides compute alike.
#include <thrust/reduce.h>
#include <thrust/scan.h>
#include <thrust/system/cuda/execution_policy.h>

#include <algorithm>
#include <stdexcept>

#include "cuda_common.cuh"
#include "linesplat/densify.hpp"
#include "rasterizer_impl.cuh"

namespace linesplat {

using namespace cudadetail;

namespace {

constexpr int kBlock = 256;

// One Adam step on n values; see adam_step in trainer.cpp.
__global__ void adam_kernel(int n, float* __restrict__ p, const float* __restrict__ g, float* __restrict__ m,
                            float* __restrict__ v, float step, float eps, float b1, float b2) {
  const int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i >= n) return;
  const float gi = g[i];
  const float mi = __fadd_rn(__fmul_rn(b1, m[i]), __fmul_rn(1.0f - b1, gi));
  const float vi = __fadd_rn(__fmul_rn(b2, v[i]), __fmul_rn(__fmul_rn(1.0f - b2, gi), gi));
  m[i] = mi;
  v[i] = vi;
  p[i] = __fadd_rn(p[i], -__fdiv_rn(__fmul_rn(step, mi), __fadd_rn(__fsqrt_rn(vi), eps)));
}

__global__ void accumulate_kernel(int n, double scale, const float* __restrict__ g_screen,
                                  const int* __restrict__ g_pairs, double* __restrict__ screen_sum,
                                  int* __restrict__ pair_count) {
  const int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i >= n) return;
  screen_sum[i] = __dadd_rn(screen_sum[i], __dmul_rn(double(g_screen[i]), scale));
  pair_count[i] += g_pairs[i];
}

__global__ void densify_wanted_kernel(int n, const double* __restrict__ screen_sum, const int* __restrict__ pair_count,
                                      DensifyThresholds th, uint32_t* __restrict__ wanted) {
  const int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i >= n) return;
  wanted[i] = densify_wanted(screen_sum[i], pair_count[i], th) ? 1u : 0u;
}

// Per Gaussian, given its rank among those that qualify: whether it stays
// (it isn't split and survives the pruning), how many new Gaussians it makes
// (2 if it splits, 1 if it clones) and how many of those survive.
__global__ void densify_plan_kernel(int n, long long room, const uint32_t* __restrict__ wanted,
                                    const uint32_t* __restrict__ rank, const float* __restrict__ log_scales,
                                    const float* __restrict__ logits, DensifyThresholds th,
                                    uint32_t* __restrict__ old_kept, uint32_t* __restrict__ born_all,
                                    uint32_t* __restrict__ born_kept) {
  const int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i >= n) return;
  const float* ls = log_scales + 3 * size_t(i);
  const bool dens = wanted[i] && (long long)rank[i] < room;
  const bool split = dens && densify_splits(ls, th);
  old_kept[i] = !split && survives(logits[i], ls, th) ? 1u : 0u;
  uint32_t born = 0, kept = 0;
  if (split) {
    float half_ls[3];
    for (int a = 0; a < 3; ++a) half_ls[a] = ls[a] - th.shrink;
    born = 2;
    kept = survives(logits[i], half_ls, th) ? 2u : 0u;
  } else if (dens) {
    born = 1;
    kept = survives(logits[i], ls, th) ? 1u : 0u;
  }
  born_all[i] = born;
  born_kept[i] = kept;
}

// The arrays densification rebuilds: a parameter array of `dim` values per
// Gaussian and its two Adam moments.
struct Rebuilt {
  const float* p;
  const float* m;
  const float* v;
  float* np;
  float* nm;
  float* nv;
  int dim;
};
constexpr int kRebuilt = 5;  // means, log scales, rotations, logits, features
struct RebuiltSet {
  Rebuilt a[kRebuilt];
};

// Moves the kept Gaussians (with their moments) and writes the new ones
// (with zero moments) into the new arrays.
__global__ void densify_write_kernel(int n, uint32_t n_old, const uint32_t* __restrict__ old_kept,
                                     const uint32_t* __restrict__ old_pos, const uint32_t* __restrict__ born_all,
                                     const uint32_t* __restrict__ born_kept, const uint32_t* __restrict__ born_pos,
                                     DensifyThresholds th, RebuiltSet r) {
  const int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i >= n) return;
  if (old_kept[i]) {
    const size_t dst = old_pos[i];
    for (int a = 0; a < kRebuilt; ++a) {
      const Rebuilt& x = r.a[a];
      for (int d = 0; d < x.dim; ++d) {
        x.np[dst * x.dim + d] = x.p[size_t(i) * x.dim + d];
        x.nm[dst * x.dim + d] = x.m[size_t(i) * x.dim + d];
        x.nv[dst * x.dim + d] = x.v[size_t(i) * x.dim + d];
      }
    }
  }
  const uint32_t kept = born_kept[i];
  const bool split = born_all[i] == 2;
  const float* mean = r.a[0].p + 3 * size_t(i);
  const float* ls = r.a[1].p + 3 * size_t(i);
  const float* quat = r.a[2].p + 4 * size_t(i);
  for (uint32_t j = 0; j < kept; ++j) {
    const size_t dst = size_t(n_old) + born_pos[i] + j;
    float half_mean[3], half_ls[3];
    if (split) split_half(mean, ls, quat, uint64_t(i), int(j), th, half_mean, half_ls);
    for (int a = 0; a < kRebuilt; ++a) {
      const Rebuilt& x = r.a[a];
      for (int d = 0; d < x.dim; ++d) {
        float val = x.p[size_t(i) * x.dim + d];
        if (split && a == 0) val = half_mean[d];
        if (split && a == 1) val = half_ls[d];
        x.np[dst * x.dim + d] = val;
        x.nm[dst * x.dim + d] = 0.0f;
        x.nv[dst * x.dim + d] = 0.0f;
      }
    }
  }
}

// Copies a [n, k] host array into a [n, kp] device one, padding with zeros.
void upload_padded(DeviceBuffer<float>& dst, const std::vector<float>& src, int n, int k, int kp) {
  std::vector<float> padded(std::max(size_t(n) * kp, size_t(1)), 0.0f);
  for (int i = 0; i < n; ++i) std::copy(&src[size_t(i) * k], &src[size_t(i) * k] + k, &padded[size_t(i) * kp]);
  dst.upload(padded.data(), size_t(n) * kp);
}

}  // namespace

struct CudaSceneOptimizer::State {
  // Adam's moments for each parameter array, padded like the scene.
  DeviceBuffer<float> m_means, v_means, m_scales, v_scales, m_rots, v_rots, m_logits, v_logits;
  DeviceBuffer<float> m_feat, v_feat, m_bg, v_bg, m_basis, v_basis;
  // Densification statistics.
  DeviceBuffer<double> screen_sum;
  DeviceBuffer<int> pair_count;
  size_t densify_peak = 0;

  size_t adam_bytes() const {
    return m_means.bytes() + v_means.bytes() + m_scales.bytes() + v_scales.bytes() + m_rots.bytes() +
           v_rots.bytes() + m_logits.bytes() + v_logits.bytes() + m_feat.bytes() + v_feat.bytes() + m_bg.bytes() +
           v_bg.bytes() + m_basis.bytes() + v_basis.bytes();
  }
};

CudaSceneOptimizer::CudaSceneOptimizer(const Dataset& data, const GaussianScene& init) : st_(new State) {
  data.validate();
  rast_.set_targets(data.lines.data(), data.num_lines(), data.width(), data.num_bands());
  rast_.set_scene(init);
  const CudaRasterizer::Impl& m = *rast_.impl_;
  const size_t n = size_t(m.n), kp = size_t(m.kp);
  State& s = *st_;
  zero(s.m_means, 3 * n);
  zero(s.v_means, 3 * n);
  zero(s.m_scales, 3 * n);
  zero(s.v_scales, 3 * n);
  zero(s.m_rots, 4 * n);
  zero(s.v_rots, 4 * n);
  zero(s.m_logits, n);
  zero(s.v_logits, n);
  zero(s.m_feat, n * kp);
  zero(s.v_feat, n * kp);
  zero(s.m_bg, kp);
  zero(s.v_bg, kp);
  zero(s.m_basis, size_t(m.bands) * m.k);
  zero(s.v_basis, size_t(m.bands) * m.k);
  zero(s.screen_sum, n);
  zero(s.pair_count, n);
}

CudaSceneOptimizer::~CudaSceneOptimizer() = default;

int CudaSceneOptimizer::size() const { return rast_.impl_->n; }

const GaussianScene& CudaSceneOptimizer::scene() const {
  if (!cache_ok_) {
    rast_.impl_->download_scene(&cache_);
    cache_ok_ = true;
  }
  return cache_;
}

double CudaSceneOptimizer::backward(const std::vector<LineCamera>& cams, const std::vector<int>& lines,
                                    std::vector<CameraGradT<float>>* cam_grad) {
  CudaRenderStats s;
  const double loss =
      rast_.impl_->backward(cams, lines, learn_basis_, cam_grad, rast_.max_pairs_per_batch, rast_.profile, &s);
  stats_ += s;
  return loss;
}

void CudaSceneOptimizer::set_gradient(const SceneGradT<float>& g) {
  CudaRasterizer::Impl& m = *rast_.impl_;
  const size_t n = size_t(m.n);
  if (g.means.size() != 3 * n || g.log_scales.size() != 3 * n || g.rotations.size() != 4 * n ||
      g.opacity_logits.size() != n || g.features.size() != n * m.k || g.background.size() != size_t(m.k) ||
      g.screen_grad.size() != n || g.pairs.size() != n)
    throw std::runtime_error("CudaSceneOptimizer::set_gradient: wrong sizes");
  m.g_mean.upload(g.means.data(), g.means.size());
  m.g_log_scales.upload(g.log_scales.data(), g.log_scales.size());
  m.g_rotations.upload(g.rotations.data(), g.rotations.size());
  m.g_logits.upload(g.opacity_logits.data(), g.opacity_logits.size());
  upload_padded(m.g_feat, g.features, m.n, m.k, m.kp);
  upload_padded(m.g_bg, g.background, 1, m.k, m.kp);
  if (!g.basis.empty()) {
    if (g.basis.size() != size_t(m.bands) * m.k) throw std::runtime_error("CudaSceneOptimizer::set_gradient: basis");
    m.g_basis.upload(g.basis.data(), g.basis.size());
  }
  m.g_screen.upload(g.screen_grad.data(), n);
  m.g_pairs.upload(g.pairs.data(), n);
}

void CudaSceneOptimizer::adam(const SceneRates& lr, int t) {
  CudaRasterizer::Impl& m = *rast_.impl_;
  State& s = *st_;
  PassClock clock(rast_.profile);
  clock.start();
  const float b1 = 0.9f, b2 = 0.999f;  // as float(kBeta1), float(kBeta2) in trainer.cpp
  auto step = [&](DeviceBuffer<float>& p, DeviceBuffer<float>& g, DeviceBuffer<float>& mo, DeviceBuffer<float>& vo,
                  size_t count, double rate) {
    if (count == 0) return;
    float sz, eps;
    adam_constants(rate, t, &sz, &eps);
    adam_kernel<<<blocks_for(count, kBlock), kBlock>>>(int(count), p.get(), g.get(), mo.get(), vo.get(), sz, eps, b1,
                                                       b2);
    LS_CUDA_CHECK(cudaGetLastError());
  };
  const size_t n = size_t(m.n), kp = size_t(m.kp);
  step(m.means, m.g_mean, s.m_means, s.v_means, 3 * n, lr.means);
  step(m.log_scales, m.g_log_scales, s.m_scales, s.v_scales, 3 * n, lr.log_scales);
  step(m.rotations, m.g_rotations, s.m_rots, s.v_rots, 4 * n, lr.rotations);
  step(m.logits, m.g_logits, s.m_logits, s.v_logits, n, lr.opacity);
  step(m.features, m.g_feat, s.m_feat, s.v_feat, n * kp, lr.features);
  step(m.background, m.g_bg, s.m_bg, s.v_bg, kp, lr.background);
  if (learn_basis_) step(m.basis, m.g_basis, s.m_basis, s.v_basis, size_t(m.bands) * m.k, lr.basis);
  m.prepare();  // the covariances for the next step; waits for the GPU
  clock.mark(kPassAdam);
  clock.collect(stats_.pass_ms);
  cache_ok_ = false;
}

void CudaSceneOptimizer::accumulate(double scale) {
  CudaRasterizer::Impl& m = *rast_.impl_;
  if (m.n == 0) return;
  PassClock clock(rast_.profile);
  clock.start();
  accumulate_kernel<<<blocks_for(size_t(m.n), kBlock), kBlock>>>(m.n, scale, m.g_screen.get(), m.g_pairs.get(),
                                                                 st_->screen_sum.get(), st_->pair_count.get());
  LS_CUDA_CHECK(cudaGetLastError());
  LS_CUDA_CHECK(cudaDeviceSynchronize());
  clock.mark(kPassAdam);
  clock.collect(stats_.pass_ms);
}

DensifyCounts CudaSceneOptimizer::densify(const DensifyParams& p) {
  CudaRasterizer::Impl& m = *rast_.impl_;
  State& s = *st_;
  const int n = m.n;
  DensifyCounts c;
  if (n == 0) return c;
  PassClock clock(rast_.profile);
  clock.start();
  const DensifyThresholds th = densify_thresholds(p);
  const unsigned grid = blocks_for(size_t(n), kBlock);
  DeviceBuffer<uint32_t> wanted, rank, old_kept, old_pos, born_all, born_kept, born_pos;
  for (DeviceBuffer<uint32_t>* b : {&wanted, &rank, &old_kept, &old_pos, &born_all, &born_kept, &born_pos})
    b->reserve(size_t(n));

  densify_wanted_kernel<<<grid, kBlock>>>(n, s.screen_sum.get(), s.pair_count.get(), th, wanted.get());
  LS_CUDA_CHECK(cudaGetLastError());
  thrust::exclusive_scan(thrust::cuda::par(m.scratch), wanted.get(), wanted.get() + n, rank.get(), 0u);
  densify_plan_kernel<<<grid, kBlock>>>(n, (long long)p.max_gaussians - n, wanted.get(), rank.get(),
                                        m.log_scales.get(), m.logits.get(), th, old_kept.get(), born_all.get(),
                                        born_kept.get());
  LS_CUDA_CHECK(cudaGetLastError());
  thrust::exclusive_scan(thrust::cuda::par(m.scratch), old_kept.get(), old_kept.get() + n, old_pos.get(), 0u);
  thrust::exclusive_scan(thrust::cuda::par(m.scratch), born_kept.get(), born_kept.get() + n, born_pos.get(), 0u);
  const uint32_t added = thrust::reduce(thrust::cuda::par(m.scratch), born_all.get(), born_all.get() + n, 0u);
  uint32_t last[4];  // the scans' last entries and the flags they leave out
  LS_CUDA_CHECK(cudaMemcpy(&last[0], old_pos.get() + n - 1, 4, cudaMemcpyDeviceToHost));
  LS_CUDA_CHECK(cudaMemcpy(&last[1], old_kept.get() + n - 1, 4, cudaMemcpyDeviceToHost));
  LS_CUDA_CHECK(cudaMemcpy(&last[2], born_pos.get() + n - 1, 4, cudaMemcpyDeviceToHost));
  LS_CUDA_CHECK(cudaMemcpy(&last[3], born_kept.get() + n - 1, 4, cudaMemcpyDeviceToHost));
  const uint32_t n_old = last[0] + last[1], n_born = last[2] + last[3];
  const int n_new = int(n_old + n_born);

  // The new arrays, written in one pass, then swapped in.
  const int kp = m.kp;
  DeviceBuffer<float> np[kRebuilt], nm[kRebuilt], nv[kRebuilt];
  DeviceBuffer<float>* params[kRebuilt] = {&m.means, &m.log_scales, &m.rotations, &m.logits, &m.features};
  DeviceBuffer<float>* ms[kRebuilt] = {&s.m_means, &s.m_scales, &s.m_rots, &s.m_logits, &s.m_feat};
  DeviceBuffer<float>* vs[kRebuilt] = {&s.v_means, &s.v_scales, &s.v_rots, &s.v_logits, &s.v_feat};
  const int dims[kRebuilt] = {3, 3, 4, 1, kp};
  RebuiltSet r;
  size_t bytes = 7 * size_t(n) * sizeof(uint32_t);
  for (int a = 0; a < kRebuilt; ++a) {
    const size_t count = size_t(std::max(n_new, 1)) * dims[a];
    np[a].reserve(count);
    nm[a].reserve(count);
    nv[a].reserve(count);
    bytes += np[a].bytes() + nm[a].bytes() + nv[a].bytes();
    r.a[a] = Rebuilt{params[a]->get(), ms[a]->get(), vs[a]->get(), np[a].get(), nm[a].get(), nv[a].get(), dims[a]};
  }
  s.densify_peak = std::max(s.densify_peak, bytes);
  densify_write_kernel<<<grid, kBlock>>>(n, n_old, old_kept.get(), old_pos.get(), born_all.get(), born_kept.get(),
                                         born_pos.get(), th, r);
  LS_CUDA_CHECK(cudaGetLastError());
  LS_CUDA_CHECK(cudaDeviceSynchronize());
  for (int a = 0; a < kRebuilt; ++a) {
    params[a]->swap(np[a]);
    ms[a]->swap(nm[a]);
    vs[a]->swap(nv[a]);
  }  // the old arrays are freed with np, nm and nv

  m.n = n_new;
  m.prepare();
  zero(s.screen_sum, size_t(n_new));
  zero(s.pair_count, size_t(n_new));
  clock.mark(kPassDensify);
  clock.collect(stats_.pass_ms);
  cache_ok_ = false;
  c.added = int(added);
  c.removed = n + c.added - n_new;
  return c;
}

CudaMemoryUse CudaSceneOptimizer::memory_use() const {
  CudaMemoryUse u = rast_.memory_use();
  u.adam = st_->adam_bytes();
  u.gradients += st_->screen_sum.bytes() + st_->pair_count.bytes();
  u.densify = st_->densify_peak;
  return u;
}

}  // namespace linesplat
