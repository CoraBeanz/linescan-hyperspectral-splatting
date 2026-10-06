// CUDA line rasterizer. See cuda_rasterizer.hpp for the passes.
//
// Kept to what CUDA 10.2 on the Jetson Nano (sm_53) supports: C++14, Thrust
// for scans and sorts, float atomics only, warp intrinsics with _sync.
#include "linesplat/cuda_rasterizer.hpp"

#include <cuda_runtime.h>
#include <thrust/execution_policy.h>
#include <thrust/reduce.h>
#include <thrust/scan.h>
#include <thrust/sort.h>
#include <thrust/transform_scan.h>

#include <algorithm>
#include <climits>
#include <cstdint>
#include <sstream>
#include <stdexcept>

#include "linesplat/gradients.hpp"
#include "linesplat/util.hpp"

namespace linesplat {

namespace {

void throw_cuda(cudaError_t e, const char* expr, const char* file, int line) {
  std::ostringstream ss;
  ss << "CUDA error " << cudaGetErrorName(e) << " (" << cudaGetErrorString(e) << ") in " << expr << " at " << file
     << ":" << line;
  throw std::runtime_error(ss.str());
}

#define LS_CUDA_CHECK(expr)                                         \
  do {                                                              \
    const cudaError_t err_ = (expr);                                \
    if (err_ != cudaSuccess) throw_cuda(err_, #expr, __FILE__, __LINE__); \
  } while (0)

constexpr int kTile = 32;          // pixels per tile: one warp, one lane per pixel
constexpr int kWarpsPerBlock = 4;  // tiles per raster block
constexpr int kProjectBlock = 256;

// A grow-only device array.
template <typename T>
class DeviceBuffer {
 public:
  DeviceBuffer() = default;
  DeviceBuffer(const DeviceBuffer&) = delete;
  DeviceBuffer& operator=(const DeviceBuffer&) = delete;
  ~DeviceBuffer() {
    if (ptr_) cudaFree(ptr_);
  }
  // Grows by at least 1.5x so a slowly rising size doesn't reallocate every
  // call. The old contents are dropped.
  void reserve(size_t n) {
    if (n <= cap_) return;
    const size_t want = std::max(n, cap_ + cap_ / 2);
    if (ptr_) cudaFree(ptr_);
    ptr_ = nullptr;
    cap_ = 0;
    LS_CUDA_CHECK(cudaMalloc(&ptr_, std::max<size_t>(want, 1) * sizeof(T)));
    cap_ = want;
  }
  void upload(const T* src, size_t n) {
    reserve(n);
    if (n) LS_CUDA_CHECK(cudaMemcpy(ptr_, src, n * sizeof(T), cudaMemcpyHostToDevice));
  }
  void download(T* dst, size_t n) const {
    if (n) LS_CUDA_CHECK(cudaMemcpy(dst, ptr_, n * sizeof(T), cudaMemcpyDeviceToHost));
  }
  T* get() const { return ptr_; }

 private:
  T* ptr_ = nullptr;
  size_t cap_ = 0;
};

unsigned blocks_for(size_t n, int per_block) { return unsigned((n + per_block - 1) / per_block); }

// Pixel range [p0, p1) packed into 32 bits; 0 means "doesn't touch the line".
__host__ __device__ inline uint32_t pack_range(int p0, int p1) { return uint32_t(p0) | (uint32_t(p1) << 16); }
__host__ __device__ inline int range_p0(uint32_t r) { return int(r & 0xffffu); }
__host__ __device__ inline int range_p1(uint32_t r) { return int(r >> 16); }

struct TilesOf {
  __host__ __device__ uint32_t operator()(uint32_t r) const {
    return r ? uint32_t((range_p1(r) - 1) / kTile - range_p0(r) / kTile + 1) : 0u;
  }
};
struct IsVisible {
  __host__ __device__ uint32_t operator()(uint32_t r) const { return r ? 1u : 0u; }
};
// thrust::plus is deprecated in CUDA 13 and cuda::std::plus doesn't exist in
// CUDA 10.2, so add with our own functor.
struct AddU32 {
  __host__ __device__ uint32_t operator()(uint32_t a, uint32_t b) const { return a + b; }
};
struct AddF64 {
  __host__ __device__ double operator()(double a, double b) const { return a + b; }
};

// Per Gaussian: (mean, opacity), (cov xx, xy, xz, yy), (cov yz, zz, max scale, 0).
__global__ void prepare_kernel(int n, const float* __restrict__ means, const float* __restrict__ log_scales,
                               const float* __restrict__ rotations, const float* __restrict__ logits,
                               float4* __restrict__ geom) {
  const int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i >= n) return;
  const GaussianGeomT<float> g =
      gaussian_geometry<float>(means + 3 * i, log_scales + 3 * i, rotations + 4 * i, logits[i]);
  geom[3 * i] = make_float4(g.mean.x, g.mean.y, g.mean.z, g.opacity);
  geom[3 * i + 1] = make_float4(g.cov.xx, g.cov.xy, g.cov.xz, g.cov.yy);
  geom[3 * i + 2] = make_float4(g.cov.yz, g.cov.zz, g.max_scale, 0.0f);
}

__device__ __forceinline__ GaussianGeomT<float> load_geom(const float4* __restrict__ geom, int i) {
  const float4 a = geom[3 * i], b = geom[3 * i + 1], c = geom[3 * i + 2];
  GaussianGeomT<float> g;
  g.mean = Vec3<float>{a.x, a.y, a.z};
  g.opacity = a.w;
  g.cov.xx = b.x; g.cov.xy = b.y; g.cov.xz = b.z; g.cov.yy = b.w;
  g.cov.yz = c.x; g.cov.zz = c.y;
  g.max_scale = c.z;
  return g;
}

// Pass 1: the pixel range of every (line, Gaussian) pair, or 0.
__global__ void count_kernel(int n, const float4* __restrict__ geom, const LineCamera* __restrict__ cams,
                             uint32_t* __restrict__ packed) {
  __shared__ LineCamera cam;
  if (threadIdx.x == 0) cam = cams[blockIdx.y];
  __syncthreads();
  const int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i >= n) return;
  LineSplatT<float> s;
  packed[size_t(blockIdx.y) * n + i] = project_to_line(cam, load_geom(geom, i), &s) ? pack_range(s.p0, s.p1) : 0u;
}

// Pass 3: the splats and their (line, tile, depth) keys. The pixel ranges
// come from pass 1, so the number of keys always matches the scan.
__global__ void emit_kernel(int n, int n_tiles, const float4* __restrict__ geom, const LineCamera* __restrict__ cams,
                            const uint32_t* __restrict__ packed, const uint32_t* __restrict__ vis_off,
                            const uint32_t* __restrict__ tile_off, float4* __restrict__ splats,
                            int* __restrict__ splat_gid, int* __restrict__ splat_line,
                            unsigned long long* __restrict__ keys, uint32_t* __restrict__ values) {
  __shared__ LineCamera cam;
  if (threadIdx.x == 0) cam = cams[blockIdx.y];
  __syncthreads();
  const int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i >= n) return;
  const size_t k = size_t(blockIdx.y) * n + i;
  const uint32_t r = packed[k];
  if (!r) return;
  LineSplatT<float> s;
  if (!project_to_line(cam, load_geom(geom, i), &s)) s.alpha = 0.0f;  // can't happen; stay safe if it does
  const uint32_t sid = vis_off[k];
  splats[sid] = make_float4(s.u, s.inv_var, s.alpha, __uint_as_float(r));
  splat_gid[sid] = i;
  splat_line[sid] = int(blockIdx.y);
  const unsigned long long depth = __float_as_uint(s.depth);
  const unsigned long long cell0 = (unsigned long long)blockIdx.y * n_tiles;
  uint32_t off = tile_off[k];
  for (int t = range_p0(r) / kTile; t <= (range_p1(r) - 1) / kTile; ++t, ++off) {
    keys[off] = ((cell0 + t) << 32) | depth;
    values[off] = sid;
  }
}

// Pass 4b: [start, end) of each (line, tile) cell in the sorted keys.
__global__ void ranges_kernel(uint32_t n, const unsigned long long* __restrict__ keys, uint2* __restrict__ ranges) {
  const uint32_t i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i >= n) return;
  const uint32_t cell = uint32_t(keys[i] >> 32);
  if (i == 0) {
    ranges[cell].x = 0;
  } else {
    const uint32_t prev = uint32_t(keys[i - 1] >> 32);
    if (prev != cell) {
      ranges[prev].y = i;
      ranges[cell].x = i;
    }
  }
  if (i == n - 1) ranges[cell].y = n;
}

// Pass 5: one warp per (line, tile), C feature channels per launch row.
template <int C>
__global__ void __launch_bounds__(kTile* kWarpsPerBlock)
    raster_kernel(int n_cells, int n_tiles, int width, int k, int kp, const uint2* __restrict__ ranges,
                  const uint32_t* __restrict__ values, const float4* __restrict__ splats,
                  const int* __restrict__ splat_gid, const float* __restrict__ features,
                  const float* __restrict__ background, float* __restrict__ out, float* __restrict__ out_trans,
                  int* __restrict__ out_contrib) {
  __shared__ float4 s_splat[kWarpsPerBlock][kTile];
  __shared__ int s_gid[kWarpsPerBlock][kTile];
  __shared__ float s_feat[kWarpsPerBlock][kTile][C];

  const int lane = threadIdx.x, w = threadIdx.y;
  const int cell = blockIdx.x * kWarpsPerBlock + w;
  if (cell >= n_cells) return;  // the whole warp leaves together
  const int chunk = blockIdx.y;
  const int line = cell / n_tiles;
  const int p = (cell - line * n_tiles) * kTile + lane;
  const bool inside = p < width;
  const float pc = float(p) + 0.5f;
  const uint2 range = ranges[cell];

  float acc[C];
#pragma unroll
  for (int c = 0; c < C; ++c) acc[c] = 0.0f;
  float trans = 1.0f;
  bool done = !inside;
  uint32_t contributor = 0, last = 0;

  for (uint32_t base = range.x; base < range.y; base += kTile) {
    if (__all_sync(0xffffffffu, done)) break;
    const int n_batch = min(kTile, int(range.y - base));
    if (lane < n_batch) {
      const uint32_t sid = values[base + lane];
      s_splat[w][lane] = splats[sid];
      s_gid[w][lane] = splat_gid[sid];
    }
    __syncwarp();
    for (int e = lane; e < n_batch * C; e += kTile) {
      const int row = e / C, col = e - row * C;
      s_feat[w][row][col] = features[size_t(s_gid[w][row]) * kp + chunk * C + col];
    }
    __syncwarp();
    for (int j = 0; j < n_batch && !done; ++j) {
      ++contributor;
      const float4 s = s_splat[w][j];
      const uint32_t r = __float_as_uint(s.w);
      if (p < range_p0(r) || p >= range_p1(r)) continue;
      const float d = pc - s.x;
      const float a = fminf(kMaxAlpha, s.z * expf(-0.5f * s.y * d * d));
      if (a < kMinAlpha) continue;
      const float next = trans * (1.0f - a);
      if (next < kMinTransmittance) {
        done = true;
        continue;
      }
      const float wgt = a * trans;
#pragma unroll
      for (int c = 0; c < C; ++c) acc[c] += s_feat[w][j][c] * wgt;
      trans = next;
      last = contributor;
    }
    __syncwarp();
  }
  if (!inside) return;
  const size_t px = size_t(line) * width + p;
  // The output has the scene's k channels, not the padded kp.
  float* o = out + px * k + chunk * C;
  const int n_out = k - chunk * C;
#pragma unroll
  for (int c = 0; c < C; ++c)
    if (c < n_out) o[c] = acc[c] + trans * background[chunk * C + c];
  if (chunk == 0) {
    out_trans[px] = trans;
    out_contrib[px] = int(last);
  }
}

// ---- Backward ----

__device__ __forceinline__ float warp_sum(float v) {
#pragma unroll
  for (int o = kTile / 2; o > 0; o >>= 1) v += __shfl_xor_sync(0xffffffffu, v, o);
  return v;
}

__device__ __forceinline__ int warp_max(int v) {
#pragma unroll
  for (int o = kTile / 2; o > 0; o >>= 1) v = max(v, __shfl_xor_sync(0xffffffffu, v, o));
  return v;
}

// Backward pass 6a: one thread per (pixel, band): the band value (basis x
// features), its squared error, and dL/dband for L = scale * sum of squares.
__global__ void band_error_kernel(int n, int width, int k, int bands, float scale, const float* __restrict__ feat,
                                  const float* __restrict__ basis, const float* __restrict__ targets,
                                  const int* __restrict__ line_ids, float* __restrict__ sq_err,
                                  float* __restrict__ g_bands) {
  const int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i >= n) return;
  const int pix = i / bands, b = i - pix * bands;
  const int line = pix / width, p = pix - line * width;
  const float* f = feat + size_t(pix) * k;
  const float* row = basis + size_t(b) * k;
  float v = 0.0f;
  for (int c = 0; c < k; ++c) v += row[c] * f[c];
  const float d = v - targets[(size_t(line_ids[line]) * width + p) * bands + b];
  sq_err[i] = d * d;
  g_bands[i] = 2.0f * scale * d;
}

// Backward pass 6b: one thread per (pixel, padded feature channel):
// dL/dfeature = basis^T dL/dbands, and zero in the padding.
__global__ void feature_grad_kernel(int n, int k, int kp, int bands, const float* __restrict__ basis,
                                    const float* __restrict__ g_bands, float* __restrict__ g_pix) {
  const int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i >= n) return;
  const int pix = i / kp, c = i - pix * kp;
  float g = 0.0f;
  if (c < k) {
    const float* gb = g_bands + size_t(pix) * bands;
    for (int b = 0; b < bands; ++b) g += basis[size_t(b) * k + c] * gb[b];
  }
  g_pix[i] = g;
}

// Backward pass 7: one warp per (line, tile), C feature channels per launch
// row, mirroring raster_kernel. Each lane walks its pixel's list back to front
// from where the forward pass stopped, recovering the transmittance in front
// of each splat as T / (1 - alpha) and keeping the color behind it (see
// backward_cpu.hpp). Per splat, the warp sums its lanes' dL/d(u, inv_var,
// alpha) and dL/dfeatures, so each value costs one atomic per tile. Every
// feature chunk adds its own share of the alpha gradient, which is a sum
// over channels.
template <int C>
__global__ void __launch_bounds__(kTile* kWarpsPerBlock)
    raster_backward_kernel(int n_cells, int n_tiles, int width, int kp, const uint2* __restrict__ ranges,
                           const uint32_t* __restrict__ values, const float4* __restrict__ splats,
                           const int* __restrict__ splat_gid, const float* __restrict__ features,
                           const float* __restrict__ background, const float* __restrict__ trans_final,
                           const int* __restrict__ contrib, const float* __restrict__ g_pix,
                           float4* __restrict__ g_splat, float* __restrict__ g_features,
                           float* __restrict__ g_background) {
  __shared__ float4 s_splat[kWarpsPerBlock][kTile];
  __shared__ uint32_t s_sid[kWarpsPerBlock][kTile];
  __shared__ int s_gid[kWarpsPerBlock][kTile];
  __shared__ float s_feat[kWarpsPerBlock][kTile][C];

  const int lane = threadIdx.x, w = threadIdx.y;
  const int cell = blockIdx.x * kWarpsPerBlock + w;
  if (cell >= n_cells) return;  // the whole warp leaves together
  const int chunk = blockIdx.y;
  const int line = cell / n_tiles;
  const int p = (cell - line * n_tiles) * kTile + lane;
  const bool inside = p < width;
  const float pc = float(p) + 0.5f;
  const uint2 range = ranges[cell];
  const size_t px = size_t(line) * width + p;

  float trans = inside ? trans_final[px] : 0.0f;
  const int last = inside ? contrib[px] : 0;
  float g_pixel[C], behind[C];
#pragma unroll
  for (int c = 0; c < C; ++c) {
    g_pixel[c] = inside ? g_pix[px * kp + chunk * C + c] : 0.0f;
    behind[c] = background[chunk * C + c];
    // The background shows through whatever transmittance is left.
    const float gb = warp_sum(trans * g_pixel[c]);
    if (lane == 0 && gb != 0.0f) atomicAdd(&g_background[chunk * C + c], gb);
  }

  // The tile's list from the furthest any lane got, in batches of 32.
  for (int top = warp_max(last); top > 0; top -= kTile) {
    const int b0 = max(0, top - kTile), n_batch = top - b0;
    __syncwarp();
    if (lane < n_batch) {
      const uint32_t sid = values[range.x + b0 + lane];
      s_sid[w][lane] = sid;
      s_splat[w][lane] = splats[sid];
      s_gid[w][lane] = splat_gid[sid];
    }
    __syncwarp();
    for (int e = lane; e < n_batch * C; e += kTile) {
      const int row = e / C, col = e - row * C;
      s_feat[w][row][col] = features[size_t(s_gid[w][row]) * kp + chunk * C + col];
    }
    __syncwarp();
    for (int j = n_batch - 1; j >= 0; --j) {
      const float4 s = s_splat[w][j];
      const uint32_t r = __float_as_uint(s.w);
      float g_u = 0.0f, g_iv = 0.0f, g_al = 0.0f, g_f[C];
#pragma unroll
      for (int c = 0; c < C; ++c) g_f[c] = 0.0f;
      bool hit = false;
      if (b0 + j < last && p >= range_p0(r) && p < range_p1(r)) {
        // The same expression as raster_kernel, so the same splats count.
        const float d = pc - s.x;
        const float a = fminf(kMaxAlpha, s.z * expf(-0.5f * s.y * d * d));
        if (a >= kMinAlpha) {
          hit = true;
          const float t_i = trans / (1.0f - a);
          float g_a = 0.0f;
#pragma unroll
          for (int c = 0; c < C; ++c) {
            const float f = s_feat[w][j][c];
            g_f[c] = a * t_i * g_pixel[c];
            g_a += g_pixel[c] * (f - behind[c]);
            behind[c] = a * f + (1.0f - a) * behind[c];
          }
          trans = t_i;
          splat_alpha_backward(s.x, s.y, s.z, p, g_a * t_i, &g_u, &g_iv, &g_al);
        }
      }
      if (!__any_sync(0xffffffffu, hit)) continue;
      g_u = warp_sum(g_u);
      g_iv = warp_sum(g_iv);
      g_al = warp_sum(g_al);
#pragma unroll
      for (int c = 0; c < C; ++c) g_f[c] = warp_sum(g_f[c]);
      if (lane == 0) {
        float4* gs = &g_splat[s_sid[w][j]];
        atomicAdd(&gs->x, g_u);
        atomicAdd(&gs->y, g_iv);
        atomicAdd(&gs->z, g_al);
        gs->w = 1.0f;  // the pair drew a pixel
        float* gf = g_features + size_t(s_gid[w][j]) * kp + chunk * C;
#pragma unroll
        for (int c = 0; c < C; ++c)
          if (g_f[c] != 0.0f) atomicAdd(&gf[c], g_f[c]);
      }
    }
  }
}

// Backward pass 8: one thread per visible pair that drew a pixel, through
// the projection into the Gaussian (mean, covariance, opacity, and the
// densification statistics) and the line's camera. Consecutive pairs mostly
// belong to one line, so a warp sums the camera gradient first when it can.
__global__ void project_backward_kernel(uint32_t n_vis, int line0, const float4* __restrict__ geom,
                                        const LineCamera* __restrict__ cams, const float4* __restrict__ g_splat,
                                        const int* __restrict__ splat_gid, const int* __restrict__ splat_line,
                                        float* __restrict__ g_mean, float* __restrict__ g_cov,
                                        float* __restrict__ g_opacity, float* __restrict__ g_screen,
                                        int* __restrict__ g_pairs, float* __restrict__ g_cam) {
  const uint32_t sid = blockIdx.x * blockDim.x + threadIdx.x;
  const float4 gs = sid < n_vis ? g_splat[sid] : make_float4(0.0f, 0.0f, 0.0f, 0.0f);
  const bool drew = gs.w > 0.0f;
  const int line = drew ? splat_line[sid] : -1;
  float cg[12];
#pragma unroll
  for (int i = 0; i < 12; ++i) cg[i] = 0.0f;
  if (drew) {
    const int gid = splat_gid[sid];
    ProjectionGradT<float> pg;
    project_to_line_backward(cams[line], load_geom(geom, gid), gs.x, gs.y, gs.z, &pg);
    atomicAdd(&g_mean[3 * gid], pg.mean.x);
    atomicAdd(&g_mean[3 * gid + 1], pg.mean.y);
    atomicAdd(&g_mean[3 * gid + 2], pg.mean.z);
    float* c = g_cov + 6 * size_t(gid);
    atomicAdd(&c[0], pg.cov.xx);
    atomicAdd(&c[1], pg.cov.xy);
    atomicAdd(&c[2], pg.cov.xz);
    atomicAdd(&c[3], pg.cov.yy);
    atomicAdd(&c[4], pg.cov.yz);
    atomicAdd(&c[5], pg.cov.zz);
    atomicAdd(&g_opacity[gid], pg.opacity);
    atomicAdd(&g_screen[gid], sqrtf(pg.mu_u * pg.mu_u + pg.mu_v * pg.mu_v));
    atomicAdd(&g_pairs[gid], 1);
#pragma unroll
    for (int i = 0; i < 9; ++i) cg[i] = pg.R[i];
#pragma unroll
    for (int i = 0; i < 3; ++i) cg[9 + i] = pg.t[i];
  }
  const unsigned drawn = __ballot_sync(0xffffffffu, drew);
  if (!drawn) return;  // the whole warp
  const int leader = __ffs(drawn) - 1;
  const int lead_line = __shfl_sync(0xffffffffu, line, leader);
  if (__all_sync(0xffffffffu, !drew || line == lead_line)) {
    const int lane = int(threadIdx.x) & (kTile - 1);
#pragma unroll
    for (int i = 0; i < 12; ++i) {
      const float v = warp_sum(cg[i]);
      if (lane == leader) atomicAdd(&g_cam[12 * size_t(line0 + lead_line) + i], v);
    }
  } else if (drew) {
#pragma unroll
    for (int i = 0; i < 12; ++i) atomicAdd(&g_cam[12 * size_t(line0 + line) + i], cg[i]);
  }
}

// Last step: per Gaussian, from the covariance and opacity gradients to the
// stored log scales, quaternion and opacity logit.
__global__ void geometry_backward_kernel(int n, const float* __restrict__ log_scales,
                                         const float* __restrict__ rotations, const float* __restrict__ logits,
                                         const float* __restrict__ g_cov, const float* __restrict__ g_opacity,
                                         float* __restrict__ g_log_scales, float* __restrict__ g_rotations,
                                         float* __restrict__ g_logits) {
  const int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i >= n) return;
  const float* c = g_cov + 6 * size_t(i);
  const Sym3<float> gc{c[0], c[1], c[2], c[3], c[4], c[5]};
  gaussian_geometry_backward(log_scales + 3 * size_t(i), rotations + 4 * size_t(i), logits[i], gc, g_opacity[i],
                             g_log_scales + 3 * size_t(i), g_rotations + 4 * size_t(i), g_logits + i);
}

}  // namespace

bool cuda_device_available(std::string* why) {
  int n = 0;
  const cudaError_t e = cudaGetDeviceCount(&n);
  if (e != cudaSuccess || n == 0) {
    if (why) *why = e != cudaSuccess ? cudaGetErrorString(e) : "no CUDA device";
    cudaGetLastError();  // clear the sticky error
    return false;
  }
  return true;
}

std::string cuda_device_name() {
  int dev = 0;
  cudaDeviceProp prop;
  if (cudaGetDevice(&dev) != cudaSuccess || cudaGetDeviceProperties(&prop, dev) != cudaSuccess) return "unknown";
  std::ostringstream ss;
  ss << prop.name << " (sm_" << prop.major << prop.minor << ", " << (prop.totalGlobalMem >> 20) << " MB)";
  return ss.str();
}

namespace {

// CUDA events that time the GPU work of a call, batch by batch.
class GpuTimer {
 public:
  GpuTimer() {
    LS_CUDA_CHECK(cudaEventCreate(&start_));
    LS_CUDA_CHECK(cudaEventCreate(&stop_));
  }
  ~GpuTimer() {
    cudaEventDestroy(start_);
    cudaEventDestroy(stop_);
  }
  void start() { LS_CUDA_CHECK(cudaEventRecord(start_)); }
  void stop() {
    LS_CUDA_CHECK(cudaEventRecord(stop_));
    LS_CUDA_CHECK(cudaEventSynchronize(stop_));
    float ms = 0.0f;
    LS_CUDA_CHECK(cudaEventElapsedTime(&ms, start_, stop_));
    total_ms += ms;
  }
  double total_ms = 0.0;

 private:
  cudaEvent_t start_, stop_;
};

template <typename T>
void zero(DeviceBuffer<T>& b, size_t n) {
  b.reserve(n);
  if (n) LS_CUDA_CHECK(cudaMemset(b.get(), 0, n * sizeof(T)));
}

}  // namespace

struct CudaRasterizer::Impl {
  int n = 0;         // Gaussians
  int k = 0;         // features
  int chunk = 16;    // feature channels per raster launch row
  int kp = 0;        // features padded to a multiple of chunk
  int bands = 0;     // rows of the spectral basis
  bool has_scene = false;
  DeviceBuffer<float4> geom;
  DeviceBuffer<float> means, log_scales, rotations, logits;
  DeviceBuffer<float> features, background, basis;
  DeviceBuffer<LineCamera> cams;
  DeviceBuffer<uint32_t> packed, vis_off, tile_off, values;
  DeviceBuffer<unsigned long long> keys;
  DeviceBuffer<float4> splats;
  DeviceBuffer<int> splat_gid, splat_line, contrib;
  DeviceBuffer<uint2> ranges;
  DeviceBuffer<float> out, trans;

  // Training.
  int t_lines = 0, t_width = 0, t_bands = 0;
  DeviceBuffer<float> targets;
  DeviceBuffer<int> line_ids;
  DeviceBuffer<float> sq_err, g_bands, g_pix;
  DeviceBuffer<float4> g_splat;
  DeviceBuffer<float> g_mean, g_cov, g_opacity, g_feat, g_bg, g_screen, g_cam;
  DeviceBuffer<int> g_pairs;
  DeviceBuffer<float> g_log_scales, g_rotations, g_logits;

  // Lines per batch for width w: bounded by the pair budget, the grid's y
  // limit, and keeping every key count inside 32 bits.
  int batch_lines(long long max_pairs, int w) const {
    const long long n_tiles = (w + kTile - 1) / kTile;
    long long b = std::max(1LL, max_pairs / std::max(1, n));
    b = std::min<long long>(b, 65535);
    b = std::min<long long>(b, std::max(1LL, (1LL << 31) / (std::max(1LL, (long long)n) * n_tiles)));
    return int(b);
  }

  // Passes 1-5 for lb lines of width w, whose cameras are already in `cams`.
  // Leaves the sorted lists, splats, features (out), transmittance and
  // contributors on the GPU.
  void forward(int lb, int w, uint32_t* n_vis_out, uint32_t* n_entries_out);
};

void CudaRasterizer::Impl::forward(int lb, int w, uint32_t* n_vis_out, uint32_t* n_entries_out) {
  const int n_tiles = (w + kTile - 1) / kTile;
  const size_t pairs = size_t(lb) * n;
  const size_t n_cells = size_t(lb) * n_tiles;
  uint32_t n_entries = 0, n_vis = 0;

  if (n > 0) {
    packed.reserve(pairs);
    vis_off.reserve(pairs);
    tile_off.reserve(pairs);
    const dim3 grid(blocks_for(size_t(n), kProjectBlock), unsigned(lb));
    count_kernel<<<grid, kProjectBlock>>>(n, geom.get(), cams.get(), packed.get());
    LS_CUDA_CHECK(cudaGetLastError());
    thrust::transform_exclusive_scan(thrust::device, packed.get(), packed.get() + pairs, tile_off.get(), TilesOf(),
                                     0u, AddU32());
    thrust::transform_exclusive_scan(thrust::device, packed.get(), packed.get() + pairs, vis_off.get(), IsVisible(),
                                     0u, AddU32());
    uint32_t last_packed = 0, last_tile = 0, last_vis = 0;
    LS_CUDA_CHECK(cudaMemcpy(&last_packed, packed.get() + pairs - 1, 4, cudaMemcpyDeviceToHost));
    LS_CUDA_CHECK(cudaMemcpy(&last_tile, tile_off.get() + pairs - 1, 4, cudaMemcpyDeviceToHost));
    LS_CUDA_CHECK(cudaMemcpy(&last_vis, vis_off.get() + pairs - 1, 4, cudaMemcpyDeviceToHost));
    n_entries = last_tile + TilesOf()(last_packed);
    n_vis = last_vis + IsVisible()(last_packed);
    if (n_entries > 0) {
      splats.reserve(n_vis);
      splat_gid.reserve(n_vis);
      splat_line.reserve(n_vis);
      keys.reserve(n_entries);
      values.reserve(n_entries);
      emit_kernel<<<grid, kProjectBlock>>>(n, n_tiles, geom.get(), cams.get(), packed.get(), vis_off.get(),
                                           tile_off.get(), splats.get(), splat_gid.get(), splat_line.get(),
                                           keys.get(), values.get());
      LS_CUDA_CHECK(cudaGetLastError());
      thrust::sort_by_key(thrust::device, keys.get(), keys.get() + n_entries, values.get());
    }
  }

  ranges.reserve(n_cells);
  LS_CUDA_CHECK(cudaMemset(ranges.get(), 0, n_cells * sizeof(uint2)));
  if (n_entries > 0) {
    ranges_kernel<<<blocks_for(n_entries, 256), 256>>>(n_entries, keys.get(), ranges.get());
    LS_CUDA_CHECK(cudaGetLastError());
  }

  const size_t px = size_t(lb) * w;
  out.reserve(px * k);
  trans.reserve(px);
  contrib.reserve(px);
  const dim3 rgrid(blocks_for(n_cells, kWarpsPerBlock), unsigned(kp / chunk));
  const dim3 rblock(kTile, kWarpsPerBlock);
  if (chunk == 8) {
    raster_kernel<8><<<rgrid, rblock>>>(int(n_cells), n_tiles, w, k, kp, ranges.get(), values.get(), splats.get(),
                                        splat_gid.get(), features.get(), background.get(), out.get(), trans.get(),
                                        contrib.get());
  } else {
    raster_kernel<16><<<rgrid, rblock>>>(int(n_cells), n_tiles, w, k, kp, ranges.get(), values.get(), splats.get(),
                                         splat_gid.get(), features.get(), background.get(), out.get(), trans.get(),
                                         contrib.get());
  }
  LS_CUDA_CHECK(cudaGetLastError());
  *n_vis_out = n_vis;
  *n_entries_out = n_entries;
}

CudaRasterizer::CudaRasterizer() : impl_(new Impl) {}
CudaRasterizer::~CudaRasterizer() = default;

void CudaRasterizer::set_scene(const GaussianScene& scene) {
  scene.validate();
  Impl& m = *impl_;
  m.n = scene.size();
  m.k = scene.num_features;
  m.chunk = m.k <= 8 ? 8 : 16;
  m.kp = (m.k + m.chunk - 1) / m.chunk * m.chunk;
  m.bands = scene.num_bands();

  std::vector<float> feat(size_t(m.n) * m.kp, 0.0f), bg(size_t(m.kp), 0.0f);
  for (int i = 0; i < m.n; ++i)
    std::copy(&scene.features[size_t(i) * m.k], &scene.features[size_t(i) * m.k] + m.k, &feat[size_t(i) * m.kp]);
  std::copy(scene.background.begin(), scene.background.end(), bg.begin());
  m.features.upload(feat.data(), feat.size());
  m.background.upload(bg.data(), bg.size());
  m.basis.upload(scene.basis.data(), scene.basis.size());

  // The stored parameters stay on the GPU for the backward pass.
  m.geom.reserve(size_t(m.n) * 3);
  if (m.n > 0) {
    m.means.upload(scene.means.data(), scene.means.size());
    m.log_scales.upload(scene.log_scales.data(), scene.log_scales.size());
    m.rotations.upload(scene.rotations.data(), scene.rotations.size());
    m.logits.upload(scene.opacity_logits.data(), scene.opacity_logits.size());
    prepare_kernel<<<blocks_for(size_t(m.n), kProjectBlock), kProjectBlock>>>(
        m.n, m.means.get(), m.log_scales.get(), m.rotations.get(), m.logits.get(), m.geom.get());
    LS_CUDA_CHECK(cudaGetLastError());
    LS_CUDA_CHECK(cudaDeviceSynchronize());
  }
  m.has_scene = true;
}

LineImage CudaRasterizer::render(const std::vector<LineCamera>& cams, CudaRenderStats* stats) {
  LineImage img;
  render(cams, &img, stats);
  return img;
}

void CudaRasterizer::render(const std::vector<LineCamera>& cams, LineImage* out, CudaRenderStats* stats) {
  Impl& m = *impl_;
  if (!m.has_scene) throw std::runtime_error("CudaRasterizer::render: call set_scene first");
  Timer timer;
  LineImage& img = *out;
  img.lines = int(cams.size());
  img.width = cams.empty() ? 0 : cams[0].width;
  img.channels = m.k;
  for (const auto& c : cams)
    if (c.width != img.width) throw std::runtime_error("CudaRasterizer::render: all cameras need the same width");
  if (img.width > 0xffff) throw std::runtime_error("CudaRasterizer::render: lines are limited to 65535 pixels");
  const int L = img.lines, W = img.width, K = m.k;
  // Every value is overwritten below, so a reused image of the right size
  // isn't cleared first (clearing 80 MB costs more than rendering it).
  img.values.resize(size_t(L) * W * K);
  img.transmittance.resize(size_t(L) * W);
  img.contributors.resize(size_t(L) * W);
  CudaRenderStats st;
  if (L == 0 || W == 0) {
    if (stats) *stats = st;
    return;
  }
  GpuTimer gpu;
  const int batch = m.batch_lines(max_pairs_per_batch, W);
  for (int l0 = 0; l0 < L; l0 += batch) {
    const int lb = std::min(batch, L - l0);
    m.cams.upload(&cams[size_t(l0)], size_t(lb));
    gpu.start();
    uint32_t n_vis = 0, n_entries = 0;
    m.forward(lb, W, &n_vis, &n_entries);
    gpu.stop();

    const size_t px = size_t(lb) * W;
    m.out.download(&img.values[size_t(l0) * W * K], px * K);
    m.trans.download(&img.transmittance[size_t(l0) * W], px);
    m.contrib.download(&img.contributors[size_t(l0) * W], px);

    st.batches += 1;
    st.visible_pairs += n_vis;
    st.tile_entries += n_entries;
  }
  LS_CUDA_CHECK(cudaDeviceSynchronize());
  st.ms = timer.ms();
  st.gpu_ms = gpu.total_ms;
  if (stats) *stats = st;
}

void CudaRasterizer::set_targets(const float* lines, int num_lines, int width, int bands) {
  if (num_lines < 0 || width < 0 || bands < 0) throw std::runtime_error("CudaRasterizer::set_targets: bad shape");
  Impl& m = *impl_;
  m.targets.upload(lines, size_t(num_lines) * width * bands);
  m.t_lines = num_lines;
  m.t_width = width;
  m.t_bands = bands;
}

double CudaRasterizer::mse_backward(const std::vector<LineCamera>& cams, const std::vector<int>& lines,
                                    SceneGradT<float>* grad, std::vector<CameraGradT<float>>* cam_grad,
                                    CudaRenderStats* stats) {
  Impl& m = *impl_;
  if (!m.has_scene) throw std::runtime_error("CudaRasterizer::mse_backward: call set_scene first");
  if (lines.size() != cams.size())
    throw std::runtime_error("CudaRasterizer::mse_backward: need one measured line per camera");
  const int L = int(cams.size()), W = m.t_width, K = m.k, B = m.bands, n = m.n;
  if (B != m.t_bands)
    throw std::runtime_error("CudaRasterizer::mse_backward: the scene has " + std::to_string(B) +
                             " bands and the measured lines " + std::to_string(m.t_bands));
  for (const auto& c : cams)
    if (c.width != W) throw std::runtime_error("CudaRasterizer::mse_backward: cameras and measured lines differ in width");
  for (int l : lines)
    if (l < 0 || l >= m.t_lines) throw std::runtime_error("CudaRasterizer::mse_backward: no such measured line");
  if (W > 0xffff) throw std::runtime_error("CudaRasterizer::mse_backward: lines are limited to 65535 pixels");
  Timer timer;
  CudaRenderStats st;
  GpuTimer gpu;

  zero(m.g_mean, 3 * size_t(n));
  zero(m.g_cov, 6 * size_t(n));
  zero(m.g_opacity, size_t(n));
  zero(m.g_screen, size_t(n));
  zero(m.g_pairs, size_t(n));
  zero(m.g_feat, size_t(n) * m.kp);
  zero(m.g_bg, size_t(m.kp));
  zero(m.g_cam, 12 * size_t(L));
  double sum_sq = 0.0;
  const float scale = float(1.0 / (double(std::max(1, L)) * std::max(1, W) * std::max(1, B)));
  const int n_tiles = (W + kTile - 1) / kTile;
  // The loss kernels index (pixel, band) and (pixel, channel) with an int.
  const int batch = std::min(m.batch_lines(max_pairs_per_batch, W),
                             std::max(1, int(INT_MAX / (std::max(1, W) * std::max(B, m.kp)))));

  for (int l0 = 0; l0 < L && W > 0; l0 += batch) {
    const int lb = std::min(batch, L - l0);
    m.cams.upload(&cams[size_t(l0)], size_t(lb));
    m.line_ids.upload(&lines[size_t(l0)], size_t(lb));
    gpu.start();
    uint32_t n_vis = 0, n_entries = 0;
    m.forward(lb, W, &n_vis, &n_entries);

    // The loss and dL/dfeatures per pixel.
    const size_t px = size_t(lb) * W;
    const size_t n_pb = px * B, n_pk = px * m.kp;
    m.sq_err.reserve(n_pb);
    m.g_bands.reserve(n_pb);
    m.g_pix.reserve(n_pk);
    if (n_pb > 0) {
      band_error_kernel<<<blocks_for(n_pb, 256), 256>>>(int(n_pb), W, K, B, scale, m.out.get(), m.basis.get(),
                                                        m.targets.get(), m.line_ids.get(), m.sq_err.get(),
                                                        m.g_bands.get());
      LS_CUDA_CHECK(cudaGetLastError());
      sum_sq += thrust::reduce(thrust::device, m.sq_err.get(), m.sq_err.get() + n_pb, 0.0, AddF64());
    }
    feature_grad_kernel<<<blocks_for(n_pk, 256), 256>>>(int(n_pk), K, m.kp, B, m.basis.get(), m.g_bands.get(),
                                                        m.g_pix.get());
    LS_CUDA_CHECK(cudaGetLastError());

    // Back through the compositing, per (line, tile).
    zero(m.g_splat, n_vis);
    const size_t n_cells = size_t(lb) * n_tiles;
    const dim3 rgrid(blocks_for(n_cells, kWarpsPerBlock), unsigned(m.kp / m.chunk));
    const dim3 rblock(kTile, kWarpsPerBlock);
    if (m.chunk == 8) {
      raster_backward_kernel<8><<<rgrid, rblock>>>(int(n_cells), n_tiles, W, m.kp, m.ranges.get(), m.values.get(),
                                                   m.splats.get(), m.splat_gid.get(), m.features.get(),
                                                   m.background.get(), m.trans.get(), m.contrib.get(), m.g_pix.get(),
                                                   m.g_splat.get(), m.g_feat.get(), m.g_bg.get());
    } else {
      raster_backward_kernel<16><<<rgrid, rblock>>>(int(n_cells), n_tiles, W, m.kp, m.ranges.get(), m.values.get(),
                                                    m.splats.get(), m.splat_gid.get(), m.features.get(),
                                                    m.background.get(), m.trans.get(), m.contrib.get(),
                                                    m.g_pix.get(), m.g_splat.get(), m.g_feat.get(), m.g_bg.get());
    }
    LS_CUDA_CHECK(cudaGetLastError());

    // Back through the projection, per visible pair.
    if (n_vis > 0) {
      project_backward_kernel<<<blocks_for(n_vis, kProjectBlock), kProjectBlock>>>(
          n_vis, l0, m.geom.get(), m.cams.get(), m.g_splat.get(), m.splat_gid.get(), m.splat_line.get(),
          m.g_mean.get(), m.g_cov.get(), m.g_opacity.get(), m.g_screen.get(), m.g_pairs.get(), m.g_cam.get());
      LS_CUDA_CHECK(cudaGetLastError());
    }
    gpu.stop();
    st.batches += 1;
    st.visible_pairs += n_vis;
    st.tile_entries += n_entries;
  }

  m.g_log_scales.reserve(3 * size_t(n));
  m.g_rotations.reserve(4 * size_t(n));
  m.g_logits.reserve(size_t(n));
  if (n > 0) {
    gpu.start();
    geometry_backward_kernel<<<blocks_for(size_t(n), kProjectBlock), kProjectBlock>>>(
        n, m.log_scales.get(), m.rotations.get(), m.logits.get(), m.g_cov.get(), m.g_opacity.get(),
        m.g_log_scales.get(), m.g_rotations.get(), m.g_logits.get());
    LS_CUDA_CHECK(cudaGetLastError());
    gpu.stop();
  }

  // Copy the gradients back, without the feature padding.
  SceneGradT<float>& g = *grad;
  g.means.resize(3 * size_t(n));
  g.log_scales.resize(3 * size_t(n));
  g.rotations.resize(4 * size_t(n));
  g.opacity_logits.resize(size_t(n));
  g.features.resize(size_t(n) * K);
  g.background.resize(size_t(K));
  g.screen_grad.resize(size_t(n));
  g.pairs.resize(size_t(n));
  m.g_mean.download(g.means.data(), 3 * size_t(n));
  m.g_log_scales.download(g.log_scales.data(), 3 * size_t(n));
  m.g_rotations.download(g.rotations.data(), 4 * size_t(n));
  m.g_logits.download(g.opacity_logits.data(), size_t(n));
  m.g_screen.download(g.screen_grad.data(), size_t(n));
  m.g_pairs.download(g.pairs.data(), size_t(n));
  std::vector<float> padded(std::max(size_t(n) * m.kp, size_t(m.kp)));
  m.g_feat.download(padded.data(), size_t(n) * m.kp);
  for (int i = 0; i < n; ++i)
    std::copy(&padded[size_t(i) * m.kp], &padded[size_t(i) * m.kp] + K, &g.features[size_t(i) * K]);
  m.g_bg.download(padded.data(), size_t(m.kp));
  std::copy(padded.begin(), padded.begin() + K, g.background.begin());
  if (cam_grad) {
    std::vector<float> c(12 * size_t(L));
    m.g_cam.download(c.data(), c.size());
    cam_grad->resize(size_t(L));
    for (int l = 0; l < L; ++l) {
      CameraGradT<float>& cg = (*cam_grad)[size_t(l)];
      std::copy(&c[12 * size_t(l)], &c[12 * size_t(l)] + 9, cg.R);
      std::copy(&c[12 * size_t(l)] + 9, &c[12 * size_t(l)] + 12, cg.t);
    }
  }
  st.ms = timer.ms();
  st.gpu_ms = gpu.total_ms;
  if (stats) *stats = st;
  return sum_sq * double(scale);
}

}  // namespace linesplat
