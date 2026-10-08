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
#include <thrust/system/cuda/execution_policy.h>
#include <thrust/transform_scan.h>

#include <algorithm>
#include <climits>
#include <cstdint>
#include <sstream>
#include <stdexcept>

#include "cuda_common.cuh"
#include "linesplat/gradients.hpp"
#include "linesplat/util.hpp"
#include "rasterizer_impl.cuh"

namespace linesplat {

using namespace cudadetail;

namespace {

constexpr int kTile = 32;          // pixels per tile: one warp, one lane per pixel
constexpr int kWarpsPerBlock = 4;  // tiles per raster block
constexpr int kProjectBlock = 256;
constexpr int kBasisBlock = 256;

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

// Backward pass 6c, when the basis is learned: dL/dbasis[b, c] is the sum
// over pixels of dL/dband b times feature c. One thread per (b, c) entry
// (grid.y covers them all); each block takes a share of the pixels, 32 at a
// time through shared memory, and adds its sums with one atomic per entry.
__global__ void basis_grad_kernel(int n_px, int k, int bands, const float* __restrict__ feat,
                                  const float* __restrict__ g_bands, float* __restrict__ g_basis) {
  extern __shared__ float sh[];  // [32, bands] of dL/dbands, then [32, k] of features
  float* s_g = sh;
  float* s_f = sh + kTile * bands;
  const int o = blockIdx.y * blockDim.x + threadIdx.x;
  const bool mine = o < bands * k;
  const int b = mine ? o / k : 0, c = mine ? o - b * k : 0;
  const int tiles = (n_px + kTile - 1) / kTile;
  const int per = (tiles + gridDim.x - 1) / gridDim.x;
  const int t0 = blockIdx.x * per, t1 = min(tiles, t0 + per);
  float acc = 0.0f;
  for (int t = t0; t < t1; ++t) {
    const int p0 = t * kTile, np = min(kTile, n_px - p0);
    __syncthreads();
    for (int e = threadIdx.x; e < np * bands; e += blockDim.x) s_g[e] = g_bands[size_t(p0) * bands + e];
    for (int e = threadIdx.x; e < np * k; e += blockDim.x) s_f[e] = feat[size_t(p0) * k + e];
    __syncthreads();
    if (mine)
      for (int j = 0; j < np; ++j) acc += s_g[j * bands + b] * s_f[j * k + c];
  }
  if (mine && acc != 0.0f) atomicAdd(&g_basis[o], acc);
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

bool cuda_memory_info(size_t* free_bytes, size_t* total_bytes) {
  if (cudaMemGetInfo(free_bytes, total_bytes) == cudaSuccess) return true;
  cudaGetLastError();
  return false;
}

const char* cuda_pass_name(int pass) {
  static const char* const names[kCudaPasses] = {"project", "scan", "emit", "sort", "raster", "loss",
                                                 "raster back", "project back", "geometry back", "copies",
                                                 "adam", "densify"};
  return pass >= 0 && pass < kCudaPasses ? names[pass] : "?";
}

int CudaRasterizer::Impl::batch_lines(long long max_pairs, int w) const {
  const long long n_tiles = (w + kTile - 1) / kTile;
  long long b = std::max(1LL, max_pairs / std::max(1, n));
  b = std::min<long long>(b, 65535);
  b = std::min<long long>(b, std::max(1LL, (1LL << 31) / (std::max(1LL, (long long)n) * n_tiles)));
  return int(b);
}

void CudaRasterizer::Impl::upload_scene(const GaussianScene& scene) {
  n = scene.size();
  k = scene.num_features;
  chunk = k <= 8 ? 8 : 16;
  kp = (k + chunk - 1) / chunk * chunk;
  bands = scene.num_bands();

  std::vector<float> feat(size_t(n) * kp, 0.0f), bg(size_t(kp), 0.0f);
  for (int i = 0; i < n; ++i)
    std::copy(&scene.features[size_t(i) * k], &scene.features[size_t(i) * k] + k, &feat[size_t(i) * kp]);
  std::copy(scene.background.begin(), scene.background.end(), bg.begin());
  features.upload(feat.data(), feat.size());
  background.upload(bg.data(), bg.size());
  basis.upload(scene.basis.data(), scene.basis.size());

  // The stored parameters stay on the GPU for the backward pass.
  if (n > 0) {
    means.upload(scene.means.data(), scene.means.size());
    log_scales.upload(scene.log_scales.data(), scene.log_scales.size());
    rotations.upload(scene.rotations.data(), scene.rotations.size());
    logits.upload(scene.opacity_logits.data(), scene.opacity_logits.size());
  }
  prepare();
  has_scene = true;
}

void CudaRasterizer::Impl::prepare() {
  geom.reserve(size_t(n) * 3);
  if (n > 0) {
    prepare_kernel<<<blocks_for(size_t(n), kProjectBlock), kProjectBlock>>>(n, means.get(), log_scales.get(),
                                                                           rotations.get(), logits.get(), geom.get());
    LS_CUDA_CHECK(cudaGetLastError());
  }
  LS_CUDA_CHECK(cudaDeviceSynchronize());
}

void CudaRasterizer::Impl::download_scene(GaussianScene* s) const {
  s->num_features = k;
  s->means.resize(3 * size_t(n));
  s->log_scales.resize(3 * size_t(n));
  s->rotations.resize(4 * size_t(n));
  s->opacity_logits.resize(size_t(n));
  s->features.resize(size_t(n) * k);
  s->background.resize(size_t(k));
  s->basis.resize(size_t(bands) * k);
  means.download(s->means.data(), s->means.size());
  log_scales.download(s->log_scales.data(), s->log_scales.size());
  rotations.download(s->rotations.data(), s->rotations.size());
  logits.download(s->opacity_logits.data(), s->opacity_logits.size());
  basis.download(s->basis.data(), s->basis.size());
  std::vector<float> padded(std::max(size_t(n) * kp, size_t(kp)));
  features.download(padded.data(), size_t(n) * kp);
  for (int i = 0; i < n; ++i)
    std::copy(&padded[size_t(i) * kp], &padded[size_t(i) * kp] + k, &s->features[size_t(i) * k]);
  background.download(padded.data(), size_t(kp));
  std::copy(padded.begin(), padded.begin() + k, s->background.begin());
}

void CudaRasterizer::Impl::forward(int lb, int w, uint32_t* n_vis_out, uint32_t* n_entries_out, PassClock& clock) {
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
    clock.mark(kPassProject);
    thrust::transform_exclusive_scan(thrust::cuda::par(scratch), packed.get(), packed.get() + pairs, tile_off.get(),
                                     TilesOf(), 0u, AddU32());
    thrust::transform_exclusive_scan(thrust::cuda::par(scratch), packed.get(), packed.get() + pairs, vis_off.get(),
                                     IsVisible(), 0u, AddU32());
    uint32_t last_packed = 0, last_tile = 0, last_vis = 0;
    LS_CUDA_CHECK(cudaMemcpy(&last_packed, packed.get() + pairs - 1, 4, cudaMemcpyDeviceToHost));
    LS_CUDA_CHECK(cudaMemcpy(&last_tile, tile_off.get() + pairs - 1, 4, cudaMemcpyDeviceToHost));
    LS_CUDA_CHECK(cudaMemcpy(&last_vis, vis_off.get() + pairs - 1, 4, cudaMemcpyDeviceToHost));
    n_entries = last_tile + TilesOf()(last_packed);
    n_vis = last_vis + IsVisible()(last_packed);
    clock.mark(kPassScan);
    if (n_entries > 0) {
      splats.reserve(n_vis);
      splat_gid.reserve(n_vis);
      splat_line.reserve(n_vis);
      keys.reserve(n_entries);
      values.reserve(n_entries);
      emit_kernel<<<grid, kProjectBlock>>>(n, n_tiles, geom.get(), cams.get(), packed.get(), vis_off.get(),
                                           tile_off.get(), splats.get(), splat_gid.get(), splat_line.get(), keys.get(),
                                           values.get());
      LS_CUDA_CHECK(cudaGetLastError());
      clock.mark(kPassEmit);
      thrust::sort_by_key(thrust::cuda::par(scratch), keys.get(), keys.get() + n_entries, values.get());
    }
  }

  ranges.reserve(n_cells);
  LS_CUDA_CHECK(cudaMemset(ranges.get(), 0, n_cells * sizeof(uint2)));
  if (n_entries > 0) {
    ranges_kernel<<<blocks_for(n_entries, 256), 256>>>(n_entries, keys.get(), ranges.get());
    LS_CUDA_CHECK(cudaGetLastError());
  }
  clock.mark(kPassSort);

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
  clock.mark(kPassRaster);
  *n_vis_out = n_vis;
  *n_entries_out = n_entries;
}

double CudaRasterizer::Impl::backward(const std::vector<LineCamera>& cams_in, const std::vector<int>& lines,
                                      bool learn_basis, std::vector<CameraGradT<float>>* cam_grad, long long max_pairs,
                                      bool profile, CudaRenderStats* stats) {
  if (!has_scene) throw std::runtime_error("CudaRasterizer::mse_backward: call set_scene first");
  if (lines.size() != cams_in.size())
    throw std::runtime_error("CudaRasterizer::mse_backward: need one measured line per camera");
  const int L = int(cams_in.size()), W = t_width, K = k, B = bands;
  if (B != t_bands)
    throw std::runtime_error("CudaRasterizer::mse_backward: the scene has " + std::to_string(B) +
                             " bands and the measured lines " + std::to_string(t_bands));
  for (const auto& c : cams_in)
    if (c.width != W) throw std::runtime_error("CudaRasterizer::mse_backward: cameras and measured lines differ in width");
  for (int l : lines)
    if (l < 0 || l >= t_lines) throw std::runtime_error("CudaRasterizer::mse_backward: no such measured line");
  if (W > 0xffff) throw std::runtime_error("CudaRasterizer::mse_backward: lines are limited to 65535 pixels");
  // basis_grad_kernel holds 32 pixels' bands and features in shared memory.
  if (learn_basis && size_t(kTile) * (B + K) * sizeof(float) > 48 * 1024)
    throw std::runtime_error("CudaRasterizer::mse_backward: too many bands and features to learn the basis");
  Timer timer;
  CudaRenderStats st;
  GpuTimer gpu;
  PassClock clock(profile);

  clock.start();
  zero(g_mean, 3 * size_t(n));
  zero(g_cov, 6 * size_t(n));
  zero(g_opacity, size_t(n));
  zero(g_screen, size_t(n));
  zero(g_pairs, size_t(n));
  zero(g_feat, size_t(n) * kp);
  zero(g_bg, size_t(kp));
  zero(g_cam, 12 * size_t(L));
  if (learn_basis) zero(g_basis, size_t(B) * K);
  clock.mark(kPassCopy);
  clock.collect(st.pass_ms);
  double sum_sq = 0.0;
  const float scale = float(1.0 / (double(std::max(1, L)) * std::max(1, W) * std::max(1, B)));
  const int n_tiles = (W + kTile - 1) / kTile;
  // The loss kernels index (pixel, band) and (pixel, channel) with an int.
  const int batch =
      std::min(batch_lines(max_pairs, W), std::max(1, int(INT_MAX / (std::max(1, W) * std::max(B, kp)))));

  for (int l0 = 0; l0 < L && W > 0; l0 += batch) {
    const int lb = std::min(batch, L - l0);
    cams.upload(&cams_in[size_t(l0)], size_t(lb));
    line_ids.upload(&lines[size_t(l0)], size_t(lb));
    gpu.start();
    clock.start();
    uint32_t n_vis = 0, n_entries = 0;
    forward(lb, W, &n_vis, &n_entries, clock);

    // The loss and dL/dfeatures per pixel.
    const size_t px = size_t(lb) * W;
    const size_t n_pb = px * B, n_pk = px * kp;
    sq_err.reserve(n_pb);
    g_bands.reserve(n_pb);
    g_pix.reserve(n_pk);
    if (n_pb > 0) {
      band_error_kernel<<<blocks_for(n_pb, 256), 256>>>(int(n_pb), W, K, B, scale, out.get(), basis.get(),
                                                        targets.get(), line_ids.get(), sq_err.get(), g_bands.get());
      LS_CUDA_CHECK(cudaGetLastError());
      sum_sq += thrust::reduce(thrust::cuda::par(scratch), sq_err.get(), sq_err.get() + n_pb, 0.0, AddF64());
    }
    feature_grad_kernel<<<blocks_for(n_pk, 256), 256>>>(int(n_pk), K, kp, B, basis.get(), g_bands.get(),
                                                        g_pix.get());
    LS_CUDA_CHECK(cudaGetLastError());
    if (learn_basis && px > 0) {
      const unsigned tiles = blocks_for(px, kTile);
      const dim3 bgrid(std::min(64u, tiles), blocks_for(size_t(B) * K, kBasisBlock));
      basis_grad_kernel<<<bgrid, kBasisBlock, size_t(kTile) * (B + K) * sizeof(float)>>>(
          int(px), K, B, out.get(), g_bands.get(), g_basis.get());
      LS_CUDA_CHECK(cudaGetLastError());
    }
    clock.mark(kPassLoss);

    // Back through the compositing, per (line, tile).
    zero(g_splat, n_vis);
    const size_t n_cells = size_t(lb) * n_tiles;
    const dim3 rgrid(blocks_for(n_cells, kWarpsPerBlock), unsigned(kp / chunk));
    const dim3 rblock(kTile, kWarpsPerBlock);
    if (chunk == 8) {
      raster_backward_kernel<8><<<rgrid, rblock>>>(int(n_cells), n_tiles, W, kp, ranges.get(), values.get(),
                                                   splats.get(), splat_gid.get(), features.get(), background.get(),
                                                   trans.get(), contrib.get(), g_pix.get(), g_splat.get(),
                                                   g_feat.get(), g_bg.get());
    } else {
      raster_backward_kernel<16><<<rgrid, rblock>>>(int(n_cells), n_tiles, W, kp, ranges.get(), values.get(),
                                                    splats.get(), splat_gid.get(), features.get(), background.get(),
                                                    trans.get(), contrib.get(), g_pix.get(), g_splat.get(),
                                                    g_feat.get(), g_bg.get());
    }
    LS_CUDA_CHECK(cudaGetLastError());
    clock.mark(kPassRasterBack);

    // Back through the projection, per visible pair.
    if (n_vis > 0) {
      project_backward_kernel<<<blocks_for(n_vis, kProjectBlock), kProjectBlock>>>(
          n_vis, l0, geom.get(), cams.get(), g_splat.get(), splat_gid.get(), splat_line.get(), g_mean.get(),
          g_cov.get(), g_opacity.get(), g_screen.get(), g_pairs.get(), g_cam.get());
      LS_CUDA_CHECK(cudaGetLastError());
    }
    clock.mark(kPassProjectBack);
    gpu.stop();
    clock.collect(st.pass_ms);
    st.batches += 1;
    st.visible_pairs += n_vis;
    st.tile_entries += n_entries;
  }

  g_log_scales.reserve(3 * size_t(n));
  g_rotations.reserve(4 * size_t(n));
  g_logits.reserve(size_t(n));
  if (n > 0) {
    gpu.start();
    clock.start();
    geometry_backward_kernel<<<blocks_for(size_t(n), kProjectBlock), kProjectBlock>>>(
        n, log_scales.get(), rotations.get(), logits.get(), g_cov.get(), g_opacity.get(), g_log_scales.get(),
        g_rotations.get(), g_logits.get());
    LS_CUDA_CHECK(cudaGetLastError());
    clock.mark(kPassGeometryBack);
    gpu.stop();
    clock.collect(st.pass_ms);
  }

  if (cam_grad) {
    clock.start();
    std::vector<float> c(12 * size_t(L));
    g_cam.download(c.data(), c.size());
    cam_grad->resize(size_t(L));
    for (int l = 0; l < L; ++l) {
      CameraGradT<float>& cg = (*cam_grad)[size_t(l)];
      std::copy(&c[12 * size_t(l)], &c[12 * size_t(l)] + 9, cg.R);
      std::copy(&c[12 * size_t(l)] + 9, &c[12 * size_t(l)] + 12, cg.t);
    }
    clock.mark(kPassCopy);
    clock.collect(st.pass_ms);
  }
  st.ms = timer.ms();
  st.gpu_ms = gpu.total_ms;
  if (stats) *stats = st;
  return sum_sq * double(scale);
}

CudaRasterizer::CudaRasterizer() : impl_(new Impl) {}
CudaRasterizer::~CudaRasterizer() = default;

void CudaRasterizer::set_scene(const GaussianScene& scene) {
  scene.validate();
  impl_->upload_scene(scene);
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
  PassClock clock(profile);
  const int batch = m.batch_lines(max_pairs_per_batch, W);
  for (int l0 = 0; l0 < L; l0 += batch) {
    const int lb = std::min(batch, L - l0);
    m.cams.upload(&cams[size_t(l0)], size_t(lb));
    gpu.start();
    clock.start();
    uint32_t n_vis = 0, n_entries = 0;
    m.forward(lb, W, &n_vis, &n_entries, clock);
    gpu.stop();

    const size_t px = size_t(lb) * W;
    m.out.download(&img.values[size_t(l0) * W * K], px * K);
    m.trans.download(&img.transmittance[size_t(l0) * W], px);
    m.contrib.download(&img.contributors[size_t(l0) * W], px);
    clock.mark(kPassCopy);
    clock.collect(st.pass_ms);

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
  CudaRenderStats st;
  const double loss = m.backward(cams, lines, grad->learn_basis, cam_grad, max_pairs_per_batch, profile, &st);

  // Copy the gradients back, without the feature padding.
  Timer timer;
  PassClock clock(profile);
  clock.start();
  const int n = m.n, K = m.k;
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
  if (g.learn_basis) {
    g.basis.resize(size_t(m.bands) * K);
    m.g_basis.download(g.basis.data(), g.basis.size());
  } else {
    g.basis.clear();
  }
  clock.mark(kPassCopy);
  clock.collect(st.pass_ms);
  st.ms += timer.ms();
  if (stats) *stats = st;
  return loss;
}

CudaMemoryUse CudaRasterizer::memory_use() const {
  const Impl& m = *impl_;
  CudaMemoryUse u;
  u.scene = m.geom.bytes() + m.means.bytes() + m.log_scales.bytes() + m.rotations.bytes() + m.logits.bytes() +
            m.features.bytes() + m.background.bytes() + m.basis.bytes();
  u.gradients = m.g_mean.bytes() + m.g_cov.bytes() + m.g_opacity.bytes() + m.g_feat.bytes() + m.g_bg.bytes() +
                m.g_basis.bytes() + m.g_screen.bytes() + m.g_pairs.bytes() + m.g_log_scales.bytes() +
                m.g_rotations.bytes() + m.g_logits.bytes();
  u.pairs = m.packed.bytes() + m.vis_off.bytes() + m.tile_off.bytes();
  u.splats = m.splats.bytes() + m.splat_gid.bytes() + m.splat_line.bytes() + m.g_splat.bytes();
  u.keys = m.keys.bytes() + m.values.bytes() + m.ranges.bytes();
  u.pixels = m.out.bytes() + m.trans.bytes() + m.contrib.bytes() + m.sq_err.bytes() + m.g_bands.bytes() +
             m.g_pix.bytes();
  u.cameras = m.cams.bytes() + m.line_ids.bytes() + m.g_cam.bytes();
  u.targets = m.targets.bytes();
  u.thrust = m.scratch.bytes();
  return u;
}

}  // namespace linesplat
