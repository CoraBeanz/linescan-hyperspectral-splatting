// CUDA line rasterizer. See cuda_rasterizer.hpp for the passes.
//
// Kept to what CUDA 10.2 on the Jetson Nano (sm_53) supports: C++14, Thrust
// for scans and sorts, float atomics only, warp intrinsics with _sync.
#include "linesplat/cuda_rasterizer.hpp"

#include <cuda_runtime.h>
#include <thrust/execution_policy.h>
#include <thrust/scan.h>
#include <thrust/sort.h>
#include <thrust/transform_scan.h>

#include <algorithm>
#include <cstdint>
#include <sstream>
#include <stdexcept>

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
                            int* __restrict__ splat_gid, unsigned long long* __restrict__ keys,
                            uint32_t* __restrict__ values) {
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

struct CudaRasterizer::Impl {
  int n = 0;         // Gaussians
  int k = 0;         // features
  int chunk = 16;    // feature channels per raster launch row
  int kp = 0;        // features padded to a multiple of chunk
  bool has_scene = false;
  DeviceBuffer<float4> geom;
  DeviceBuffer<float> features, background;
  DeviceBuffer<LineCamera> cams;
  DeviceBuffer<uint32_t> packed, vis_off, tile_off, values;
  DeviceBuffer<unsigned long long> keys;
  DeviceBuffer<float4> splats;
  DeviceBuffer<int> splat_gid, contrib;
  DeviceBuffer<uint2> ranges;
  DeviceBuffer<float> out, trans;
};

CudaRasterizer::CudaRasterizer() : impl_(new Impl) {}
CudaRasterizer::~CudaRasterizer() = default;

void CudaRasterizer::set_scene(const GaussianScene& scene) {
  scene.validate();
  Impl& m = *impl_;
  m.n = scene.size();
  m.k = scene.num_features;
  m.chunk = m.k <= 8 ? 8 : 16;
  m.kp = (m.k + m.chunk - 1) / m.chunk * m.chunk;

  std::vector<float> feat(size_t(m.n) * m.kp, 0.0f), bg(size_t(m.kp), 0.0f);
  for (int i = 0; i < m.n; ++i)
    std::copy(&scene.features[size_t(i) * m.k], &scene.features[size_t(i) * m.k] + m.k, &feat[size_t(i) * m.kp]);
  std::copy(scene.background.begin(), scene.background.end(), bg.begin());
  m.features.upload(feat.data(), feat.size());
  m.background.upload(bg.data(), bg.size());

  m.geom.reserve(size_t(m.n) * 3);
  if (m.n > 0) {
    DeviceBuffer<float> means, scales, rots, logits;
    means.upload(scene.means.data(), scene.means.size());
    scales.upload(scene.log_scales.data(), scene.log_scales.size());
    rots.upload(scene.rotations.data(), scene.rotations.size());
    logits.upload(scene.opacity_logits.data(), scene.opacity_logits.size());
    prepare_kernel<<<blocks_for(size_t(m.n), kProjectBlock), kProjectBlock>>>(m.n, means.get(), scales.get(), rots.get(),
                                                                             logits.get(), m.geom.get());
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

namespace {

// CUDA events that time the GPU work of a render, batch by batch.
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

}  // namespace

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
  const int L = img.lines, W = img.width, K = m.k, n = m.n;
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

  const int n_tiles = (W + kTile - 1) / kTile;
  // Lines per batch: bounded by the pair budget, the grid's y limit, and
  // keeping every key count inside 32 bits.
  long long batch = std::max(1LL, max_pairs_per_batch / std::max(1, n));
  batch = std::min<long long>(batch, 65535);
  batch = std::min<long long>(batch, std::max(1LL, (1LL << 31) / (std::max(1LL, (long long)n) * n_tiles)));

  for (int l0 = 0; l0 < L; l0 += int(batch)) {
    const int lb = int(std::min<long long>(batch, L - l0));
    m.cams.upload(&cams[size_t(l0)], size_t(lb));
    gpu.start();
    const size_t pairs = size_t(lb) * n;
    const size_t n_cells = size_t(lb) * n_tiles;
    uint32_t n_entries = 0, n_vis = 0;

    if (n > 0) {
      m.packed.reserve(pairs);
      m.vis_off.reserve(pairs);
      m.tile_off.reserve(pairs);
      const dim3 grid(blocks_for(size_t(n), kProjectBlock), unsigned(lb));
      count_kernel<<<grid, kProjectBlock>>>(n, m.geom.get(), m.cams.get(), m.packed.get());
      LS_CUDA_CHECK(cudaGetLastError());
      thrust::transform_exclusive_scan(thrust::device, m.packed.get(), m.packed.get() + pairs, m.tile_off.get(),
                                       TilesOf(), 0u, AddU32());
      thrust::transform_exclusive_scan(thrust::device, m.packed.get(), m.packed.get() + pairs, m.vis_off.get(),
                                       IsVisible(), 0u, AddU32());
      uint32_t last_packed = 0, last_tile = 0, last_vis = 0;
      LS_CUDA_CHECK(cudaMemcpy(&last_packed, m.packed.get() + pairs - 1, 4, cudaMemcpyDeviceToHost));
      LS_CUDA_CHECK(cudaMemcpy(&last_tile, m.tile_off.get() + pairs - 1, 4, cudaMemcpyDeviceToHost));
      LS_CUDA_CHECK(cudaMemcpy(&last_vis, m.vis_off.get() + pairs - 1, 4, cudaMemcpyDeviceToHost));
      n_entries = last_tile + TilesOf()(last_packed);
      n_vis = last_vis + IsVisible()(last_packed);
      if (n_entries > 0) {
        m.splats.reserve(n_vis);
        m.splat_gid.reserve(n_vis);
        m.keys.reserve(n_entries);
        m.values.reserve(n_entries);
        emit_kernel<<<grid, kProjectBlock>>>(n, n_tiles, m.geom.get(), m.cams.get(), m.packed.get(), m.vis_off.get(),
                                             m.tile_off.get(), m.splats.get(), m.splat_gid.get(), m.keys.get(),
                                             m.values.get());
        LS_CUDA_CHECK(cudaGetLastError());
        thrust::sort_by_key(thrust::device, m.keys.get(), m.keys.get() + n_entries, m.values.get());
      }
    }

    m.ranges.reserve(n_cells);
    LS_CUDA_CHECK(cudaMemset(m.ranges.get(), 0, n_cells * sizeof(uint2)));
    if (n_entries > 0) {
      ranges_kernel<<<blocks_for(n_entries, 256), 256>>>(n_entries, m.keys.get(), m.ranges.get());
      LS_CUDA_CHECK(cudaGetLastError());
    }

    const size_t px = size_t(lb) * W;
    m.out.reserve(px * K);
    m.trans.reserve(px);
    m.contrib.reserve(px);
    const dim3 rgrid(blocks_for(n_cells, kWarpsPerBlock), unsigned(m.kp / m.chunk));
    const dim3 rblock(kTile, kWarpsPerBlock);
    if (m.chunk == 8) {
      raster_kernel<8><<<rgrid, rblock>>>(int(n_cells), n_tiles, W, K, m.kp, m.ranges.get(), m.values.get(),
                                          m.splats.get(), m.splat_gid.get(), m.features.get(), m.background.get(),
                                          m.out.get(), m.trans.get(), m.contrib.get());
    } else {
      raster_kernel<16><<<rgrid, rblock>>>(int(n_cells), n_tiles, W, K, m.kp, m.ranges.get(), m.values.get(),
                                           m.splats.get(), m.splat_gid.get(), m.features.get(), m.background.get(),
                                           m.out.get(), m.trans.get(), m.contrib.get());
    }
    LS_CUDA_CHECK(cudaGetLastError());
    gpu.stop();

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

}  // namespace linesplat
