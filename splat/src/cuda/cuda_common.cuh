// Helpers shared by the CUDA files: error checks, device buffers, Thrust's
// scratch memory and timers. Kept to what CUDA 10.2 supports (see
// rasterizer.cu).
#pragma once

#include <cuda_runtime.h>

#include <algorithm>
#include <cstddef>
#include <map>
#include <sstream>
#include <stdexcept>
#include <vector>

#include "linesplat/cuda_rasterizer.hpp"

namespace linesplat {
namespace cudadetail {

inline void throw_cuda(cudaError_t e, const char* expr, const char* file, int line) {
  std::ostringstream ss;
  ss << "CUDA error " << cudaGetErrorName(e) << " (" << cudaGetErrorString(e) << ") in " << expr << " at " << file
     << ":" << line;
  throw std::runtime_error(ss.str());
}

#define LS_CUDA_CHECK(expr)                                                           \
  do {                                                                                \
    const cudaError_t err_ = (expr);                                                  \
    if (err_ != cudaSuccess) ::linesplat::cudadetail::throw_cuda(err_, #expr, __FILE__, __LINE__); \
  } while (0)

inline unsigned blocks_for(size_t n, int per_block) { return unsigned((n + per_block - 1) / per_block); }

// A grow-only device array.
template <typename T>
class DeviceBuffer {
 public:
  DeviceBuffer() = default;
  DeviceBuffer(const DeviceBuffer&) = delete;
  DeviceBuffer& operator=(const DeviceBuffer&) = delete;
  ~DeviceBuffer() { release(); }
  // Grows by at least 1.5x so a slowly rising size doesn't reallocate every
  // call. The old contents are dropped.
  void reserve(size_t n) {
    if (n <= cap_) return;
    const size_t want = std::max(n, cap_ + cap_ / 2);
    release();
    LS_CUDA_CHECK(cudaMalloc(&ptr_, std::max<size_t>(want, 1) * sizeof(T)));
    cap_ = want;
  }
  void release() {
    if (ptr_) cudaFree(ptr_);
    ptr_ = nullptr;
    cap_ = 0;
  }
  void upload(const T* src, size_t n) {
    reserve(n);
    if (n) LS_CUDA_CHECK(cudaMemcpy(ptr_, src, n * sizeof(T), cudaMemcpyHostToDevice));
  }
  void download(T* dst, size_t n) const {
    if (n) LS_CUDA_CHECK(cudaMemcpy(dst, ptr_, n * sizeof(T), cudaMemcpyDeviceToHost));
  }
  void swap(DeviceBuffer& o) {
    std::swap(ptr_, o.ptr_);
    std::swap(cap_, o.cap_);
  }
  T* get() const { return ptr_; }
  size_t bytes() const { return cap_ * sizeof(T); }

 private:
  T* ptr_ = nullptr;
  size_t cap_ = 0;
};

template <typename T>
void zero(DeviceBuffer<T>& b, size_t n) {
  b.reserve(n);
  if (n) LS_CUDA_CHECK(cudaMemset(b.get(), 0, n * sizeof(T)));
}

// Thrust's scratch memory. Every sort, scan and sum needs some, and Thrust
// would cudaMalloc and cudaFree it on each call; cudaFree waits for the GPU,
// and both are slow on the Jetson. This keeps the blocks and hands them out
// again (pass it as thrust::cuda::par(scratch)).
class ThrustScratch {
 public:
  typedef char value_type;
  ThrustScratch() = default;
  ThrustScratch(const ThrustScratch&) = delete;
  ThrustScratch& operator=(const ThrustScratch&) = delete;
  ~ThrustScratch() {
    for (auto& f : free_) cudaFree(f.second);
    for (auto& u : used_) cudaFree(u.first);
  }
  char* allocate(std::ptrdiff_t n) {
    const size_t want = size_t(std::max<std::ptrdiff_t>(n, 1));
    auto it = free_.lower_bound(want);
    char* p = nullptr;
    size_t size = want;
    if (it != free_.end()) {
      size = it->first;
      p = it->second;
      free_.erase(it);
    } else {
      LS_CUDA_CHECK(cudaMalloc(&p, want));
      bytes_ += want;
    }
    used_[p] = size;
    return p;
  }
  void deallocate(char* p, size_t) {
    auto it = used_.find(p);
    if (it == used_.end()) return;
    free_.insert(std::make_pair(it->second, p));
    used_.erase(it);
  }
  size_t bytes() const { return bytes_; }

 private:
  std::multimap<size_t, char*> free_;  // by size
  std::map<char*, size_t> used_;
  size_t bytes_ = 0;
};

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
  GpuTimer(const GpuTimer&) = delete;
  GpuTimer& operator=(const GpuTimer&) = delete;
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

// Per-pass GPU times, when profiling: an event at the end of each pass, and
// the time since the previous event goes to that pass. Does nothing when off.
class PassClock {
 public:
  explicit PassClock(bool on) : on_(on) {}
  ~PassClock() {
    for (cudaEvent_t e : pool_) cudaEventDestroy(e);
  }
  PassClock(const PassClock&) = delete;
  PassClock& operator=(const PassClock&) = delete;
  // Starts the clock: work from here on belongs to the next mark's pass.
  void start() { mark(-1); }
  // The GPU work queued since the last mark belongs to `pass`.
  void mark(int pass) {
    if (!on_) return;
    if (used_ == pool_.size()) {
      cudaEvent_t e;
      LS_CUDA_CHECK(cudaEventCreate(&e));
      pool_.push_back(e);
    }
    cudaEvent_t e = pool_[used_++];
    LS_CUDA_CHECK(cudaEventRecord(e));
    marks_.push_back(std::make_pair(pass, e));
  }
  // Waits for the GPU and adds each pass's time to ms[pass].
  void collect(double* ms) {
    if (!on_ || marks_.empty()) return;
    LS_CUDA_CHECK(cudaEventSynchronize(marks_.back().second));
    for (size_t i = 1; i < marks_.size(); ++i) {
      float t = 0.0f;
      LS_CUDA_CHECK(cudaEventElapsedTime(&t, marks_[i - 1].second, marks_[i].second));
      if (marks_[i].first >= 0) ms[marks_[i].first] += t;
    }
    marks_.clear();
    used_ = 0;
  }

 private:
  bool on_;
  std::vector<cudaEvent_t> pool_;
  size_t used_ = 0;
  std::vector<std::pair<int, cudaEvent_t>> marks_;
};

}  // namespace cudadetail
}  // namespace linesplat
