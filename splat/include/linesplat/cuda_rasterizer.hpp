// The CUDA line rasterizer: renders many scan lines at once, with the same
// results as render_lines_cpu.
//
// How it works, per batch of lines (one thread per (line, Gaussian) pair in
// the first two passes):
//   1. count   project every Gaussian onto every line of the batch, keep the
//              pixel range of each pair that reaches the line
//   2. scan    prefix sums give each visible pair a slot, and each
//              (pair, 32-pixel tile) overlap a slot for a sort key
//   3. emit    write the 1D splats and the keys (line, tile, depth)
//   4. sort    Thrust radix sort: per tile, front to back
//   5. raster  one warp per (line, tile): each lane is a pixel, the warp walks
//              the tile's sorted list in batches of 32 through shared memory
//              and composites front to back until the pixel is opaque.
//              Features are done 8 or 16 channels at a time (grid.y), so any
//              number of spectral features fits in registers.
// This is the 3DGS tile rasterizer with the image collapsed to rows of tiles.
//
// For training, mse_backward() runs the same passes and then three more:
//   6. loss    per pixel, the bands, the squared error against the measured
//              line and dL/dfeatures
//   7. raster  one warp per (line, tile) again, walking each pixel's list
//      back    back to front from where the forward pass stopped (as
//              backward_cpu.hpp does per pixel); per splat the warp sums its
//              pixels' gradients and adds them with one atomic per value
//   8. project one thread per visible pair: through the projection to the
//      back    Gaussian's mean, covariance and opacity, and the line's camera
// and finally turns covariance gradients into scale and rotation ones.
//
// CudaSceneOptimizer (below) keeps a whole training run on the GPU: the
// Gaussians, their gradients and their Adam state stay there, and Adam and
// densification run as kernels (src/cuda/optimizer.cu).
#pragma once

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include "linesplat/backward_cpu.hpp"
#include "linesplat/dataset.hpp"
#include "linesplat/line_camera.hpp"
#include "linesplat/render_cpu.hpp"
#include "linesplat/scene.hpp"
#include "linesplat/trainer.hpp"

namespace linesplat {

// The GPU's passes, for per-pass timings (CudaRasterizer::profile).
enum CudaPass {
  kPassProject,       // 1. count: project every (line, Gaussian) pair
  kPassScan,          // 2. prefix sums, and reading back their totals
  kPassEmit,          // 3. write the splats and the sort keys
  kPassSort,          // 4. sort the keys, find each tile's range
  kPassRaster,        // 5. composite front to back
  kPassLoss,          // 6. bands, squared error, dL/dfeatures (and dL/dbasis)
  kPassRasterBack,    // 7. compositing backwards
  kPassProjectBack,   // 8. projection backwards
  kPassGeometryBack,  // covariance to scales and rotations
  kPassCopy,          // copies between the host and the GPU
  kPassAdam,          // Adam and the densification statistics (CudaSceneOptimizer)
  kPassDensify,       // clone, split and prune (CudaSceneOptimizer)
  kCudaPasses
};
const char* cuda_pass_name(int pass);

struct CudaRenderStats {
  int batches = 0;
  long long visible_pairs = 0;  // (line, Gaussian) pairs that reach a pixel
  long long tile_entries = 0;   // sorted (line, tile, depth) keys
  double gpu_ms = 0.0;          // GPU time of the passes, without the copies
  double ms = 0.0;              // wall time of the whole call, copies included
  double pass_ms[kCudaPasses] = {};  // GPU time per pass, when profiling

  CudaRenderStats& operator+=(const CudaRenderStats& o) {
    batches += o.batches;
    visible_pairs += o.visible_pairs;
    tile_entries += o.tile_entries;
    gpu_ms += o.gpu_ms;
    ms += o.ms;
    for (int p = 0; p < kCudaPasses; ++p) pass_ms[p] += o.pass_ms[p];
    return *this;
  }
};

// The GPU memory a rasterizer holds, by what it is for, in bytes. Its
// buffers only grow, so this is also the most it has held, apart from
// densification's (densify).
struct CudaMemoryUse {
  size_t scene = 0;      // Gaussians: parameters, covariances, padded features, basis
  size_t gradients = 0;  // per Gaussian: gradients and densification statistics
  size_t adam = 0;       // Adam's moments (CudaSceneOptimizer)
  size_t pairs = 0;      // per (line, Gaussian) pair of a batch: 12 bytes
  size_t splats = 0;     // per visible pair: the 1D splats and their gradients
  size_t keys = 0;       // per (pair, tile) overlap: sort keys and values
  size_t pixels = 0;     // per pixel of a batch: features, transmittance, loss terms
  size_t cameras = 0;    // per line of a batch: cameras and their gradients
  size_t targets = 0;    // the measured lines
  size_t thrust = 0;     // Thrust's scratch for sorts, scans and sums
  size_t densify = 0;    // the most densification held at once, on top of the rest
  size_t total() const {
    return scene + gradients + adam + pairs + splats + keys + pixels + cameras + targets + thrust + densify;
  }
};

// True if a CUDA device can be used; otherwise *why (if given) says why not.
bool cuda_device_available(std::string* why = nullptr);
std::string cuda_device_name();
// Free and total memory on the device, from cudaMemGetInfo. On a Jetson the
// GPU shares the system's memory, so this is the whole board's.
bool cuda_memory_info(size_t* free_bytes, size_t* total_bytes);

class CudaRasterizer {
 public:
  CudaRasterizer();
  ~CudaRasterizer();
  CudaRasterizer(const CudaRasterizer&) = delete;
  CudaRasterizer& operator=(const CudaRasterizer&) = delete;

  // Copies the scene to the GPU and precomputes each Gaussian's covariance.
  void set_scene(const GaussianScene& scene);
  // Renders the scene's features on every camera (all the same width).
  LineImage render(const std::vector<LineCamera>& cams, CudaRenderStats* stats = nullptr);
  // The same into *out, reusing its memory when it already has the right size.
  void render(const std::vector<LineCamera>& cams, LineImage* out, CudaRenderStats* stats = nullptr);

  // Training. Copies measured lines [num_lines, width, bands] to the GPU,
  // where they stay for mse_backward.
  void set_targets(const float* lines, int num_lines, int width, int bands);
  // Renders the scene's bands on cams, where camera i sees measured line
  // lines[i], and backpropagates the mean squared error over all their pixels
  // and bands into *grad (the basis too if grad->learn_basis) and, if not
  // null, *cam_grad (one per camera), as render_backward_cpu does. Returns
  // that error.
  double mse_backward(const std::vector<LineCamera>& cams, const std::vector<int>& lines, SceneGradT<float>* grad,
                      std::vector<CameraGradT<float>>* cam_grad = nullptr, CudaRenderStats* stats = nullptr);

  // What the GPU buffers hold now, which is the most they have held.
  CudaMemoryUse memory_use() const;

  // Cap on lines x Gaussians per batch. It sets the scratch memory: about
  // 12 bytes per pair, so the default 2^24 needs about 200 MB.
  long long max_pairs_per_batch = 1LL << 24;
  // Time every pass (CudaRenderStats::pass_ms), with an event after each.
  bool profile = false;

 private:
  friend class CudaSceneOptimizer;
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

// Training with everything on the GPU: the scene, its gradients and its Adam
// state stay there between steps, Adam and the densification statistics run
// as one kernel per array, and densification as a few kernels and scans.
// Each step only the batch's cameras go up and, once the poses are refined,
// their gradients come down. Gives the same results as HostSceneOptimizer
// (the tests compare them), with the split offsets drawn alike.
class CudaSceneOptimizer : public SceneOptimizer {
 public:
  // Copies the dataset's measured lines and the scene to the GPU. `data`
  // need not outlive this.
  CudaSceneOptimizer(const Dataset& data, const GaussianScene& init);
  ~CudaSceneOptimizer() override;

  double backward(const std::vector<LineCamera>& cams, const std::vector<int>& lines,
                  std::vector<CameraGradT<float>>* cam_grad) override;
  void set_gradient(const SceneGradT<float>& g) override;
  void adam(const SceneRates& lr, int t) override;
  void accumulate(double scale) override;
  DensifyCounts densify(const DensifyParams& p) override;
  int size() const override;
  const GaussianScene& scene() const override;

  CudaRasterizer& rasterizer() { return rast_; }
  // The GPU's work since the start, summed over every call.
  const CudaRenderStats& stats() const { return stats_; }
  CudaMemoryUse memory_use() const;

 private:
  struct State;
  CudaRasterizer rast_;
  std::unique_ptr<State> st_;
  CudaRenderStats stats_;
  mutable GaussianScene cache_;
  mutable bool cache_ok_ = false;
};

}  // namespace linesplat
