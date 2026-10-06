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
#pragma once

#include <memory>
#include <string>
#include <vector>

#include "linesplat/backward_cpu.hpp"
#include "linesplat/line_camera.hpp"
#include "linesplat/render_cpu.hpp"
#include "linesplat/scene.hpp"

namespace linesplat {

struct CudaRenderStats {
  int batches = 0;
  long long visible_pairs = 0;  // (line, Gaussian) pairs that reach a pixel
  long long tile_entries = 0;   // sorted (line, tile, depth) keys
  double gpu_ms = 0.0;          // GPU time of the passes, without the copies
  double ms = 0.0;              // wall time of the whole call, copies included
};

// True if a CUDA device can be used; otherwise *why (if given) says why not.
bool cuda_device_available(std::string* why = nullptr);
std::string cuda_device_name();

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
  // and bands into *grad and, if not null, *cam_grad (one per camera), as
  // render_backward_cpu does. Returns that error.
  double mse_backward(const std::vector<LineCamera>& cams, const std::vector<int>& lines, SceneGradT<float>* grad,
                      std::vector<CameraGradT<float>>* cam_grad = nullptr, CudaRenderStats* stats = nullptr);

  // Cap on lines x Gaussians per batch. It sets the scratch memory: about
  // 12 bytes per pair, so the default 2^24 needs about 200 MB.
  long long max_pairs_per_batch = 1LL << 24;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace linesplat
