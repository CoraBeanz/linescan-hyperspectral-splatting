// CudaRasterizer's GPU state, shared by rasterizer.cu (the passes) and
// optimizer.cu (Adam and densification on the same arrays).
#pragma once

#include <cstdint>
#include <vector>

#include "cuda_common.cuh"
#include "linesplat/cuda_rasterizer.hpp"

namespace linesplat {

struct CudaRasterizer::Impl {
  int n = 0;         // Gaussians
  int k = 0;         // features
  int chunk = 16;    // feature channels per raster launch row
  int kp = 0;        // features padded to a multiple of chunk
  int bands = 0;     // rows of the spectral basis
  bool has_scene = false;
  // The scene: the stored parameters, and per Gaussian its covariance in
  // the form the projection reads (3 float4s). Features and background are
  // padded to kp channels with zeros.
  cudadetail::DeviceBuffer<float4> geom;
  cudadetail::DeviceBuffer<float> means, log_scales, rotations, logits;
  cudadetail::DeviceBuffer<float> features, background, basis;
  cudadetail::DeviceBuffer<float> basis_t;  // [k, bands], made from basis by each backward call
  // Per batch.
  cudadetail::DeviceBuffer<LineCamera> cams;
  cudadetail::DeviceBuffer<uint32_t> packed, vis_off, tile_off, values;
  cudadetail::DeviceBuffer<unsigned long long> keys;
  cudadetail::DeviceBuffer<float4> splats;
  cudadetail::DeviceBuffer<int> splat_gid, splat_line, contrib;
  cudadetail::DeviceBuffer<uint2> ranges;
  cudadetail::DeviceBuffer<float> out, trans;
  cudadetail::ThrustScratch scratch;

  // Training.
  int t_lines = 0, t_width = 0, t_bands = 0;
  cudadetail::DeviceBuffer<float> targets;
  cudadetail::DeviceBuffer<int> line_ids;
  cudadetail::DeviceBuffer<float> sq_err, g_bands, g_pix;
  cudadetail::DeviceBuffer<float4> g_splat;
  cudadetail::DeviceBuffer<float> g_mean, g_cov, g_opacity, g_feat, g_bg, g_basis, g_screen, g_cam;
  cudadetail::DeviceBuffer<int> g_pairs;
  cudadetail::DeviceBuffer<float> g_log_scales, g_rotations, g_logits;

  // Lines per batch for width w: bounded by the pair budget, the grid's y
  // limit, and keeping every key count inside 32 bits.
  int batch_lines(long long max_pairs, int w) const;

  // Copies a scene up (features padded to kp) and computes its geometry.
  void upload_scene(const GaussianScene& scene);
  // The scene as it is on the GPU, without the padding.
  void download_scene(GaussianScene* scene) const;
  // Each Gaussian's covariance and opacity from its parameters (after they
  // change on the GPU).
  void prepare();

  // Passes 1-5 for lb lines of width w, whose cameras are already in `cams`.
  // Leaves the sorted lists, splats, features (out), transmittance and
  // contributors on the GPU.
  void forward(int lb, int w, uint32_t* n_vis_out, uint32_t* n_entries_out, cudadetail::PassClock& clock);

  // mse_backward, leaving the scene's gradient in g_* on the GPU (with
  // dL/dbasis in g_basis if learn_basis). Only the cameras' gradients come
  // back, into *cam_grad if it isn't null.
  double backward(const std::vector<LineCamera>& cams, const std::vector<int>& lines, bool learn_basis,
                  std::vector<CameraGradT<float>>* cam_grad, long long max_pairs, bool profile,
                  CudaRenderStats* stats);
};

}  // namespace linesplat
