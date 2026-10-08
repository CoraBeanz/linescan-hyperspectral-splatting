// The CPU reference backward pass: renders lines, asks a loss for dL/dbands,
// and backpropagates into the scene and the cameras.
//
// Per line it does what 3DGS does per image. The forward pass composites
// front to back and remembers, per pixel, how far down the depth-sorted list
// it got and what transmittance was left. The backward pass walks the same
// list back to front, recovering each splat's transmittance by dividing by
// (1 - alpha), and keeps the color "behind" each splat:
//
//   pixel = sum_i c_i a_i T_i + T_final bg,     T_i = prod_{j<i} (1 - a_j)
//   dL/dc_i = a_i T_i dL/dpixel
//   dL/da_i = T_i (c_i - behind_i) . dL/dpixel,  behind_i = what the pixel would
//             show if splat i were removed, divided by T_i (1 - a_i)
//
// The per-pixel alpha gradients collect into each (line, Gaussian) splat's
// u, inv_var and alpha, then flow through project_to_line_backward to the
// Gaussian and the camera. The spectral basis gets a gradient too if asked
// for (SceneGradT::learn_basis): bands = basis * features per pixel, so
// dL/dbasis is the sum over pixels of dL/dbands times the pixel's features.
#pragma once

#include <functional>
#include <vector>

#include "linesplat/gradients.hpp"
#include "linesplat/render_cpu.hpp"
#include "linesplat/scene.hpp"

namespace linesplat {

// dL/d(every trainable array of a GaussianScene), same layouts.
template <typename T>
struct SceneGradT {
  std::vector<T> means;           // [N, 3]
  std::vector<T> log_scales;      // [N, 3]
  std::vector<T> rotations;       // [N, 4]
  std::vector<T> opacity_logits;  // [N]
  std::vector<T> features;        // [N, K]
  std::vector<T> background;      // [K]
  std::vector<T> basis;           // [bands, K], only with learn_basis
  // For densification: per Gaussian, the summed norm of dL/d(projected
  // centre) in px over the (line, Gaussian) pairs that drew a pixel, and the
  // number of those pairs.
  std::vector<T> screen_grad;     // [N]
  std::vector<int> pairs;         // [N]
  // Set by the caller: also backpropagate into the spectral basis. It costs
  // bands x K multiply-adds per pixel, as much as applying the basis.
  bool learn_basis = false;

  void reset(const GaussianScene& s);  // sized for s, all zero (basis only with learn_basis)
};

// dL/d(world -> camera rotation and translation) of one line's camera.
template <typename T>
struct CameraGradT {
  T R[9];
  T t[3];
};

// Called once per line with that line's rendered bands [W, B]. It writes
// dL/dbands into grad [W, B] (zeroed before the call) and returns the line's
// loss. Lines are processed in parallel, so it must be safe to call
// concurrently for different lines.
template <typename T>
using LineLossFn = std::function<double(int line, const T* bands, T* grad)>;

// Renders every camera, evaluates the loss, and fills *grad (and *cam_grad,
// one entry per camera, if not null). Returns the summed loss. If bands_out
// is not null it receives the rendered bands [lines, W, B].
template <typename T>
double render_backward_cpu(const GaussianScene& scene, const std::vector<LineCameraT<T>>& cams,
                           const LineLossFn<T>& loss, SceneGradT<T>* grad,
                           std::vector<CameraGradT<T>>* cam_grad = nullptr, std::vector<T>* bands_out = nullptr);

}  // namespace linesplat
