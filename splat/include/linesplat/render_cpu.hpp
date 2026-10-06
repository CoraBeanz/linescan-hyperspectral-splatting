// The CPU reference renderer.
//
// It renders scan lines with exactly the semantics of the CUDA rasterizer
// (same projection, same cutoffs, same front-to-back compositing), but in the
// most direct way: for each line, project every Gaussian, sort the hits by
// depth, and composite each pixel. Tests compare the GPU against it, and it
// runs in double precision for gradient checks.
#pragma once

#include <vector>

#include "linesplat/line_camera.hpp"
#include "linesplat/scene.hpp"

namespace linesplat {

template <typename T>
struct LineImageT {
  int lines = 0;
  int width = 0;
  int channels = 0;
  std::vector<T> values;          // [lines, width, channels]
  std::vector<T> transmittance;   // [lines, width]: the fraction left for the background
  std::vector<int> contributors;  // [lines, width]: CPU, splats blended into the pixel;
                                  // GPU, how far into its tile's list the pixel read

  T at(int line, int p, int c) const { return values[(size_t(line) * width + p) * channels + c]; }
};
using LineImage = LineImageT<float>;

// Renders the scene's features (K channels, background included) on every
// camera. All cameras must have the same width.
template <typename T>
LineImageT<T> render_lines_cpu(const GaussianScene& scene, const std::vector<LineCameraT<T>>& cams);

// Applies the scene's spectral basis per pixel: [.., K] -> [.., bands].
template <typename T>
LineImageT<T> features_to_bands(const GaussianScene& scene, const LineImageT<T>& features);

// Visible (line, Gaussian) pairs of one camera, sorted front to back, for
// tests and debugging.
template <typename T>
std::vector<std::pair<LineSplatT<T>, int>> project_scene(const GaussianScene& scene, const LineCameraT<T>& cam);

}  // namespace linesplat
