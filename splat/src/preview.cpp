#include "linesplat/preview.hpp"

#include <cmath>

#include "linesplat/png.hpp"
#include "linesplat/spectra.hpp"

namespace linesplat {

void write_spectral_png(const std::string& path, const float* spectra, int rows, int cols,
                        const std::vector<double>& wl, PreviewMode mode, float gain) {
  const size_t B = wl.size();
  std::vector<uint8_t> rgb(size_t(rows) * cols * 3);
  for (size_t i = 0; i < size_t(rows) * cols; ++i) {
    const auto c = mode == PreviewMode::kTrueColor ? true_color(spectra + i * B, wl) : color_infrared(spectra + i * B, wl);
    for (int k = 0; k < 3; ++k) rgb[3 * i + k] = to_srgb8(gain * c[size_t(k)]);
  }
  write_png_rgb(path, cols, rows, rgb);
}

std::vector<LineCamera> pinhole_rows(const Pose& camera_in_world, int width, int height, double f) {
  LineIntrinsics in;
  in.width = width;
  in.f = f;
  in.cu = 0.5 * width;
  in.sigma_u = in.sigma_v = std::sqrt(1.0 / 12.0);
  in.near_z = 0.005;
  std::vector<LineCamera> cams(static_cast<size_t>(height));
  for (int r = 0; r < height; ++r) {
    in.v_slit = r + 0.5 - 0.5 * height;
    cams[size_t(r)] = cast_camera<float>(line_camera_from_pose(camera_in_world, in));
  }
  return cams;
}

}  // namespace linesplat
