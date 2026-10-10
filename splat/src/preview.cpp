#include "linesplat/preview.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

#include "linesplat/dataset.hpp"
#include "linesplat/png.hpp"
#include "linesplat/spectra.hpp"

namespace linesplat {

void write_spectral_png(const std::string& path, const float* spectra, int rows, int cols,
                        const std::vector<double>& wl, PreviewMode mode, float gain) {
  const size_t B = wl.size();
  const auto w = color_weights(mode == PreviewMode::kTrueColor ? true_color : color_infrared, wl);
  std::vector<uint8_t> rgb(size_t(rows) * cols * 3);
  for (size_t i = 0; i < size_t(rows) * cols; ++i) {
    const float* s = spectra + i * B;
    float c[3] = {0.0f, 0.0f, 0.0f};
    for (size_t b = 0; b < B; ++b)
      for (int k = 0; k < 3; ++k) c[k] += w[b][size_t(k)] * s[b];
    for (int k = 0; k < 3; ++k) rgb[3 * i + k] = to_srgb8(gain * c[k]);
  }
  write_png_rgb(path, cols, rows, rgb);
}

void write_sweep_pngs(const std::string& prefix, const Dataset& d, const std::vector<const float*>& panels) {
  const int W = d.width(), B = d.num_bands(), gap = 4, P = int(panels.size());
  const int cols = P * W + (P - 1) * gap;
  for (int s = 0; s < d.num_sweeps(); ++s) {
    std::vector<int> rows;
    for (int l = 0; l < d.num_lines(); ++l)
      if (d.line_sweep[size_t(l)] == s) rows.push_back(l);
    std::vector<float> img(rows.size() * size_t(cols) * B, 1.0f);  // white between panels
    for (size_t r = 0; r < rows.size(); ++r)
      for (int p = 0; p < P; ++p)
        std::copy_n(panels[size_t(p)] + size_t(rows[r]) * W * B, size_t(W) * B,
                    &img[(r * cols + size_t(p) * (W + gap)) * B]);
    char name[32];
    std::snprintf(name, sizeof name, "_sweep_%02d.png", s);
    write_spectral_png(prefix + name, img.data(), int(rows.size()), cols, d.wavelengths_nm, PreviewMode::kTrueColor);
  }
}

std::vector<LineCameraT<double>> pinhole_rows_d(const Pose& camera_in_world, int width, int height, double f) {
  LineIntrinsics in;
  in.width = width;
  in.f = f;
  in.cu = 0.5 * width;
  in.sigma_u = in.sigma_v = std::sqrt(1.0 / 12.0);
  in.near_z = 0.005;
  std::vector<LineCameraT<double>> cams(static_cast<size_t>(height));
  for (int r = 0; r < height; ++r) {
    in.v_slit = r + 0.5 - 0.5 * height;
    cams[size_t(r)] = line_camera_from_pose(camera_in_world, in);
  }
  return cams;
}

std::vector<LineCamera> pinhole_rows(const Pose& camera_in_world, int width, int height, double f) {
  std::vector<LineCamera> cams;
  for (const auto& c : pinhole_rows_d(camera_in_world, width, height, f)) cams.push_back(cast_camera<float>(c));
  return cams;
}

}  // namespace linesplat
