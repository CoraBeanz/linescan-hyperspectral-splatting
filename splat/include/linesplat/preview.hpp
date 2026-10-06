// Quick-look images of spectral data: true color and color-infrared PNGs,
// and a pinhole camera built from scan lines.
#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "linesplat/line_camera.hpp"
#include "linesplat/scan_model.hpp"

namespace linesplat {

enum class PreviewMode { kTrueColor, kColorInfrared };

// spectra: rows * cols spectra of `wavelengths.size()` bands each, row-major.
void write_spectral_png(const std::string& path, const float* spectra, int rows, int cols,
                        const std::vector<double>& wavelengths, PreviewMode mode, float gain = 1.0f);

// A regular pinhole image is a stack of line cameras, one per row, that share
// a pose and differ only in where their "slit" sits. Row r gets
// v_slit = r + 0.5 - height / 2, and the blur is one pixel's box.
std::vector<LineCamera> pinhole_rows(const Pose& camera_in_world, int width, int height, double f);

}  // namespace linesplat
