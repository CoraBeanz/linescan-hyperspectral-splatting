// Wavelength grids, the synthetic scene's material spectra, and turning
// spectra into preview colors.
#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace linesplat {

// n band centres spread evenly from lo to hi (nm), both ends included.
std::vector<double> wavelength_grid(double lo_nm, double hi_nm, int n);

// Reflectance spectra (0..1) of the synthetic scene's materials, as smooth
// analytic curves loosely shaped after real ones.
enum class Material : int {
  kWhitePaper = 0,
  kCarbonBlack,    // black in the visible and the NIR
  kIrBlackDye,     // black in the visible, bright past ~740 nm (IR-transparent ink)
  kLeaf,           // chlorophyll: green bump, red absorption, red edge at ~715 nm
  kRedPaint,
  kOrangePlastic,
  kYellowPaint,
  kGreenPaint,     // looks like the leaf by eye, but has no red edge
  kBluePaint,
  kGray18,
  kWood,           // the table under the board
  kCount
};
const char* material_name(Material m);
double reflectance(Material m, double nm);
std::vector<float> material_spectrum(Material m, const std::vector<double>& wavelengths_nm);

// Preview colors of a spectrum sampled at `wavelengths_nm`.
//  true_color: the CIE 1931 observer under equal-energy light, white balanced
//              so reflectance 1 is white. Below the first band (the GG-495
//              filter cuts there) the first band's value is held, so blues
//              are only approximate.
//  color_infrared: the classic CIR false color, R = 800-900 nm, G = 620-680 nm,
//              B = 520-580 nm. Vegetation comes out red.
std::array<float, 3> true_color(const float* spectrum, const std::vector<double>& wavelengths_nm);
std::array<float, 3> color_infrared(const float* spectrum, const std::vector<double>& wavelengths_nm);

// Linear [0, 1] -> 8-bit sRGB.
uint8_t to_srgb8(float linear);

}  // namespace linesplat
