#include "linesplat/spectra.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace linesplat {

namespace {

double sig(double x) { return 1.0 / (1.0 + std::exp(-x)); }
double bump(double x, double mu, double s) { return std::exp(-0.5 * (x - mu) * (x - mu) / (s * s)); }

// Spectrum value at nm by linear interpolation, held constant past either end.
double sample(const float* spec, const std::vector<double>& wl, double nm) {
  const size_t n = wl.size();
  if (nm <= wl.front()) return spec[0];
  if (nm >= wl.back()) return spec[n - 1];
  const size_t i = size_t(std::upper_bound(wl.begin(), wl.end(), nm) - wl.begin());
  const double t = (nm - wl[i - 1]) / (wl[i] - wl[i - 1]);
  return (1.0 - t) * spec[i - 1] + t * spec[i];
}

// Analytic fit to the CIE 1931 color matching functions (Wyman, Sloan and
// Shirley, "Simple Analytic Approximations to the CIE XYZ Color Matching
// Functions", JCGT 2013).
double lobe(double x, double mu, double s1, double s2) {
  const double s = x < mu ? s1 : s2;
  return std::exp(-0.5 * (x - mu) * (x - mu) / (s * s));
}
void cie_xyz(double nm, double* x, double* y, double* z) {
  *x = 1.056 * lobe(nm, 599.8, 37.9, 31.0) + 0.362 * lobe(nm, 442.0, 16.0, 26.7) -
       0.065 * lobe(nm, 501.1, 20.4, 26.2);
  *y = 0.821 * lobe(nm, 568.8, 46.9, 40.5) + 0.286 * lobe(nm, 530.9, 16.3, 31.1);
  *z = 1.217 * lobe(nm, 437.0, 11.8, 36.0) + 0.681 * lobe(nm, 459.0, 26.0, 13.8);
}

std::array<double, 3> xyz_to_linear_srgb(double X, double Y, double Z) {
  return {3.2406 * X - 1.5372 * Y - 0.4986 * Z, -0.9689 * X + 1.8758 * Y + 0.0415 * Z,
          0.0557 * X - 0.2040 * Y + 1.0570 * Z};
}

double band_mean(const float* spec, const std::vector<double>& wl, double lo, double hi) {
  double s = 0.0;
  int n = 0;
  for (double nm = lo; nm <= hi + 1e-9; nm += 1.0, ++n) s += sample(spec, wl, nm);
  return s / n;
}

}  // namespace

std::vector<double> wavelength_grid(double lo_nm, double hi_nm, int n) {
  if (n < 1) throw std::runtime_error("wavelength_grid: need at least one band");
  std::vector<double> w(static_cast<size_t>(n));
  for (int i = 0; i < n; ++i) w[size_t(i)] = n == 1 ? lo_nm : lo_nm + (hi_nm - lo_nm) * i / (n - 1);
  return w;
}

const char* material_name(Material m) {
  switch (m) {
    case Material::kWhitePaper: return "white paper";
    case Material::kCarbonBlack: return "carbon black";
    case Material::kIrBlackDye: return "IR-transparent black dye";
    case Material::kLeaf: return "leaf";
    case Material::kRedPaint: return "red paint";
    case Material::kOrangePlastic: return "orange plastic";
    case Material::kYellowPaint: return "yellow paint";
    case Material::kGreenPaint: return "green paint";
    case Material::kBluePaint: return "blue paint";
    case Material::kGray18: return "18% gray";
    case Material::kWood: return "wood";
    default: return "?";
  }
}

double reflectance(Material m, double nm) {
  switch (m) {
    case Material::kWhitePaper: return 0.86 - 0.05 * (nm - 500.0) / 450.0;
    case Material::kCarbonBlack: return 0.04;
    case Material::kIrBlackDye: return 0.04 + 0.74 * sig((nm - 740.0) / 16.0);
    case Material::kLeaf:
      return 0.045 + 0.085 * bump(nm, 553.0, 32.0) - 0.015 * bump(nm, 676.0, 16.0) +
             0.46 * sig((nm - 714.0) / 13.0) - 0.05 * bump(nm, 970.0, 22.0);
    case Material::kRedPaint: return 0.05 + 0.72 * sig((nm - 603.0) / 11.0);
    case Material::kOrangePlastic: return 0.06 + 0.74 * sig((nm - 572.0) / 11.0);
    case Material::kYellowPaint: return 0.08 + 0.74 * sig((nm - 518.0) / 10.0);
    case Material::kGreenPaint: return 0.05 + 0.26 * bump(nm, 535.0, 32.0) + 0.40 * sig((nm - 775.0) / 20.0);
    case Material::kBluePaint:
      return 0.06 + 0.28 * (1.0 - sig((nm - 535.0) / 18.0)) + 0.48 * sig((nm - 745.0) / 22.0);
    case Material::kGray18: return 0.18;
    case Material::kWood: return 0.10 + 0.30 * sig((nm - 610.0) / 45.0);
    default: return 0.0;
  }
}

std::vector<float> material_spectrum(Material m, const std::vector<double>& wl) {
  std::vector<float> s(wl.size());
  for (size_t i = 0; i < wl.size(); ++i) s[i] = float(reflectance(m, wl[i]));
  return s;
}

std::array<float, 3> true_color(const float* spectrum, const std::vector<double>& wl) {
  double X = 0, Y = 0, Z = 0, Xw = 0, Yw = 0, Zw = 0;
  for (double nm = 380.0; nm <= 780.0; nm += 2.0) {
    double x, y, z;
    cie_xyz(nm, &x, &y, &z);
    const double r = sample(spectrum, wl, nm);
    X += x * r;
    Y += y * r;
    Z += z * r;
    Xw += x;
    Yw += y;
    Zw += z;
  }
  // White balance: divide by the color of a perfect white, channel by channel.
  const auto c = xyz_to_linear_srgb(X / Yw, Y / Yw, Z / Yw);
  const auto w = xyz_to_linear_srgb(Xw / Yw, 1.0, Zw / Yw);
  return {float(c[0] / w[0]), float(c[1] / w[1]), float(c[2] / w[2])};
}

std::array<float, 3> color_infrared(const float* spectrum, const std::vector<double>& wl) {
  return {float(band_mean(spectrum, wl, 800.0, 900.0)), float(band_mean(spectrum, wl, 620.0, 680.0)),
          float(band_mean(spectrum, wl, 520.0, 580.0))};
}

uint8_t to_srgb8(float x) {
  x = std::min(std::max(x, 0.0f), 1.0f);
  const float s = x <= 0.0031308f ? 12.92f * x : 1.055f * std::pow(x, 1.0f / 2.4f) - 0.055f;
  return uint8_t(std::lround(255.0f * s));
}

}  // namespace linesplat
