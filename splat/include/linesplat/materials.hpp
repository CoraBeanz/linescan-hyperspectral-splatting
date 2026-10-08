// Material maps: what each Gaussian of a hyperspectral splat is made of,
// worked out from its spectrum alone. Three standard methods, each a little
// linear algebra per Gaussian:
//
//  - Spectral angle mapping (SAM). A spectrum is a vector with one entry per
//    band; the angle between it and each spectrum in a library says how alike
//    their shapes are, and the smallest angle names the material. An angle
//    ignores brightness, so shading doesn't change it, but neither does black
//    against gray against white (all flat), so only library spectra that match
//    within a brightness window are candidates.
//  - k-means clustering. Groups of Gaussians with similar spectra, found
//    without a library: each Gaussian joins the nearest of k mean spectra,
//    the means move to the middle of their groups, and that repeats.
//  - Linear unmixing. Each spectrum as a mix of a few pure "endmember"
//    spectra, with abundances that are non-negative and sum to one (fully
//    constrained least squares), for Gaussians that straddle two materials.
#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace linesplat {

struct SpectralLibrary {
  std::vector<std::string> names;
  std::vector<std::string> colors;     // "#rrggbb", each material's color on a map
  std::vector<double> wavelengths_nm;  // [B]
  std::vector<float> spectra;          // [M, B]

  int size() const { return int(names.size()); }
  int bands() const { return int(wavelengths_nm.size()); }
  const float* spectrum(int m) const { return &spectra[size_t(m) * wavelengths_nm.size()]; }
};

// The synthetic scene's materials (spectra.hpp), in Material order, sampled at
// `wavelengths_nm`.
SpectralLibrary builtin_library(const std::vector<double>& wavelengths_nm);

// A CSV file: a header row "nm,<name>,<name>,...", then one row per
// wavelength, ascending. An optional row whose first cell is "color" gives
// each material's map color as #rrggbb. Values are interpolated linearly onto
// `wavelengths_nm`, which must lie within the file's wavelengths.
SpectralLibrary load_library_csv(const std::string& path, const std::vector<double>& wavelengths_nm);

// Up to 20 well-separated colors for categories with no color of their own.
const std::vector<std::string>& category_colors();

// The angle between two spectra in radians: 0 for the same shape, whatever
// the brightness.
double spectral_angle(const float* a, const float* b, int bands);

struct SamOptions {
  double max_angle_deg = 10.0;     // farther than this from every candidate: unknown
  double brightness_window = 2.0;  // candidates match after scaling by 1/window..window (0: any)
};

struct SamResult {
  std::vector<int> label;    // [N] library index, -1 for unknown
  std::vector<float> angle;  // [N] the angle to the best candidate (rad), or -1 if there was none
};

// spectra: [n, lib.bands()].
SamResult classify_sam(const float* spectra, int n, const SpectralLibrary& lib, const SamOptions& opt);

struct KMeansResult {
  std::vector<int> label;        // [N] cluster, largest first
  std::vector<float> centroids;  // [k, B] each cluster's weighted mean spectrum
  std::vector<double> weight;    // [k] the weights of each cluster's members, summed
  double inertia = 0.0;          // weighted sum of squared distances to the centroids
  int iterations = 0;
};

// Weighted k-means on spectra as points in `bands` dimensions: Lloyd's
// iterations from k-means++ seeds, until no Gaussian changes cluster.
// weights: [n] (opacity, say), or empty for all ones. Clusters are numbered
// by total weight, largest first.
KMeansResult kmeans(const float* spectra, int n, int bands, const std::vector<float>& weights, int k, uint64_t seed,
                    int max_iterations = 200);

// Non-negative least squares from the normal equations: the a >= 0 that
// minimizes |E a - s|^2, given G = E^T E (m x m, row-major) and b = E^T s.
// Lawson and Hanson's active-set method.
std::vector<double> nnls_normal(const std::vector<double>& G, const std::vector<double>& b, int m);

struct Unmixing {
  std::vector<float> abundances;  // [N, M], each row non-negative and summing to 1
  std::vector<float> residual;    // [N] RMS over the bands of the spectrum minus the mix
};

// Fully constrained least squares (Heinz and Chang, 2001): sum-to-one is added
// as one heavily weighted extra equation, and NNLS does the rest.
// endmembers: [m, bands].
Unmixing unmix_fcls(const float* spectra, int n, const float* endmembers, int m, int bands);

// How alike two labelings of the same items are, 1 for the same grouping
// (whatever the numbers) and about 0 for chance. Items with a negative label
// in either are left out.
double adjusted_rand_index(const std::vector<int>& a, const std::vector<int>& b);

}  // namespace linesplat
