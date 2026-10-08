#include "linesplat/materials.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <sstream>
#include <stdexcept>

#include "linesplat/math.hpp"
#include "linesplat/rng.hpp"
#include "linesplat/spectra.hpp"
#include "linesplat/util.hpp"

namespace linesplat {

namespace {

// Map colors of the synthetic materials, in Material order. Hues hint at the
// material where they can, but the two blacks and the two greens, which look
// alike to the eye, get colors far apart: telling those apart is the point.
const char* const kBuiltinColors[] = {
    "#d8d3c4",  // white paper
    "#5b626c",  // carbon black
    "#9a6fdc",  // IR-transparent black dye
    "#4caf50",  // leaf
    "#e0463c",  // red paint
    "#f08a24",  // orange plastic
    "#f2d03b",  // yellow paint
    "#1fb5a8",  // green paint
    "#3f7fe0",  // blue paint
    "#9aa0a8",  // 18% gray
    "#a87648",  // wood
};
static_assert(sizeof(kBuiltinColors) / sizeof(kBuiltinColors[0]) == size_t(Material::kCount),
              "a color for every material");

std::vector<std::string> split_csv_row(const std::string& line) {
  std::vector<std::string> cells;
  std::stringstream ss(line);
  std::string cell;
  while (std::getline(ss, cell, ',')) {
    const size_t a = cell.find_first_not_of(" \t\r\""), b = cell.find_last_not_of(" \t\r\"");
    cells.push_back(a == std::string::npos ? std::string() : cell.substr(a, b - a + 1));
  }
  return cells;
}

double dot(const float* a, const float* b, int n) {
  double s = 0.0;
  for (int i = 0; i < n; ++i) s += double(a[i]) * b[i];
  return s;
}

double sq_dist(const float* a, const float* b, int n) {
  double s = 0.0;
  for (int i = 0; i < n; ++i) {
    const double d = double(a[i]) - b[i];
    s += d * d;
  }
  return s;
}

// Solves A x = y for a symmetric positive definite A (n x n, row-major) by
// Cholesky. A tiny ridge keeps nearly dependent columns solvable.
std::vector<double> solve_spd(std::vector<double> A, std::vector<double> y, int n) {
  double scale = 0.0;
  for (int i = 0; i < n; ++i) scale = std::max(scale, A[size_t(i) * n + i]);
  for (int i = 0; i < n; ++i) A[size_t(i) * n + i] += 1e-12 * scale + 1e-300;
  for (int j = 0; j < n; ++j) {
    double d = A[size_t(j) * n + j];
    for (int k = 0; k < j; ++k) d -= A[size_t(j) * n + k] * A[size_t(j) * n + k];
    d = std::sqrt(std::max(d, 1e-300));
    A[size_t(j) * n + j] = d;
    for (int i = j + 1; i < n; ++i) {
      double s = A[size_t(i) * n + j];
      for (int k = 0; k < j; ++k) s -= A[size_t(i) * n + k] * A[size_t(j) * n + k];
      A[size_t(i) * n + j] = s / d;
    }
  }
  for (int i = 0; i < n; ++i) {  // L z = y
    double s = y[size_t(i)];
    for (int k = 0; k < i; ++k) s -= A[size_t(i) * n + k] * y[size_t(k)];
    y[size_t(i)] = s / A[size_t(i) * n + i];
  }
  for (int i = n - 1; i >= 0; --i) {  // L^T x = z
    double s = y[size_t(i)];
    for (int k = i + 1; k < n; ++k) s -= A[size_t(k) * n + i] * y[size_t(k)];
    y[size_t(i)] = s / A[size_t(i) * n + i];
  }
  return y;
}

}  // namespace

const std::vector<std::string>& category_colors() {
  // Tableau 20, which keeps its colors apart on dark and light backgrounds.
  static const std::vector<std::string> colors = {
      "#4e79a7", "#f28e2b", "#e15759", "#76b7b2", "#59a14f", "#edc948", "#b07aa1", "#ff9da7", "#9c755f", "#bab0ac",
      "#a0cbe8", "#ffbe7d", "#8cd17d", "#b6992d", "#f1ce63", "#499894", "#86bcb6", "#d37295", "#fabfd2", "#d4a6c8"};
  return colors;
}

SpectralLibrary builtin_library(const std::vector<double>& wl) {
  SpectralLibrary lib;
  lib.wavelengths_nm = wl;
  for (int m = 0; m < int(Material::kCount); ++m) {
    lib.names.push_back(material_name(Material(m)));
    lib.colors.push_back(kBuiltinColors[m]);
    const std::vector<float> s = material_spectrum(Material(m), wl);
    lib.spectra.insert(lib.spectra.end(), s.begin(), s.end());
  }
  return lib;
}

SpectralLibrary load_library_csv(const std::string& path, const std::vector<double>& wl) {
  std::stringstream text(read_text_file(path));
  std::string line;
  std::vector<std::string> names, colors;
  std::vector<double> nm;
  std::vector<std::vector<double>> rows;
  while (std::getline(text, line)) {
    const std::vector<std::string> cells = split_csv_row(line);
    if (cells.empty() || cells[0].empty() || cells[0][0] == '#') continue;
    if (names.empty()) {
      names.assign(cells.begin() + 1, cells.end());
      if (names.empty()) throw std::runtime_error(path + ": the header names no materials");
      continue;
    }
    if (cells.size() != names.size() + 1)
      throw std::runtime_error(path + ": a row has " + std::to_string(cells.size()) + " cells, not " +
                               std::to_string(names.size() + 1));
    if (cells[0] == "color") {
      colors.assign(cells.begin() + 1, cells.end());
      continue;
    }
    nm.push_back(std::stod(cells[0]));
    std::vector<double> r;
    for (size_t c = 1; c < cells.size(); ++c) r.push_back(std::stod(cells[c]));
    rows.push_back(r);
    if (nm.size() > 1 && !(nm.back() > nm[nm.size() - 2]))
      throw std::runtime_error(path + ": wavelengths must go up row by row");
  }
  if (nm.size() < 2) throw std::runtime_error(path + ": needs at least two wavelengths");

  SpectralLibrary lib;
  lib.names = names;
  lib.wavelengths_nm = wl;
  for (size_t m = 0; m < names.size(); ++m)
    lib.colors.push_back(m < colors.size() && colors[m].size() == 7 && colors[m][0] == '#'
                             ? colors[m]
                             : category_colors()[m % category_colors().size()]);
  lib.spectra.resize(names.size() * wl.size());
  for (size_t b = 0; b < wl.size(); ++b) {
    const double x = wl[b];
    if (x < nm.front() - 1e-6 || x > nm.back() + 1e-6)
      throw std::runtime_error(path + " covers " + std::to_string(nm.front()) + " to " + std::to_string(nm.back()) +
                               " nm, not " + std::to_string(x) + " nm");
    size_t i = size_t(std::upper_bound(nm.begin(), nm.end(), x) - nm.begin());
    i = std::min(std::max<size_t>(i, 1), nm.size() - 1);
    const double t = std::min(std::max((x - nm[i - 1]) / (nm[i] - nm[i - 1]), 0.0), 1.0);
    for (size_t m = 0; m < names.size(); ++m)
      lib.spectra[m * wl.size() + b] = float((1.0 - t) * rows[i - 1][m] + t * rows[i][m]);
  }
  return lib;
}

double spectral_angle(const float* a, const float* b, int n) {
  const double na = std::sqrt(dot(a, a, n)), nb = std::sqrt(dot(b, b, n));
  if (!(na > 0.0) || !(nb > 0.0)) return 0.5 * kPi;
  return std::acos(std::min(1.0, std::max(-1.0, dot(a, b, n) / (na * nb))));
}

SamResult classify_sam(const float* spectra, int n, const SpectralLibrary& lib, const SamOptions& opt) {
  const int B = lib.bands(), M = lib.size();
  const double max_angle = opt.max_angle_deg * kPi / 180.0;
  std::vector<double> lib_sq(static_cast<size_t>(M));
  for (int m = 0; m < M; ++m) lib_sq[size_t(m)] = dot(lib.spectrum(m), lib.spectrum(m), B);
  SamResult r;
  r.label.assign(size_t(n), -1);
  r.angle.assign(size_t(n), -1.0f);
#pragma omp parallel for schedule(static)
  for (int i = 0; i < n; ++i) {
    const float* s = spectra + size_t(i) * B;
    double best = std::numeric_limits<double>::infinity();
    int arg = -1;
    for (int m = 0; m < M; ++m) {
      // The scale that fits the library spectrum to this one best; outside
      // the window, it's too bright or too dark to be that material.
      const double scale = dot(s, lib.spectrum(m), B) / lib_sq[size_t(m)];
      if (opt.brightness_window > 0.0 && !(scale >= 1.0 / opt.brightness_window && scale <= opt.brightness_window))
        continue;
      const double a = spectral_angle(s, lib.spectrum(m), B);
      if (a < best) {
        best = a;
        arg = m;
      }
    }
    if (arg < 0) continue;
    r.angle[size_t(i)] = float(best);
    if (best <= max_angle) r.label[size_t(i)] = arg;
  }
  return r;
}

KMeansResult kmeans(const float* spectra, int n, int B, const std::vector<float>& weights, int k, uint64_t seed,
                    int max_iterations) {
  if (k < 1 || n < k) throw std::runtime_error("k-means needs 1 <= k <= the number of points");
  auto w = [&](int i) { return weights.empty() ? 1.0 : std::max(0.0, double(weights[size_t(i)])); };
  Rng rng(seed);
  KMeansResult r;
  std::vector<float>& c = r.centroids;
  // k-means++: each new seed is a point picked with probability in
  // proportion to its weighted squared distance from the seeds so far.
  std::vector<double> d2(size_t(n), std::numeric_limits<double>::infinity());
  auto add_seed = [&](int i) {
    c.insert(c.end(), spectra + size_t(i) * B, spectra + size_t(i + 1) * B);
    for (int j = 0; j < n; ++j)
      d2[size_t(j)] = std::min(d2[size_t(j)], sq_dist(spectra + size_t(j) * B, spectra + size_t(i) * B, B));
  };
  {
    double total = 0.0;
    for (int i = 0; i < n; ++i) total += w(i);
    double pick = rng.uniform() * total;
    int first = n - 1;
    for (int i = 0; i < n; ++i)
      if ((pick -= w(i)) < 0.0) {
        first = i;
        break;
      }
    add_seed(first);
  }
  while (int(c.size()) < k * B) {
    double total = 0.0;
    for (int i = 0; i < n; ++i) total += w(i) * d2[size_t(i)];
    int next = -1;
    if (total > 0.0) {
      double pick = rng.uniform() * total;
      for (int i = 0; i < n && next < 0; ++i)
        if ((pick -= w(i) * d2[size_t(i)]) < 0.0) next = i;
    }
    if (next < 0)  // every point sits on a seed already: take the farthest
      next = int(std::max_element(d2.begin(), d2.end()) - d2.begin());
    add_seed(next);
  }

  r.label.assign(size_t(n), -1);
  for (r.iterations = 1; r.iterations <= max_iterations; ++r.iterations) {
    int changed = 0;
#pragma omp parallel for schedule(static) reduction(+ : changed)
    for (int i = 0; i < n; ++i) {
      double best = std::numeric_limits<double>::infinity();
      int arg = 0;
      for (int j = 0; j < k; ++j) {
        const double d = sq_dist(spectra + size_t(i) * B, &c[size_t(j) * B], B);
        if (d < best) {
          best = d;
          arg = j;
        }
      }
      if (r.label[size_t(i)] != arg) ++changed;
      r.label[size_t(i)] = arg;
    }
    // Move each centroid to its members' weighted mean; an empty cluster
    // restarts at the point farthest from its own centroid.
    std::vector<double> sum(size_t(k) * B, 0.0), wsum(size_t(k), 0.0);
    for (int i = 0; i < n; ++i) {
      const int j = r.label[size_t(i)];
      wsum[size_t(j)] += w(i);
      for (int b = 0; b < B; ++b) sum[size_t(j) * B + b] += w(i) * spectra[size_t(i) * B + b];
    }
    for (int j = 0; j < k; ++j) {
      if (wsum[size_t(j)] > 0.0) {
        for (int b = 0; b < B; ++b) c[size_t(j) * B + b] = float(sum[size_t(j) * B + b] / wsum[size_t(j)]);
        continue;
      }
      int far = 0;
      double fd = -1.0;
      for (int i = 0; i < n; ++i) {
        const double d = sq_dist(spectra + size_t(i) * B, &c[size_t(r.label[size_t(i)]) * B], B);
        if (d > fd) {
          fd = d;
          far = i;
        }
      }
      std::copy(spectra + size_t(far) * B, spectra + size_t(far + 1) * B, c.begin() + std::ptrdiff_t(j) * B);
      ++changed;
    }
    if (changed == 0) break;
  }
  r.iterations = std::min(r.iterations, max_iterations);

  // Number the clusters by weight, largest first.
  std::vector<double> wsum(size_t(k), 0.0);
  for (int i = 0; i < n; ++i) wsum[size_t(r.label[size_t(i)])] += w(i);
  std::vector<int> order(static_cast<size_t>(k));
  for (int j = 0; j < k; ++j) order[size_t(j)] = j;
  std::stable_sort(order.begin(), order.end(), [&](int a, int b) { return wsum[size_t(a)] > wsum[size_t(b)]; });
  std::vector<int> rank(static_cast<size_t>(k));
  std::vector<float> sorted(c.size());
  for (int j = 0; j < k; ++j) {
    rank[size_t(order[size_t(j)])] = j;
    std::copy(c.begin() + std::ptrdiff_t(order[size_t(j)]) * B, c.begin() + std::ptrdiff_t(order[size_t(j)] + 1) * B,
              sorted.begin() + std::ptrdiff_t(j) * B);
    r.weight.push_back(wsum[size_t(order[size_t(j)])]);
  }
  c = sorted;
  for (int& l : r.label) l = rank[size_t(l)];
  for (int i = 0; i < n; ++i) r.inertia += w(i) * sq_dist(spectra + size_t(i) * B, &c[size_t(r.label[size_t(i)]) * B], B);
  return r;
}

std::vector<double> nnls_normal(const std::vector<double>& G, const std::vector<double>& b, int m) {
  std::vector<double> a(size_t(m), 0.0);
  std::vector<char> passive(size_t(m), 0);
  double bmax = 0.0;
  for (double v : b) bmax = std::max(bmax, std::fabs(v));
  const double tol = 1e-12 * std::max(bmax, 1e-300);
  auto gradient = [&] {  // w = b - G a: how much each coefficient wants to grow
    std::vector<double> w(b);
    for (int i = 0; i < m; ++i)
      for (int j = 0; j < m; ++j) w[size_t(i)] -= G[size_t(i) * m + j] * a[size_t(j)];
    return w;
  };
  for (int outer = 0; outer < 3 * m + 3; ++outer) {
    const std::vector<double> w = gradient();
    int j = -1;
    for (int i = 0; i < m; ++i)
      if (!passive[size_t(i)] && w[size_t(i)] > tol && (j < 0 || w[size_t(i)] > w[size_t(j)])) j = i;
    if (j < 0) break;
    passive[size_t(j)] = 1;
    for (int inner = 0; inner < 3 * m + 3; ++inner) {
      // The unconstrained least-squares solution on the passive set.
      std::vector<int> P;
      for (int i = 0; i < m; ++i)
        if (passive[size_t(i)]) P.push_back(i);
      const int p = int(P.size());
      std::vector<double> A(size_t(p) * p), y(static_cast<size_t>(p));
      for (int r = 0; r < p; ++r) {
        y[size_t(r)] = b[size_t(P[size_t(r)])];
        for (int c = 0; c < p; ++c) A[size_t(r) * p + c] = G[size_t(P[size_t(r)]) * m + P[size_t(c)]];
      }
      const std::vector<double> zp = solve_spd(A, y, p);
      std::vector<double> z(size_t(m), 0.0);
      bool positive = true;
      for (int r = 0; r < p; ++r) {
        z[size_t(P[size_t(r)])] = zp[size_t(r)];
        if (!(zp[size_t(r)] > 0.0)) positive = false;
      }
      if (positive) {
        a = z;
        break;
      }
      // Step from a toward z until the first coefficient reaches zero, and
      // drop the ones that did.
      double alpha = 1.0;
      for (int i : P)
        if (!(z[size_t(i)] > 0.0)) alpha = std::min(alpha, a[size_t(i)] / std::max(a[size_t(i)] - z[size_t(i)], 1e-300));
      for (int i = 0; i < m; ++i) a[size_t(i)] += alpha * (z[size_t(i)] - a[size_t(i)]);
      for (int i : P)
        if (a[size_t(i)] <= 1e-15) {
          a[size_t(i)] = 0.0;
          passive[size_t(i)] = 0;
        }
    }
  }
  return a;
}

Unmixing unmix_fcls(const float* spectra, int n, const float* E, int m, int B) {
  // G = E^T E over the bands, plus delta^2 for the row of ones that asks the
  // abundances to sum to one; delta^2 is 10^4 times the endmembers' mean
  // squared length, so that equation wins by far.
  std::vector<double> G(size_t(m) * m, 0.0);
  for (int i = 0; i < m; ++i)
    for (int j = 0; j < m; ++j) G[size_t(i) * m + j] = dot(E + size_t(i) * B, E + size_t(j) * B, B);
  double mean_sq = 0.0;
  for (int i = 0; i < m; ++i) mean_sq += G[size_t(i) * m + i] / m;
  const double delta2 = 1e4 * std::max(mean_sq, 1e-12);
  for (double& g : G) g += delta2;

  Unmixing u;
  u.abundances.assign(size_t(n) * m, 0.0f);
  u.residual.assign(size_t(n), 0.0f);
#pragma omp parallel for schedule(dynamic, 64)
  for (int i = 0; i < n; ++i) {
    const float* s = spectra + size_t(i) * B;
    std::vector<double> rhs(static_cast<size_t>(m));
    for (int j = 0; j < m; ++j) rhs[size_t(j)] = dot(E + size_t(j) * B, s, B) + delta2;
    std::vector<double> a = nnls_normal(G, rhs, m);
    double sum = 0.0;
    for (double v : a) sum += v;
    if (!(sum > 0.0)) continue;
    double r2 = 0.0;
    for (int b = 0; b < B; ++b) {
      double mix = 0.0;
      for (int j = 0; j < m; ++j) mix += a[size_t(j)] / sum * E[size_t(j) * B + b];
      r2 += (s[b] - mix) * (s[b] - mix);
    }
    for (int j = 0; j < m; ++j) u.abundances[size_t(i) * m + j] = float(a[size_t(j)] / sum);
    u.residual[size_t(i)] = float(std::sqrt(r2 / B));
  }
  return u;
}

double adjusted_rand_index(const std::vector<int>& a, const std::vector<int>& b) {
  if (a.size() != b.size()) throw std::runtime_error("adjusted_rand_index: different lengths");
  std::map<std::pair<int, int>, double> table;
  std::map<int, double> rows, cols;
  double n = 0.0;
  for (size_t i = 0; i < a.size(); ++i) {
    if (a[i] < 0 || b[i] < 0) continue;
    table[{a[i], b[i]}] += 1.0;
    rows[a[i]] += 1.0;
    cols[b[i]] += 1.0;
    n += 1.0;
  }
  auto pairs = [](double x) { return 0.5 * x * (x - 1.0); };
  double index = 0.0, ra = 0.0, cb = 0.0;
  for (const auto& t : table) index += pairs(t.second);
  for (const auto& r : rows) ra += pairs(r.second);
  for (const auto& c : cols) cb += pairs(c.second);
  const double expected = n > 1.0 ? ra * cb / pairs(n) : 0.0, best = 0.5 * (ra + cb);
  return best > expected ? (index - expected) / (best - expected) : 1.0;
}

}  // namespace linesplat
