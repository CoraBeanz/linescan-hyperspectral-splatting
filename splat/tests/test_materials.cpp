// Material maps (materials.hpp) and the viewer's file (lsplat.hpp).
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iterator>

#include "fixtures.hpp"
#include "linesplat/lsplat.hpp"
#include "linesplat/materials.hpp"
#include "linesplat/rng.hpp"
#include "linesplat/spectra.hpp"
#include "linesplat/synthetic.hpp"
#include "linesplat/util.hpp"
#include "test.hpp"

using namespace linesplat;

namespace {

std::string tmp_path(const char* name) {
  make_dirs(LINESPLAT_TEST_TMP);
  return join_path(LINESPLAT_TEST_TMP, name);
}

std::vector<char> read_bytes(const std::string& path) {
  std::ifstream f(path, std::ios::binary);
  return std::vector<char>((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
}

const std::vector<double> kWl = wavelength_grid(500.0, 950.0, 46);

// Spectra of library materials, each scaled by its own brightness.
std::vector<float> scaled_spectra(const SpectralLibrary& lib, const std::vector<int>& which,
                                  const std::vector<float>& scale) {
  std::vector<float> out;
  for (size_t i = 0; i < which.size(); ++i)
    for (int b = 0; b < lib.bands(); ++b) out.push_back(scale[i] * lib.spectrum(which[i])[b]);
  return out;
}

}  // namespace

TEST(spectral_angle_ignores_brightness) {
  const float a[3] = {1.0f, 2.0f, 3.0f}, b[3] = {0.5f, 1.0f, 1.5f}, c[3] = {-2.0f, 1.0f, 0.0f};
  CHECK_NEAR(spectral_angle(a, b, 3), 0.0, 1e-7);
  CHECK_NEAR(spectral_angle(a, c, 3), 0.5 * kPi, 1e-12);  // orthogonal
  const float d[2] = {1.0f, 0.0f}, e[2] = {1.0f, 1.0f};
  CHECK_NEAR(spectral_angle(d, e, 2), 0.25 * kPi, 1e-12);
}

TEST(builtin_library_is_the_synthetic_materials) {
  const SpectralLibrary lib = builtin_library(kWl);
  CHECK(lib.size() == int(Material::kCount));
  CHECK(lib.bands() == 46);
  for (int m = 0; m < lib.size(); ++m) {
    CHECK(lib.names[size_t(m)] == material_name(Material(m)));
    CHECK(lib.colors[size_t(m)].size() == 7 && lib.colors[size_t(m)][0] == '#');
    CHECK_NEAR(lib.spectrum(m)[10], reflectance(Material(m), kWl[10]), 1e-6);
  }
  // Every material gets its own color.
  std::vector<std::string> colors = lib.colors;
  std::sort(colors.begin(), colors.end());
  CHECK(std::unique(colors.begin(), colors.end()) == colors.end());
}

TEST(sam_names_scaled_library_spectra) {
  const SpectralLibrary lib = builtin_library(kWl);
  std::vector<int> which;
  std::vector<float> scale;
  for (int m = 0; m < lib.size(); ++m)
    for (float s : {0.6f, 1.0f, 1.7f}) {
      which.push_back(m);
      scale.push_back(s);
    }
  const std::vector<float> spectra = scaled_spectra(lib, which, scale);
  const SamResult r = classify_sam(spectra.data(), int(which.size()), lib, SamOptions());
  for (size_t i = 0; i < which.size(); ++i) {
    CHECK(r.label[i] == which[i]);
    CHECK(r.angle[i] < 1e-3f);
  }
}

TEST(sam_brightness_window_tells_black_from_gray) {
  // Carbon black (4%) and 18% gray are both flat, so their angle is 0; only
  // brightness tells them apart.
  const SpectralLibrary lib = builtin_library(kWl);
  const int black = int(Material::kCarbonBlack), gray = int(Material::kGray18);
  CHECK_NEAR(spectral_angle(lib.spectrum(black), lib.spectrum(gray), lib.bands()), 0.0, 1e-6);
  const std::vector<float> spectra = scaled_spectra(lib, {black, gray}, {1.0f, 1.0f});
  const SamResult r = classify_sam(spectra.data(), 2, lib, SamOptions());
  CHECK(r.label[0] == black);
  CHECK(r.label[1] == gray);

  // Far brighter than any material: no candidate, so unknown.
  const std::vector<float> bright = scaled_spectra(lib, {int(Material::kWhitePaper)}, {5.0f});
  SamResult u = classify_sam(bright.data(), 1, lib, SamOptions());
  CHECK(u.label[0] == -1 && u.angle[0] < 0.0f);
  // With no window, the shape alone names it.
  SamOptions any;
  any.brightness_window = 0.0;
  u = classify_sam(bright.data(), 1, lib, any);
  CHECK(u.label[0] == int(Material::kWhitePaper));
}

TEST(sam_leaves_unlike_spectra_unknown) {
  const SpectralLibrary lib = builtin_library(kWl);
  // A narrow peak at 650 nm, like a red LED: no material has that shape.
  std::vector<float> led(kWl.size());
  for (size_t b = 0; b < kWl.size(); ++b) led[b] = float(0.3 + 3.0 * std::exp(-0.5 * std::pow((kWl[b] - 650.0) / 8.0, 2)));
  const SamResult r = classify_sam(led.data(), 1, lib, SamOptions());
  CHECK(r.label[0] == -1);
  CHECK(r.angle[0] > float(10.0 * kPi / 180.0));
}

TEST(library_csv_interpolates_and_reads_colors) {
  const std::string path = tmp_path("library.csv");
  write_text_file(path,
                  "nm,first,second\n"
                  "color,#112233,not-a-color\n"
                  "# comments and blank lines are skipped\n"
                  "\n"
                  "400,0.0,1.0\n"
                  "600,0.2,1.0\n"
                  "1000,0.6,0.0\n");
  const SpectralLibrary lib = load_library_csv(path, {400.0, 500.0, 800.0});
  CHECK(lib.size() == 2);
  CHECK(lib.names[0] == "first" && lib.names[1] == "second");
  CHECK(lib.colors[0] == "#112233");
  CHECK(lib.colors[1] == category_colors()[1]);  // a default where the file's isn't a color
  CHECK_NEAR(lib.spectrum(0)[0], 0.0, 1e-7);
  CHECK_NEAR(lib.spectrum(0)[1], 0.1, 1e-7);
  CHECK_NEAR(lib.spectrum(0)[2], 0.4, 1e-7);
  CHECK_NEAR(lib.spectrum(1)[2], 0.5, 1e-7);
  bool threw = false;
  try {
    load_library_csv(path, {350.0});
  } catch (const std::exception&) {
    threw = true;
  }
  CHECK(threw);  // outside the file's wavelengths
}

TEST(kmeans_finds_separated_groups) {
  // Three groups of noisy points in 5 dimensions, of 60, 40 and 20 points.
  Rng rng(3);
  const float centres[3][5] = {{0, 0, 0, 0, 0}, {1, 1, 0, 0, 1}, {0, 1, 1, 1, 0}};
  std::vector<float> pts;
  std::vector<int> truth;
  for (int g = 0; g < 3; ++g)
    for (int i = 0; i < 60 - 20 * g; ++i) {
      for (int b = 0; b < 5; ++b) pts.push_back(centres[g][b] + float(0.05 * rng.normal()));
      truth.push_back(g);
    }
  const KMeansResult r = kmeans(pts.data(), int(truth.size()), 5, {}, 3, 7);
  CHECK_NEAR(adjusted_rand_index(truth, r.label), 1.0, 1e-12);
  // Largest cluster first, with the centroids at the group means.
  CHECK(r.weight[0] == 60 && r.weight[1] == 40 && r.weight[2] == 20);
  for (int b = 0; b < 5; ++b) CHECK_NEAR(r.centroids[size_t(b)], centres[0][b], 0.03);
  CHECK(r.iterations < 20);

  // Weights: a heavy point drags its cluster's mean toward it.
  std::vector<float> w(truth.size(), 1.0f);
  w[0] = 1000.0f;
  const KMeansResult h = kmeans(pts.data(), int(truth.size()), 5, w, 3, 7);
  CHECK_NEAR(h.centroids[0], pts[0], 0.01);
}

TEST(adjusted_rand_index_basics) {
  CHECK_NEAR(adjusted_rand_index({0, 0, 1, 1, 2, 2}, {5, 5, 3, 3, 9, 9}), 1.0, 1e-12);  // same grouping
  CHECK(adjusted_rand_index({0, 0, 1, 1}, {0, 1, 0, 1}) < 0.0);                       // worse than chance
  CHECK_NEAR(adjusted_rand_index({0, 0, -1, 1}, {2, 2, 0, 3}), 1.0, 1e-12);            // -1 is left out
}

TEST(nnls_matches_brute_force) {
  // For a small problem, the best non-negative solution is the best of the
  // unconstrained solutions on every subset of the coefficients.
  Rng rng(11);
  for (int trial = 0; trial < 50; ++trial) {
    const int m = 4, rows = 6;
    std::vector<double> E(size_t(rows) * m), s(static_cast<size_t>(rows));
    for (double& v : E) v = rng.normal();
    for (double& v : s) v = rng.normal();
    std::vector<double> G(size_t(m) * m, 0.0), b(static_cast<size_t>(m), 0.0);
    for (int i = 0; i < m; ++i) {
      for (int r = 0; r < rows; ++r) b[size_t(i)] += E[size_t(r) * m + i] * s[size_t(r)];
      for (int j = 0; j < m; ++j)
        for (int r = 0; r < rows; ++r) G[size_t(i) * m + j] += E[size_t(r) * m + i] * E[size_t(r) * m + j];
    }
    auto cost = [&](const std::vector<double>& a) {
      double c = 0.0;
      for (int r = 0; r < rows; ++r) {
        double e = -s[size_t(r)];
        for (int i = 0; i < m; ++i) e += E[size_t(r) * m + i] * a[size_t(i)];
        c += e * e;
      }
      return c;
    };
    const std::vector<double> a = nnls_normal(G, b, m);
    for (double v : a) CHECK(v >= 0.0);
    double best = cost(std::vector<double>(static_cast<size_t>(m), 0.0));
    for (int mask = 1; mask < (1 << m); ++mask) {
      std::vector<int> P;
      for (int i = 0; i < m; ++i)
        if (mask & (1 << i)) P.push_back(i);
      // Solve the normal equations on P by Gaussian elimination.
      const int p = int(P.size());
      std::vector<double> A(size_t(p) * (p + 1));
      for (int r = 0; r < p; ++r) {
        for (int c = 0; c < p; ++c) A[size_t(r) * (p + 1) + c] = G[size_t(P[size_t(r)]) * m + P[size_t(c)]];
        A[size_t(r) * (p + 1) + p] = b[size_t(P[size_t(r)])];
      }
      for (int c = 0; c < p; ++c)
        for (int r = c + 1; r < p; ++r) {
          const double f = A[size_t(r) * (p + 1) + c] / A[size_t(c) * (p + 1) + c];
          for (int k = c; k <= p; ++k) A[size_t(r) * (p + 1) + k] -= f * A[size_t(c) * (p + 1) + k];
        }
      std::vector<double> z(static_cast<size_t>(m), 0.0);
      bool feasible = true;
      for (int r = p - 1; r >= 0; --r) {
        double v = A[size_t(r) * (p + 1) + p];
        for (int k = r + 1; k < p; ++k) v -= A[size_t(r) * (p + 1) + k] * z[size_t(P[size_t(k)])];
        z[size_t(P[size_t(r)])] = v / A[size_t(r) * (p + 1) + r];
        if (z[size_t(P[size_t(r)])] < 0.0) feasible = false;
      }
      if (feasible) best = std::min(best, cost(z));
    }
    CHECK_NEAR(cost(a), best, 1e-9 * std::max(1.0, best));
  }
}

TEST(fcls_recovers_mixtures) {
  const SpectralLibrary lib = builtin_library(kWl);
  const int B = lib.bands();
  // Mixes of leaf, red paint and the IR dye, and pure white paper.
  const int leaf = int(Material::kLeaf), red = int(Material::kRedPaint), dye = int(Material::kIrBlackDye);
  const float mixes[4][3] = {{0.6f, 0.3f, 0.1f}, {0.2f, 0.2f, 0.6f}, {0.0f, 1.0f, 0.0f}, {0.5f, 0.0f, 0.5f}};
  std::vector<float> spectra;
  for (const auto& w : mixes)
    for (int b = 0; b < B; ++b)
      spectra.push_back(w[0] * lib.spectrum(leaf)[b] + w[1] * lib.spectrum(red)[b] + w[2] * lib.spectrum(dye)[b]);
  for (int b = 0; b < B; ++b) spectra.push_back(lib.spectrum(int(Material::kWhitePaper))[b]);
  // Against the whole library; its flat spectra are nearly dependent, but these mixes are unique.
  const Unmixing u = unmix_fcls(spectra.data(), 5, lib.spectra.data(), lib.size(), B);
  const int M = lib.size();
  for (int i = 0; i < 5; ++i) {
    double sum = 0.0;
    for (int m = 0; m < M; ++m) {
      CHECK(u.abundances[size_t(i) * M + m] >= 0.0f);
      sum += u.abundances[size_t(i) * M + m];
    }
    CHECK_NEAR(sum, 1.0, 1e-5);
    CHECK(u.residual[size_t(i)] < 1e-3f);
  }
  for (int i = 0; i < 4; ++i) {
    CHECK_NEAR(u.abundances[size_t(i) * M + leaf], mixes[i][0], 2e-3);
    CHECK_NEAR(u.abundances[size_t(i) * M + red], mixes[i][1], 2e-3);
    CHECK_NEAR(u.abundances[size_t(i) * M + dye], mixes[i][2], 2e-3);
  }
  CHECK_NEAR(u.abundances[size_t(4) * M + int(Material::kWhitePaper)], 1.0, 2e-3);

  // A spectrum outside the endmembers' hull still gets abundances that sum to
  // one, and a residual that says how far off it is.
  std::vector<float> off(static_cast<size_t>(B), 2.0f);
  const Unmixing o = unmix_fcls(off.data(), 1, lib.spectra.data(), M, B);
  double sum = 0.0;
  for (int m = 0; m < M; ++m) sum += o.abundances[size_t(m)];
  CHECK_NEAR(sum, 1.0, 1e-5);
  CHECK(o.residual[0] > 1.0f);
}

TEST(synthetic_scene_records_materials) {
  SyntheticOptions o;
  o.bands = 12;
  o.spacing = 2e-3;
  std::vector<int32_t> m;
  const GaussianScene s = make_synthetic_scene(o, wavelength_grid(500.0, 950.0, 12), &m);
  CHECK(int(m.size()) == s.size());
  // The features are each material's spectrum with a few percent of tint, so
  // the spectral angle names every Gaussian's material.
  const SpectralLibrary lib = builtin_library(wavelength_grid(500.0, 950.0, 12));
  const SamResult r = classify_sam(s.features.data(), s.size(), lib, SamOptions());
  std::vector<int> seen(size_t(Material::kCount), 0);
  for (int i = 0; i < s.size(); ++i) {
    CHECK(r.label[size_t(i)] == m[size_t(i)]);
    ++seen[size_t(m[size_t(i)])];
  }
  for (int c : seen) CHECK(c > 0);  // every material is in the scene
}

TEST(lsplat_round_trip) {
  GaussianScene s = lsfix::random_scene(40, 5, 21);
  for (float& v : s.features) v = 0.2f + 0.6f * v;
  // A basis that isn't the identity: 7 bands from 5 features.
  s.basis.assign(7 * 5, 0.0f);
  for (size_t i = 0; i < s.basis.size(); ++i) s.basis[i] = float((i * 37) % 11) / 10.0f;
  LsplatFile f;
  f.header = {{"format", "linesplat-view"}, {"version", 1}, {"title", "test"}, {"wavelengths_nm", wavelength_grid(500, 800, 7)}};
  put_scene(f, s);
  CHECK(f.header["basis"].is_array() && f.header["basis"].size() == 7);
  const std::string path = tmp_path("round_trip.lsplat");
  write_lsplat(path, f);

  LsplatFile g = read_lsplat(path);
  CHECK(g.header["title"] == "test");
  CHECK(g.blocks.size() == 6);
  CHECK(g.blocks[0].name == "means" && g.blocks[5].name == "features");  // in file order
  const GaussianScene t = unpack_scene(g);
  CHECK(t.size() == 40 && t.num_features == 5 && t.num_bands() == 7);
  CHECK(t.means == s.means);
  CHECK(t.basis == s.basis);
  for (size_t i = 0; i < s.features.size(); ++i) CHECK_NEAR(t.features[i], s.features[i], 0.6 / 255.0 * 0.51 + 1e-6);
  for (int i = 0; i < 40; ++i) {
    const double o0 = 1.0 / (1.0 + std::exp(-double(s.opacity_logits[size_t(i)])));
    const double o1 = 1.0 / (1.0 + std::exp(-double(t.opacity_logits[size_t(i)])));
    CHECK_NEAR(o0, o1, 0.5 / 255.0 + 1e-6);
  }

  // Writing what was read gives the same bytes back.
  write_lsplat(tmp_path("round_trip_2.lsplat"), g);
  CHECK(read_bytes(path) == read_bytes(tmp_path("round_trip_2.lsplat")));

  // Extra arrays go at the end and leave the scene's where they were; set
  // replaces in place.
  g.set(make_block("material_label", std::vector<uint8_t>(40, 3), {40}));
  g.set(make_block("abundances", std::vector<uint8_t>(80, 9), {40, 2}));
  g.set(make_block("material_label", std::vector<uint8_t>(40, 4), {40}));
  g.header["materials"] = {{"version", 1}};
  write_lsplat(tmp_path("with_materials.lsplat"), g);
  const LsplatFile h = read_lsplat(tmp_path("with_materials.lsplat"));
  CHECK(h.blocks.size() == 8);
  CHECK(h.at("material_label").values<uint8_t>() == std::vector<uint8_t>(40, 4));
  CHECK((h.at("abundances").shape == std::vector<size_t>{40, 2}));
  CHECK(h.header["materials"]["version"] == 1);
  const std::vector<char> a = read_bytes(path), b = read_bytes(tmp_path("with_materials.lsplat"));
  CHECK(unpack_scene(h).features == t.features);
  CHECK(b.size() > a.size());

  // Not a splat file.
  write_text_file(tmp_path("not.lsplat"), "hello, world");
  bool threw = false;
  try {
    read_lsplat(tmp_path("not.lsplat"));
  } catch (const std::exception&) {
    threw = true;
  }
  CHECK(threw);
}
