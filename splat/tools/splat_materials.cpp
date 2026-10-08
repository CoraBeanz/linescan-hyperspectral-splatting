// splat_materials: material maps for a splat, added to the web viewer's file.
//
//   splat_materials IN.lsplat OUT.lsplat [--library builtin|FILE.csv] [--max-angle DEG]
//                   [--brightness F] [--clusters K] [--seed N] [--endmembers library|clusters]
//                   [--truth DATASET] [--truth-labels LABELS.npy] [--truth-map MAP.npy]
//                   [--dataset DIR [--poses POSES.npy] [--truth-lines LINES.npy]] [--png DIR]
//
// IN is a file splat_export wrote (unzipped); the maps are made from the
// Gaussians as the viewer decodes them. Each Gaussian's spectrum gets:
//
//   a label     the library material with the smallest spectral angle, among
//               those within a factor of --brightness (2) of its brightness,
//               or unknown if every candidate is more than --max-angle (10)
//               degrees away. The library is the synthetic scene's 11
//               materials, or a CSV file (materials.hpp says the layout).
//   a cluster   from weighted k-means with --clusters (12) clusters, weighted
//               by opacity so that faint Gaussians don't pull the means.
//   abundances  the mix of endmembers (the library's spectra, or with
//               --endmembers clusters the cluster means) that fits it best,
//               non-negative and summing to one.
//
// OUT is IN with these as extra arrays and a "materials" entry in the header
// (viewer/README.md has the layout); OUT may be IN. Files without them still
// load in the viewer, which then offers no material views.
//
// --truth scores the maps against a synthetic dataset's ground truth (its
// gt/scene and gt/materials.npy): each Gaussian is scored as the material of
// the nearest true Gaussian within 1.5 mm, and the material map rendered from
// the file's default view is scored against the truth rendered the same way,
// on the pixels that show one material. --truth-labels gives each Gaussian's
// true material as a library index (-1 for none) instead, from any source,
// and --truth-map the default view's (int32 [360, 480], -1 where a pixel
// shows no single material). With --dataset, the maps are also rendered
// through the dataset's line cameras, at --poses (the trained ones; else the
// recorded ones), and scored against what each line really saw: the truth's
// scene at its true poses for a synthetic dataset, or --truth-lines (int32
// [lines, width]). splat/tools/sim_material_truth.py writes the labels, the
// map and the lines for a simulator scan.
// --png writes the default view as true color, labels, clusters and truth.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <functional>
#include <map>
#include <nlohmann/json.hpp>
#include <stdexcept>
#include <string>
#include <vector>

#include "linesplat/dataset.hpp"
#include "linesplat/lsplat.hpp"
#include "linesplat/materials.hpp"
#include "linesplat/npy.hpp"
#include "linesplat/png.hpp"
#include "linesplat/preview.hpp"
#include "linesplat/render_cpu.hpp"
#include "linesplat/spectra.hpp"
#include "linesplat/util.hpp"

using namespace linesplat;
using nlohmann::json;

namespace {

constexpr uint8_t kNone = 255;           // no label, in the uint8 arrays
constexpr double kAngleStepDeg = 0.1;    // material_angle's unit
constexpr double kTruthRadius = 1.5e-3;  // a Gaussian farther than this from every true one has no truth
constexpr int kMapWidth = 480, kMapHeight = 360;
constexpr double kPureFraction = 0.9;    // a pixel shows one material if that material covers this much of it

void usage() {
  std::fprintf(stderr,
               "usage: splat_materials IN.lsplat OUT.lsplat [--library builtin|FILE.csv] [--max-angle DEG]\n"
               "                       [--brightness F] [--clusters K] [--seed N] [--endmembers library|clusters]\n"
               "                       [--truth DATASET] [--truth-labels LABELS.npy] [--truth-map MAP.npy]\n"
               "                       [--dataset DIR [--poses POSES.npy] [--truth-lines LINES.npy]] [--png DIR]\n");
  std::exit(2);
}

double rounded(double x, int decimals) {
  const double s = std::pow(10.0, decimals);
  return std::round(x * s) / s;
}

json spectrum_json(const float* s, int n) {
  json a = json::array();
  for (int i = 0; i < n; ++i) a.push_back(rounded(s[i], 4));
  return a;
}

// "#rrggbb" -> linear RGB.
std::array<float, 3> linear_rgb(const std::string& hex) {
  std::array<float, 3> c{0.5f, 0.5f, 0.5f};
  if (hex.size() != 7 || hex[0] != '#') return c;
  for (int k = 0; k < 3; ++k) {
    const float v = float(std::stoi(hex.substr(size_t(1 + 2 * k), 2), nullptr, 16)) / 255.0f;
    c[size_t(k)] = v <= 0.04045f ? v / 12.92f : std::pow((v + 0.055f) / 1.055f, 2.4f);
  }
  return c;
}

// The scene with each Gaussian's features replaced by `values` ([N, C]) and
// an identity basis, so rendering it composites those values.
GaussianScene with_values(const GaussianScene& s, const std::vector<float>& values, int channels) {
  GaussianScene v = s;
  v.num_features = channels;
  v.features = values;
  v.set_identity_basis(channels);
  v.background.assign(size_t(channels), 0.0f);
  return v;
}

std::vector<float> one_hot(const std::vector<int>& labels, int classes) {
  std::vector<float> v(labels.size() * size_t(classes), 0.0f);
  for (size_t i = 0; i < labels.size(); ++i)
    if (labels[i] >= 0 && labels[i] < classes) v[i * classes + size_t(labels[i])] = 1.0f;
  return v;
}

LineImage render_default_view(const GaussianScene& s, const json& view) {
  double f = 0.0;
  const Pose cam = lsplat_view_pose(view, kMapWidth, kMapHeight, &f);
  return render_lines_cpu<float>(s, pinhole_rows(cam, kMapWidth, kMapHeight, f));
}

// Per pixel, the class with the largest share and that share (0 if none).
void dominant(const LineImage& img, std::vector<int>* cls, std::vector<float>* share) {
  const size_t n = size_t(img.lines) * img.width;
  cls->assign(n, -1);
  share->assign(n, 0.0f);
  for (size_t p = 0; p < n; ++p)
    for (int c = 0; c < img.channels; ++c)
      if (img.values[p * img.channels + c] > (*share)[p]) {
        (*share)[p] = img.values[p * img.channels + c];
        (*cls)[p] = c;
      }
}

void write_map_png(const std::string& path, const GaussianScene& s, const json& view, const std::vector<int>& labels,
                   const std::vector<std::string>& colors) {
  std::vector<float> rgb(labels.size() * 3, 0.0f);
  for (size_t i = 0; i < labels.size(); ++i) {
    // Unknown Gaussians stay dark, so they read as gaps.
    const std::array<float, 3> c =
        labels[i] >= 0 ? linear_rgb(colors[size_t(labels[i])]) : std::array<float, 3>{0.02f, 0.02f, 0.02f};
    for (int k = 0; k < 3; ++k) rgb[3 * i + size_t(k)] = c[size_t(k)];
  }
  const LineImage img = render_default_view(with_values(s, rgb, 3), view);
  std::vector<uint8_t> px;
  for (float v : img.values) px.push_back(to_srgb8(v));
  write_png_rgb(path, kMapWidth, kMapHeight, px);
}

struct Truth {
  std::vector<std::string> names;  // the truth's material names
  GaussianScene scene;             // the true Gaussians, for the rendered comparison
  std::vector<int> materials;      // [N_true] index into names
  bool has_scene = false;
};

Truth load_truth(const std::string& dir) {
  Truth t;
  const json meta = json::parse(read_text_file(join_path(dir, "dataset.json"))).value("metadata", json::object());
  const json gt = meta.value("ground_truth", json::object());
  if (!gt.contains("materials"))
    throw std::runtime_error(dir + " has no gt/materials.npy; remake it with this version of splat_synth");
  if (gt.contains("material_names")) {
    t.names = gt["material_names"].get<std::vector<std::string>>();
  } else {
    for (int m = 0; m < int(Material::kCount); ++m) t.names.push_back(material_name(Material(m)));
  }
  t.scene = GaussianScene::load(join_path(dir, gt.value("scene", std::string("gt/scene"))));
  const std::vector<int32_t> m = npy_load_i32(join_path(dir, gt["materials"].get<std::string>()));
  t.materials.assign(m.begin(), m.end());
  if (int(t.materials.size()) != t.scene.size()) throw std::runtime_error("gt/materials.npy and gt/scene disagree");
  t.has_scene = true;
  return t;
}

// The library index of each truth name, or -1 where the library lacks it.
std::vector<int> map_names(const std::vector<std::string>& names, const SpectralLibrary& lib) {
  std::vector<int> out;
  for (const std::string& n : names) {
    const auto it = std::find(lib.names.begin(), lib.names.end(), n);
    out.push_back(it == lib.names.end() ? -1 : int(it - lib.names.begin()));
  }
  return out;
}

// Each Gaussian's true material: that of the nearest true Gaussian within
// kTruthRadius, as a library index (-1: none near, or not in the library).
std::vector<int> nearest_truth(const GaussianScene& s, const Truth& t, const std::vector<int>& to_lib) {
  const int n = s.size(), m = t.scene.size();
  std::vector<int> out(size_t(n), -1);
#pragma omp parallel for schedule(static)
  for (int i = 0; i < n; ++i) {
    const float* p = &s.means[3 * size_t(i)];
    double best = kTruthRadius * kTruthRadius;
    int arg = -1;
    for (int j = 0; j < m; ++j) {
      const float* q = &t.scene.means[3 * size_t(j)];
      const double dx = p[0] - q[0], dy = p[1] - q[1], dz = p[2] - q[2];
      const double d = dx * dx + dy * dy + dz * dz;
      if (d < best) {
        best = d;
        arg = j;
      }
    }
    if (arg >= 0) out[size_t(i)] = to_lib[size_t(t.materials[size_t(arg)])];
  }
  return out;
}

std::string pct(double x) {
  char b[32];
  std::snprintf(b, sizeof b, "%.1f%%", 100.0 * x);
  return b;
}

// An image of the true materials to score a rendered map against: per pixel,
// the dominant true material (a library index, -1 for none) and its share.
struct PixelTruth {
  std::string key, heading;  // the score's key in the header, and its column in the printout
  json about;                // more for the header (the image's size)
  std::function<LineImage(const GaussianScene&)> render;  // renders a scene the same way
  std::vector<int> labels;
  std::vector<float> shares;
};

struct PixelScore {
  std::vector<int> count, right;  // per true material
  int scored = 0, correct = 0, unmix_right = 0;
  double abundance = 0.0;         // the true material's abundance, summed
};

// Scores and prints the maps against the truth; returns the scores for the header.
json score(const GaussianScene& s, const SpectralLibrary& lib, const SamResult& sam, const KMeansResult& km,
           const Unmixing& um, const std::vector<int>& endmember_class, const std::vector<int>& truth,
           const std::vector<PixelTruth>& images) {
  const int n = s.size(), M = lib.size(), E = int(endmember_class.size());
  std::vector<double> opacity(static_cast<size_t>(n));
  for (int i = 0; i < n; ++i) opacity[size_t(i)] = 1.0 / (1.0 + std::exp(-double(s.opacity_logits[size_t(i)])));

  // Per Gaussian; confused[c * (M + 1) + l]: Gaussians of material c labelled l (M: unknown).
  std::vector<int> count(size_t(M), 0), right(size_t(M), 0), confused(size_t(M) * (M + 1), 0);
  int scored = 0, correct = 0, unmixed_right = 0;
  double w_all = 0.0, w_right = 0.0, abundance_sum = 0.0;
  for (int i = 0; i < n; ++i) {
    const int c = truth[size_t(i)];
    if (c < 0) continue;
    ++scored;
    ++count[size_t(c)];
    w_all += opacity[size_t(i)];
    const bool ok = sam.label[size_t(i)] == c;
    ++confused[size_t(c) * (M + 1) + size_t(sam.label[size_t(i)] < 0 ? M : sam.label[size_t(i)])];
    if (ok) {
      ++correct;
      ++right[size_t(c)];
      w_right += opacity[size_t(i)];
    }
    // Unmixing: the largest abundance, and how much of the true material it found.
    int best = 0;
    double truth_share = 0.0;
    for (int e = 0; e < E; ++e) {
      const float a = um.abundances[size_t(i) * E + e];
      if (a > um.abundances[size_t(i) * E + best]) best = e;
      if (endmember_class[size_t(e)] == c) truth_share += a;
    }
    if (endmember_class[size_t(best)] == c) ++unmixed_right;
    abundance_sum += truth_share;
  }
  std::vector<int> sam_truth = truth, clusters = km.label;
  const double ari = adjusted_rand_index(truth, clusters);
  // Purity: the share of Gaussians whose cluster's commonest true material is theirs.
  std::map<int, std::map<int, int>> per_cluster;
  for (int i = 0; i < n; ++i)
    if (truth[size_t(i)] >= 0) ++per_cluster[clusters[size_t(i)]][truth[size_t(i)]];
  int pure = 0;
  for (const auto& c : per_cluster) {
    int most = 0;
    for (const auto& kv : c.second) most = std::max(most, kv.second);
    pure += most;
  }

  json out = {{"gaussians", scored},
              {"label_accuracy", rounded(double(correct) / std::max(1, scored), 4)},
              {"label_accuracy_opacity_weighted", rounded(w_right / std::max(1e-12, w_all), 4)},
              {"cluster_purity", rounded(double(pure) / std::max(1, scored), 4)},
              {"cluster_adjusted_rand_index", rounded(ari, 4)},
              {"unmixing_top_endmember_accuracy", rounded(double(unmixed_right) / std::max(1, scored), 4)},
              {"unmixing_mean_true_abundance", rounded(abundance_sum / std::max(1, scored), 4)}};

  // The material maps rendered as each image of the truth was, on the pixels
  // that show one material where the splat has something to show (a trained
  // splat stops where the scans did).
  std::vector<PixelScore> px;
  for (const PixelTruth& img : images) {
    std::vector<int> pcls, ucls;
    std::vector<float> pshare, ushare;
    const LineImage predicted = img.render(with_values(s, one_hot(sam.label, M), M));
    dominant(predicted, &pcls, &pshare);
    const LineImage abundance = img.render(with_values(s, um.abundances, E));
    dominant(abundance, &ucls, &ushare);
    PixelScore r;
    r.count.assign(size_t(M), 0);
    r.right.assign(size_t(M), 0);
    for (size_t p = 0; p < img.labels.size(); ++p) {
      const int c = img.labels[p];
      if (c < 0 || img.shares[p] < kPureFraction || predicted.transmittance[p] > 1.0f - kPureFraction) continue;
      ++r.scored;
      ++r.count[size_t(c)];
      if (pcls[p] == c) {
        ++r.correct;
        ++r.right[size_t(c)];
      }
      if (ucls[p] >= 0 && endmember_class[size_t(ucls[p])] == c) ++r.unmix_right;
      for (int e = 0; e < E; ++e)
        if (endmember_class[size_t(e)] == c) r.abundance += abundance.values[p * E + size_t(e)];
    }
    json j = img.about;
    j["pixels"] = r.scored;
    j["label_accuracy"] = rounded(double(r.correct) / std::max(1, r.scored), 4);
    j["unmixing_top_endmember_accuracy"] = rounded(double(r.unmix_right) / std::max(1, r.scored), 4);
    j["unmixing_mean_true_abundance"] = rounded(r.abundance / std::max(1, r.scored), 4);
    out[img.key] = j;
    px.push_back(r);
  }

  std::printf("\nagainst the truth          Gaussians  labelled right");
  for (const PixelTruth& img : images) std::printf("  %11s  labelled right", img.heading.c_str());
  std::printf("   most often taken for\n");
  for (int c = 0; c < M; ++c) {
    bool any = count[size_t(c)] > 0;
    for (const PixelScore& r : px) any = any || r.count[size_t(c)] > 0;
    if (!any) continue;
    std::printf("  %-24s %9d  %14s", lib.names[size_t(c)].c_str(), count[size_t(c)],
                count[size_t(c)] ? pct(double(right[size_t(c)]) / count[size_t(c)]).c_str() : "-");
    for (const PixelScore& r : px)
      std::printf("  %11d  %14s", r.count[size_t(c)],
                  r.count[size_t(c)] ? pct(double(r.right[size_t(c)]) / r.count[size_t(c)]).c_str() : "-");
    // The commonest wrong label among this material's Gaussians.
    int worst = -1;
    for (int l = 0; l <= M; ++l)
      if (l != c && confused[size_t(c) * (M + 1) + l] > 0 &&
          (worst < 0 || confused[size_t(c) * (M + 1) + l] > confused[size_t(c) * (M + 1) + worst]))
        worst = l;
    if (worst >= 0)
      std::printf("   %s (%s)", worst == M ? "unknown" : lib.names[size_t(worst)].c_str(),
                  pct(double(confused[size_t(c) * (M + 1) + worst]) / count[size_t(c)]).c_str());
    std::printf("\n");
  }
  std::printf("  %-24s %9d  %14s", "all", scored, pct(double(correct) / std::max(1, scored)).c_str());
  for (const PixelScore& r : px)
    std::printf("  %11d  %14s", r.scored, pct(double(r.correct) / std::max(1, r.scored)).c_str());
  std::printf("\n  weighted by opacity                %14s\n", pct(w_right / std::max(1e-12, w_all)).c_str());
  int no_match = 0;
  for (int c = 0; c < M; ++c) no_match += confused[size_t(c) * (M + 1) + M];
  std::printf("  the rest: no match for %s, another material for %s\n", pct(double(no_match) / std::max(1, scored)).c_str(),
              pct(double(scored - correct - no_match) / std::max(1, scored)).c_str());
  std::printf("clusters   purity %s, adjusted Rand index %.3f\n", pct(double(pure) / std::max(1, scored)).c_str(), ari);
  std::printf("unmixing   largest abundance is the true material for %s of Gaussians",
              pct(double(unmixed_right) / std::max(1, scored)).c_str());
  for (size_t k = 0; k < px.size(); ++k)
    std::printf(", %s of %s", pct(double(px[k].unmix_right) / std::max(1, px[k].scored)).c_str(),
                images[k].heading.c_str());
  std::printf(";\n           the true material's abundance averages %.2f", abundance_sum / std::max(1, scored));
  for (size_t k = 0; k < px.size(); ++k)
    std::printf(", %.2f on %s", px[k].abundance / std::max(1, px[k].scored), images[k].heading.c_str());
  std::printf("\n");
  return out;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 3 || argv[1][0] == '-' || argv[2][0] == '-') usage();
  const std::string in = argv[1], out = argv[2];
  std::string library = "builtin", endmembers = "library", truth_dir, truth_labels, truth_map_path, png_dir;
  std::string dataset_dir, poses_path, truth_lines_path;
  SamOptions sam_opt;
  int clusters = 12;
  uint64_t seed = 1;
  for (int i = 3; i < argc; ++i) {
    const std::string a = argv[i];
    auto next = [&]() -> const char* {
      if (i + 1 >= argc) usage();
      return argv[++i];
    };
    if (a == "--library") library = next();
    else if (a == "--max-angle") sam_opt.max_angle_deg = std::atof(next());
    else if (a == "--brightness") sam_opt.brightness_window = std::atof(next());
    else if (a == "--clusters") clusters = std::atoi(next());
    else if (a == "--seed") seed = std::strtoull(next(), nullptr, 10);
    else if (a == "--endmembers") endmembers = next();
    else if (a == "--truth") truth_dir = next();
    else if (a == "--truth-labels") truth_labels = next();
    else if (a == "--truth-map") truth_map_path = next();
    else if (a == "--dataset") dataset_dir = next();
    else if (a == "--poses") poses_path = next();
    else if (a == "--truth-lines") truth_lines_path = next();
    else if (a == "--png") png_dir = next();
    else usage();
  }
  if (endmembers != "library" && endmembers != "clusters") usage();

  try {
    LsplatFile file = read_lsplat(in);
    const GaussianScene scene = unpack_scene(file);
    const int n = scene.size(), K = scene.num_features, B = scene.num_bands();
    const std::vector<double> wl = file.header.at("wavelengths_nm").get<std::vector<double>>();
    if (int(wl.size()) != B) throw std::runtime_error("the file's wavelengths and basis disagree");
    if (n < clusters) throw std::runtime_error("fewer Gaussians than clusters");

    // Each Gaussian's spectrum, basis times features, and its opacity.
    std::vector<float> spectra(size_t(n) * B, 0.0f), opacity(static_cast<size_t>(n));
    for (int i = 0; i < n; ++i) {
      for (int b = 0; b < B; ++b) {
        double v = 0.0;
        for (int k = 0; k < K; ++k) v += double(scene.basis[size_t(b) * K + k]) * scene.features[size_t(i) * K + k];
        spectra[size_t(i) * B + b] = float(v);
      }
      opacity[size_t(i)] = float(1.0 / (1.0 + std::exp(-double(scene.opacity_logits[size_t(i)]))));
    }

    const SpectralLibrary lib = library == "builtin" ? builtin_library(wl) : load_library_csv(library, wl);
    const int M = lib.size();
    if (M > 254) throw std::runtime_error("a library of up to 254 materials fits the file");
    if (clusters < 1 || clusters > 254) throw std::runtime_error("--clusters must be 1 to 254");

    Timer timer;
    const SamResult sam = classify_sam(spectra.data(), n, lib, sam_opt);
    const double t_sam = timer.ms();
    const KMeansResult km = kmeans(spectra.data(), n, B, opacity, clusters, seed);
    const double t_km = timer.ms() - t_sam;
    // The cluster means against the library, without the angle limit, to name them.
    SamOptions loose = sam_opt;
    loose.max_angle_deg = 180.0;
    SamResult named = classify_sam(km.centroids.data(), clusters, lib, loose);
    // For a mean too bright or dark for any, the nearest shape within the angle
    // limit; a mean like none of them (floaters, often) stays unnamed (-1).
    loose.brightness_window = 0.0;
    loose.max_angle_deg = sam_opt.max_angle_deg;
    const SamResult any = classify_sam(km.centroids.data(), clusters, lib, loose);
    std::vector<bool> named_by_shape(static_cast<size_t>(clusters), false);
    for (int c = 0; c < clusters; ++c)
      if (named.label[size_t(c)] < 0 || named.angle[size_t(c)] * 180.0 / kPi > sam_opt.max_angle_deg) {
        named.label[size_t(c)] = any.label[size_t(c)];
        named.angle[size_t(c)] = any.angle[size_t(c)];
        named_by_shape[size_t(c)] = any.label[size_t(c)] >= 0;
      }

    const bool from_library = endmembers == "library";
    const int E = from_library ? M : clusters;
    const float* E_spectra = from_library ? lib.spectra.data() : km.centroids.data();
    const Unmixing um = unmix_fcls(spectra.data(), n, E_spectra, E, B);
    const double t_um = timer.ms() - t_sam - t_km;
    std::vector<int> endmember_class;  // each endmember's library material, for scoring
    for (int e = 0; e < E; ++e) endmember_class.push_back(from_library ? e : named.label[size_t(e)]);

    // The arrays.
    std::vector<uint8_t> label(static_cast<size_t>(n)), angle(static_cast<size_t>(n)), cluster(static_cast<size_t>(n)),
        abundance(size_t(n) * E);
    std::vector<int> per_class(size_t(M), 0), per_cluster(size_t(clusters), 0);
    int unknown = 0;
    double residual = 0.0;
    for (int i = 0; i < n; ++i) {
      const int l = sam.label[size_t(i)];
      label[size_t(i)] = l < 0 ? kNone : uint8_t(l);
      if (l < 0) ++unknown;
      else ++per_class[size_t(l)];
      const float a = sam.angle[size_t(i)];
      angle[size_t(i)] = a < 0.0f ? kNone : uint8_t(std::min(254.0, std::round(a * 180.0 / kPi / kAngleStepDeg)));
      cluster[size_t(i)] = uint8_t(km.label[size_t(i)]);
      ++per_cluster[size_t(km.label[size_t(i)])];
      for (int e = 0; e < E; ++e)
        abundance[size_t(i) * E + e] = uint8_t(std::lround(255.0f * um.abundances[size_t(i) * E + e]));
      residual += double(um.residual[size_t(i)]) * um.residual[size_t(i)];
    }
    residual = std::sqrt(residual / std::max(1, n));
    const size_t N = size_t(n);
    file.set(make_block("material_label", label, {N}));
    file.set(make_block("material_angle", angle, {N}));
    file.set(make_block("cluster", cluster, {N}));
    file.set(make_block("abundances", abundance, {N, size_t(E)}));

    json classes = json::array();
    for (int m = 0; m < M; ++m)
      classes.push_back({{"name", lib.names[size_t(m)]},
                         {"color", lib.colors[size_t(m)]},
                         {"spectrum", spectrum_json(lib.spectrum(m), B)},
                         {"count", per_class[size_t(m)]}});
    json cluster_list = json::array();
    for (int c = 0; c < clusters; ++c) {
      const int near = named.label[size_t(c)];
      cluster_list.push_back({{"color", category_colors()[size_t(c) % category_colors().size()]},
                              {"spectrum", spectrum_json(&km.centroids[size_t(c) * B], B)},
                              {"count", per_cluster[size_t(c)]},
                              {"nearest", near},
                              {"angle_deg", near < 0 ? json(nullptr) : json(rounded(named.angle[size_t(c)] * 180.0 / kPi, 2))},
                              {"brightness_matches", !named_by_shape[size_t(c)]}});
    }
    json endmember_list = json::array();
    for (int e = 0; e < E; ++e) {
      if (from_library) {
        endmember_list.push_back({{"name", lib.names[size_t(e)]}, {"color", lib.colors[size_t(e)]}, {"class", e}});
      } else {
        endmember_list.push_back({{"name", "Cluster " + std::to_string(e + 1)},
                                  {"color", cluster_list[size_t(e)]["color"]},
                                  {"cluster", e},
                                  {"spectrum", cluster_list[size_t(e)]["spectrum"]}});
      }
    }
    json materials = {
        {"version", 1},
        {"library", {{"source", library}, {"classes", classes}}},
        {"labels",
         {{"method", "spectral angle"},
          {"max_angle_deg", sam_opt.max_angle_deg},
          {"brightness_window", sam_opt.brightness_window},
          {"angle_step_deg", kAngleStepDeg},
          {"unknown", unknown}}},
        {"clusters", {{"method", "k-means"}, {"k", clusters}, {"seed", seed}, {"weights", "opacity"}, {"list", cluster_list}}},
        {"unmixing",
         {{"method", "fully constrained least squares"},
          {"endmembers_from", endmembers},
          {"endmembers", endmember_list},
          {"residual_rms", rounded(residual, 5)}}}};

    // The printout.
    std::printf("read      %s: %d Gaussians, %d bands (%.0f to %.0f nm)\n", in.c_str(), n, B, wl.front(), wl.back());
    std::printf("labels    spectral angle against %d %s materials, within %gx in brightness and %g deg: %.0f ms\n", M,
                library == "builtin" ? "built-in" : "library", sam_opt.brightness_window, sam_opt.max_angle_deg, t_sam);
    std::printf("          %-24s %9s %8s\n", "material", "Gaussians", "share");
    for (int m = 0; m < M; ++m)
      std::printf("          %-24s %9d %8s\n", lib.names[size_t(m)].c_str(), per_class[size_t(m)],
                  pct(double(per_class[size_t(m)]) / n).c_str());
    std::printf("          %-24s %9d %8s\n", "unknown", unknown, pct(double(unknown) / n).c_str());
    std::printf("clusters  k-means, k = %d, %d iterations: %.0f ms\n", clusters, km.iterations, t_km);
    for (int c = 0; c < clusters; ++c) {
      if (named.label[size_t(c)] < 0)
        std::printf("          %2d  %6d Gaussians, like no library spectrum\n", c + 1, per_cluster[size_t(c)]);
      else
        std::printf("          %2d  %6d Gaussians, nearest %s (%.1f deg%s)\n", c + 1, per_cluster[size_t(c)],
                    lib.names[size_t(named.label[size_t(c)])].c_str(), named.angle[size_t(c)] * 180.0 / kPi,
                    named_by_shape[size_t(c)] ? ", in shape only" : "");
    }
    std::printf("unmixing  %d endmembers from the %s, fully constrained: residual %.4f rms: %.0f ms\n", E,
                from_library ? "library" : "clusters", residual, t_um);

    // Scoring.
    if (!truth_map_path.empty() && truth_labels.empty()) throw std::runtime_error("--truth-map goes with --truth-labels");
    if (!truth_lines_path.empty() && dataset_dir.empty()) throw std::runtime_error("--truth-lines needs --dataset");
    if (!truth_dir.empty() || !truth_labels.empty()) {
      Truth t;
      std::vector<int> truth, to_lib, true_labels;
      if (!truth_dir.empty()) {
        t = load_truth(truth_dir);
        to_lib = map_names(t.names, lib);
        truth = nearest_truth(scene, t, to_lib);
        for (int m : t.materials) true_labels.push_back(to_lib[size_t(m)]);
      } else {
        std::vector<size_t> shape;
        const std::vector<int32_t> v = npy_load_i32(truth_labels, &shape);
        if (int(v.size()) != n) throw std::runtime_error(truth_labels + " needs one label per Gaussian");
        for (int32_t x : v) truth.push_back(x >= 0 && x < M ? int(x) : -1);
      }
      auto labels_from = [&](const std::string& path, const std::vector<size_t>& want, std::vector<int>* out) {
        std::vector<size_t> shape;
        const std::vector<int32_t> v = npy_load_i32(path, &shape);
        if (shape != want) throw std::runtime_error(path + " has the wrong shape");
        out->clear();
        for (int32_t x : v) out->push_back(x >= 0 && x < M ? int(x) : -1);
      };

      // The default view, against the true scene rendered the same way or a map of it.
      std::vector<PixelTruth> images;
      const json view = file.header.at("view");
      PixelTruth map{"view", "map pixels", {{"width", kMapWidth}, {"height", kMapHeight}},
                     [&](const GaussianScene& g) { return render_default_view(g, view); }, {}, {}};
      if (t.has_scene) {
        dominant(map.render(with_values(t.scene, one_hot(true_labels, M), M)), &map.labels, &map.shares);
      } else if (!truth_map_path.empty()) {
        labels_from(truth_map_path, {size_t(kMapHeight), size_t(kMapWidth)}, &map.labels);
        map.shares.assign(map.labels.size(), 1.0f);
      }
      if (!map.labels.empty()) images.push_back(map);

      // The scan lines, from the poses given, against what each line saw.
      if (!dataset_dir.empty()) {
        Dataset d = Dataset::load(dataset_dir);
        if (!poses_path.empty()) {
          const std::vector<double> p = npy_load_f64(poses_path);
          if (p.size() != 7 * size_t(d.num_sweeps())) throw std::runtime_error(poses_path + " needs 7 values a sweep");
          for (int k = 0; k < d.num_sweeps(); ++k) d.sweep_head_pose[size_t(k)] = Pose::from_array(&p[7 * size_t(k)]);
        }
        const std::vector<LineCamera> cams = d.cameras();
        PixelTruth lines{"lines", "line pixels", {{"lines", d.num_lines()}, {"width", d.width()}},
                         [cams](const GaussianScene& g) { return render_lines_cpu<float>(g, cams); }, {}, {}};
        const std::string gt_pose = join_path(dataset_dir, "gt/sweep_head_pose.npy");
        if (!truth_lines_path.empty()) {
          labels_from(truth_lines_path, {size_t(d.num_lines()), size_t(d.width())}, &lines.labels);
          lines.shares.assign(lines.labels.size(), 1.0f);
        } else if (t.has_scene && file_exists(gt_pose)) {
          // The true scene at the true poses: what the scan really saw.
          Dataset g = d;
          const std::vector<double> p = npy_load_f64(gt_pose);
          for (int k = 0; k < g.num_sweeps(); ++k) g.sweep_head_pose[size_t(k)] = Pose::from_array(&p[7 * size_t(k)]);
          g.line_mirror_angle = npy_load_f64(join_path(dataset_dir, "gt/line_mirror_angle.npy"));
          dominant(render_lines_cpu<float>(with_values(t.scene, one_hot(true_labels, M), M), g.cameras()),
                   &lines.labels, &lines.shares);
        }
        if (!lines.labels.empty()) images.push_back(lines);
      }

      materials["score"] = score(scene, lib, sam, km, um, endmember_class, truth, images);
      if (!png_dir.empty() && !map.labels.empty()) {
        make_dirs(png_dir);
        if (t.has_scene) {
          write_map_png(join_path(png_dir, "truth.png"), t.scene, view, true_labels, lib.colors);
        } else {
          std::vector<uint8_t> px;
          for (int c : map.labels)
            for (float v : c >= 0 ? linear_rgb(lib.colors[size_t(c)]) : std::array<float, 3>{0.0f, 0.0f, 0.0f})
              px.push_back(to_srgb8(v));
          write_png_rgb(join_path(png_dir, "truth.png"), kMapWidth, kMapHeight, px);
        }
      }
    }
    file.header["materials"] = materials;
    write_lsplat(out, file);
    std::printf("wrote     %s\n", out.c_str());

    if (!png_dir.empty()) {
      make_dirs(png_dir);
      const json& view = file.header.at("view");
      const LineImage bands = features_to_bands(scene, render_default_view(scene, view));
      write_spectral_png(join_path(png_dir, "true_color.png"), bands.values.data(), kMapHeight, kMapWidth, wl,
                         PreviewMode::kTrueColor);
      write_map_png(join_path(png_dir, "labels.png"), scene, view, sam.label, lib.colors);
      std::vector<std::string> cluster_colors;
      for (int c = 0; c < clusters; ++c) cluster_colors.push_back(category_colors()[size_t(c) % category_colors().size()]);
      write_map_png(join_path(png_dir, "clusters.png"), scene, view, km.label, cluster_colors);
      std::printf("pictures  %s\n", png_dir.c_str());
    }
  } catch (const std::exception& e) {
    std::fprintf(stderr, "splat_materials: %s\n", e.what());
    return 1;
  }
  return 0;
}
