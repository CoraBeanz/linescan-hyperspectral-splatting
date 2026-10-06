// splat_render: renders a scene through a dataset's line cameras.
//
//   splat_render compare  DATASET [SCENE]   CPU vs GPU on every line, with timings
//   splat_render residual DATASET [SCENE] [--png PREFIX]
//                                           how well the scene explains the lines,
//                                           at the recorded poses and (for
//                                           synthetic data) at the true ones;
//                                           --png writes one true color image per
//                                           sweep: measured | recorded | true
//   splat_render view     SCENE OUT_PREFIX  true color and CIR images from the
//                                           synthetic scene's overview camera
//
// SCENE defaults to DATASET/gt/scene (the synthetic ground truth).
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

#include "linesplat/dataset.hpp"
#include "linesplat/npy.hpp"
#include "linesplat/preview.hpp"
#include "linesplat/render_cpu.hpp"
#include "linesplat/spectra.hpp"
#include "linesplat/synthetic.hpp"
#include "linesplat/util.hpp"
#ifdef LINESPLAT_WITH_CUDA
#include "linesplat/cuda_rasterizer.hpp"
#endif

using namespace linesplat;

namespace {

void usage() {
  std::fprintf(stderr,
               "usage: splat_render compare  DATASET [SCENE]\n"
               "       splat_render residual DATASET [SCENE] [--png PREFIX]\n"
               "       splat_render view     SCENE OUT_PREFIX\n");
  std::exit(2);
}

LineImage render_any(const GaussianScene& s, const std::vector<LineCamera>& cams, std::string* device) {
#ifdef LINESPLAT_WITH_CUDA
  if (cuda_device_available()) {
    CudaRasterizer gpu;
    gpu.set_scene(s);
    if (device) *device = cuda_device_name();
    return gpu.render(cams);
  }
#endif
  if (device) *device = "CPU";
  return render_lines_cpu<float>(s, cams);
}

double rmse(const std::vector<float>& a, const std::vector<float>& b) {
  double s = 0;
  for (size_t i = 0; i < a.size(); ++i) s += double(a[i] - b[i]) * double(a[i] - b[i]);
  return std::sqrt(s / std::max<size_t>(1, a.size()));
}

int cmd_compare(const Dataset& d, const GaussianScene& scene) {
  const auto cams = d.cameras();
  Timer tc;
  const LineImage cpu = render_lines_cpu<float>(scene, cams);
  const double cpu_ms = tc.ms();
  std::printf("CPU  %d lines x %d px x %d features in %.1f ms\n", cpu.lines, cpu.width, cpu.channels, cpu_ms);
#ifdef LINESPLAT_WITH_CUDA
  std::string why;
  if (!cuda_device_available(&why)) {
    std::printf("GPU  not available: %s\n", why.c_str());
    return 0;
  }
  CudaRasterizer gpu;
  gpu.set_scene(scene);
  gpu.render(cams);  // warm up
  CudaRenderStats st;
  double best = 1e30;
  LineImage g;
  for (int rep = 0; rep < 5; ++rep) {
    g = gpu.render(cams, &st);
    best = std::min(best, st.ms);
  }
  double worst = 0, mean = 0;
  size_t over = 0;
  for (size_t i = 0; i < g.values.size(); ++i) {
    const double e = std::fabs(double(g.values[i]) - double(cpu.values[i]));
    worst = std::max(worst, e);
    mean += e;
    over += e > 1e-4;
  }
  mean /= double(g.values.size());
  std::printf("GPU  %s: %.2f ms (best of 5), %d batches, %lld visible pairs, %lld tile entries\n",
              cuda_device_name().c_str(), best, st.batches, st.visible_pairs, st.tile_entries);
  std::printf("diff max %.2e, mean %.2e, %.4f%% of values over 1e-4, speedup %.0fx\n", worst, mean,
              100.0 * over / g.values.size(), cpu_ms / best);
  return (worst < 0.05 && over < g.values.size() / 1000) ? 0 : 1;
#else
  std::printf("GPU  not built (no CUDA compiler at configure time)\n");
  return 0;
#endif
}

// One true color image per sweep with the panels side by side, each the
// sweep's lines stacked in the order they were taken.
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
  std::printf("wrote %s_sweep_XX.png for %d sweeps\n", prefix.c_str(), d.num_sweeps());
}

int cmd_residual(const std::string& dir, const Dataset& d, const GaussianScene& scene, const std::string& png) {
  std::string device;
  const LineImage rec = features_to_bands(scene, render_any(scene, d.cameras(), &device));
  std::printf("rendered on %s\n", device.c_str());
  std::printf("RMSE vs measured lines, recorded poses: %.4f\n", rmse(rec.values, d.lines));
  std::vector<const float*> panels{d.lines.data(), rec.values.data()};
  LineImage tru;
  const std::string gt_pose = join_path(dir, "gt/sweep_head_pose.npy");
  if (file_exists(gt_pose)) {
    Dataset t = d;
    const std::vector<double> p = npy_load_f64(gt_pose);
    for (int s = 0; s < t.num_sweeps(); ++s) t.sweep_head_pose[size_t(s)] = Pose::from_array(&p[7 * size_t(s)]);
    t.line_mirror_angle = npy_load_f64(join_path(dir, "gt/line_mirror_angle.npy"));
    // The ground truth integrates the slit as a box; the renderer's Gaussian
    // approximation of it is one part of what's left.
    tru = features_to_bands(scene, render_any(scene, t.cameras(), nullptr));
    std::printf("RMSE vs measured lines, true poses:     %.4f\n", rmse(tru.values, d.lines));
    const std::string clean = join_path(dir, "gt/lines_clean.npy");
    if (file_exists(clean))
      std::printf("RMSE vs noise-free lines, true poses:   %.4f\n", rmse(tru.values, npy_load_f32(clean)));
    panels.push_back(tru.values.data());
  }
  if (!png.empty()) write_sweep_pngs(png, d, panels);
  return 0;
}

int cmd_view(const GaussianScene& scene, const std::string& prefix) {
  const int w = 480, h = 360;
  std::string device;
  const LineImage img = features_to_bands(scene, render_any(scene, pinhole_rows(overview_camera(), w, h, 640.0), &device));
  const int B = scene.num_bands();
  // Synthetic scenes cover 500-950 nm; anything else gets the same spacing.
  const std::vector<double> wl = wavelength_grid(500.0, 950.0, B);
  write_spectral_png(prefix + "_rgb.png", img.values.data(), h, w, wl, PreviewMode::kTrueColor);
  write_spectral_png(prefix + "_cir.png", img.values.data(), h, w, wl, PreviewMode::kColorInfrared);
  std::printf("wrote %s_rgb.png and %s_cir.png (rendered on %s)\n", prefix.c_str(), prefix.c_str(), device.c_str());
  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 3) usage();
  const std::string cmd = argv[1];
  try {
    if (cmd == "view") {
      if (argc < 4) usage();
      return cmd_view(GaussianScene::load(argv[2]), argv[3]);
    }
    const std::string dir = argv[2];
    std::string scene_dir = join_path(dir, "gt/scene"), png;
    for (int i = 3; i < argc; ++i) {
      if (std::strcmp(argv[i], "--png") == 0 && i + 1 < argc)
        png = argv[++i];
      else if (argv[i][0] != '-')
        scene_dir = argv[i];
      else
        usage();
    }
    const Dataset d = Dataset::load(dir);
    const GaussianScene scene = GaussianScene::load(scene_dir);
    if (cmd == "compare") return cmd_compare(d, scene);
    if (cmd == "residual") return cmd_residual(dir, d, scene, png);
    usage();
  } catch (const std::exception& e) {
    std::fprintf(stderr, "splat_render: %s\n", e.what());
    return 1;
  }
  return 0;
}
