// The GPU rasterizer against the CPU reference. Skipped without CUDA.
#include <algorithm>

#include "fixtures.hpp"
#include "linesplat/render_cpu.hpp"
#include "linesplat/spectra.hpp"
#include "linesplat/synthetic.hpp"
#include "test.hpp"

#ifdef LINESPLAT_WITH_CUDA
#include "linesplat/cuda_rasterizer.hpp"
#endif

using namespace linesplat;

#ifdef LINESPLAT_WITH_CUDA

namespace {

struct Diff {
  double worst = 0, mean = 0, frac_over = 0;
};

// Float rounding differs between host and device, and can tip a splat over
// a cutoff or swap two nearly equal depths, so compare in bulk: almost every
// value must match to 1e-4 and none may be far off.
Diff compare(const LineImage& a, const LineImage& b) {
  Diff d;
  size_t over = 0;
  for (size_t i = 0; i < a.values.size(); ++i) {
    const double e = std::fabs(double(a.values[i]) - double(b.values[i]));
    d.worst = std::max(d.worst, e);
    d.mean += e;
    over += e > 1e-4;
  }
  d.mean /= std::max<size_t>(1, a.values.size());
  d.frac_over = double(over) / std::max<size_t>(1, a.values.size());
  return d;
}

void check_close(const LineImage& gpu, const LineImage& cpu) {
  CHECK(gpu.lines == cpu.lines && gpu.width == cpu.width && gpu.channels == cpu.channels);
  const Diff d = compare(gpu, cpu);
  std::printf("       max diff %.2e, mean %.2e, %.4f%% over 1e-4\n", d.worst, d.mean, 100 * d.frac_over);
  CHECK(d.frac_over < 1e-3);
  CHECK(d.worst < 0.05);
  for (size_t i = 0; i < gpu.transmittance.size(); ++i)
    CHECK_NEAR(gpu.transmittance[i], cpu.transmittance[i], 0.05);
}

bool have_gpu() {
  std::string why;
  if (cuda_device_available(&why)) return true;
  lstest::skip_reason() = "no CUDA device: " + why;
  return false;
}

}  // namespace

TEST(gpu_matches_cpu_on_random_lines) {
  if (!have_gpu()) return;
  // Width not a multiple of 32, and 5 features (one 8-channel pass).
  const GaussianScene s = lsfix::random_scene(4000, 5, 51);
  const auto cams = lsfix::cast_all<float>(lsfix::sweep_cameras(300, 77, 500.0, 0.15, 2.0));
  CudaRasterizer gpu;
  gpu.set_scene(s);
  check_close(gpu.render(cams), render_lines_cpu<float>(s, cams));
}

TEST(gpu_matches_cpu_with_many_features) {
  if (!have_gpu()) return;
  // 37 features: three 16-channel passes with padding.
  const GaussianScene s = lsfix::random_scene(1500, 37, 52);
  const auto cams = lsfix::cast_all<float>(lsfix::sweep_cameras(120, 130, 800.0, 0.1));
  CudaRasterizer gpu;
  gpu.set_scene(s);
  check_close(gpu.render(cams), render_lines_cpu<float>(s, cams));
}

TEST(gpu_batches_give_the_same_answer) {
  if (!have_gpu()) return;
  const GaussianScene s = lsfix::random_scene(2000, 3, 53);
  const auto cams = lsfix::cast_all<float>(lsfix::sweep_cameras(200, 64, 400.0, 0.12));
  CudaRasterizer gpu;
  gpu.set_scene(s);
  const LineImage one = gpu.render(cams);
  gpu.max_pairs_per_batch = 2000 * 7;  // 7 lines per batch
  CudaRenderStats st;
  const LineImage many = gpu.render(cams, &st);
  CHECK(st.batches == 29);
  CHECK(one.values == many.values);
  CHECK(one.contributors == many.contributors);
}

TEST(gpu_matches_cpu_on_the_synthetic_scan) {
  if (!have_gpu()) return;
  SyntheticOptions o;
  o.width = 96;
  o.bands = 20;
  o.sweeps = 2;
  o.microsteps_per_line = 4;
  const std::vector<double> wl = wavelength_grid(o.wl_min_nm, o.wl_max_nm, o.bands);
  const GaussianScene scene = make_synthetic_scene(o, wl);
  Dataset d = make_synthetic_dataset(o).dataset;
  const auto cams = d.cameras();
  CudaRasterizer gpu;
  gpu.set_scene(scene);
  CudaRenderStats st;
  const LineImage g = gpu.render(cams, &st);
  std::printf("       %d lines, %lld visible pairs, %lld tile entries, %.1f ms\n", int(cams.size()),
              st.visible_pairs, st.tile_entries, st.ms);
  check_close(g, render_lines_cpu<float>(scene, cams));
}

TEST(gpu_renders_an_empty_scene_as_background) {
  if (!have_gpu()) return;
  GaussianScene s;
  s.num_features = 2;
  s.set_identity_basis(2);
  s.background = {0.3f, 0.6f};
  CudaRasterizer gpu;
  gpu.set_scene(s);
  const LineImage img = gpu.render(lsfix::cast_all<float>(lsfix::sweep_cameras(5, 40, 300.0, 0.1)));
  for (size_t i = 0; i < img.values.size(); ++i) CHECK(img.values[i] == (i % 2 ? 0.6f : 0.3f));
}

#else

TEST(gpu_tests_need_cuda) { SKIP("built without CUDA"); }

#endif
