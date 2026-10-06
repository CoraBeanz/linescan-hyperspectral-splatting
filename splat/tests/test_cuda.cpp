// The GPU rasterizer and backward pass against the CPU reference. Skipped
// without CUDA.
#include <algorithm>
#include <cmath>
#include <memory>

#include "fixtures.hpp"
#include "linesplat/backward_cpu.hpp"
#include "linesplat/render_cpu.hpp"
#include "linesplat/spectra.hpp"
#include "linesplat/synthetic.hpp"
#include "linesplat/trainer.hpp"
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
  // Rendering into a used image overwrites all of it, whatever it held.
  LineImage reused = gpu.render(lsfix::cast_all<float>(lsfix::sweep_cameras(200, 64, 300.0, 0.05)));
  for (float& v : reused.values) v = -1.0f;
  gpu.render(cams, &reused);
  CHECK(reused.values == one.values);
  CHECK(reused.transmittance == one.transmittance);
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
  std::printf("       %d lines, %lld visible pairs, %lld tile entries, %.1f ms on the GPU\n", int(cams.size()),
              st.visible_pairs, st.tile_entries, st.gpu_ms);
  check_close(g, render_lines_cpu<float>(scene, cams));
}

namespace {

// |a - b| / |b| over a whole array.
double rel_err(const std::vector<float>& a, const std::vector<float>& b) {
  double d = 0, n = 0;
  for (size_t i = 0; i < b.size(); ++i) {
    d += (double(a[i]) - b[i]) * (double(a[i]) - b[i]);
    n += double(b[i]) * b[i];
  }
  return std::sqrt(d / std::max(n, 1e-300));
}

std::vector<float> flat(const std::vector<CameraGradT<float>>& g) {
  std::vector<float> v;
  for (const auto& c : g) {
    v.insert(v.end(), c.R, c.R + 9);
    v.insert(v.end(), c.t, c.t + 3);
  }
  return v;
}

struct BackwardResult {
  double loss = 0;
  SceneGradT<float> g;
  std::vector<CameraGradT<float>> cam;
};

// Like render_backward_cpu, the GPU's float sums can tip a splat across a
// cutoff, so compare whole arrays: each within 1% of the CPU's in norm (a
// wrong term would be off by order 1), and the pair counts nearly the same.
void check_backward_close(const BackwardResult& gpu, const BackwardResult& cpu) {
  CHECK(std::fabs(gpu.loss - cpu.loss) <= 1e-4 * cpu.loss);
  const struct {
    const char* name;
    const std::vector<float>& a;
    const std::vector<float>& b;
  } arrays[] = {{"means", gpu.g.means, cpu.g.means},
                {"log scales", gpu.g.log_scales, cpu.g.log_scales},
                {"rotations", gpu.g.rotations, cpu.g.rotations},
                {"opacity logits", gpu.g.opacity_logits, cpu.g.opacity_logits},
                {"features", gpu.g.features, cpu.g.features},
                {"background", gpu.g.background, cpu.g.background},
                {"screen grad", gpu.g.screen_grad, cpu.g.screen_grad}};
  for (const auto& a : arrays) {
    CHECK(a.a.size() == a.b.size());
    if (a.a.size() != a.b.size()) continue;
    const double e = rel_err(a.a, a.b);
    std::printf("       %-15s relative error %.2e\n", a.name, e);
    CHECK(e < 1e-2);
  }
  const double ec = rel_err(flat(gpu.cam), flat(cpu.cam));
  std::printf("       %-15s relative error %.2e\n", "cameras", ec);
  CHECK(ec < 1e-2);
  int differ = 0, total = 0;
  for (size_t i = 0; i < cpu.g.pairs.size(); ++i) {
    differ += gpu.g.pairs[i] != cpu.g.pairs[i];
    total += cpu.g.pairs[i];
  }
  std::printf("       loss %.6e vs %.6e; pair counts differ for %d of %zu Gaussians (%d pairs)\n", gpu.loss, cpu.loss,
              differ, cpu.g.pairs.size(), total);
  CHECK(total > 0);
  CHECK(differ <= int(cpu.g.pairs.size() / 100) + 1);
}

// The mean squared error against `targets` [L, W, B], both ways.
BackwardResult cpu_mse_backward(const GaussianScene& s, const std::vector<LineCamera>& cams,
                                const std::vector<float>& targets) {
  const int W = cams[0].width, B = s.num_bands();
  const double inv = 1.0 / (double(cams.size()) * W * B);
  BackwardResult r;
  r.loss = render_backward_cpu<float>(
      s, cams,
      [&](int l, const float* bands, float* g) {
        double sum = 0;
        for (int i = 0; i < W * B; ++i) {
          const double d = double(bands[i]) - targets[size_t(l) * W * B + i];
          g[i] = float(2.0 * d * inv);
          sum += d * d;
        }
        return sum * inv;
      },
      &r.g, &r.cam);
  return r;
}

BackwardResult gpu_mse_backward(CudaRasterizer& gpu, const std::vector<LineCamera>& cams) {
  std::vector<int> lines(cams.size());
  for (size_t i = 0; i < lines.size(); ++i) lines[i] = int(i);
  BackwardResult r;
  CudaRenderStats st;
  r.loss = gpu.mse_backward(cams, lines, &r.g, &r.cam, &st);
  std::printf("       GPU backward: %d batches, %lld visible pairs, %.2f ms on the GPU\n", st.batches,
              st.visible_pairs, st.gpu_ms);
  return r;
}

}  // namespace

TEST(gpu_backward_matches_cpu) {
  if (!have_gpu()) return;
  // 5 features through a random 7-band basis (one 8-channel pass), then 37
  // features as bands (three 16-channel passes with padding).
  for (int K : {5, 37}) {
    GaussianScene s = lsfix::random_scene(K == 5 ? 3000 : 1200, K, 60 + K);
    Rng rng(70 + K);
    if (K == 5) {
      s.basis.resize(size_t(7) * K);
      for (float& v : s.basis) v = float(rng.uniform(-0.5, 1.0));
    }
    const auto cams = lsfix::cast_all<float>(lsfix::sweep_cameras(90, 77, 500.0, 0.15, 2.0));
    const int W = 77, B = s.num_bands();
    std::vector<float> targets(cams.size() * W * B);
    for (float& v : targets) v = float(rng.uniform(0.0, 1.0));
    CudaRasterizer gpu;
    gpu.set_scene(s);
    gpu.set_targets(targets.data(), int(cams.size()), W, B);
    std::printf("       %d features, %d bands\n", K, B);
    check_backward_close(gpu_mse_backward(gpu, cams), cpu_mse_backward(s, cams, targets));
  }
}

TEST(gpu_backward_batches_give_the_same_answer) {
  if (!have_gpu()) return;
  const GaussianScene s = lsfix::random_scene(2000, 3, 81);
  const auto cams = lsfix::cast_all<float>(lsfix::sweep_cameras(100, 64, 400.0, 0.12));
  std::vector<float> targets(cams.size() * 64 * 3);
  Rng rng(82);
  for (float& v : targets) v = float(rng.uniform(0.0, 1.0));
  // The measured lines in reverse order, so camera i reads line 99 - i.
  std::vector<float> reversed(targets.size());
  for (size_t l = 0; l < cams.size(); ++l)
    std::copy(&targets[l * 64 * 3], &targets[l * 64 * 3] + 64 * 3, &reversed[(cams.size() - 1 - l) * 64 * 3]);
  CudaRasterizer gpu;
  gpu.set_scene(s);
  gpu.set_targets(targets.data(), int(cams.size()), 64, 3);
  const BackwardResult one = gpu_mse_backward(gpu, cams);
  gpu.max_pairs_per_batch = 2000 * 7;  // 7 lines per batch
  gpu.set_targets(reversed.data(), int(cams.size()), 64, 3);
  std::vector<int> lines(cams.size());
  for (size_t i = 0; i < lines.size(); ++i) lines[i] = int(cams.size() - 1 - i);
  BackwardResult many;
  CudaRenderStats st;
  many.loss = gpu.mse_backward(cams, lines, &many.g, &many.cam, &st);
  CHECK(st.batches == 15);
  // Only the order of the atomic sums differs.
  CHECK(std::fabs(many.loss - one.loss) <= 1e-6 * one.loss);
  CHECK(rel_err(many.g.means, one.g.means) < 1e-5);
  CHECK(rel_err(many.g.features, one.g.features) < 1e-5);
  CHECK(rel_err(many.g.log_scales, one.g.log_scales) < 1e-5);
  CHECK(rel_err(flat(many.cam), flat(one.cam)) < 1e-5);
  CHECK(many.g.pairs == one.g.pairs);
}

TEST(gpu_training_fits_the_lines_and_improves_the_poses) {
  if (!have_gpu()) return;
  // The CPU trainer test's scan, with the GPU's backward pass.
  SyntheticOptions so;
  so.width = 32;
  so.bands = 3;
  so.sweeps = 4;
  so.microsteps_per_line = 8;
  so.slit_samples = 3;
  const SyntheticData sd = make_synthetic_dataset(so);
  const Dataset& d = sd.dataset;
  std::vector<LineCameraT<double>> truth;
  for (int l = 0; l < d.num_lines(); ++l)
    truth.push_back(make_line_camera_d(d.head, sd.true_head_pose[size_t(d.line_sweep[size_t(l)])],
                                       sd.true_mirror_angle[size_t(l)], d.intrinsics));
  auto gpu = std::make_shared<CudaRasterizer>();
  gpu->set_targets(d.lines.data(), d.num_lines(), d.width(), d.num_bands());
  const BatchBackwardFn backward = [gpu](const GaussianScene& s, const std::vector<LineCamera>& cams,
                                         const std::vector<int>& lines, SceneGradT<float>* g,
                                         std::vector<CameraGradT<float>>* cg) {
    gpu->set_scene(s);
    return gpu->mse_backward(cams, lines, g, cg);
  };
  TrainOptions o;
  o.iterations = 200;
  o.batch_lines = 32;
  o.pose_from = 20;
  o.densify_from = 50;
  o.densify_every = 50;
  o.densify_until = 150;
  o.max_gaussians = 4000;
  Trainer t(d, init_on_plane(d, 3e-3), o, backward);
  const PoseErrorPx e0 = line_pose_error_px(t.cameras(), truth);
  double first = 0, last = 0;
  for (int i = 0; i < o.iterations; ++i) {
    const TrainStep st = t.step();
    if (i < 20) first += st.loss / 20;
    if (i >= o.iterations - 20) last += st.loss / 20;
  }
  const PoseErrorPx e1 = line_pose_error_px(t.cameras(), truth);
  std::printf("       loss %.4f -> %.4f, pose %.2f -> %.2f px, %d Gaussians\n", first, last, e0.rms, e1.rms,
              t.scene().size());
  CHECK(last < 0.25 * first);
  CHECK(e1.rms < 0.8 * e0.rms);
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
