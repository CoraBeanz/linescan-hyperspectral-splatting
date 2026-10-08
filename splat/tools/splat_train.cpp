// splat_train: trains a hyperspectral splat from a pushbroom dataset.
//
//   splat_train DATASET OUT_DIR [--iterations N] [--batch N] [--spacing MM]
//               [--no-poses] [--pose-from N] [--densify-grad X] [--log-every N]
//               [--seed N] [--cpu] [--gpu-adam] [--basis K] [--profile]
//               [--no-preview]
//
// Starts from Gaussians on the board plane (z = 0), trains (rendering and
// backpropagating on the GPU if there is one, unless --cpu), and writes
// OUT_DIR/scene/ (the Gaussians), OUT_DIR/sweep_head_pose.npy (the refined
// head poses), OUT_DIR/log.csv and previews: per sweep, the measured
// lines next to the trained scene's, and the scene from the overview camera.
// For a synthetic dataset it also reports how far the line cameras are from
// the true ones, in pixels, before and after.
//
//   --gpu-adam    keep the Gaussians, Adam and densification on the GPU, so
//                 that only cameras cross over each step
//   --basis K     K features through a learned spectral basis, instead of one
//                 feature per band (starts from the spectra's top K
//                 principal directions)
//   --profile     time every GPU pass and print where the time went
//   --no-preview  skip the previews at the end
// At the end it prints the time per step (and what took it), the GPU memory
// by buffer, and the process's peak memory, for docs/nano_budget.md.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "linesplat/dataset.hpp"
#include "linesplat/npy.hpp"
#include "linesplat/preview.hpp"
#include "linesplat/render_cpu.hpp"
#include "linesplat/synthetic.hpp"
#include "linesplat/trainer.hpp"
#include "linesplat/util.hpp"
#ifdef LINESPLAT_WITH_CUDA
#include "linesplat/cuda_rasterizer.hpp"
#endif

using namespace linesplat;

namespace {

void usage() {
  std::fprintf(stderr,
               "usage: splat_train DATASET OUT_DIR [--iterations N] [--batch N] [--spacing MM]\n"
               "                   [--no-poses] [--pose-from N] [--densify-grad X] [--log-every N]\n"
               "                   [--seed N] [--cpu] [--gpu-adam] [--basis K] [--profile] [--no-preview]\n");
  std::exit(2);
}

double mb(size_t bytes) { return double(bytes) / (1024.0 * 1024.0); }

LineImage render_any(const GaussianScene& s, const std::vector<LineCamera>& cams) {
#ifdef LINESPLAT_WITH_CUDA
  if (cuda_device_available()) {
    CudaRasterizer gpu;
    gpu.set_scene(s);
    return gpu.render(cams);
  }
#endif
  return render_lines_cpu<float>(s, cams);
}

// The RMSE of a scene's bands against the measured lines and, if there are
// any, the noise-free ones. Renders 256 lines at a time: all at once, the
// rendered features and bands would be two more copies of the dataset in
// memory, which on the Nano is shared with the GPU. Keeps the rendered bands
// in *bands only if it isn't null (for the previews).
void compare_lines(const GaussianScene& s, const std::vector<LineCamera>& cams, const Dataset& d,
                   const std::vector<float>& clean, double* rmse, double* rmse_clean, std::vector<float>* bands) {
  const size_t per_line = size_t(d.width()) * d.num_bands();
  double se = 0, se_clean = 0;
  for (size_t l0 = 0; l0 < cams.size(); l0 += 256) {
    const std::vector<LineCamera> part(cams.begin() + l0, cams.begin() + std::min(cams.size(), l0 + 256));
    const LineImage b = features_to_bands(s, render_any(s, part));
    const size_t o = l0 * per_line;
    for (size_t i = 0; i < b.values.size(); ++i) {
      const double e = double(b.values[i]) - d.lines[o + i];
      se += e * e;
      if (!clean.empty()) {
        const double c = double(b.values[i]) - clean[o + i];
        se_clean += c * c;
      }
    }
    if (bands) bands->insert(bands->end(), b.values.begin(), b.values.end());
  }
  const double n = double(std::max<size_t>(1, cams.size() * per_line));
  *rmse = std::sqrt(se / n);
  *rmse_clean = std::sqrt(se_clean / n);
}

std::string pose_text(const PoseErrorPx& e) {
  char buf[128];
  std::snprintf(buf, sizeof buf, "%.2f px rms (along the slit %.2f, across %.2f), max %.2f", e.rms, e.rms_along,
                e.rms_across, e.max);
  return buf;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 3 || argv[1][0] == '-' || argv[2][0] == '-') usage();
  const std::string dir = argv[1], out = argv[2];
  TrainOptions o;
  double spacing_mm = 1.0;
  int log_every = 100;
  bool cpu = false, gpu_adam = false, profile = false, preview = true;
  int basis = 0;
  for (int i = 3; i < argc; ++i) {
    const std::string a = argv[i];
    auto next = [&]() -> const char* {
      if (i + 1 >= argc) usage();
      return argv[++i];
    };
    if (a == "--iterations") o.iterations = std::atoi(next());
    else if (a == "--batch") o.batch_lines = std::atoi(next());
    else if (a == "--spacing") spacing_mm = std::atof(next());
    else if (a == "--no-poses") o.refine_poses = false;
    else if (a == "--pose-from") o.pose_from = std::atoi(next());
    else if (a == "--densify-grad") o.densify_grad = std::atof(next());
    else if (a == "--log-every") log_every = std::max(1, std::atoi(next()));
    else if (a == "--seed") o.seed = std::strtoull(next(), nullptr, 10);
    else if (a == "--cpu") cpu = true;
    else if (a == "--gpu-adam") gpu_adam = true;
    else if (a == "--basis") basis = std::atoi(next());
    else if (a == "--profile") profile = true;
    else if (a == "--no-preview") preview = false;
    else usage();
  }
  if (basis < 0) usage();
  o.learn_basis = basis > 0;

  try {
    Timer total;
    const Dataset d = Dataset::load(dir);
    // The true line cameras, if this is a synthetic dataset.
    std::vector<LineCameraT<double>> truth;
    const std::string gt_pose = join_path(dir, "gt/sweep_head_pose.npy");
    if (file_exists(gt_pose)) {
      const std::vector<double> p = npy_load_f64(gt_pose);
      const std::vector<double> angle = npy_load_f64(join_path(dir, "gt/line_mirror_angle.npy"));
      for (int l = 0; l < d.num_lines(); ++l)
        truth.push_back(make_line_camera_d(d.head, Pose::from_array(&p[7 * size_t(d.line_sweep[size_t(l)])]),
                                           angle[size_t(l)], d.intrinsics));
    }

    std::printf("dataset   %d sweeps, %d lines of %d px x %d bands (%.1f MB of lines)\n", d.num_sweeps(),
                d.num_lines(), d.width(), d.num_bands(), mb(d.lines.size() * sizeof(float)));
    GaussianScene init = init_on_plane(d, spacing_mm * 1e-3);
    const int start_n = init.size();
    if (basis > 0) init = reduce_features(init, basis);
    std::unique_ptr<SceneOptimizer> opt;
    std::string device = "the CPU", where = "Adam on the CPU";
#ifdef LINESPLAT_WITH_CUDA
    std::shared_ptr<CudaRasterizer> gpu;  // GPU backward, Adam on the CPU
    auto gpu_stats = std::make_shared<CudaRenderStats>();
    CudaSceneOptimizer* resident = nullptr;  // everything on the GPU
    if (!cpu && cuda_device_available()) {
      device = cuda_device_name();
      if (gpu_adam) {
        resident = new CudaSceneOptimizer(d, init);
        resident->rasterizer().profile = profile;
        opt.reset(resident);
        where = "Adam and densification on the GPU";
      } else {
        // The measured lines go to the GPU once; the scene goes every step.
        gpu = std::make_shared<CudaRasterizer>();
        gpu->profile = profile;
        gpu->set_targets(d.lines.data(), d.num_lines(), d.width(), d.num_bands());
        opt.reset(new HostSceneOptimizer(
            init, [gpu, gpu_stats](const GaussianScene& s, const std::vector<LineCamera>& cams,
                                   const std::vector<int>& lines, SceneGradT<float>* g,
                                   std::vector<CameraGradT<float>>* cg) {
              const Timer up;
              gpu->set_scene(s);
              CudaRenderStats st;
              const double loss = gpu->mse_backward(cams, lines, g, cg, &st);
              st.pass_ms[kPassCopy] += up.ms() - st.ms;  // set_scene's copies, as wall time
              *gpu_stats += st;
              return loss;
            }));
      }
    }
#endif
    if (!opt) opt.reset(new HostSceneOptimizer(init, cpu_batch_backward(d)));
    (void)cpu;  // these are read only when built with CUDA
    (void)gpu_adam;
    (void)profile;
    Trainer t(d, std::move(opt), o);
    std::printf("device    %s, %s\n", device.c_str(), where.c_str());
    std::printf("start     %d Gaussians on the board plane, %.2f mm apart", start_n, spacing_mm);
    if (basis > 0) std::printf(", %d features through a learned basis", basis);
    std::printf("\n");
    if (!truth.empty())
      std::printf("pose err  %s at the recorded poses\n", pose_text(line_pose_error_px(t.cameras(), truth)).c_str());

    make_dirs(out);
    std::string csv = truth.empty() ? "iteration,rmse,gaussians,ms,backward_ms,update_ms\n"
                                    : "iteration,rmse,gaussians,ms,backward_ms,update_ms,pose_rms_px\n";
    double loss = 0, ms = 0, back_ms = 0, up_ms = 0;
    int added = 0, removed = 0, n = 0;
    // Steady-state timings: every step after the first 10% (the GPU's
    // buffers grow and the CPU's caches warm up early on).
    double sum_ms = 0, sum_back = 0, sum_up = 0;
    int timed = 0;
    const Timer train_clock;
    for (int i = 0; i < o.iterations; ++i) {
      const TrainStep st = t.step();
      loss += st.loss;
      ms += st.ms;
      back_ms += st.backward_ms;
      up_ms += st.update_ms;
      added += st.densified;
      removed += st.pruned;
      ++n;
      if (st.iteration > o.iterations / 10) {
        sum_ms += st.ms;
        sum_back += st.backward_ms;
        sum_up += st.update_ms;
        ++timed;
      }
      if (st.iteration % log_every != 0 && st.iteration != o.iterations) continue;
      char row[200];
      std::string pose;
      if (!truth.empty()) {
        const PoseErrorPx e = line_pose_error_px(t.cameras(), truth);
        pose = "  pose " + std::to_string(e.rms).substr(0, 5) + " px";
        std::snprintf(row, sizeof row, "%d,%.6f,%d,%.2f,%.2f,%.2f,%.4f\n", st.iteration, std::sqrt(loss / n),
                      st.gaussians, ms / n, back_ms / n, up_ms / n, e.rms);
      } else {
        std::snprintf(row, sizeof row, "%d,%.6f,%d,%.2f,%.2f,%.2f\n", st.iteration, std::sqrt(loss / n),
                      st.gaussians, ms / n, back_ms / n, up_ms / n);
      }
      csv += row;
      std::printf("iter %5d  rmse %.4f  %6d Gaussians (+%d -%d)  %4.0f ms/iter%s\n", st.iteration,
                  std::sqrt(loss / n), st.gaussians, added, removed, ms / n, pose.c_str());
      std::fflush(stdout);
      loss = ms = back_ms = up_ms = 0;
      added = removed = n = 0;
    }
    const double train_s = train_clock.ms() / 1000.0;
    if (timed > 0)
      std::printf("time      %.2f ms/step after the first 10%%: backward %.2f, Adam and densify %.2f, the rest %.2f; "
                  "%.1f s for %d steps\n",
                  sum_ms / timed, sum_back / timed, sum_up / timed, (sum_ms - sum_back - sum_up) / timed, train_s,
                  o.iterations);
#ifdef LINESPLAT_WITH_CUDA
    if (gpu || resident) {
      const CudaMemoryUse u = resident ? resident->memory_use() : gpu->memory_use();
      std::printf("gpu mem   %.1f MB in splat's buffers: scene %.1f, gradients %.1f, Adam %.1f, pairs %.1f, "
                  "splats %.1f, sort keys %.1f, pixels %.1f, cameras %.2f, measured lines %.1f, Thrust %.1f, "
                  "densify %.1f\n",
                  mb(u.total()), mb(u.scene), mb(u.gradients), mb(u.adam), mb(u.pairs), mb(u.splats), mb(u.keys),
                  mb(u.pixels), mb(u.cameras), mb(u.targets), mb(u.thrust), mb(u.densify));
      size_t free_b = 0, total_b = 0;
      if (cuda_memory_info(&free_b, &total_b))
        std::printf("device    %.0f MB used of %.0f MB (everything on the device, the CUDA context included)\n",
                    mb(total_b - free_b), mb(total_b));
      if (profile) {
        const CudaRenderStats& ps = resident ? resident->stats() : *gpu_stats;
        double sum = 0;
        std::printf("gpu time  per step, every step:");
        for (int p = 0; p < kCudaPasses; ++p) {
          if (ps.pass_ms[p] <= 0) continue;
          std::printf(" %s %.3f,", cuda_pass_name(p), ps.pass_ms[p] / o.iterations);
          sum += ps.pass_ms[p];
        }
        std::printf(" in all %.2f ms; %.0f visible pairs and %.0f sort keys a step\n", sum / o.iterations,
                    double(ps.visible_pairs) / o.iterations, double(ps.tile_entries) / o.iterations);
      }
    }
#endif

    // Results.
    const GaussianScene& scene = t.scene();
    scene.save(join_path(out, "scene"));
    std::vector<double> poses;
    for (const Pose& p : t.head_poses()) {
      double v[7];
      p.to_array(v);
      poses.insert(poses.end(), v, v + 7);
    }
    npy_save(join_path(out, "sweep_head_pose.npy"), poses, {size_t(d.num_sweeps()), 7});
    write_text_file(join_path(out, "log.csv"), csv);

    std::vector<LineCamera> cams;
    for (const auto& c : t.cameras()) cams.push_back(cast_camera<float>(c));
    std::vector<float> trained;  // every line's bands, for the previews
    {
      const std::string clean_path = join_path(dir, "gt/lines_clean.npy");
      const std::vector<float> clean = file_exists(clean_path) ? npy_load_f32(clean_path) : std::vector<float>();
      double fit = 0, fit_clean = 0;
      compare_lines(scene, cams, d, clean, &fit, &fit_clean, preview ? &trained : nullptr);
      std::printf("done      %d Gaussians in %.1f s; RMSE vs measured lines %.4f", scene.size(), total.ms() / 1000.0,
                  fit);
      if (!clean.empty()) std::printf(", vs noise-free lines %.4f", fit_clean);
      std::printf("\n");
    }
    if (!truth.empty())
      std::printf("pose err  %s at the refined poses\n", pose_text(line_pose_error_px(t.cameras(), truth)).c_str());

    const double rss = peak_rss_mb();
    if (rss > 0) std::printf("host mem  %.0f MB at most (the process's peak resident memory)\n", rss);
    if (!preview) {
      std::printf("wrote     %s (scene/, sweep_head_pose.npy, log.csv)\n", out.c_str());
      return 0;
    }
    const std::string pv = join_path(out, "preview");
    make_dirs(pv);
    write_sweep_pngs(join_path(pv, "measured_vs_trained"), d, {d.lines.data(), trained.data()});
    const int w = 480, h = 360;
    const LineImage view = features_to_bands(scene, render_any(scene, pinhole_rows(overview_camera(), w, h, 640.0)));
    write_spectral_png(join_path(pv, "scene_rgb.png"), view.values.data(), h, w, d.wavelengths_nm,
                       PreviewMode::kTrueColor);
    write_spectral_png(join_path(pv, "scene_cir.png"), view.values.data(), h, w, d.wavelengths_nm,
                       PreviewMode::kColorInfrared);
    std::printf("wrote     %s (scene/, sweep_head_pose.npy, log.csv, preview/)\n", out.c_str());
  } catch (const std::exception& e) {
    std::fprintf(stderr, "splat_train: %s\n", e.what());
    return 1;
  }
  return 0;
}
