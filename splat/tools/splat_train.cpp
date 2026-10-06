// splat_train: trains a hyperspectral splat from a pushbroom dataset.
//
//   splat_train DATASET OUT_DIR [--iterations N] [--batch N] [--spacing MM]
//               [--no-poses] [--pose-from N] [--densify-grad X] [--log-every N]
//               [--seed N] [--cpu]
//
// Starts from Gaussians on the board plane (z = 0), trains (rendering and
// backpropagating on the GPU if there is one, unless --cpu), and writes
// OUT_DIR/scene/ (the Gaussians), OUT_DIR/sweep_head_pose.npy (the refined
// head poses), OUT_DIR/log.csv and previews: per sweep, the measured
// lines next to the trained scene's, and the scene from the overview camera.
// For a synthetic dataset it also reports how far the line cameras are from
// the true ones, in pixels, before and after.
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
               "                   [--seed N] [--cpu]\n");
  std::exit(2);
}

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

double rmse(const std::vector<float>& a, const std::vector<float>& b) {
  double s = 0;
  for (size_t i = 0; i < a.size(); ++i) s += double(a[i] - b[i]) * double(a[i] - b[i]);
  return std::sqrt(s / std::max<size_t>(1, a.size()));
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
  bool cpu = false;
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
    else usage();
  }

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

    std::printf("dataset   %d sweeps, %d lines of %d px x %d bands\n", d.num_sweeps(), d.num_lines(), d.width(),
                d.num_bands());
    BatchBackwardFn backward;  // empty: the CPU reference
    std::string device = "the CPU";
#ifdef LINESPLAT_WITH_CUDA
    if (!cpu && cuda_device_available()) {
      // The measured lines go to the GPU once; the scene goes every step.
      auto gpu = std::make_shared<CudaRasterizer>();
      gpu->set_targets(d.lines.data(), d.num_lines(), d.width(), d.num_bands());
      backward = [gpu](const GaussianScene& s, const std::vector<LineCamera>& cams, const std::vector<int>& lines,
                       SceneGradT<float>* g, std::vector<CameraGradT<float>>* cg) {
        gpu->set_scene(s);
        return gpu->mse_backward(cams, lines, g, cg);
      };
      device = cuda_device_name();
    }
#endif
    (void)cpu;  // read only when built with CUDA
    Trainer t(d, init_on_plane(d, spacing_mm * 1e-3), o, backward);
    std::printf("device    %s\n", device.c_str());
    std::printf("start     %d Gaussians on the board plane, %.2f mm apart\n", t.scene().size(), spacing_mm);
    if (!truth.empty())
      std::printf("pose err  %s at the recorded poses\n", pose_text(line_pose_error_px(t.cameras(), truth)).c_str());

    make_dirs(out);
    std::string csv = truth.empty() ? "iteration,rmse,gaussians,ms\n" : "iteration,rmse,gaussians,ms,pose_rms_px\n";
    double loss = 0, ms = 0;
    int added = 0, removed = 0, n = 0;
    for (int i = 0; i < o.iterations; ++i) {
      const TrainStep st = t.step();
      loss += st.loss;
      ms += st.ms;
      added += st.densified;
      removed += st.pruned;
      ++n;
      if (st.iteration % log_every != 0 && st.iteration != o.iterations) continue;
      char row[160];
      std::string pose;
      if (!truth.empty()) {
        const PoseErrorPx e = line_pose_error_px(t.cameras(), truth);
        pose = "  pose " + std::to_string(e.rms).substr(0, 5) + " px";
        std::snprintf(row, sizeof row, "%d,%.6f,%d,%.1f,%.4f\n", st.iteration, std::sqrt(loss / n), st.gaussians,
                      ms / n, e.rms);
      } else {
        std::snprintf(row, sizeof row, "%d,%.6f,%d,%.1f\n", st.iteration, std::sqrt(loss / n), st.gaussians, ms / n);
      }
      csv += row;
      std::printf("iter %5d  rmse %.4f  %6d Gaussians (+%d -%d)  %4.0f ms/iter%s\n", st.iteration,
                  std::sqrt(loss / n), st.gaussians, added, removed, ms / n, pose.c_str());
      std::fflush(stdout);
      loss = ms = 0;
      added = removed = n = 0;
    }

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
    const LineImage bands = features_to_bands(scene, render_any(scene, cams));
    std::printf("done      %d Gaussians in %.1f s; RMSE vs measured lines %.4f", scene.size(), total.ms() / 1000.0,
                rmse(bands.values, d.lines));
    const std::string clean = join_path(dir, "gt/lines_clean.npy");
    if (file_exists(clean)) std::printf(", vs noise-free lines %.4f", rmse(bands.values, npy_load_f32(clean)));
    std::printf("\n");
    if (!truth.empty())
      std::printf("pose err  %s at the refined poses\n", pose_text(line_pose_error_px(t.cameras(), truth)).c_str());

    const std::string pv = join_path(out, "preview");
    make_dirs(pv);
    write_sweep_pngs(join_path(pv, "measured_vs_trained"), d, {d.lines.data(), bands.values.data()});
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
