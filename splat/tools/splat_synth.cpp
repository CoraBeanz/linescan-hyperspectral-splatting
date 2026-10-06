// splat_synth: writes a synthetic pushbroom dataset with known answers.
//
//   splat_synth OUT_DIR [--preset tiny|small|default|full] [--width N]
//               [--bands N] [--sweeps N] [--microsteps N] [--slit-samples N]
//               [--spacing MM] [--seed N] [--errors SCALE] [--no-errors] [--cpu]
//
// --errors scales every pose and mirror error (1 is the default; 0 is the
// same as --no-errors).
//
// Renders on the GPU when one is available (unless --cpu).
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <stdexcept>
#include <string>

#include "linesplat/synthetic.hpp"
#include "linesplat/util.hpp"
#ifdef LINESPLAT_WITH_CUDA
#include "linesplat/cuda_rasterizer.hpp"
#endif

using namespace linesplat;

namespace {

void usage() {
  std::fprintf(stderr,
               "usage: splat_synth OUT_DIR [--preset tiny|small|default|full] [--width N] [--bands N]\n"
               "                   [--sweeps N] [--microsteps N] [--slit-samples N] [--spacing MM]\n"
               "                   [--seed N] [--errors SCALE] [--no-errors] [--cpu]\n");
  std::exit(2);
}

void apply_preset(const std::string& p, SyntheticOptions& o) {
  if (p == "tiny") {
    o.width = 64; o.bands = 12; o.sweeps = 3; o.microsteps_per_line = 4; o.spacing = 1.5e-3;
  } else if (p == "small") {
    o.width = 128; o.bands = 24; o.sweeps = 6; o.microsteps_per_line = 2;
  } else if (p == "default") {
    o.width = 256; o.bands = 46; o.sweeps = 8; o.microsteps_per_line = 1;
  } else if (p == "full") {
    o.width = 512; o.bands = 91; o.sweeps = 12; o.microsteps_per_line = 1; o.spacing = 0.7e-3;
  } else {
    usage();
  }
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 2 || argv[1][0] == '-') usage();
  const std::string out = argv[1];
  SyntheticOptions o;
  bool use_cpu = false;
  for (int i = 2; i < argc; ++i) {
    const std::string a = argv[i];
    auto next = [&]() -> const char* {
      if (i + 1 >= argc) usage();
      return argv[++i];
    };
    if (a == "--preset") apply_preset(next(), o);
    else if (a == "--width") o.width = std::atoi(next());
    else if (a == "--bands") o.bands = std::atoi(next());
    else if (a == "--sweeps") o.sweeps = std::atoi(next());
    else if (a == "--microsteps") o.microsteps_per_line = std::atoi(next());
    else if (a == "--slit-samples") o.slit_samples = std::atoi(next());
    else if (a == "--spacing") o.spacing = std::atof(next()) * 1e-3;
    else if (a == "--seed") o.seed = std::strtoull(next(), nullptr, 10);
    else if (a == "--errors" || a == "--no-errors") {
      const double k = a == "--errors" ? std::atof(next()) : 0.0;
      o.head_trans_sigma *= k;
      o.head_rot_sigma_deg *= k;
      o.mirror_offset_sigma_deg *= k;
      o.mirror_jitter_sigma_deg *= k;
    } else if (a == "--cpu") use_cpu = true;
    else usage();
  }

  try {
    LineRenderFn render = cpu_renderer();
    std::string device = "CPU";
#ifdef LINESPLAT_WITH_CUDA
    std::string why;
    if (!use_cpu && cuda_device_available(&why)) {
      auto gpu = std::make_shared<CudaRasterizer>();
      render = [gpu](const GaussianScene& s, const std::vector<LineCamera>& c) {
        gpu->set_scene(s);
        return gpu->render(c);
      };
      device = cuda_device_name();
    } else if (!use_cpu) {
      std::printf("no GPU (%s), rendering on the CPU\n", why.c_str());
    }
#endif
    (void)use_cpu;
    Timer t;
    const SyntheticData data = make_synthetic_dataset(o, render);
    const double gen_ms = t.ms();
    save_synthetic(data, out, render);
    const Dataset& d = data.dataset;
    std::printf("wrote %s\n", out.c_str());
    std::printf("  scene     %d Gaussians, %d bands from %.0f to %.0f nm\n", data.scene.size(), d.num_bands(),
                d.wavelengths_nm.front(), d.wavelengths_nm.back());
    std::printf("  scan      %d sweeps, %d lines of %d px (%.1f MB)\n", d.num_sweeps(), d.num_lines(), d.width(),
                d.lines.size() * 4.0 / (1 << 20));
    std::printf("  rendered  on %s in %.1f s (%d sub-lines per line across the slit)\n", device.c_str(),
                gen_ms / 1000.0, o.slit_samples);
    double t_err = 0, r_err = 0;
    for (int s = 0; s < d.num_sweeps(); ++s) {
      const Pose e = data.true_head_pose[size_t(s)].inverse() * d.sweep_head_pose[size_t(s)];
      t_err = std::max(t_err, norm(e.t));
      r_err = std::max(r_err, norm(rotation_log(e.R)));
    }
    std::printf("  pose err  up to %.2f mm and %.2f deg per sweep (recorded vs true)\n", t_err * 1e3,
                r_err * 180.0 / kPi);
  } catch (const std::exception& e) {
    std::fprintf(stderr, "splat_synth: %s\n", e.what());
    return 1;
  }
  return 0;
}
