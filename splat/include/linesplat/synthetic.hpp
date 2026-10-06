// A synthetic pushbroom dataset with known answers.
//
// The scene is made of Gaussians, so the renderer can represent it exactly:
// an 80 x 64 mm board on a table with a checker border (standing in for the
// AprilTag board), a row of paint patches, a panel that is black to the eye
// but hides the word "NIR" past 740 nm, a leaf-green ball and an orange box.
// Every material has a known reflectance spectrum (spectra.hpp).
//
// The scan follows the real rig: each sweep holds the head at one arm pose
// and steps the mirror one or more microsteps per line. The ground-truth
// lines are rendered through the true poses with the slit integrated as a box
// (several sub-lines across its width), then noise is added. The dataset
// stores the poses the rig would have recorded: the arm pose with
// millimetre-level error per sweep, and the commanded mirror angles, which
// miss the true ones by a homing offset per sweep and a little jitter per
// line. A trainer that refines poses should recover the true ones.
#pragma once

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

#include "linesplat/dataset.hpp"
#include "linesplat/render_cpu.hpp"
#include "linesplat/scene.hpp"

namespace linesplat {

struct SyntheticOptions {
  // Instrument
  int width = 256;                  // pixels along the slit
  int bands = 46;                   // spectral bands
  double wl_min_nm = 500.0;
  double wl_max_nm = 950.0;
  int slit_samples = 5;             // sub-lines across the slit for the ground truth
  // Scene
  double spacing = 1.0e-3;          // Gaussian spacing on the surfaces (m)
  // Scan
  int sweeps = 8;
  double distance = 0.150;          // objective to the aim point (m), the focus distance
  double scan_half_deg = 12.0;      // half-angle of the view fan; the mirror turns half as much
  double microstep_deg = 1.8 / 32;  // mirror turn per microstep (NEMA 8 at 1/32)
  int microsteps_per_line = 1;
  // Errors in what the rig records
  double head_trans_sigma = 1.0e-3;        // per axis, per sweep (m)
  double head_rot_sigma_deg = 0.3;         // per axis, per sweep
  double mirror_offset_sigma_deg = 0.05;   // homing error, per sweep
  double mirror_jitter_sigma_deg = 0.005;  // microstep error, per line
  // Sensor noise on the line values (reflectance units)
  double read_noise = 0.003;
  double shot_noise = 0.0004;       // variance per unit of signal
  uint64_t seed = 1;
};

// Renders feature lines for a batch of cameras: the CPU reference by default,
// or the CUDA rasterizer.
using LineRenderFn = std::function<LineImage(const GaussianScene&, const std::vector<LineCamera>&)>;
LineRenderFn cpu_renderer();

struct SyntheticData {
  GaussianScene scene;                     // ground truth, identity basis
  Dataset dataset;                         // recorded poses and noisy lines
  std::vector<Pose> true_head_pose;        // [S]
  std::vector<double> true_mirror_angle;   // [L]
  std::vector<float> clean_lines;          // [L, W, B] before noise
};

GaussianScene make_synthetic_scene(const SyntheticOptions& opt, const std::vector<double>& wavelengths_nm);
SyntheticData make_synthetic_dataset(const SyntheticOptions& opt, const LineRenderFn& render = cpu_renderer());

// Writes the dataset to dir, the ground truth to dir/gt/ (scene/, poses,
// mirror angles, clean lines) and PNG previews to dir/preview/.
void save_synthetic(const SyntheticData& data, const std::string& dir, const LineRenderFn& render = cpu_renderer());

// The camera -> world pose of the overview image in the previews.
Pose overview_camera();

}  // namespace linesplat
