// Training a hyperspectral splat from scan lines, on the CPU.
//
// Each iteration takes a random batch of lines, renders them at the current
// poses, compares them with the measured lines (mean squared error over every
// pixel and band), backpropagates (backward_cpu.hpp) and takes an Adam step on
// the Gaussians.
//
// Once the scene has settled, it also refines one head pose per sweep: each
// line camera's gradient is carried back to a small rigid correction of its
// sweep's head pose. A mirror homing offset moves a sweep's lines the same
// way a small head motion does (to about a micron), so the head correction
// absorbs it and there is no separate mirror parameter. No sweep is held at
// its recorded pose: moving the scene and every pose together changes
// nothing in the data, so the reconstruction stays about where the recorded
// poses put it on average.
//
// Densification follows 3DGS, counted per (line, Gaussian) pair instead of
// per image: Gaussians whose projected centres keep getting large gradients
// are cloned if they are small or split if they are large, and nearly
// transparent ones are pruned.
#pragma once

#include <cstdint>
#include <functional>
#include <vector>

#include "linesplat/backward_cpu.hpp"
#include "linesplat/dataset.hpp"
#include "linesplat/rng.hpp"
#include "linesplat/scene.hpp"

namespace linesplat {

struct TrainOptions {
  int iterations = 3000;
  int batch_lines = 128;
  uint64_t seed = 7;

  // Adam step sizes. Positions decay to 1% by the end, as in 3DGS.
  double lr_means = 2e-5;  // m
  double lr_log_scales = 5e-3;
  double lr_rotations = 1e-3;
  double lr_opacity = 2.5e-2;
  double lr_features = 5e-3;
  double lr_background = 1e-3;

  // Pose refinement: one rigid correction per sweep, made in the camera's
  // frame at the mirror's rest angle, so its translation moves the camera
  // along the slit, across it and along its line of sight, and its rotation
  // turns it about its own centre.
  bool refine_poses = true;
  int pose_from = 100;          // iteration it starts at
  double lr_pose_trans = 2e-5;  // m, decays to 10% by the end
  double lr_pose_rot = 1e-4;    // rad, likewise
  // A sweep that keeps its recorded pose, or -1 for none. Holding one fixed
  // makes the whole scene shift to match that sweep's pose error, which
  // gradient steps on thousands of Gaussians do badly; with none fixed the
  // scene stays where the recorded poses put it on average.
  int fixed_sweep = -1;

  // Densification and pruning.
  int densify_from = 200;
  int densify_until = 2000;
  int densify_every = 100;
  // Mean |dL/d(projected centre)| per (line, Gaussian) pair, for one line's
  // loss and the centre in line widths.
  double densify_grad = 6e-3;
  double split_scale = 1.0e-3;   // larger Gaussians split, smaller ones clone (m)
  double prune_opacity = 0.005;
  double max_scale = 0.01;       // larger ones are pruned (m)
  int max_gaussians = 300000;
};

struct TrainStep {
  int iteration = 0;
  double loss = 0.0;  // mean squared error over the batch's pixels and bands
  int gaussians = 0;
  int densified = 0;  // Gaussians added this step (clones and splits)
  int pruned = 0;     // Gaussians removed this step
  double ms = 0.0;
};

// Renders the dataset's lines `lines` through `cams` (camera i sees line
// lines[i]) and backpropagates the mean squared error over all their pixels
// and bands into *grad and, if not null, *cam_grad. Returns that error.
using BatchBackwardFn =
    std::function<double(const GaussianScene& scene, const std::vector<LineCamera>& cams,
                         const std::vector<int>& lines, SceneGradT<float>* grad,
                         std::vector<CameraGradT<float>>* cam_grad)>;

// The CPU reference (backward_cpu.hpp). `data` must outlive it. On a GPU,
// CudaRasterizer::mse_backward does the same.
BatchBackwardFn cpu_batch_backward(const Dataset& data);

// Gaussians on the plane z = plane_z (the board), where the dataset's pixel
// rays meet it: one per cell of a `spacing` grid, with the mean spectrum of
// the pixels that land in the cell. Features are the bands (identity basis).
GaussianScene init_on_plane(const Dataset& d, double spacing, double plane_z = 0.0);

class Trainer {
 public:
  // `data` must outlive the trainer. Without a backward function it uses
  // cpu_batch_backward(data).
  Trainer(const Dataset& data, GaussianScene init, const TrainOptions& opt, BatchBackwardFn backward = nullptr);

  TrainStep step();
  int iteration() const { return iter_; }
  const GaussianScene& scene() const { return scene_; }
  // Each sweep's head pose with its learned correction.
  std::vector<Pose> head_poses() const;
  // Line cameras for every line of the dataset at the current poses.
  std::vector<LineCameraT<double>> cameras() const;

 private:
  struct Adam {
    std::vector<float> m, v;
    void resize(size_t n) {
      m.assign(n, 0.0f);
      v.assign(n, 0.0f);
    }
  };
  struct PoseState {
    double zeta[6] = {0, 0, 0, 0, 0, 0};  // translation (m), rotation vector (rad)
    double m[6] = {0, 0, 0, 0, 0, 0}, v[6] = {0, 0, 0, 0, 0, 0};
    int steps = 0;
  };

  Pose head_pose(int sweep) const;
  LineCameraT<double> camera(int line, const Pose& head) const;
  void adam_scene(const SceneGradT<float>& g);
  void adam_poses(const std::vector<int>& lines, const std::vector<LineCameraT<double>>& cams,
                  const std::vector<CameraGradT<float>>& cam_grad);
  void densify(TrainStep* st);
  double decayed(double lr, double final_fraction) const;

  const Dataset& data_;
  TrainOptions opt_;
  BatchBackwardFn backward_;
  GaussianScene scene_;
  Pose pose_frame_;         // the frame G the pose corrections are made in
  std::vector<Pose> virt_;  // [L] each line's virtual camera in the head
  std::vector<double> v_sign_;
  Adam a_means_, a_scales_, a_rots_, a_opacity_, a_features_, a_background_;
  std::vector<PoseState> poses_;
  std::vector<double> screen_sum_;
  std::vector<int> pair_count_;
  std::vector<int> order_;
  size_t next_ = 0;
  Rng rng_;
  int iter_ = 0;
};

// Pose refinement maths, exposed for the tests.
//
// A sweep's head pose is its recorded one with a correction zeta = (rho, phi)
// made in a frame G (given in head coordinates):
//   head = recorded * G * [exp(phi), rho] * G^-1
// so rho moves the head along G's axes and phi turns it about G's origin.
Pose corrected_head(const Pose& recorded, const Pose& G, const double zeta[6]);

// Adds one line camera's share of dL/d(eta) to g_eta[6] (v first), for a
// small motion head * exp(eta) of the head pose it was made from. `virt` is
// the line's virtual camera in the head, `cam` the line camera and `g` the
// loss gradient with respect to its world -> camera R and t.
template <typename T>
void add_head_twist_grad(const LineCameraT<double>& cam, const Pose& virt, const CameraGradT<T>& g,
                         double g_eta[6]);

// dL/d(zeta) of corrected_head from dL/d(eta) of the head it returned.
void correction_grad(const Pose& G, const double zeta[6], const double g_eta[6], double g_zeta[6]);

// How far apart two sets of line cameras put the scene, in pixels. For a few
// pixels of every line, the point the `truth` camera sees on the plane
// z = plane_z goes through the `estimate` camera, after the rigid transform
// that best lines the two sets up (moving everything rigidly changes nothing
// in the data, so no trainer can recover it).
struct PoseErrorPx {
  double rms = 0.0, max = 0.0;    // distance from the pixel it should hit
  double rms_along = 0.0;         // along the slit
  double rms_across = 0.0;        // across the slit
  double scale = 1.0;             // of the alignment, with_scale only
  int points = 0;
};
PoseErrorPx line_pose_error_px(const std::vector<LineCameraT<double>>& estimate,
                               const std::vector<LineCameraT<double>>& truth, double plane_z = 0.0,
                               bool with_scale = false);

}  // namespace linesplat
