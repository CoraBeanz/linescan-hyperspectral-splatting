// A pushbroom dataset: scan lines, the poses they were taken from, and the
// instrument model. The synthetic generator writes this format, and the
// capture side (ROS 2 on the Jetson) will write the same thing from the real
// rig, so the trainer can't tell them apart.
//
// On disk it is a directory:
//   dataset.json            instrument, wavelengths, file names, metadata
//   lines.npy               float32 [L, W, B]   one spectrum per pixel per line
//   line_sweep.npy          int32   [L]         which sweep each line belongs to
//   line_mirror_angle.npy   float64 [L]         mirror angle per line (rad)
//   sweep_head_pose.npy     float64 [S, 7]      head -> world per sweep,
//                                               [tx, ty, tz, qw, qx, qy, qz]
// Everything is in metres and radians; wavelengths in nm.
#pragma once

#include <string>
#include <vector>

#include "linesplat/line_camera.hpp"
#include "linesplat/scan_model.hpp"

namespace linesplat {

struct Dataset {
  LineIntrinsics intrinsics;
  HeadModel head;
  std::vector<double> wavelengths_nm;  // [B]
  std::string values = "reflectance";  // what the line values mean

  std::vector<Pose> sweep_head_pose;      // [S] head_in_world, as recorded
  std::vector<int> line_sweep;            // [L]
  std::vector<double> line_mirror_angle;  // [L] as recorded
  std::vector<float> lines;               // [L, W, B]

  // Extra JSON object stored under "metadata" in dataset.json (e.g. how a
  // synthetic dataset was made). Must be a JSON object or empty.
  std::string metadata_json;

  int num_lines() const { return int(line_sweep.size()); }
  int num_sweeps() const { return int(sweep_head_pose.size()); }
  int width() const { return intrinsics.width; }
  int num_bands() const { return int(wavelengths_nm.size()); }
  const float* line(int l) const { return &lines[size_t(l) * width() * num_bands()]; }

  // The line camera for line l, from the recorded poses.
  LineCamera camera(int l) const;
  std::vector<LineCamera> cameras() const;

  // Throws std::runtime_error if the arrays disagree.
  void validate() const;
  void save(const std::string& dir) const;
  static Dataset load(const std::string& dir);
};

}  // namespace linesplat
