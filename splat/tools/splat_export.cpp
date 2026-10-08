// splat_export: packs a splat into one compact file for the web viewer.
//
//   splat_export SCENE OUT.lsplat [--dataset DIR] [--poses POSES.npy] [--title TEXT]
//                [--reference OUT.json]
//
// SCENE is a scene directory: splat_train's OUT_DIR/scene, or a synthetic
// dataset's gt/scene. --dataset gives the wavelengths and the scan, so the
// viewer can draw each sweep's fan of lines; the fans use the dataset's
// recorded head poses, or --poses (splat_train's refined
// OUT_DIR/sweep_head_pose.npy, or gt/sweep_head_pose.npy for the truth).
// Without a dataset the bands are spread evenly over 500-950 nm.
//
// The file is a JSON header and then little-endian arrays (the layout is in
// viewer/README.md). Positions stay float32; log scales become uint16,
// rotations int16 and opacity uint8; each Gaussian's features become uint8
// between that Gaussian's own minimum and maximum, so a flat spectrum keeps
// its detail. That's 35 + K bytes per Gaussian, against 4 * (11 + K) as .npy.
//
// It then renders the default view from the scene as given and as the viewer
// will decode it, and prints the difference. --reference writes spectra that
// the C++ renderer gives at a grid of pixels, which the viewer's tests check
// their own rendering against.
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <limits>
#include <nlohmann/json.hpp>
#include <stdexcept>
#include <string>
#include <vector>

#include "linesplat/dataset.hpp"
#include "linesplat/lsplat.hpp"
#include "linesplat/npy.hpp"
#include "linesplat/preview.hpp"
#include "linesplat/render_cpu.hpp"
#include "linesplat/scene.hpp"
#include "linesplat/spectra.hpp"
#include "linesplat/synthetic.hpp"
#include "linesplat/util.hpp"

using namespace linesplat;
using nlohmann::json;

namespace {

constexpr uint32_t kVersion = 1;
constexpr double kDeg = kPi / 180.0;

void usage() {
  std::fprintf(stderr,
               "usage: splat_export SCENE OUT.lsplat [--dataset DIR] [--poses POSES.npy] [--title TEXT]\n"
               "                    [--reference OUT.json]\n");
  std::exit(2);
}

// x rounded to `decimals` places. Dividing by a power of ten gives the double
// nearest the decimal, so the JSON prints it short.
double round_to(double x, int decimals) {
  const double scale = std::pow(10.0, decimals);
  return std::round(x * scale) / scale;
}

// Points to the micron.
json vec_json(const Vec3d& v) { return json::array({round_to(v.x, 6), round_to(v.y, 6), round_to(v.z, 6)}); }

// Where a ray from `from` along `dir` meets the board plane z = 0, or a point
// 0.3 m along it if it never gets there.
Vec3d hit_board(const Vec3d& from, const Vec3d& dir) {
  const double t = dir.z < -1e-9 ? -from.z / dir.z : -1.0;
  if (t > 0.0 && t < 1.0) return Vec3d{from.x + t * dir.x, from.y + t * dir.y, 0.0};
  return from + 0.3 * dir;
}

// The default camera: the synthetic previews' overview camera for synthetic
// data, otherwise the same direction pulled back to see most of the scene.
// The pivot for orbiting is where the view's centre meets the board.
json default_view(const GaussianScene& s, bool synthetic) {
  const double fov_short = 2.0 * std::atan(180.0 / 640.0);  // the previews: 480 x 360 px at f = 640 px
  Pose cam = overview_camera();
  if (!synthetic && s.size() > 0) {
    // The box between the 5th and 95th percentiles of the centres, so a few
    // stray Gaussians don't shrink the view.
    double mid[3], half[3];
    for (int a = 0; a < 3; ++a) {
      std::vector<double> c;
      for (int i = 0; i < s.size(); ++i) c.push_back(s.means[3 * size_t(i) + a]);
      std::sort(c.begin(), c.end());
      const double lo = c[c.size() / 20], hi = c[c.size() - 1 - c.size() / 20];
      mid[a] = 0.5 * (lo + hi);
      half[a] = 0.5 * (hi - lo);
    }
    const double radius = std::max(norm(Vec3d{half[0], half[1], half[2]}), 1e-3);
    const Vec3d centre{mid[0], mid[1], mid[2]};
    const Vec3d dir = normalized(cam.t - Vec3d{0.0, -0.002, 0.004});
    cam = look_at(centre + (1.15 * radius / std::sin(0.5 * fov_short)) * dir, centre, Vec3d{0.0, 0.0, 1.0});
  }
  const Vec3d forward = cam.R.col(2);
  const Vec3d target = hit_board(cam.t, forward);
  return {{"eye", vec_json(cam.t)},
          {"target", vec_json(target)},
          {"up", json::array({0.0, 0.0, 1.0})},
          {"fov_deg", fov_short / kDeg},
          {"fov_axis", "short"}};
}

// Each sweep's fan: the virtual camera's centre and where the first and the
// last line's slit ends meet the board.
json sweep_fans(const Dataset& d, const std::vector<Pose>& poses) {
  json out = json::array();
  for (int s = 0; s < d.num_sweeps(); ++s) {
    double lo = std::numeric_limits<double>::max(), hi = -lo;
    int lines = 0;
    for (int l = 0; l < d.num_lines(); ++l)
      if (d.line_sweep[size_t(l)] == s) {
        lo = std::min(lo, d.line_mirror_angle[size_t(l)]);
        hi = std::max(hi, d.line_mirror_angle[size_t(l)]);
        ++lines;
      }
    if (lines == 0) continue;
    Vec3d apex{0, 0, 0};
    std::vector<Vec3d> ends;
    for (double angle : {lo, hi}) {
      const LineCameraT<double> c = make_line_camera_d(d.head, poses[size_t(s)], angle, d.intrinsics);
      const Mat3d R{{c.R[0], c.R[1], c.R[2], c.R[3], c.R[4], c.R[5], c.R[6], c.R[7], c.R[8]}};
      const Mat3d Rt = transpose(R);
      const Vec3d centre = -(Rt * Vec3d{c.t[0], c.t[1], c.t[2]});
      apex = apex + 0.5 * centre;
      for (double u : {0.0, double(c.width)})
        ends.push_back(hit_board(centre, normalized(Rt * Vec3d{(u - c.cu) / c.f, c.v_slit / c.f, 1.0})));
    }
    // First line left to right, then the last line back, so the outline doesn't cross itself.
    out.push_back({{"apex", vec_json(apex)},
                   {"corners", json::array({vec_json(ends[0]), vec_json(ends[1]), vec_json(ends[3]), vec_json(ends[2])})},
                   {"lines", lines}});
  }
  return out;
}

LineImage render_view(const GaussianScene& s, const json& view, int width, int height) {
  double f = 0.0;
  const Pose cam = lsplat_view_pose(view, width, height, &f);
  return features_to_bands(s, render_lines_cpu<float>(s, pinhole_rows(cam, width, height, f)));
}

// Spectra rendered by the C++ renderer at a grid of pixels of two cameras,
// with the color weights, for the viewer's tests.
json reference(const GaussianScene& s, const json& view, const std::vector<double>& wl) {
  json cams = json::array();
  const Vec3d target = [&] {
    const auto t = view.at("target").get<std::vector<double>>();
    return Vec3d{t[0], t[1], t[2]};
  }();
  // A grid of cols x rows pixels over the part of the image between the
  // fractions x0..x1 across and y0..y1 down.
  struct Cam {
    json view;
    int width, height, cols, rows;
    double x0, x1, y0, y1;
  };
  // The default view at the previews' size, where the scene sits in the
  // middle, and a closer one from the other side.
  const json close = {{"eye", vec_json(target + Vec3d{-0.045, 0.06, 0.075})},
                      {"target", vec_json(target)},
                      {"up", json::array({0.0, 0.0, 1.0})},
                      {"fov_deg", 38.0},
                      {"fov_axis", "short"}};
  for (const Cam& c : {Cam{view, 480, 360, 8, 6, 0.2, 0.8, 0.3, 0.85}, Cam{close, 320, 240, 8, 6, 0.0, 1.0, 0.0, 1.0}}) {
    double f = 0.0;
    const Pose pose = lsplat_view_pose(c.view, c.width, c.height, &f);
    const std::vector<LineCamera> all_rows = pinhole_rows(pose, c.width, c.height, f);
    std::vector<int> ys;
    std::vector<LineCamera> rows;
    for (int j = 0; j < c.rows; ++j) {
      ys.push_back(int(c.height * (c.y0 + (c.y1 - c.y0) * (j + 0.5) / c.rows)));
      rows.push_back(all_rows[size_t(ys.back())]);
    }
    const LineImage img = features_to_bands(s, render_lines_cpu<float>(s, rows));
    json pixels = json::array();
    for (int j = 0; j < c.rows; ++j)
      for (int i = 0; i < c.cols; ++i) {
        const int x = int(c.width * (c.x0 + (c.x1 - c.x0) * (i + 0.5) / c.cols));
        const size_t px = size_t(j) * c.width + x;
        json spec = json::array();
        for (int b = 0; b < img.channels; ++b) spec.push_back(round_to(img.values[px * img.channels + b], 4));
        pixels.push_back({{"x", x}, {"y", ys[size_t(j)]}, {"coverage", round_to(1.0 - img.transmittance[px], 4)},
                          {"spectrum", spec}});
      }
    cams.push_back({{"view", c.view}, {"width", c.width}, {"height", c.height}, {"f_px", f}, {"pixels", pixels}});
  }
  auto weights = [&](SpectrumColorFn fn) {
    json w = json::array();
    for (const auto& rgb : color_weights(fn, wl)) w.push_back({rgb[0], rgb[1], rgb[2]});
    return w;
  };
  return {{"wavelengths_nm", wl},
          {"true_color_weights", weights(true_color)},
          {"color_infrared_weights", weights(color_infrared)},
          {"cameras", cams}};
}

double rmse(const std::vector<float>& a, const std::vector<float>& b, double* max_abs) {
  double s = 0.0;
  *max_abs = 0.0;
  for (size_t i = 0; i < a.size(); ++i) {
    const double d = double(a[i]) - double(b[i]);
    s += d * d;
    *max_abs = std::max(*max_abs, std::fabs(d));
  }
  return std::sqrt(s / std::max<size_t>(1, a.size()));
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 3 || argv[1][0] == '-' || argv[2][0] == '-') usage();
  const std::string scene_dir = argv[1], out = argv[2];
  std::string dataset_dir, poses_file, title, reference_file;
  for (int i = 3; i < argc; ++i) {
    const std::string a = argv[i];
    auto next = [&]() -> const char* {
      if (i + 1 >= argc) usage();
      return argv[++i];
    };
    if (a == "--dataset") dataset_dir = next();
    else if (a == "--poses") poses_file = next();
    else if (a == "--title") title = next();
    else if (a == "--reference") reference_file = next();
    else usage();
  }

  try {
    const GaussianScene scene = GaussianScene::load(scene_dir);
    const int B = scene.num_bands();
    json header = {{"format", "linesplat-view"},
                   {"version", kVersion},
                   {"title", title.empty() ? scene_dir : title},
                   {"count", scene.size()},
                   {"num_features", scene.num_features},
                   {"num_bands", B},
                   {"units", "metres"}};

    std::vector<double> wl = wavelength_grid(500.0, 950.0, B);
    bool synthetic = false;
    json source = {{"scene", scene_dir}};
    if (!dataset_dir.empty()) {
      const Dataset d = Dataset::load(dataset_dir);
      if (d.num_bands() != B) throw std::runtime_error("the scene's basis and the dataset disagree on the bands");
      wl = d.wavelengths_nm;
      header["values"] = d.values;
      synthetic = !d.metadata_json.empty() && json::parse(d.metadata_json).contains("synthetic");
      std::vector<Pose> poses = d.sweep_head_pose;
      if (!poses_file.empty()) {
        std::vector<size_t> shape;
        const std::vector<double> v = npy_load_f64(poses_file, &shape);
        if (shape.size() != 2 || shape[1] != 7 || int(shape[0]) != d.num_sweeps())
          throw std::runtime_error(poses_file + " must be [num_sweeps, 7]");
        for (int s = 0; s < d.num_sweeps(); ++s) poses[size_t(s)] = Pose::from_array(&v[7 * size_t(s)]);
        source["poses"] = poses_file;
      }
      header["sweeps"] = sweep_fans(d, poses);
      source["dataset"] = dataset_dir;
      source["lines"] = d.num_lines();
      source["line_width_px"] = d.width();
    } else if (!poses_file.empty()) {
      throw std::runtime_error("--poses needs --dataset for the mirror angles");
    }
    header["wavelengths_nm"] = wl;
    source["synthetic"] = synthetic;
    header["source"] = source;

    header["view"] = default_view(scene, synthetic);

    LsplatFile file;
    file.header = header;
    put_scene(file, scene);
    write_lsplat(out, file);
    const size_t bytes = size_t(std::ifstream(out, std::ios::binary | std::ios::ate).tellg());
    std::printf("wrote     %s: %d Gaussians, %d features, %d bands (%.0f to %.0f nm), %.0f KB\n", out.c_str(),
                scene.size(), scene.num_features, B, wl.front(), wl.back(), bytes / 1024.0);
    if (header.contains("sweeps")) std::printf("sweeps    %d fans\n", int(header["sweeps"].size()));

    const GaussianScene decoded = unpack_scene(file);
    double max_abs = 0.0;
    const double err = rmse(render_view(scene, header["view"], 240, 180).values,
                            render_view(decoded, header["view"], 240, 180).values, &max_abs);
    std::printf("decoded   default view at 240 x 180 differs by %.5f rms, %.4f at most (values in %s)\n", err,
                max_abs, header.value("values", std::string("reflectance")).c_str());

    if (!reference_file.empty()) {
      write_text_file(reference_file, reference(decoded, header["view"], wl).dump() + "\n");
      std::printf("reference %s\n", reference_file.c_str());
    }
  } catch (const std::exception& e) {
    std::fprintf(stderr, "splat_export: %s\n", e.what());
    return 1;
  }
  return 0;
}
