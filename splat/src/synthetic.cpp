#include "linesplat/synthetic.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <nlohmann/json.hpp>
#include <stdexcept>

#include "linesplat/npy.hpp"
#include "linesplat/preview.hpp"
#include "linesplat/rng.hpp"
#include "linesplat/spectra.hpp"
#include "linesplat/util.hpp"

namespace linesplat {

namespace {

constexpr double kDeg = kPi / 180.0;

// 5 x 7 glyphs for the hidden word, top row first.
const char* const kGlyphN[7] = {"X...X", "XX..X", "X.X.X", "X..XX", "X...X", "X...X", "X...X"};
const char* const kGlyphI[7] = {".XXX.", "..X..", "..X..", "..X..", "..X..", "..X..", ".XXX."};
const char* const kGlyphR[7] = {"XXXX.", "X...X", "X...X", "XXXX.", "X.X..", "X..X.", "X...X"};

// Board layout, in mm from the board's centre (the world origin).
constexpr double kBoardHalfX = 40.0, kBoardHalfY = 32.0;
constexpr double kTableHalfX = 70.0, kTableHalfY = 60.0;

Material board_material(double x, double y) {
  // Checker border with 4 mm squares, standing in for the AprilTag board.
  if (std::fabs(x) > 32.0 || std::fabs(y) > 24.0) {
    const int cx = int(std::floor(x / 4.0)), cy = int(std::floor(y / 4.0));
    return ((cx + cy) & 1) ? Material::kCarbonBlack : Material::kWhitePaper;
  }
  // Six paint patches along the top edge.
  if (y >= 13.0 && y <= 21.0 && x >= -30.0 && x <= 29.0) {
    const double fx = x + 30.0;
    const int i = std::min(int(fx / 10.0), 5);
    if (fx - 10.0 * i <= 9.0) {
      static const Material kPatches[6] = {Material::kRedPaint,   Material::kOrangePlastic, Material::kYellowPaint,
                                           Material::kGreenPaint, Material::kBluePaint,     Material::kGray18};
      return kPatches[i];
    }
  }
  // "NIR" in carbon black on IR-transparent black dye: one black panel to
  // the eye, a word past 740 nm.
  if (x >= -30.0 && x <= -2.0 && y >= -21.0 && y <= -7.0) {
    const double px = 1.4;                            // mm per glyph pixel
    const double x0 = -16.0 - 0.5 * 17 * px;          // three 5-wide glyphs with 1-pixel gaps
    const double y_top = -14.0 + 3.5 * px;
    const int col = int(std::floor((x - x0) / px));
    const int row = int(std::floor((y_top - y) / px));
    if (row >= 0 && row < 7 && col >= 0 && col < 17 && col % 6 != 5) {
      const char* const* glyph = col < 6 ? kGlyphN : (col < 12 ? kGlyphI : kGlyphR);
      if (glyph[row][col % 6] == 'X') return Material::kCarbonBlack;
    }
    return Material::kIrBlackDye;
  }
  return Material::kWhitePaper;
}

// Quaternion (w, x, y, z) turning +z onto the unit vector n.
void quat_z_to(const Vec3d& n, float q[4]) {
  if (n.z < -0.999999) {
    q[0] = 0.0f; q[1] = 1.0f; q[2] = 0.0f; q[3] = 0.0f;
    return;
  }
  const double w = 1.0 + n.z, x = -n.y, y = n.x;
  const double s = 1.0 / std::sqrt(w * w + x * x + y * y);
  q[0] = float(w * s); q[1] = float(x * s); q[2] = float(y * s); q[3] = 0.0f;
}

class SceneBuilder {
 public:
  SceneBuilder(const std::vector<double>& wl, double spacing, uint64_t seed) : spacing_(spacing), rng_(seed) {
    scene_.num_features = int(wl.size());
    for (int m = 0; m < int(Material::kCount); ++m) spectra_.push_back(material_spectrum(Material(m), wl));
  }

  // A flat Gaussian ("surfel") lying on a surface with normal n. `size` scales
  // its footprint relative to the spacing.
  void surfel(const Vec3d& p, const Vec3d& n, Material m, double size = 1.0) {
    const float mean[3] = {float(p.x), float(p.y), float(p.z)};
    const float sc[3] = {float(0.65 * spacing_ * size), float(0.65 * spacing_ * size), float(0.08 * spacing_)};
    float q[4];
    quat_z_to(normalized(n), q);
    // A few percent of brightness variation per Gaussian, like real texture.
    const float tint = float(1.0 + 0.04 * (2.0 * rng_.uniform() - 1.0));
    std::vector<float> f = spectra_[size_t(m)];
    for (float& v : f) v *= tint;
    scene_.add(mean, sc, q, 0.98f, f.data());
  }

  GaussianScene finish() {
    scene_.set_identity_basis(scene_.num_features);
    scene_.background.assign(size_t(scene_.num_features), 0.0f);
    return scene_;
  }

  double spacing() const { return spacing_; }

 private:
  double spacing_;
  Rng rng_;
  GaussianScene scene_;
  std::vector<std::vector<float>> spectra_;
};

// Grid of n points across [lo, hi] at cell centres.
int grid_count(double lo, double hi, double step) { return std::max(1, int(std::lround((hi - lo) / step))); }
double grid_at(double lo, double hi, int n, int i) { return lo + (hi - lo) * (i + 0.5) / n; }

// The arm poses: one straight down, the rest on two rings around the board,
// alternating the slit between across and along the ring.
std::vector<Pose> plan_sweeps(const SyntheticOptions& opt, const HeadModel& head) {
  const Pose v0_inv = virtual_camera_in_head(head, 0.0).inverse();
  std::vector<Pose> poses;
  for (int k = 0; k < opt.sweeps; ++k) {
    double elev = 89.0, azim = 0.0;
    Vec3d target{0.0, 0.0, 0.003};
    if (k > 0) {
      elev = (k % 2) ? 62.0 : 48.0;
      azim = 15.0 + 360.0 * (k - 1) / std::max(1, opt.sweeps - 1);
      target = Vec3d{0.012 * std::cos(azim * kDeg), 0.009 * std::sin(azim * kDeg), 0.003};
    }
    const double ce = std::cos(elev * kDeg), se = std::sin(elev * kDeg);
    const double ca = std::cos(azim * kDeg), sa = std::sin(azim * kDeg);
    const Vec3d toward_eye{ce * ca, ce * sa, se};
    const Vec3d z = -toward_eye;
    const Vec3d hint = (k % 2 == 0) ? Vec3d{-sa, ca, 0.0} : Vec3d{ca, sa, 0.0};
    const Vec3d x = normalized(hint - dot(hint, z) * z);
    Pose cam;
    cam.R = mat3_from_cols(x, cross(z, x), z);
    cam.t = target + opt.distance * toward_eye;
    poses.push_back(cam * v0_inv);  // head_in_world = camera_in_world * head_from_camera
  }
  return poses;
}

nlohmann::json options_json(const SyntheticOptions& o) {
  return {{"width", o.width},
          {"bands", o.bands},
          {"wl_min_nm", o.wl_min_nm},
          {"wl_max_nm", o.wl_max_nm},
          {"slit_samples", o.slit_samples},
          {"spacing_m", o.spacing},
          {"sweeps", o.sweeps},
          {"distance_m", o.distance},
          {"scan_half_deg", o.scan_half_deg},
          {"microstep_deg", o.microstep_deg},
          {"microsteps_per_line", o.microsteps_per_line},
          {"head_trans_sigma_m", o.head_trans_sigma},
          {"head_rot_sigma_deg", o.head_rot_sigma_deg},
          {"mirror_offset_sigma_deg", o.mirror_offset_sigma_deg},
          {"mirror_jitter_sigma_deg", o.mirror_jitter_sigma_deg},
          {"read_noise", o.read_noise},
          {"shot_noise", o.shot_noise},
          {"seed", o.seed}};
}

}  // namespace

LineRenderFn cpu_renderer() {
  return [](const GaussianScene& s, const std::vector<LineCamera>& c) { return render_lines_cpu<float>(s, c); };
}

Pose overview_camera() {
  return look_at(Vec3d{0.105, -0.135, 0.125}, Vec3d{0.0, -0.002, 0.004}, Vec3d{0.0, 0.0, 1.0});
}

GaussianScene make_synthetic_scene(const SyntheticOptions& opt, const std::vector<double>& wl) {
  SceneBuilder b(wl, opt.spacing, opt.seed);
  const double s_mm = opt.spacing * 1e3;
  const Vec3d up{0.0, 0.0, 1.0};

  // The board, z = 0.
  const int nx = grid_count(-kBoardHalfX, kBoardHalfX, s_mm), ny = grid_count(-kBoardHalfY, kBoardHalfY, s_mm);
  for (int iy = 0; iy < ny; ++iy)
    for (int ix = 0; ix < nx; ++ix) {
      const double x = grid_at(-kBoardHalfX, kBoardHalfX, nx, ix), y = grid_at(-kBoardHalfY, kBoardHalfY, ny, iy);
      b.surfel(Vec3d{x * 1e-3, y * 1e-3, 0.0}, up, board_material(x, y));
    }

  // The table around it, with coarser Gaussians, half a millimetre lower.
  const double ts = 2.0 * s_mm;
  const int tx = grid_count(-kTableHalfX, kTableHalfX, ts), ty = grid_count(-kTableHalfY, kTableHalfY, ts);
  for (int iy = 0; iy < ty; ++iy)
    for (int ix = 0; ix < tx; ++ix) {
      const double x = grid_at(-kTableHalfX, kTableHalfX, tx, ix), y = grid_at(-kTableHalfY, kTableHalfY, ty, iy);
      if (std::fabs(x) < kBoardHalfX - 1.0 && std::fabs(y) < kBoardHalfY - 1.0) continue;
      b.surfel(Vec3d{x * 1e-3, y * 1e-3, -0.0005}, up, Material::kWood, 2.0);
    }

  // A leaf-green ball resting on the board (Fibonacci points on a sphere).
  const Vec3d c{0.014, -0.012, 0.009};
  const double r = 0.009;
  const int n = int(std::lround(4.0 * kPi * r * r / (opt.spacing * opt.spacing)));
  const double golden = kPi * (3.0 - std::sqrt(5.0));
  for (int i = 0; i < n; ++i) {
    const double z = 1.0 - 2.0 * (i + 0.5) / n;
    const double rad = std::sqrt(std::max(0.0, 1.0 - z * z));
    const Vec3d nrm{rad * std::cos(golden * i), rad * std::sin(golden * i), z};
    const Vec3d p = c + r * nrm;
    if (p.z < 0.0003) continue;
    b.surfel(p, nrm, Material::kLeaf);
  }

  // An orange box, 12 x 10 x 10 mm, open at the bottom.
  const double x0 = 10.0, x1 = 22.0, y0 = -1.0, y1 = 9.0, h = 10.0;
  auto face = [&](int axis, double fixed, double a0, double a1, double b0, double b1, const Vec3d& nrm) {
    const int na = grid_count(a0, a1, s_mm), nb = grid_count(b0, b1, s_mm);
    for (int i = 0; i < na; ++i)
      for (int j = 0; j < nb; ++j) {
        const double a = grid_at(a0, a1, na, i), bb = grid_at(b0, b1, nb, j);
        Vec3d p{0, 0, 0};
        if (axis == 0) p = Vec3d{fixed, a, bb};
        if (axis == 1) p = Vec3d{a, fixed, bb};
        if (axis == 2) p = Vec3d{a, bb, fixed};
        b.surfel(1e-3 * p, nrm, Material::kOrangePlastic);
      }
  };
  face(2, h, x0, x1, y0, y1, Vec3d{0, 0, 1});
  face(0, x0, y0, y1, 0.0, h, Vec3d{-1, 0, 0});
  face(0, x1, y0, y1, 0.0, h, Vec3d{1, 0, 0});
  face(1, y0, x0, x1, 0.0, h, Vec3d{0, -1, 0});
  face(1, y1, x0, x1, 0.0, h, Vec3d{0, 1, 0});
  return b.finish();
}

SyntheticData make_synthetic_dataset(const SyntheticOptions& opt, const LineRenderFn& render) {
  if (opt.sweeps < 1 || opt.width < 2 || opt.bands < 1 || opt.microsteps_per_line < 1)
    throw std::runtime_error("synthetic: bad options");
  SyntheticData out;
  const std::vector<double> wl = wavelength_grid(opt.wl_min_nm, opt.wl_max_nm, opt.bands);
  out.scene = make_synthetic_scene(opt, wl);

  Dataset& d = out.dataset;
  d.intrinsics = intrinsics_from_optics(opt.width);
  d.head = cad_head_model();
  d.wavelengths_nm = wl;
  d.values = "reflectance";

  Rng rng(opt.seed * 7919u + 17u);
  out.true_head_pose = plan_sweeps(opt, d.head);
  for (const Pose& p : out.true_head_pose) {
    const Vec3d rho{opt.head_trans_sigma * rng.normal(), opt.head_trans_sigma * rng.normal(),
                    opt.head_trans_sigma * rng.normal()};
    const double sr = opt.head_rot_sigma_deg * kDeg;
    const Vec3d phi{sr * rng.normal(), sr * rng.normal(), sr * rng.normal()};
    d.sweep_head_pose.push_back(perturb(p, rho, phi));
  }

  const double half = 0.5 * opt.scan_half_deg * kDeg;  // the mirror turns half the view angle
  const double step = opt.microsteps_per_line * opt.microstep_deg * kDeg;
  const int per_sweep = int(std::floor(2.0 * half / step + 1e-9)) + 1;
  for (int s = 0; s < opt.sweeps; ++s) {
    const double offset = opt.mirror_offset_sigma_deg * kDeg * rng.normal();
    for (int j = 0; j < per_sweep; ++j) {
      const double commanded = (j - 0.5 * (per_sweep - 1)) * step;
      d.line_sweep.push_back(s);
      d.line_mirror_angle.push_back(commanded);
      out.true_mirror_angle.push_back(commanded + offset + opt.mirror_jitter_sigma_deg * kDeg * rng.normal());
    }
  }

  // Render the clean lines through the true poses. The slit integrates a box
  // across its width, so average several sub-lines spread across it, each
  // blurred only by the optics and its share of the slit.
  const int L = d.num_lines(), W = opt.width, B = opt.bands;
  const int n = std::max(1, opt.slit_samples);
  const double slit = slit_width_px(W), blur = optics_blur_px(W);
  LineIntrinsics sub = d.intrinsics;
  if (n > 1) sub.sigma_v = std::sqrt(blur * blur + (slit / n) * (slit / n) / 12.0);
  out.clean_lines.assign(size_t(L) * W * B, 0.0f);
  const int chunk = 128;
  for (int l0 = 0; l0 < L; l0 += chunk) {
    const int l1 = std::min(L, l0 + chunk);
    std::vector<LineCamera> cams;
    for (int l = l0; l < l1; ++l)
      for (int j = 0; j < n; ++j) {
        sub.v_slit = n == 1 ? 0.0 : (j + 0.5) * slit / n - 0.5 * slit;
        cams.push_back(make_line_camera(d.head, out.true_head_pose[size_t(d.line_sweep[size_t(l)])],
                                        out.true_mirror_angle[size_t(l)], sub));
      }
    const LineImage bands = features_to_bands(out.scene, render(out.scene, cams));
    for (int l = l0; l < l1; ++l)
      for (int j = 0; j < n; ++j) {
        const float* src = &bands.values[size_t((l - l0) * n + j) * W * B];
        float* dst = &out.clean_lines[size_t(l) * W * B];
        for (int i = 0; i < W * B; ++i) dst[i] += src[i] / float(n);
      }
  }

  d.lines = out.clean_lines;
  for (float& v : d.lines) {
    const double sd = std::sqrt(opt.read_noise * opt.read_noise + opt.shot_noise * std::max(0.0f, v));
    v = float(v + sd * rng.normal());
  }

  nlohmann::json meta;
  meta["synthetic"] = options_json(opt);
  meta["ground_truth"] = {{"scene", "gt/scene"},
                          {"sweep_head_pose", "gt/sweep_head_pose.npy"},
                          {"line_mirror_angle", "gt/line_mirror_angle.npy"},
                          {"clean_lines", "gt/lines_clean.npy"}};
  d.metadata_json = meta.dump();
  return out;
}

void save_synthetic(const SyntheticData& data, const std::string& dir, const LineRenderFn& render) {
  const Dataset& d = data.dataset;
  d.save(dir);
  const std::string gt = join_path(dir, "gt");
  data.scene.save(join_path(gt, "scene"));
  std::vector<double> poses;
  for (const Pose& p : data.true_head_pose) {
    double v[7];
    p.to_array(v);
    poses.insert(poses.end(), v, v + 7);
  }
  npy_save(join_path(gt, "sweep_head_pose.npy"), poses, {data.true_head_pose.size(), 7});
  npy_save(join_path(gt, "line_mirror_angle.npy"), data.true_mirror_angle, {data.true_mirror_angle.size()});
  npy_save(join_path(gt, "lines_clean.npy"), data.clean_lines,
           {size_t(d.num_lines()), size_t(d.width()), size_t(d.num_bands())});

  // Previews: each sweep stacked line by line (the pushbroom image), and the
  // scene from an oblique pinhole camera.
  const std::string pv = join_path(dir, "preview");
  make_dirs(pv);
  const size_t line_size = size_t(d.width()) * d.num_bands();
  for (int s = 0; s < d.num_sweeps(); ++s) {
    std::vector<float> img;
    for (int l = 0; l < d.num_lines(); ++l)
      if (d.line_sweep[size_t(l)] == s) img.insert(img.end(), d.line(l), d.line(l) + line_size);
    const int rows = int(img.size() / line_size);
    char name[64];
    snprintf(name, sizeof(name), "sweep_%02d_rgb.png", s);
    write_spectral_png(join_path(pv, name), img.data(), rows, d.width(), d.wavelengths_nm, PreviewMode::kTrueColor);
    snprintf(name, sizeof(name), "sweep_%02d_cir.png", s);
    write_spectral_png(join_path(pv, name), img.data(), rows, d.width(), d.wavelengths_nm,
                       PreviewMode::kColorInfrared);
  }
  const int pw = 480, ph = 360;
  const LineImage view = features_to_bands(data.scene, render(data.scene, pinhole_rows(overview_camera(), pw, ph, 640.0)));
  write_spectral_png(join_path(pv, "scene_rgb.png"), view.values.data(), ph, pw, d.wavelengths_nm,
                     PreviewMode::kTrueColor);
  write_spectral_png(join_path(pv, "scene_cir.png"), view.values.data(), ph, pw, d.wavelengths_nm,
                     PreviewMode::kColorInfrared);
}

}  // namespace linesplat
