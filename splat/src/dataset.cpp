#include "linesplat/dataset.hpp"

#include <nlohmann/json.hpp>
#include <stdexcept>

#include "linesplat/npy.hpp"
#include "linesplat/util.hpp"

namespace linesplat {

using nlohmann::json;

namespace {

json vec_json(const Vec3d& v) { return json::array({v.x, v.y, v.z}); }

Vec3d vec_from(const json& j) {
  if (!j.is_array() || j.size() != 3) throw std::runtime_error("dataset.json: expected a 3-vector");
  return Vec3d{j[0].get<double>(), j[1].get<double>(), j[2].get<double>()};
}

json pose_json(const Pose& p) {
  double v[7];
  p.to_array(v);
  return json(std::vector<double>(v, v + 7));
}

Pose pose_from(const json& j) {
  const std::vector<double> v = j.get<std::vector<double>>();
  if (v.size() != 7) throw std::runtime_error("dataset.json: a pose is [tx, ty, tz, qw, qx, qy, qz]");
  return Pose::from_array(v.data());
}

}  // namespace

LineCamera Dataset::camera(int l) const {
  return make_line_camera(head, sweep_head_pose[size_t(line_sweep[size_t(l)])], line_mirror_angle[size_t(l)],
                          intrinsics);
}

std::vector<LineCamera> Dataset::cameras() const {
  std::vector<LineCamera> c(static_cast<size_t>(num_lines()));
  for (int l = 0; l < num_lines(); ++l) c[size_t(l)] = camera(l);
  return c;
}

void Dataset::validate() const {
  const size_t L = size_t(num_lines());
  if (line_mirror_angle.size() != L) throw std::runtime_error("dataset: line_mirror_angle needs one value per line");
  if (lines.size() != L * size_t(width()) * size_t(num_bands()))
    throw std::runtime_error("dataset: lines must be [num_lines, width, num_bands]");
  for (int s : line_sweep)
    if (s < 0 || s >= num_sweeps()) throw std::runtime_error("dataset: line_sweep refers to a missing sweep");
  if (width() <= 0 || num_bands() <= 0) throw std::runtime_error("dataset: empty width or wavelength list");
}

void Dataset::save(const std::string& dir) const {
  validate();
  make_dirs(dir);
  json j;
  j["format"] = "linesplat-dataset";
  j["version"] = 1;
  j["units"] = "metres, radians, nanometres";
  j["num_lines"] = num_lines();
  j["num_sweeps"] = num_sweeps();
  j["width"] = width();
  j["num_bands"] = num_bands();
  j["values"] = values;
  j["wavelengths_nm"] = wavelengths_nm;
  j["intrinsics"] = {{"width", intrinsics.width},       {"f_px", intrinsics.f},
                     {"cu_px", intrinsics.cu},          {"v_slit_px", intrinsics.v_slit},
                     {"sigma_u_px", intrinsics.sigma_u}, {"sigma_v_px", intrinsics.sigma_v},
                     {"near_m", intrinsics.near_z}};
  j["head"] = {{"camera_in_head", pose_json(head.camera_in_head)},
               {"mirror",
                {{"axis_point", vec_json(head.mirror.axis_point)},
                 {"axis_dir", vec_json(head.mirror.axis_dir)},
                 {"normal_at_zero", vec_json(head.mirror.normal_at_zero)},
                 {"face_offset", head.mirror.face_offset}}}};
  j["files"] = {{"lines", "lines.npy"},
                {"line_sweep", "line_sweep.npy"},
                {"line_mirror_angle", "line_mirror_angle.npy"},
                {"sweep_head_pose", "sweep_head_pose.npy"}};
  j["metadata"] = metadata_json.empty() ? json::object() : json::parse(metadata_json);
  write_text_file(join_path(dir, "dataset.json"), j.dump(2) + "\n");

  const size_t L = size_t(num_lines()), S = size_t(num_sweeps());
  npy_save(join_path(dir, "lines.npy"), lines, {L, size_t(width()), size_t(num_bands())});
  npy_save(join_path(dir, "line_sweep.npy"), std::vector<int32_t>(line_sweep.begin(), line_sweep.end()), {L});
  npy_save(join_path(dir, "line_mirror_angle.npy"), line_mirror_angle, {L});
  std::vector<double> poses(S * 7);
  for (size_t s = 0; s < S; ++s) sweep_head_pose[s].to_array(&poses[7 * s]);
  npy_save(join_path(dir, "sweep_head_pose.npy"), poses, {S, 7});
}

Dataset Dataset::load(const std::string& dir) {
  const json j = json::parse(read_text_file(join_path(dir, "dataset.json")));
  if (j.value("format", "") != "linesplat-dataset") throw std::runtime_error(dir + " is not a linesplat dataset");
  Dataset d;
  d.values = j.value("values", "reflectance");
  d.wavelengths_nm = j.at("wavelengths_nm").get<std::vector<double>>();
  const json& in = j.at("intrinsics");
  d.intrinsics.width = in.at("width").get<int>();
  d.intrinsics.f = in.at("f_px").get<double>();
  d.intrinsics.cu = in.at("cu_px").get<double>();
  d.intrinsics.v_slit = in.value("v_slit_px", 0.0);
  d.intrinsics.sigma_u = in.at("sigma_u_px").get<double>();
  d.intrinsics.sigma_v = in.at("sigma_v_px").get<double>();
  d.intrinsics.near_z = in.value("near_m", 0.01);
  const json& h = j.at("head");
  d.head.camera_in_head = pose_from(h.at("camera_in_head"));
  const json& m = h.at("mirror");
  d.head.mirror.axis_point = vec_from(m.at("axis_point"));
  d.head.mirror.axis_dir = normalized(vec_from(m.at("axis_dir")));
  d.head.mirror.normal_at_zero = normalized(vec_from(m.at("normal_at_zero")));
  d.head.mirror.face_offset = m.at("face_offset").get<double>();
  if (j.contains("metadata") && !j["metadata"].empty()) d.metadata_json = j["metadata"].dump();

  const json files = j.value("files", json::object());
  auto file = [&](const char* key) { return join_path(dir, files.value(key, std::string(key) + ".npy")); };
  std::vector<size_t> shape;
  d.lines = npy_load_f32(file("lines"), &shape);
  if (shape.size() != 3 || int(shape[1]) != d.intrinsics.width || shape[2] != d.wavelengths_nm.size())
    throw std::runtime_error("lines.npy must be [num_lines, width, num_bands]");
  const std::vector<int32_t> ls = npy_load_i32(file("line_sweep"));
  d.line_sweep.assign(ls.begin(), ls.end());
  d.line_mirror_angle = npy_load_f64(file("line_mirror_angle"));
  const std::vector<double> poses = npy_load_f64(file("sweep_head_pose"), &shape);
  if (shape.size() != 2 || shape[1] != 7) throw std::runtime_error("sweep_head_pose.npy must be [num_sweeps, 7]");
  for (size_t s = 0; s < shape[0]; ++s) d.sweep_head_pose.push_back(Pose::from_array(&poses[7 * s]));
  d.validate();
  return d;
}

}  // namespace linesplat
