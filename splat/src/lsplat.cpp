#include "linesplat/lsplat.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iterator>

namespace linesplat {

namespace {

constexpr char kMagic[4] = {'L', 'S', 'P', 'V'};
constexpr uint32_t kVersion = 1;

template <typename T>
T quantize(double x, double lo, double hi, double levels) {
  const double t = hi > lo ? (x - lo) / (hi - lo) : 0.0;
  return T(std::lround(std::min(std::max(t, 0.0), 1.0) * levels));
}

}  // namespace

const LsplatBlock* LsplatFile::find(const std::string& name) const {
  for (const LsplatBlock& b : blocks)
    if (b.name == name) return &b;
  return nullptr;
}

const LsplatBlock& LsplatFile::at(const std::string& name) const {
  const LsplatBlock* b = find(name);
  if (!b) throw std::runtime_error("the file has no \"" + name + "\" array");
  return *b;
}

void LsplatFile::set(LsplatBlock b) {
  for (LsplatBlock& old : blocks)
    if (old.name == b.name) {
      old = std::move(b);
      return;
    }
  blocks.push_back(std::move(b));
}

void LsplatFile::remove(const std::string& name) {
  blocks.erase(std::remove_if(blocks.begin(), blocks.end(), [&](const LsplatBlock& b) { return b.name == name; }),
               blocks.end());
}

void write_lsplat(const std::string& path, const LsplatFile& f) {
  std::vector<char> bin;
  nlohmann::json entries = nlohmann::json::object();
  for (const LsplatBlock& b : f.blocks) {
    while (bin.size() % 4) bin.push_back(0);
    nlohmann::json e = b.extra;
    e["type"] = b.type;
    e["shape"] = b.shape;
    e["offset"] = bin.size();
    e["bytes"] = b.bytes.size();
    entries[b.name] = e;
    bin.insert(bin.end(), b.bytes.begin(), b.bytes.end());
  }
  while (bin.size() % 4) bin.push_back(0);
  nlohmann::json header = f.header;
  header["blocks"] = entries;

  std::string text = header.dump();
  while (text.size() % 4) text.push_back(' ');
  std::ofstream out(path, std::ios::binary);
  if (!out) throw std::runtime_error("cannot write " + path);
  const uint32_t words[2] = {kVersion, uint32_t(text.size())};
  out.write(kMagic, 4);
  out.write(reinterpret_cast<const char*>(words), sizeof words);  // little-endian, like the .npy files
  out.write(text.data(), std::streamsize(text.size()));
  out.write(bin.data(), std::streamsize(bin.size()));
  if (!out) throw std::runtime_error("failed writing " + path);
}

LsplatFile read_lsplat(const std::string& path) {
  std::ifstream in(path, std::ios::binary);
  if (!in) throw std::runtime_error("cannot read " + path);
  const std::vector<char> bytes((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
  if (bytes.size() >= 2 && uint8_t(bytes[0]) == 0x1f && uint8_t(bytes[1]) == 0x8b)
    throw std::runtime_error(path + " is gzipped; gunzip it first");
  if (bytes.size() < 12 || !std::equal(kMagic, kMagic + 4, bytes.begin()))
    throw std::runtime_error(path + " is not a splat_export file (it should start with LSPV)");
  uint32_t words[2];
  std::memcpy(words, bytes.data() + 4, sizeof words);
  if (words[0] != kVersion) throw std::runtime_error(path + " is version " + std::to_string(words[0]) + ", not 1");
  const size_t start = 12 + size_t(words[1]);
  if (start > bytes.size()) throw std::runtime_error(path + " is cut short");

  LsplatFile f;
  f.header = nlohmann::json::parse(bytes.begin() + 12, bytes.begin() + std::ptrdiff_t(start));
  if (f.header.value("format", std::string()) != "linesplat-view")
    throw std::runtime_error(path + " is not a linesplat-view file");
  const nlohmann::json entries = f.header.value("blocks", nlohmann::json::object());
  f.header.erase("blocks");
  for (auto it = entries.begin(); it != entries.end(); ++it) {
    LsplatBlock b;
    b.name = it.key();
    b.extra = it.value();
    b.type = b.extra.at("type").get<std::string>();
    b.shape = b.extra.at("shape").get<std::vector<size_t>>();
    const size_t offset = b.extra.at("offset").get<size_t>(), n = b.extra.at("bytes").get<size_t>();
    if (start + offset + n > bytes.size()) throw std::runtime_error(path + ": the \"" + b.name + "\" array is cut short");
    b.bytes.assign(bytes.begin() + std::ptrdiff_t(start + offset), bytes.begin() + std::ptrdiff_t(start + offset + n));
    for (const char* key : {"type", "shape", "offset", "bytes"}) b.extra.erase(key);
    f.blocks.push_back(std::move(b));
  }
  // Keep them in the order they sit in the file.
  std::stable_sort(f.blocks.begin(), f.blocks.end(), [&](const LsplatBlock& a, const LsplatBlock& b) {
    return entries.at(a.name).at("offset").get<size_t>() < entries.at(b.name).at("offset").get<size_t>();
  });
  return f;
}

std::vector<LsplatBlock> pack_scene(const GaussianScene& s) {
  const int n = s.size(), k = s.num_features;
  const auto ls = std::minmax_element(s.log_scales.begin(), s.log_scales.end());
  const float lo = n ? *ls.first : 0.0f, hi = n ? *ls.second : 0.0f;
  std::vector<uint16_t> log_scales;
  for (float v : s.log_scales) log_scales.push_back(quantize<uint16_t>(v, lo, hi, 65535));

  std::vector<int16_t> rotations;
  std::vector<uint8_t> opacity, features;
  std::vector<float> feature_range;
  for (int i = 0; i < n; ++i) {
    const float* q = &s.rotations[4 * size_t(i)];
    const double len = std::sqrt(double(q[0]) * q[0] + double(q[1]) * q[1] + double(q[2]) * q[2] + double(q[3]) * q[3]);
    double u[4] = {1.0, 0.0, 0.0, 0.0};
    if (len > 0.0)
      for (int c = 0; c < 4; ++c) u[c] = q[c] / len;
    // q and -q are the same rotation; keeping w >= 0 makes the file canonical.
    const double sign = u[0] < 0.0 ? -1.0 : 1.0;
    for (int c = 0; c < 4; ++c) rotations.push_back(int16_t(std::lround(sign * u[c] * 32767.0)));

    const double o = 1.0 / (1.0 + std::exp(-double(s.opacity_logits[size_t(i)])));
    opacity.push_back(quantize<uint8_t>(o, 0.0, 1.0, 255));

    const float* f = &s.features[size_t(i) * k];
    const auto mm = std::minmax_element(f, f + k);
    const double off = *mm.first, range = double(*mm.second) - off;
    feature_range.push_back(float(off));
    feature_range.push_back(float(range));
    for (int c = 0; c < k; ++c) features.push_back(quantize<uint8_t>(f[c], off, off + range, 255));
  }

  const size_t N = size_t(n), K = size_t(k);
  std::vector<LsplatBlock> out;
  out.push_back(make_block("means", s.means, {N, 3}));
  out.push_back(make_block("log_scales", log_scales, {N, 3}));
  out.back().extra["range"] = {lo, hi};
  out.push_back(make_block("rotations", rotations, {N, 4}));
  out.push_back(make_block("opacity", opacity, {N}));
  out.push_back(make_block("feature_range", feature_range, {N, 2}));
  out.push_back(make_block("features", features, {N, K}));
  return out;
}

void put_scene(LsplatFile& f, const GaussianScene& s) {
  s.validate();
  for (LsplatBlock& b : pack_scene(s)) f.set(std::move(b));
  const int B = s.num_bands(), K = s.num_features;
  f.header["count"] = s.size();
  f.header["num_features"] = K;
  f.header["num_bands"] = B;
  // An identity basis (features are the bands) is the common case, so the
  // file says so rather than storing B x B numbers.
  bool identity = K == B;
  for (int b = 0; b < B && identity; ++b)
    for (int k = 0; k < K; ++k)
      if (s.basis[size_t(b) * K + k] != (b == k ? 1.0f : 0.0f)) identity = false;
  if (identity) {
    f.header["basis"] = "identity";
  } else {
    nlohmann::json rows = nlohmann::json::array();
    for (int b = 0; b < B; ++b)
      rows.push_back(std::vector<float>(s.basis.begin() + size_t(b) * K, s.basis.begin() + size_t(b + 1) * K));
    f.header["basis"] = rows;
  }
  f.header["background"] = s.background;
}

GaussianScene unpack_scene(const LsplatFile& f) {
  GaussianScene s;
  const int n = f.header.at("count").get<int>();
  const int k = s.num_features = f.header.at("num_features").get<int>();
  const int B = f.header.at("num_bands").get<int>();
  s.means = f.at("means").values<float>();

  const LsplatBlock& ls = f.at("log_scales");
  const auto range = ls.extra.at("range").get<std::vector<float>>();
  const float ls_step = (range.at(1) - range.at(0)) / 65535.0f;
  for (uint16_t q : ls.values<uint16_t>()) s.log_scales.push_back(range[0] + float(q) * ls_step);
  for (int16_t q : f.at("rotations").values<int16_t>()) s.rotations.push_back(float(q) / 32767.0f);
  for (uint8_t q : f.at("opacity").values<uint8_t>()) {
    // Opacity 0 and 1 would need infinite logits; +-40 gives the same floats.
    const double o = q / 255.0;
    s.opacity_logits.push_back(q == 0 ? -40.0f : q == 255 ? 40.0f : float(std::log(o / (1.0 - o))));
  }
  const std::vector<float> fr = f.at("feature_range").values<float>();
  const std::vector<uint8_t> fq = f.at("features").values<uint8_t>();
  for (int i = 0; i < n; ++i) {
    const float off = fr.at(2 * size_t(i)), scale = fr.at(2 * size_t(i) + 1);
    for (int c = 0; c < k; ++c) s.features.push_back(off + scale * float(fq.at(size_t(i) * k + c)) / 255.0f);
  }

  const nlohmann::json& basis = f.header.at("basis");
  if (basis.is_string()) {
    if (basis.get<std::string>() != "identity" || B != k) throw std::runtime_error("unknown basis in the file");
    s.set_identity_basis(B);
  } else {
    for (const auto& row : basis)
      for (float v : row.get<std::vector<float>>()) s.basis.push_back(v);
  }
  s.background = f.header.at("background").get<std::vector<float>>();
  if (s.size() != n) throw std::runtime_error("the file's arrays disagree with its count");
  s.validate();
  return s;
}

Pose lsplat_view_pose(const nlohmann::json& view, int width, int height, double* f_px) {
  auto vec = [&](const char* key) {
    const auto a = view.at(key).get<std::vector<double>>();
    return Vec3d{a.at(0), a.at(1), a.at(2)};
  };
  *f_px = 0.5 * std::min(width, height) / std::tan(0.5 * view.at("fov_deg").get<double>() * kPi / 180.0);
  return look_at(vec("eye"), vec("target"), vec("up"));
}

}  // namespace linesplat
