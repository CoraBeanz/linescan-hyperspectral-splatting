// The web viewer's file, .lsplat (format "linesplat-view", version 1): a JSON
// header, then little-endian arrays, each on a 4-byte boundary. The layout is
// in viewer/README.md. splat_export writes one from a scene directory, and
// splat_materials reads one, adds material maps and writes it back.
#pragma once

#include <cstdint>
#include <cstring>
#include <nlohmann/json.hpp>
#include <stdexcept>
#include <string>
#include <vector>

#include "linesplat/scan_model.hpp"
#include "linesplat/scene.hpp"

namespace linesplat {

// The numpy-style name of each array type the format uses.
template <typename T> const char* lsplat_type();
template <> inline const char* lsplat_type<float>() { return "float32"; }
template <> inline const char* lsplat_type<uint16_t>() { return "uint16"; }
template <> inline const char* lsplat_type<int16_t>() { return "int16"; }
template <> inline const char* lsplat_type<uint8_t>() { return "uint8"; }

// One array: its name, type, shape and bytes. `extra` holds any other keys
// its entry in the header has (log_scales keeps its "range" there).
struct LsplatBlock {
  std::string name, type;
  std::vector<size_t> shape;
  std::vector<char> bytes;
  nlohmann::json extra = nlohmann::json::object();

  size_t count() const {
    size_t n = 1;
    for (size_t s : shape) n *= s;
    return n;
  }
  // The values, copied out as T, which must be the block's type.
  template <typename T>
  std::vector<T> values() const {
    if (type != lsplat_type<T>()) throw std::runtime_error("block " + name + " is " + type + ", not " + lsplat_type<T>());
    std::vector<T> v(bytes.size() / sizeof(T));
    if (!v.empty()) std::memcpy(v.data(), bytes.data(), v.size() * sizeof(T));
    return v;
  }
};

template <typename T>
LsplatBlock make_block(const std::string& name, const std::vector<T>& v, const std::vector<size_t>& shape) {
  LsplatBlock b;
  b.name = name;
  b.type = lsplat_type<T>();
  b.shape = shape;
  if (b.count() != v.size()) throw std::runtime_error("block " + name + ": shape and values disagree");
  b.bytes.resize(v.size() * sizeof(T));
  if (!v.empty()) std::memcpy(b.bytes.data(), v.data(), b.bytes.size());
  return b;
}

struct LsplatFile {
  nlohmann::json header = nlohmann::json::object();  // everything but "blocks"
  std::vector<LsplatBlock> blocks;                    // in file order

  // nullptr if there's no such block.
  const LsplatBlock* find(const std::string& name) const;
  const LsplatBlock& at(const std::string& name) const;
  // Adds a block at the end, or replaces the one of the same name in place.
  void set(LsplatBlock b);
  void remove(const std::string& name);
};

void write_lsplat(const std::string& path, const LsplatFile& f);
// Reads a file splat_export (or splat_materials) wrote. Gzipped files have to
// be unzipped first.
LsplatFile read_lsplat(const std::string& path);

// The scene's arrays as the file stores them: positions float32, log scales
// uint16 over their range, unit quaternions int16 with w >= 0, opacity uint8,
// and each Gaussian's features uint8 between its own minimum and maximum, so a
// flat spectrum keeps its detail. That's 35 + K bytes a Gaussian.
std::vector<LsplatBlock> pack_scene(const GaussianScene& s);
// Writes the scene's arrays into the file, with its basis (as "identity" when
// it is one) and background in the header.
void put_scene(LsplatFile& f, const GaussianScene& s);
// The scene a file holds, decoded the way the viewer decodes it.
GaussianScene unpack_scene(const LsplatFile& f);

// The camera -> world pose of a view as the header stores one ({eye, target,
// up, fov_deg} with the field of view across the shorter side), and its focal
// length in pixels for an image of width x height.
Pose lsplat_view_pose(const nlohmann::json& view, int width, int height, double* f_px);

}  // namespace linesplat
