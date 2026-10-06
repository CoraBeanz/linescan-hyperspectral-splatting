#include <cstdio>
#include <cstring>
#include <fstream>

#include "fixtures.hpp"
#include "linesplat/dataset.hpp"
#include "linesplat/npy.hpp"
#include "linesplat/png.hpp"
#include "linesplat/util.hpp"
#include "test.hpp"

using namespace linesplat;

namespace {
std::string tmp_dir(const char* name) {
  const std::string d = join_path(LINESPLAT_TEST_TMP, name);
  make_dirs(d);
  return d;
}
std::vector<char> read_bytes(const std::string& path) {
  std::ifstream f(path, std::ios::binary);
  return std::vector<char>((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
}
}  // namespace

TEST(npy_round_trip) {
  const std::string d = tmp_dir("npy");
  std::vector<float> a(24);
  for (size_t i = 0; i < a.size(); ++i) a[i] = 0.5f * i - 3.0f;
  npy_save(join_path(d, "a.npy"), a, {2, 3, 4});
  std::vector<size_t> shape;
  CHECK(npy_load_f32(join_path(d, "a.npy"), &shape) == a);
  CHECK((shape == std::vector<size_t>{2, 3, 4}));
  // Loads convert between types.
  const std::vector<double> ad = npy_load_f64(join_path(d, "a.npy"));
  CHECK(ad[5] == double(a[5]));

  const std::vector<int32_t> b = {1, -2, 3};
  npy_save(join_path(d, "b.npy"), b, {3});
  CHECK(npy_load_i32(join_path(d, "b.npy"), &shape) == b);
  CHECK((shape == std::vector<size_t>{3}));

  // The header is what numpy writes: magic, version 1.0, and the data on a
  // 64-byte boundary.
  const std::vector<char> raw = read_bytes(join_path(d, "a.npy"));
  CHECK(std::memcmp(raw.data(), "\x93NUMPY\x01\x00", 8) == 0);
  const size_t hlen = size_t((unsigned char)raw[8]) | (size_t((unsigned char)raw[9]) << 8);
  CHECK((10 + hlen) % 64 == 0);
  CHECK(raw[10 + hlen - 1] == '\n');
  CHECK(raw.size() == 10 + hlen + 24 * 4);
}

TEST(scene_round_trip) {
  const GaussianScene s = lsfix::random_scene(50, 3, 2);
  const std::string d = tmp_dir("scene");
  s.save(d);
  const GaussianScene t = GaussianScene::load(d);
  CHECK(t.num_features == 3);
  CHECK(t.means == s.means);
  CHECK(t.log_scales == s.log_scales);
  CHECK(t.rotations == s.rotations);
  CHECK(t.opacity_logits == s.opacity_logits);
  CHECK(t.features == s.features);
  CHECK(t.basis == s.basis);
  CHECK(t.background == s.background);
}

TEST(dataset_round_trip) {
  Dataset d;
  d.intrinsics = intrinsics_from_optics(16);
  d.head = cad_head_model();
  d.wavelengths_nm = {500, 600, 700};
  Pose p;
  p.R = rotation_exp(Vec3d{0.1, 0.2, 0.3});
  p.t = Vec3d{0.01, 0.02, 0.03};
  d.sweep_head_pose = {Pose(), p};
  d.line_sweep = {0, 0, 1};
  d.line_mirror_angle = {-0.01, 0.0, 0.01};
  d.lines.resize(3 * 16 * 3);
  for (size_t i = 0; i < d.lines.size(); ++i) d.lines[i] = float(i) * 0.01f;
  d.metadata_json = "{\"note\": \"test\"}";
  const std::string dir = tmp_dir("dataset");
  d.save(dir);
  const Dataset e = Dataset::load(dir);
  CHECK(e.num_lines() == 3);
  CHECK(e.num_sweeps() == 2);
  CHECK(e.lines == d.lines);
  CHECK(e.line_sweep == d.line_sweep);
  CHECK(e.line_mirror_angle == d.line_mirror_angle);
  CHECK(e.wavelengths_nm == d.wavelengths_nm);
  CHECK(e.metadata_json.find("test") != std::string::npos);
  CHECK_NEAR(e.intrinsics.f, d.intrinsics.f, 1e-12);
  CHECK_NEAR(e.head.mirror.face_offset, d.head.mirror.face_offset, 1e-15);
  // The cameras come out the same.
  for (int l = 0; l < 3; ++l) {
    const LineCamera a = d.camera(l), b = e.camera(l);
    for (int i = 0; i < 9; ++i) CHECK_NEAR(a.R[i], b.R[i], 1e-7);
    for (int i = 0; i < 3; ++i) CHECK_NEAR(a.t[i], b.t[i], 1e-7);
  }
}

TEST(png_is_well_formed) {
  // Write 3 x 2 pixels, then parse the file back: chunk CRCs, the stored
  // deflate block, and the pixels.
  const std::vector<uint8_t> rgb = {255, 0, 0, 0, 255, 0, 0, 0, 255, 10, 20, 30, 40, 50, 60, 70, 80, 90};
  const std::string path = join_path(tmp_dir("png"), "t.png");
  write_png_rgb(path, 3, 2, rgb);
  const std::vector<char> raw = read_bytes(path);
  CHECK(std::memcmp(raw.data(), "\x89PNG\r\n\x1a\n", 8) == 0);
  size_t pos = 8;
  std::vector<uint8_t> idat;
  auto u32 = [&](size_t o) {
    return (uint32_t((unsigned char)raw[o]) << 24) | (uint32_t((unsigned char)raw[o + 1]) << 16) |
           (uint32_t((unsigned char)raw[o + 2]) << 8) | uint32_t((unsigned char)raw[o + 3]);
  };
  // CRC-32 to check each chunk.
  auto crc = [](const char* p, size_t n) {
    uint32_t c = 0xffffffffu;
    for (size_t i = 0; i < n; ++i) {
      c ^= (unsigned char)p[i];
      for (int k = 0; k < 8; ++k) c = (c & 1) ? 0xEDB88320u ^ (c >> 1) : c >> 1;
    }
    return ~c;
  };
  int chunks = 0;
  while (pos < raw.size()) {
    const uint32_t len = u32(pos);
    const std::string type(raw.data() + pos + 4, 4);
    CHECK(crc(raw.data() + pos + 4, len + 4) == u32(pos + 8 + len));
    if (type == "IDAT") idat.insert(idat.end(), raw.begin() + long(pos + 8), raw.begin() + long(pos + 8 + len));
    if (type == "IHDR") CHECK(u32(pos + 8) == 3 && u32(pos + 12) == 2);
    pos += 12 + len;
    ++chunks;
  }
  CHECK(chunks == 3);
  // zlib header, one final stored block of 2 * (1 + 9) bytes, adler32.
  CHECK(idat[0] == 0x78 && idat[2] == 1);
  const size_t n = idat[3] | (idat[4] << 8);
  CHECK(n == 20);
  CHECK(idat[7] == 0 && std::memcmp(&idat[8], rgb.data(), 9) == 0);
  CHECK(idat[17] == 0 && std::memcmp(&idat[18], rgb.data() + 9, 9) == 0);
}
