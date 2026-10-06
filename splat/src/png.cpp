#include "linesplat/png.hpp"

#include <algorithm>
#include <fstream>
#include <stdexcept>

namespace linesplat {

namespace {

uint32_t crc32(const uint8_t* data, size_t n, uint32_t crc = 0) {
  static uint32_t table[256];
  static bool init = false;
  if (!init) {
    for (uint32_t i = 0; i < 256; ++i) {
      uint32_t c = i;
      for (int k = 0; k < 8; ++k) c = (c & 1) ? 0xEDB88320u ^ (c >> 1) : c >> 1;
      table[i] = c;
    }
    init = true;
  }
  crc = ~crc;
  for (size_t i = 0; i < n; ++i) crc = table[(crc ^ data[i]) & 0xff] ^ (crc >> 8);
  return ~crc;
}

void put_u32(std::vector<uint8_t>& v, uint32_t x) {
  v.push_back(uint8_t(x >> 24));
  v.push_back(uint8_t(x >> 16));
  v.push_back(uint8_t(x >> 8));
  v.push_back(uint8_t(x));
}

void chunk(std::ofstream& f, const char type[4], const std::vector<uint8_t>& payload) {
  std::vector<uint8_t> buf;
  put_u32(buf, uint32_t(payload.size()));
  buf.insert(buf.end(), type, type + 4);
  buf.insert(buf.end(), payload.begin(), payload.end());
  put_u32(buf, crc32(buf.data() + 4, buf.size() - 4));
  f.write(reinterpret_cast<const char*>(buf.data()), std::streamsize(buf.size()));
}

}  // namespace

void write_png_rgb(const std::string& path, int width, int height, const std::vector<uint8_t>& rgb) {
  if (width <= 0 || height <= 0 || rgb.size() != size_t(width) * height * 3)
    throw std::runtime_error("write_png_rgb: bad image size");
  // Raw scanlines, each prefixed with filter type 0.
  std::vector<uint8_t> raw;
  raw.reserve(size_t(height) * (width * 3 + 1));
  for (int y = 0; y < height; ++y) {
    raw.push_back(0);
    raw.insert(raw.end(), rgb.begin() + size_t(y) * width * 3, rgb.begin() + size_t(y + 1) * width * 3);
  }
  // zlib stream of stored deflate blocks.
  std::vector<uint8_t> z = {0x78, 0x01};
  uint32_t a = 1, b = 0;
  for (uint8_t c : raw) {
    a = (a + c) % 65521;
    b = (b + a) % 65521;
  }
  for (size_t off = 0; off < raw.size() || off == 0; off += 65535) {
    const size_t n = std::min<size_t>(65535, raw.size() - off);
    z.push_back(off + n >= raw.size() ? 1 : 0);
    z.push_back(uint8_t(n & 0xff));
    z.push_back(uint8_t(n >> 8));
    z.push_back(uint8_t(~n & 0xff));
    z.push_back(uint8_t((~n >> 8) & 0xff));
    z.insert(z.end(), raw.begin() + off, raw.begin() + off + n);
    if (off + n >= raw.size()) break;
  }
  put_u32(z, (b << 16) | a);

  std::ofstream f(path, std::ios::binary);
  if (!f) throw std::runtime_error("cannot write " + path);
  const uint8_t sig[8] = {0x89, 'P', 'N', 'G', '\r', '\n', 0x1a, '\n'};
  f.write(reinterpret_cast<const char*>(sig), 8);
  std::vector<uint8_t> ihdr;
  put_u32(ihdr, uint32_t(width));
  put_u32(ihdr, uint32_t(height));
  ihdr.insert(ihdr.end(), {8, 2, 0, 0, 0});  // 8-bit RGB, deflate, no filter, no interlace
  chunk(f, "IHDR", ihdr);
  chunk(f, "IDAT", z);
  chunk(f, "IEND", {});
  if (!f) throw std::runtime_error("failed writing " + path);
}

}  // namespace linesplat
