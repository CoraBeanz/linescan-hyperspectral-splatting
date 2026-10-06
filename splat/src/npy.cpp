#include "linesplat/npy.hpp"

#include <cstring>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace linesplat {

namespace {

size_t dtype_size(const std::string& d) {
  if (d == "<f4" || d == "<i4" || d == "<u4") return 4;
  if (d == "<f8" || d == "<i8" || d == "<u8") return 8;
  if (d == "<u2" || d == "<i2" || d == "<f2") return 2;
  if (d == "|u1" || d == "|i1" || d == "|b1") return 1;
  throw std::runtime_error("unsupported npy dtype " + d);
}

// Pulls the value of 'key' out of the header dict, e.g. "'descr': '<f4'".
std::string dict_value(const std::string& h, const std::string& key) {
  const size_t k = h.find("'" + key + "'");
  if (k == std::string::npos) throw std::runtime_error("npy header has no " + key);
  size_t i = h.find(':', k) + 1;
  while (i < h.size() && h[i] == ' ') ++i;
  if (h[i] == '(') return h.substr(i, h.find(')', i) - i + 1);
  if (h[i] == '\'') return h.substr(i + 1, h.find('\'', i + 1) - i - 1);
  size_t j = i;
  while (j < h.size() && h[j] != ',' && h[j] != '}') ++j;
  return h.substr(i, j - i);
}

std::string normalize_dtype(std::string d) {
  if (d == "=f4" || d == "f4") d = "<f4";
  if (d == "=f8" || d == "f8") d = "<f8";
  if (d == "=i4" || d == "i4") d = "<i4";
  if (d == "<u1" || d == "u1") d = "|u1";
  return d;
}

template <typename Dst>
std::vector<Dst> convert(const NpyArray& a, const std::string& path) {
  const size_t n = a.count();
  std::vector<Dst> out(n);
  const char* p = a.bytes.data();
  if (a.dtype == "<f4") {
    for (size_t i = 0; i < n; ++i) { float v; std::memcpy(&v, p + 4 * i, 4); out[i] = Dst(v); }
  } else if (a.dtype == "<f8") {
    for (size_t i = 0; i < n; ++i) { double v; std::memcpy(&v, p + 8 * i, 8); out[i] = Dst(v); }
  } else if (a.dtype == "<i4") {
    for (size_t i = 0; i < n; ++i) { int32_t v; std::memcpy(&v, p + 4 * i, 4); out[i] = Dst(v); }
  } else if (a.dtype == "<i8") {
    for (size_t i = 0; i < n; ++i) { int64_t v; std::memcpy(&v, p + 8 * i, 8); out[i] = Dst(v); }
  } else {
    throw std::runtime_error(path + ": cannot convert dtype " + a.dtype);
  }
  return out;
}

}  // namespace

size_t NpyArray::count() const {
  size_t n = 1;
  for (size_t s : shape) n *= s;
  return n;
}

NpyArray npy_read(const std::string& path) {
  std::ifstream f(path, std::ios::binary);
  if (!f) throw std::runtime_error("cannot open " + path);
  char magic[8];
  f.read(magic, 8);
  if (!f || std::memcmp(magic, "\x93NUMPY", 6) != 0) throw std::runtime_error(path + " is not an .npy file");
  const int major = static_cast<unsigned char>(magic[6]);
  uint32_t hlen = 0;
  if (major == 1) {
    unsigned char b[2];
    f.read(reinterpret_cast<char*>(b), 2);
    hlen = b[0] | (b[1] << 8);
  } else {
    unsigned char b[4];
    f.read(reinterpret_cast<char*>(b), 4);
    hlen = b[0] | (b[1] << 8) | (b[2] << 16) | (uint32_t(b[3]) << 24);
  }
  std::string header(hlen, ' ');
  f.read(&header[0], hlen);
  NpyArray a;
  a.dtype = normalize_dtype(dict_value(header, "descr"));
  if (dict_value(header, "fortran_order").find("True") != std::string::npos)
    throw std::runtime_error(path + ": Fortran-order arrays are not supported");
  const std::string shp = dict_value(header, "shape");
  std::string num;
  for (char c : shp) {
    if (c >= '0' && c <= '9') {
      num += c;
    } else if (!num.empty()) {
      a.shape.push_back(std::stoull(num));
      num.clear();
    }
  }
  a.bytes.resize(a.count() * dtype_size(a.dtype));
  f.read(a.bytes.data(), std::streamsize(a.bytes.size()));
  if (!f) throw std::runtime_error(path + ": file is shorter than its header says");
  return a;
}

void npy_write(const std::string& path, const std::string& dtype, const std::vector<size_t>& shape,
               const void* data) {
  std::ostringstream h;
  h << "{'descr': '" << dtype << "', 'fortran_order': False, 'shape': (";
  size_t n = 1;
  for (size_t i = 0; i < shape.size(); ++i) {
    h << shape[i] << (shape.size() == 1 ? "," : (i + 1 < shape.size() ? ", " : ""));
    n *= shape[i];
  }
  h << "), }";
  std::string header = h.str();
  // Pad with spaces so the data starts on a 64-byte boundary, ending in '\n'.
  const size_t total = 10 + header.size() + 1;
  header.append((64 - total % 64) % 64, ' ');
  header += '\n';
  std::ofstream f(path, std::ios::binary);
  if (!f) throw std::runtime_error("cannot write " + path);
  const unsigned char pre[10] = {0x93, 'N', 'U', 'M', 'P', 'Y', 1, 0,
                                 static_cast<unsigned char>(header.size() & 0xff),
                                 static_cast<unsigned char>(header.size() >> 8)};
  f.write(reinterpret_cast<const char*>(pre), 10);
  f.write(header.data(), std::streamsize(header.size()));
  f.write(static_cast<const char*>(data), std::streamsize(n * dtype_size(dtype)));
  if (!f) throw std::runtime_error("failed writing " + path);
}

void npy_save(const std::string& path, const std::vector<float>& v, const std::vector<size_t>& shape) {
  npy_write(path, "<f4", shape, v.data());
}
void npy_save(const std::string& path, const std::vector<double>& v, const std::vector<size_t>& shape) {
  npy_write(path, "<f8", shape, v.data());
}
void npy_save(const std::string& path, const std::vector<int32_t>& v, const std::vector<size_t>& shape) {
  npy_write(path, "<i4", shape, v.data());
}

std::vector<float> npy_load_f32(const std::string& path, std::vector<size_t>* shape) {
  NpyArray a = npy_read(path);
  if (shape) *shape = a.shape;
  return convert<float>(a, path);
}
std::vector<double> npy_load_f64(const std::string& path, std::vector<size_t>* shape) {
  NpyArray a = npy_read(path);
  if (shape) *shape = a.shape;
  return convert<double>(a, path);
}
std::vector<int32_t> npy_load_i32(const std::string& path, std::vector<size_t>* shape) {
  NpyArray a = npy_read(path);
  if (shape) *shape = a.shape;
  if (a.dtype != "<i4" && a.dtype != "<i8") throw std::runtime_error(path + ": expected an integer array");
  return convert<int32_t>(a, path);
}

}  // namespace linesplat
