// Reading and writing NumPy .npy files, so Python tools (numpy.load /
// numpy.save) can read and write everything the C++ side produces.
// Supports little-endian float32, float64, int32, uint8 and uint16 arrays in
// C order, which covers every array in a dataset or scene.
#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace linesplat {

struct NpyArray {
  std::string dtype;            // numpy descr, e.g. "<f4"
  std::vector<size_t> shape;
  std::vector<char> bytes;
  size_t count() const;
};

NpyArray npy_read(const std::string& path);
void npy_write(const std::string& path, const std::string& dtype, const std::vector<size_t>& shape,
               const void* data);

// Typed helpers. The loaders convert between float32 and float64 (and from
// int32 to the floating types) as needed. `shape` is optional.
void npy_save(const std::string& path, const std::vector<float>& v, const std::vector<size_t>& shape);
void npy_save(const std::string& path, const std::vector<double>& v, const std::vector<size_t>& shape);
void npy_save(const std::string& path, const std::vector<int32_t>& v, const std::vector<size_t>& shape);
std::vector<float> npy_load_f32(const std::string& path, std::vector<size_t>* shape = nullptr);
std::vector<double> npy_load_f64(const std::string& path, std::vector<size_t>* shape = nullptr);
std::vector<int32_t> npy_load_i32(const std::string& path, std::vector<size_t>* shape = nullptr);

}  // namespace linesplat
