// Small host-side helpers: paths, files, timing.
#pragma once

#include <chrono>
#include <string>

namespace linesplat {

// Creates a directory and its parents (like mkdir -p). Throws on failure.
void make_dirs(const std::string& path);
bool file_exists(const std::string& path);
std::string join_path(const std::string& a, const std::string& b);
std::string read_text_file(const std::string& path);
void write_text_file(const std::string& path, const std::string& text);

class Timer {
 public:
  Timer() : t0_(std::chrono::steady_clock::now()) {}
  double ms() const {
    return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0_).count();
  }

 private:
  std::chrono::steady_clock::time_point t0_;
};

}  // namespace linesplat
