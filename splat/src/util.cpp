#include "linesplat/util.hpp"

#include <sys/stat.h>

#include <cerrno>
#include <fstream>
#include <sstream>
#include <stdexcept>

#ifdef _WIN32
#include <direct.h>
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#include <psapi.h>
#else
#include <sys/resource.h>
#endif

namespace linesplat {

namespace {
bool is_dir(const std::string& p) {
  struct stat st;
  return stat(p.c_str(), &st) == 0 && (st.st_mode & S_IFDIR);
}
int make_one_dir(const std::string& p) {
#ifdef _WIN32
  return _mkdir(p.c_str());
#else
  return mkdir(p.c_str(), 0755);
#endif
}
}  // namespace

void make_dirs(const std::string& path) {
  if (path.empty() || is_dir(path)) return;
  // Create each prefix that ends at a separator, then the path itself.
  for (size_t i = 1; i <= path.size(); ++i) {
    if (i < path.size() && path[i] != '/' && path[i] != '\\') continue;
    const std::string prefix = path.substr(0, i);
    if (prefix.empty() || prefix.back() == ':' || is_dir(prefix)) continue;
    if (make_one_dir(prefix) != 0 && errno != EEXIST)
      throw std::runtime_error("cannot create directory " + prefix);
  }
}

bool file_exists(const std::string& path) {
  struct stat st;
  return stat(path.c_str(), &st) == 0;
}

std::string join_path(const std::string& a, const std::string& b) {
  if (a.empty()) return b;
  if (a.back() == '/' || a.back() == '\\') return a + b;
  return a + "/" + b;
}

std::string read_text_file(const std::string& path) {
  std::ifstream f(path, std::ios::binary);
  if (!f) throw std::runtime_error("cannot open " + path);
  std::ostringstream ss;
  ss << f.rdbuf();
  return ss.str();
}

void write_text_file(const std::string& path, const std::string& text) {
  std::ofstream f(path, std::ios::binary);
  if (!f) throw std::runtime_error("cannot write " + path);
  f << text;
}

double peak_rss_mb() {
#ifdef _WIN32
  PROCESS_MEMORY_COUNTERS pmc;
  if (K32GetProcessMemoryInfo(GetCurrentProcess(), &pmc, sizeof pmc)) return double(pmc.PeakWorkingSetSize) / 1048576.0;
  return -1.0;
#else
  struct rusage ru;
  if (getrusage(RUSAGE_SELF, &ru) != 0) return -1.0;
#ifdef __APPLE__
  return double(ru.ru_maxrss) / 1048576.0;  // bytes
#else
  return double(ru.ru_maxrss) / 1024.0;  // kilobytes
#endif
#endif
}

}  // namespace linesplat
