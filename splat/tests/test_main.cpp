#include <cstdio>
#include <cstring>
#include <exception>

#include "test.hpp"

namespace lstest {
std::vector<Case>& registry() {
  static std::vector<Case> r;
  return r;
}
std::string& skip_reason() {
  static std::string s;
  return s;
}
}  // namespace lstest

int main(int argc, char** argv) {
  const char* filter = argc > 1 ? argv[1] : "";
  int run = 0, failed = 0, skipped = 0;
  for (const auto& c : lstest::registry()) {
    if (*filter && !std::strstr(c.name, filter)) continue;
    ++run;
    lstest::skip_reason().clear();
    try {
      c.fn();
      if (!lstest::skip_reason().empty()) {
        ++skipped;
        std::printf("[skip] %s: %s\n", c.name, lstest::skip_reason().c_str());
      } else {
        std::printf("[ ok ] %s\n", c.name);
      }
    } catch (const lstest::Failure& f) {
      ++failed;
      std::printf("[FAIL] %s\n       %s\n", c.name, f.what.c_str());
    } catch (const std::exception& e) {
      ++failed;
      std::printf("[FAIL] %s\n       exception: %s\n", c.name, e.what());
    }
    std::fflush(stdout);
  }
  std::printf("\n%d tests, %d failed, %d skipped\n", run, failed, skipped);
  return failed ? 1 : 0;
}
