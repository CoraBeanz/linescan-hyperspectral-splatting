// A minimal test harness: TEST(name) { CHECK(...); } and a main that runs
// them all, or only those whose names contain argv[1].
#pragma once

#include <cmath>
#include <functional>
#include <sstream>
#include <string>
#include <vector>

namespace lstest {

struct Case {
  const char* name;
  std::function<void()> fn;
};
std::vector<Case>& registry();
struct Register {
  Register(const char* name, std::function<void()> fn) { registry().push_back({name, std::move(fn)}); }
};

struct Failure {
  std::string what;
};
// Set when a test is skipped (e.g. no GPU); printed instead of "ok".
std::string& skip_reason();

}  // namespace lstest

#define LS_CAT2(a, b) a##b
#define LS_CAT(a, b) LS_CAT2(a, b)
#define TEST(name)                                                   \
  static void name();                                                \
  static lstest::Register LS_CAT(reg_, name)(#name, name);           \
  static void name()

#define CHECK(cond)                                                                  \
  do {                                                                               \
    if (!(cond)) {                                                                   \
      std::ostringstream ss_;                                                        \
      ss_ << __FILE__ << ":" << __LINE__ << ": CHECK(" #cond ") failed";             \
      throw lstest::Failure{ss_.str()};                                              \
    }                                                                                \
  } while (0)

#define CHECK_NEAR(a, b, tol)                                                                   \
  do {                                                                                          \
    const double a_ = double(a), b_ = double(b), t_ = double(tol);                              \
    if (!(std::fabs(a_ - b_) <= t_)) {                                                          \
      std::ostringstream ss_;                                                                   \
      ss_ << __FILE__ << ":" << __LINE__ << ": " #a " = " << a_ << ", " #b " = " << b_          \
          << ", diff " << std::fabs(a_ - b_) << " > " << t_;                                    \
      throw lstest::Failure{ss_.str()};                                                         \
    }                                                                                           \
  } while (0)

#define SKIP(why)                    \
  do {                               \
    lstest::skip_reason() = (why);   \
    return;                          \
  } while (0)
