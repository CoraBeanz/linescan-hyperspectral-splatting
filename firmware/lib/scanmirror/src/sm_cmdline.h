// Parsing of command lines ("VERB key=value ...") and formatting of replies.
#pragma once

#include <stddef.h>
#include <stdint.h>

namespace sm {

// Splits one command line in place. Verbs are matched case-insensitively and
// keys are lowercased. Getters mark a key as used; unused() then catches
// typos such as "perod=33333" instead of silently ignoring them.
class Args {
 public:
  static constexpr int kMaxPairs = 16;

  // Returns false on a syntax error; error() then says why.
  bool parse(char* line);

  const char* verb() const { return verb_; }
  const char* id() const { return id_; }
  const char* error() const { return err_; }
  bool has(const char* key) const;

  // Each getter returns true when the key is absent (leaving *out alone) or
  // holds a valid value, false (with error() set) when the value is bad.
  bool getI32(const char* key, int32_t* out, int32_t min, int32_t max);
  bool getI64(const char* key, int64_t* out, int64_t min, int64_t max);
  // Decimal microseconds with up to 6 fraction digits, e.g. 33333.333,
  // into Q16 microseconds.
  bool getQ16(const char* key, uint64_t* out, uint64_t min_us, uint64_t max_us);
  bool getStr(const char* key, const char** out);

  // The first key no getter asked for, or nullptr.
  const char* unused() const;
  int count() const { return n_; }
  const char* key(int i) const { return kv_[i].key; }
  const char* value(int i) const { return kv_[i].value; }
  void markUsed(int i) { kv_[i].used = true; }

 private:
  struct Pair {
    const char* key;
    const char* value;
    bool used;
  };
  int find(const char* key) const;
  bool fail(const char* what, const char* key);

  Pair kv_[kMaxPairs];
  int n_ = 0;
  const char* verb_ = "";
  const char* id_ = nullptr;
  char err_[64] = {0};
};

// Parsers shared with Args, exposed for the tests.
bool parseI64(const char* s, int64_t* out);
bool parseQ16(const char* s, uint64_t* out);

// Builds one output line. Fields are separated by single spaces; the line is
// silently cut at kCap characters (nothing the firmware prints is that long).
class LineBuf {
 public:
  static constexpr size_t kCap = 480;

  LineBuf& word(const char* s);
  LineBuf& kv(const char* key, int64_t value);
  LineBuf& kvStr(const char* key, const char* value);
  LineBuf& kvHex(const char* key, uint32_t value);
  LineBuf& kvQ16(const char* key, uint64_t q16_us);  // prints 33333.333
  // Appends " msg=<text>"; text may contain spaces, so it must come last.
  LineBuf& msg(const char* text);

  const char* c_str() const { return buf_; }
  size_t size() const { return n_; }
  void clear() {
    n_ = 0;
    buf_[0] = '\0';
  }

 private:
  void sep();
  void raw(const char* s);
  void rawI64(int64_t v);

  char buf_[kCap + 1] = {0};
  size_t n_ = 0;
};

}  // namespace sm
