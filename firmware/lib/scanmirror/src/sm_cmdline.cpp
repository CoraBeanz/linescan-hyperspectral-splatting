#include "sm_cmdline.h"

#include <string.h>

namespace sm {

namespace {

bool isSpace(char c) { return c == ' ' || c == '\t' || c == '\r' || c == '\n'; }

char lower(char c) { return (c >= 'A' && c <= 'Z') ? static_cast<char>(c - 'A' + 'a') : c; }
char upper(char c) { return (c >= 'a' && c <= 'z') ? static_cast<char>(c - 'a' + 'A') : c; }

}  // namespace

// Strict decimal integer: optional sign, digits only, no overflow.
bool parseI64(const char* s, int64_t* out) {
  if (s == nullptr || *s == '\0') return false;
  bool neg = false;
  if (*s == '+' || *s == '-') {
    neg = *s == '-';
    ++s;
  }
  if (*s == '\0') return false;
  uint64_t v = 0;
  for (; *s; ++s) {
    if (*s < '0' || *s > '9') return false;
    const uint64_t d = static_cast<uint64_t>(*s - '0');
    if (v > (UINT64_C(9223372036854775807) - d) / 10) return false;
    v = v * 10 + d;
  }
  *out = neg ? -static_cast<int64_t>(v) : static_cast<int64_t>(v);
  return true;
}

// "33333.333" -> 33333.333 * 65536, rounded. Up to 6 fraction digits and at
// most about 1.4e14 us in the integer part.
bool parseQ16(const char* s, uint64_t* out) {
  if (s == nullptr || *s == '\0' || *s == '-') return false;
  if (*s == '+') ++s;
  uint64_t whole = 0;
  int digits = 0;
  for (; *s >= '0' && *s <= '9'; ++s, ++digits) {
    if (whole > UINT64_C(140000000000000)) return false;
    whole = whole * 10 + static_cast<uint64_t>(*s - '0');
  }
  uint64_t frac = 0;
  uint64_t scale = 1;
  if (*s == '.') {
    ++s;
    for (; *s >= '0' && *s <= '9'; ++s, ++digits) {
      if (scale >= 1000000) return false;
      frac = frac * 10 + static_cast<uint64_t>(*s - '0');
      scale *= 10;
    }
  }
  if (*s != '\0' || digits == 0) return false;
  *out = whole * 65536 + (frac * 65536 + scale / 2) / scale;
  return true;
}

bool Args::parse(char* line) {
  n_ = 0;
  verb_ = "";
  id_ = nullptr;
  err_[0] = '\0';

  // Split on whitespace, ending each token by overwriting the space after it.
  char* tokens[kMaxPairs + 2];
  int count = 0;
  bool too_many = false;
  char* p = line;
  for (;;) {
    while (isSpace(*p)) *p++ = '\0';
    if (*p == '\0') break;
    if (count == kMaxPairs + 2) {
      too_many = true;
      break;
    }
    tokens[count++] = p;
    while (*p && !isSpace(*p)) ++p;
  }
  if (count == 0) return fail("empty line", nullptr);

  verb_ = tokens[0];
  for (char* v = tokens[0]; *v; ++v) *v = upper(*v);

  // Split every pair before judging any, so that even the error reply to a
  // bad line carries the line's id.
  char* keys[kMaxPairs + 1];
  char* values[kMaxPairs + 1];
  int pairs = 0;
  const char* bad = nullptr;
  for (int i = 1; i < count; ++i) {
    char* tok = tokens[i];
    char* eq = strchr(tok, '=');
    if (eq == nullptr || eq == tok) {
      if (bad == nullptr) bad = tok;
      continue;
    }
    *eq = '\0';
    for (char* k = tok; *k; ++k) *k = lower(*k);
    if (strcmp(tok, "id") == 0) {
      id_ = eq + 1;
      continue;
    }
    keys[pairs] = tok;
    values[pairs] = eq + 1;
    ++pairs;
  }
  if (too_many || pairs > kMaxPairs) return fail("too many arguments", nullptr);
  if (bad) return fail("expected key=value, got", bad);
  for (int i = 0; i < pairs; ++i) {
    if (find(keys[i]) >= 0) return fail("repeated key", keys[i]);
    kv_[n_].key = keys[i];
    kv_[n_].value = values[i];
    kv_[n_].used = false;
    ++n_;
  }
  return true;
}

int Args::find(const char* key) const {
  for (int i = 0; i < n_; ++i) {
    if (strcmp(kv_[i].key, key) == 0) return i;
  }
  return -1;
}

bool Args::has(const char* key) const { return find(key) >= 0; }

bool Args::fail(const char* what, const char* key) {
  size_t n = 0;
  for (const char* s = what; *s && n + 1 < sizeof(err_); ++s) err_[n++] = *s;
  if (key) {
    if (n + 1 < sizeof(err_)) err_[n++] = ' ';
    for (const char* s = key; *s && n + 1 < sizeof(err_); ++s) err_[n++] = *s;
  }
  err_[n] = '\0';
  return false;
}

bool Args::getI64(const char* key, int64_t* out, int64_t min, int64_t max) {
  const int i = find(key);
  if (i < 0) return true;
  kv_[i].used = true;
  int64_t v;
  if (!parseI64(kv_[i].value, &v)) return fail("not an integer:", key);
  if (v < min || v > max) return fail("out of range:", key);
  *out = v;
  return true;
}

bool Args::getI32(const char* key, int32_t* out, int32_t min, int32_t max) {
  int64_t v = 0;
  const bool present = has(key);
  if (!getI64(key, &v, min, max)) return false;
  if (present) *out = static_cast<int32_t>(v);
  return true;
}

bool Args::getQ16(const char* key, uint64_t* out, uint64_t min_us, uint64_t max_us) {
  const int i = find(key);
  if (i < 0) return true;
  kv_[i].used = true;
  uint64_t v;
  if (!parseQ16(kv_[i].value, &v)) return fail("not a number:", key);
  if (v < min_us * 65536 || v > max_us * 65536) return fail("out of range:", key);
  *out = v;
  return true;
}

bool Args::getStr(const char* key, const char** out) {
  const int i = find(key);
  if (i < 0) return true;
  kv_[i].used = true;
  *out = kv_[i].value;
  return true;
}

const char* Args::unused() const {
  for (int i = 0; i < n_; ++i) {
    if (!kv_[i].used) return kv_[i].key;
  }
  return nullptr;
}

void LineBuf::raw(const char* s) {
  while (*s && n_ < kCap) buf_[n_++] = *s++;
  buf_[n_] = '\0';
}

void LineBuf::sep() {
  if (n_ > 0) raw(" ");
}

void LineBuf::rawI64(int64_t v) {
  char tmp[24];
  int i = 0;
  uint64_t u = v < 0 ? static_cast<uint64_t>(-(v + 1)) + 1 : static_cast<uint64_t>(v);
  do {
    tmp[i++] = static_cast<char>('0' + u % 10);
    u /= 10;
  } while (u);
  if (v < 0) tmp[i++] = '-';
  char out[24];
  for (int j = 0; j < i; ++j) out[j] = tmp[i - 1 - j];
  out[i] = '\0';
  raw(out);
}

LineBuf& LineBuf::word(const char* s) {
  sep();
  raw(s);
  return *this;
}

LineBuf& LineBuf::kv(const char* key, int64_t value) {
  sep();
  raw(key);
  raw("=");
  rawI64(value);
  return *this;
}

LineBuf& LineBuf::kvStr(const char* key, const char* value) {
  sep();
  raw(key);
  raw("=");
  raw(value);
  return *this;
}

LineBuf& LineBuf::kvHex(const char* key, uint32_t value) {
  static const char kDigits[] = "0123456789abcdef";
  char hex[11] = "0x";
  int n = 2;
  bool started = false;
  for (int shift = 28; shift >= 0; shift -= 4) {
    const uint32_t d = (value >> shift) & 0xF;
    if (d || started || shift == 0) {
      hex[n++] = kDigits[d];
      started = true;
    }
  }
  hex[n] = '\0';
  return kvStr(key, hex);
}

LineBuf& LineBuf::kvQ16(const char* key, uint64_t q16_us) {
  // Three decimals, rounded.
  const uint64_t milli = (q16_us * 1000 + 32768) / 65536;
  sep();
  raw(key);
  raw("=");
  rawI64(static_cast<int64_t>(milli / 1000));
  char frac[5] = {'.', static_cast<char>('0' + (milli / 100) % 10), static_cast<char>('0' + (milli / 10) % 10),
                  static_cast<char>('0' + milli % 10), '\0'};
  raw(frac);
  return *this;
}

LineBuf& LineBuf::msg(const char* text) {
  sep();
  raw("msg=");
  raw(text);
  return *this;
}

}  // namespace sm
