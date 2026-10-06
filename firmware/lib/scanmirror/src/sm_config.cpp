#include "sm_config.h"

#include <string.h>

namespace sm {

namespace {

// Defaults for the NEMA 8 (8HS11-0204S, 0.2 A) on a BTT TMC2209 at 1/32.
// One microstep turns the mirror 0.05625 deg and the view twice that.
const ConfigKey kKeys[] = {
    {"usteps", &Config::usteps, 1, 256, 32, kCfgDriver | kCfgUnhome | kCfgPow2},
    {"irun", &Config::irun, 50, 1200, 200, kCfgDriver},
    {"ihold", &Config::ihold, 0, 100, 50, kCfgDriver},
    {"stealth", &Config::stealth, 0, 1, 1, kCfgDriver},
    {"dir_inv", &Config::dir_inv, 0, 1, 0, kCfgCore | kCfgUnhome},
    {"vmax", &Config::vmax, 1, 10000, 3200, 0},
    {"vstart", &Config::vstart, 1, 10000, 1600, 0},
    {"accel", &Config::accel, 100, 1000000, 20000, 0},
    {"settle", &Config::settle, 0, 1000000, 3000, kCfgCore},
    {"min", &Config::lim_min, -1000000, 1000000, -3200, 0},
    {"max", &Config::lim_max, -1000000, 1000000, 3200, 0},
    // The CAD puts the magnet at the hall sensor with the mirror turned 40 deg
    // back from its 45 deg working angle: 40 / 0.05625 = 711 microsteps.
    {"home_pos", &Config::home_pos, -1000000, 1000000, -711, kCfgUnhome},
    {"home_park", &Config::home_park, -1000000, 1000000, 0, 0},
    {"home_fast", &Config::home_fast, 1, 10000, 1600, 0},
    {"home_slow", &Config::home_slow, 1, 10000, 100, 0},
    {"home_range", &Config::home_range, 1, 1000000, 7200, 0},
    {"home_clear", &Config::home_clear, 1, 100000, 160, 0},
    {"home_width", &Config::home_width, 1, 100000, 1600, 0},
    {"hall_high", &Config::hall_high, 0, 1, 0, kCfgCore},
    {"debounce", &Config::debounce, 1, 1000, 4, kCfgCore},
    {"trig_us", &Config::trig_us, 0, 100000, 100, kCfgCore},
    {"scan_hold", &Config::scan_hold, 0, 1, 1, 0},
};

}  // namespace

size_t configKeyCount() { return sizeof(kKeys) / sizeof(kKeys[0]); }

const ConfigKey& configKey(size_t i) { return kKeys[i]; }

const ConfigKey* findConfigKey(const char* name) {
  for (size_t i = 0; i < configKeyCount(); ++i) {
    if (strcmp(kKeys[i].name, name) == 0) return &kKeys[i];
  }
  return nullptr;
}

void configDefaults(Config* c) {
  memset(c, 0, sizeof(*c));
  c->version = kConfigVersion;
  for (size_t i = 0; i < configKeyCount(); ++i) c->*(kKeys[i].field) = kKeys[i].def;
}

const char* configProblem(const Config& c, const ConfigKey** bad_key) {
  if (bad_key) *bad_key = nullptr;
  if (c.version != kConfigVersion) return "wrong config version";
  for (size_t i = 0; i < configKeyCount(); ++i) {
    const ConfigKey& k = kKeys[i];
    const int32_t v = c.*(k.field);
    const bool pow2_ok = !(k.flags & kCfgPow2) || (v & (v - 1)) == 0;
    if (v < k.min || v > k.max || !pow2_ok) {
      if (bad_key) *bad_key = &k;
      return pow2_ok ? "out of range" : "not a power of two";
    }
  }
  if (c.vstart > c.vmax) return "vstart must not exceed vmax";
  if (c.lim_min >= c.lim_max) return "min must be below max";
  if (c.home_slow > c.home_fast) return "home_slow must not exceed home_fast";
  if (c.home_park < c.lim_min || c.home_park > c.lim_max) return "home_park must be inside min..max";
  return nullptr;
}

}  // namespace sm
