// Runtime settings, changed with the CFG command and stored in flash by SAVE.
#pragma once

#include <stddef.h>
#include <stdint.h>

namespace sm {

#define SM_FW_VERSION "0.1.0"
constexpr int32_t kProtocolVersion = 1;
constexpr int32_t kFullStepsPerRev = 200;  // 1.8 degree motor
constexpr int32_t kConfigVersion = 1;      // bump when Config's layout changes

// All fields are int32 so one table can parse, check, print and store them.
// Positions and speeds are in microsteps at the current `usteps` setting.
struct Config {
  int32_t version;
  int32_t usteps;      // microsteps per full step (TMC2209 MRES)
  int32_t irun;        // motor current, mA RMS
  int32_t ihold;       // standstill current when idle, % of irun
  int32_t stealth;     // 1 = StealthChop (quiet, smooth), 0 = SpreadCycle
  int32_t dir_inv;     // 1 = flip the DIR output
  int32_t vmax;        // move speed, microsteps/s
  int32_t vstart;      // start/stop speed, microsteps/s
  int32_t accel;       // microsteps/s^2
  int32_t settle;      // us after the last step before the mirror counts as still
  int32_t lim_min;     // soft limits for MOVE and SCAN
  int32_t lim_max;
  int32_t home_pos;    // position given to the middle of the hall window
  int32_t home_park;   // where HOME leaves the mirror
  int32_t home_fast;   // search speed, microsteps/s
  int32_t home_slow;   // speed for measuring the window edges
  int32_t home_range;  // furthest a search may go (more than one turn)
  int32_t home_clear;  // back-off past the window before the slow pass
  int32_t home_width;  // widest window accepted
  int32_t hall_high;   // 1 = sensor output is high near the magnet
  int32_t debounce;    // hall debounce, 50 us ticks
  int32_t trig_us;     // TRIG pulse width, 0 = off
  int32_t scan_hold;   // 1 = keep full current at standstill during scans
};

enum ConfigFlags : uint8_t {
  kCfgDriver = 1,  // needs the TMC2209 reconfigured
  kCfgCore = 2,    // needs MirrorCore updated
  kCfgUnhome = 4,  // invalidates the homed position
  kCfgPow2 = 8,    // must be a power of two
};

struct ConfigKey {
  const char* name;
  int32_t Config::*field;
  int32_t min;
  int32_t max;
  int32_t def;
  uint8_t flags;
};

size_t configKeyCount();
const ConfigKey& configKey(size_t i);
const ConfigKey* findConfigKey(const char* name);
void configDefaults(Config* c);
// Range checks plus the checks between fields. Returns nullptr when fine,
// otherwise a short reason; for a single bad field, *bad_key names it.
const char* configProblem(const Config& c, const ConfigKey** bad_key);

}  // namespace sm
