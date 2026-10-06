// A simulated rig for the native tests: a fake clock, a fake TMC2209, and a
// rotor that follows the step pulses past a hall sensor at a known angle.
// Each tick() runs the same MirrorCore::tick() the ESP32 timer interrupt runs,
// then the controller's poll() as the main loop would.
#pragma once

#include <stdlib.h>
#include <string.h>

#include <string>
#include <utility>
#include <vector>

#include "sm_controller.h"
#include "sm_core.h"

struct SimPort : sm::Port {
  int64_t now = 1000000;  // the ESP32 clock starts at boot; begin at 1 s
  std::vector<std::string> out;
  sm::Config stored = {};
  bool has_stored = false;
  bool dir_inv = false;
  int reboots = 0;

  int64_t nowUs() override { return now; }
  void lock() override {}
  void unlock() override {}
  void writeLine(const char* s, size_t n) override { out.push_back(std::string(s, n)); }
  bool saveConfig(const sm::Config& c) override {
    stored = c;
    has_stored = true;
    return true;
  }
  bool loadConfig(sm::Config* c) override {
    if (!has_stored) return false;
    *c = stored;
    return true;
  }
  void setDirInvert(bool invert) override { dir_inv = invert; }
  void reboot() override { ++reboots; }
};

struct FakeDriver : sm::Driver {
  bool present = true;
  bool enabled = false;
  bool full_hold = false;
  int configures = 0;
  sm::DriverStatus st = {0x21000000u, 0, sm::kDrvStst, 0};

  bool configure(const sm::Config&) override {
    ++configures;
    return present;
  }
  bool setFullHold(bool full, const sm::Config&) override {
    full_hold = full;
    return present;
  }
  bool read(sm::DriverStatus* s) override {
    if (!present) return false;
    *s = st;
    if (!enabled) s->ioin |= sm::kIoinEnn;  // the EN pin as wired
    st.gstat = 0;  // read() clears GSTAT
    return true;
  }
  void enable(bool on) override { enabled = on; }
};

struct SimRig {
  sm::MirrorCore core;
  SimPort port;
  FakeDriver drv;
  sm::Controller ctl;

  // The true rotor angle in microsteps. It moves only with step pulses (and
  // the DIR pin's inversion), never when the firmware renumbers its position.
  // The hall sensor turns on when the rotor comes within half_on of
  // hall_center and off beyond half_off (hysteresis), once per `rev`.
  int32_t rotor = 0;
  bool magnet = true;
  int32_t hall_center = 0;
  int32_t half_on = 60;
  int32_t half_off = 70;
  int32_t rev = 6400;
  bool hall_on = false;

  // What the pins did.
  std::vector<int64_t> step_times;
  std::vector<int8_t> step_dirs;
  std::vector<std::pair<int64_t, bool> > moving_edges;
  std::vector<int64_t> trig_rises;
  bool moving = false;
  bool trig = false;

  SimRig() : ctl(core, port, drv) {}

  void boot() { ctl.begin("poweron"); }

  bool hallRaw() {
    if (!magnet) return true;
    int32_t x = (rotor - hall_center) % rev;
    if (x < -rev / 2) x += rev;
    if (x >= rev / 2) x -= rev;
    const int32_t d = x < 0 ? -x : x;
    if (hall_on && d > half_off) hall_on = false;
    if (!hall_on && d <= half_on) hall_on = true;
    return !hall_on;  // the A3144 pulls its output low near the magnet
  }

  void tick() {
    tickCore();
    ctl.poll();
  }

  // The interrupt alone, as if the main loop were stuck.
  void tickCore() {
    port.now += sm::kTickUs;
    const sm::TickOut o = core.tick(port.now, hallRaw());
    if (o.step) {
      rotor += port.dir_inv ? -o.step : o.step;
      step_times.push_back(port.now);
      step_dirs.push_back(o.step);
    }
    if (o.moving != moving) {
      moving = o.moving;
      moving_edges.push_back(std::make_pair(port.now, moving));
    }
    if (o.trig && !trig) trig_rises.push_back(port.now);
    trig = o.trig;
  }

  void run(int64_t us) {
    for (int64_t t = 0; t < us; t += sm::kTickUs) tick();
  }

  void cmd(const char* line) {
    std::vector<char> buf(line, line + strlen(line) + 1);
    ctl.handleLine(buf.data());
  }

  // Send a command and return its reply (the OK or ERR line it printed).
  std::string ask(const char* line) {
    const size_t from = port.out.size();
    cmd(line);
    for (size_t i = from; i < port.out.size(); ++i) {
      const std::string& l = port.out[i];
      if (l.compare(0, 3, "OK ") == 0 || l.compare(0, 4, "ERR ") == 0) return l;
    }
    return "";
  }

  size_t mark() const { return port.out.size(); }

  // Run until a line starting with prefix has been printed since index
  // `from` (by default, from now on); returns it, or "" on timeout.
  std::string runUntil(const char* prefix, int64_t max_us, size_t from = static_cast<size_t>(-1)) {
    size_t seen = from == static_cast<size_t>(-1) ? port.out.size() : from;
    for (int64_t t = 0; t <= max_us; t += sm::kTickUs) {
      for (; seen < port.out.size(); ++seen) {
        if (port.out[seen].compare(0, strlen(prefix), prefix) == 0) return port.out[seen];
      }
      tick();
    }
    return "";
  }

  // Lines printed since index `from` that start with prefix.
  std::vector<std::string> linesWith(const char* prefix, size_t from = 0) const {
    std::vector<std::string> r;
    for (size_t i = from; i < port.out.size(); ++i) {
      if (port.out[i].compare(0, strlen(prefix), prefix) == 0) r.push_back(port.out[i]);
    }
    return r;
  }

  int32_t pos() const { return core.snapshot().pos; }

  std::string last() const { return port.out.empty() ? std::string() : port.out.back(); }
};

// Value of key=... in a reply or event line, as a number (-987654321 if absent).
inline long long field(const std::string& line, const char* key) {
  const std::string k = std::string(" ") + key + "=";
  const size_t i = line.find(k);
  if (i == std::string::npos) return -987654321;
  return strtoll(line.c_str() + i + k.size(), nullptr, 10);
}

inline bool hasField(const std::string& line, const char* key) {
  return line.find(std::string(" ") + key + "=") != std::string::npos;
}

inline std::string fieldStr(const std::string& line, const char* key) {
  const std::string k = std::string(" ") + key + "=";
  const size_t i = line.find(k);
  if (i == std::string::npos) return "";
  const size_t s = i + k.size();
  const size_t e = line.find(' ', s);
  return line.substr(s, e == std::string::npos ? std::string::npos : e - s);
}
