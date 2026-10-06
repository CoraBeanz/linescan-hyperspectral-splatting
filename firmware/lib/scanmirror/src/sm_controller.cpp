#include "sm_controller.h"

#include <stdio.h>
#include <string.h>

namespace sm {

namespace {

// Holds the core lock (the timer interrupt is locked out) for one scope.
class Locked {
 public:
  explicit Locked(Port& p) : p_(p) { p_.lock(); }
  ~Locked() { p_.unlock(); }

 private:
  Locked(const Locked&);
  Locked& operator=(const Locked&);
  Port& p_;
};

constexpr int64_t kDriverCheckUs = 500000;  // driver watchdog period
constexpr int64_t kHomeTimeoutUs = 60000000;
constexpr int64_t kScanLeadUs = 2000;  // a scan's first tick is at least this far ahead
constexpr int64_t kMaxScanDelayUs = 600000000;
constexpr uint32_t kMaxLines = 1000000;

int64_t floorHalf(int64_t v) { return v >= 0 ? v / 2 : -((-v + 1) / 2); }

}  // namespace

const char* Controller::stateName() const {
  switch (state_) {
    case kDisabled: return "disabled";
    case kIdle: return "idle";
    case kMoving: return "moving";
    case kStopping: return "stopping";
    case kHoming: return "homing";
    case kScanning: return "scanning";
    case kFault: return "fault";
  }
  return "?";
}

Profile Controller::profile(int32_t vmax, int32_t vstart, int32_t accel) const {
  Profile p;
  p.v_max = velocityQ32(vmax);
  p.v_start = velocityQ32(vstart < vmax ? vstart : vmax);
  p.accel = accelQ32(accel);
  return p;
}

CoreSnapshot Controller::snapshot() {
  Locked l(port_);
  return core_.snapshot();
}

uint16_t Controller::newTag() {
  if (++tag_seq_ == 0) tag_seq_ = 1;
  return tag_seq_;
}

void Controller::applyCoreConfig() {
  Locked l(port_);
  core_.setSettle(static_cast<uint32_t>(cfg_.settle));
  core_.setHall(cfg_.hall_high != 0, static_cast<uint16_t>(cfg_.debounce));
  core_.setTrigWidth(static_cast<uint32_t>(cfg_.trig_us));
  port_.setDirInvert(cfg_.dir_inv != 0);
}

bool Controller::applyDriverConfig() {
  if (drv_.configure(cfg_)) {
    drv_state_ = kDrvOk;
    hold_full_ = false;
    return true;
  }
  drv_state_ = kDrvNoResp;
  return false;
}

void Controller::restoreHold() {
  if (hold_full_ && drv_state_ == kDrvOk) drv_.setFullHold(false, cfg_);
  hold_full_ = false;
}

void Controller::begin(const char* reset_reason) {
  drv_.enable(false);
  if (!port_.loadConfig(&cfg_) || configProblem(cfg_, nullptr) != nullptr) configDefaults(&cfg_);
  applyCoreConfig();
  // Probe the driver now so EV BOOT can say whether it answers; the motor
  // stays off until ENABLE.
  applyDriverConfig();
  state_ = kDisabled;
  LineBuf b;
  b.word("EV").word("BOOT").kvStr("fw", SM_FW_VERSION).kv("proto", kProtocolVersion);
  b.kvStr("reset", reset_reason ? reset_reason : "unknown");
  b.kvStr("drv", drv_state_ == kDrvOk ? "ok" : "noresp");
  b.kv("t", port_.nowUs());
  send(b);
}

// ---------------------------------------------------------------- replies

void Controller::send(const LineBuf& b) { port_.writeLine(b.c_str(), b.size()); }

void Controller::replyOk(LineBuf& b) {
  if (args_.id()) b.kvStr("id", args_.id());
  send(b);
}

void Controller::replyErr(const char* code, const char* msg) {
  LineBuf b;
  b.word("ERR").word(args_.verb()).kvStr("code", code);
  if (args_.id()) b.kvStr("id", args_.id());
  if (msg) b.msg(msg);
  send(b);
}

bool Controller::argsDone() {
  const char* k = args_.unused();
  if (k == nullptr) return true;
  char m[64];
  snprintf(m, sizeof(m), "unknown key '%s'", k);
  replyErr("bad_arg", m);
  return false;
}

bool Controller::requireState(bool ok) {
  if (ok) return true;
  char m[48];
  snprintf(m, sizeof(m), "not allowed while %s", stateName());
  replyErr(state_ == kDisabled || state_ == kFault ? "disabled" : "busy", m);
  return false;
}

// ---------------------------------------------------------------- dispatch

void Controller::handleLine(char* line) {
  // Blank lines and # comments (handy in scripts) get no reply.
  const char* p = line;
  while (*p == ' ' || *p == '\t' || *p == '\r') ++p;
  if (*p == '\0' || *p == '#') return;

  if (!args_.parse(line)) {
    replyErr("syntax", args_.error());
    return;
  }

  struct Entry {
    const char* verb;
    void (Controller::*fn)();
  };
  static const Entry kTable[] = {
      {"INFO", &Controller::cmdInfo},       {"PING", &Controller::cmdPing},
      {"STATUS", &Controller::cmdStatus},   {"ENABLE", &Controller::cmdEnable},
      {"DISABLE", &Controller::cmdDisable}, {"HOME", &Controller::cmdHome},
      {"MOVE", &Controller::cmdMove},       {"STOP", &Controller::cmdStop},
      {"SCAN", &Controller::cmdScan},       {"NUDGE", &Controller::cmdNudge},
      {"PERIOD", &Controller::cmdPeriod},   {"ZERO", &Controller::cmdZero},
      {"CFG", &Controller::cmdCfg},         {"SAVE", &Controller::cmdSave},
      {"DEFAULTS", &Controller::cmdDefaults}, {"DRV", &Controller::cmdDrv},
      {"REBOOT", &Controller::cmdReboot},   {"HELP", &Controller::cmdHelp},
  };
  for (size_t i = 0; i < sizeof(kTable) / sizeof(kTable[0]); ++i) {
    if (strcmp(kTable[i].verb, args_.verb()) == 0) {
      (this->*kTable[i].fn)();
      return;
    }
  }
  replyErr("unknown_cmd", "try HELP");
}

// ---------------------------------------------------------------- commands

void Controller::cmdHelp() {
  if (!argsDone()) return;
  LineBuf b;
  b.word("OK").word("HELP").kvStr("verbs",
      "INFO,PING,STATUS,ENABLE,DISABLE,HOME,MOVE,STOP,SCAN,NUDGE,PERIOD,ZERO,CFG,SAVE,DEFAULTS,DRV,REBOOT,HELP");
  replyOk(b);
}

void Controller::cmdInfo() {
  if (!argsDone()) return;
  LineBuf b;
  b.word("OK").word("INFO").kvStr("fw", SM_FW_VERSION).kv("proto", kProtocolVersion);
  b.kv("usteps", cfg_.usteps).kv("full_steps", kFullStepsPerRev).kv("tick_us", kTickUs);
  b.kv("max_rate", kMaxStepsPerSec);
  replyOk(b);
}

void Controller::cmdPing() {
  if (!argsDone()) return;
  LineBuf b;
  b.word("OK").word("PING").kv("t", port_.nowUs());
  replyOk(b);
}

void Controller::cmdStatus() {
  if (!argsDone()) return;
  const CoreSnapshot s = snapshot();
  LineBuf b;
  b.word("OK").word("STATUS").kvStr("state", stateName());
  b.kv("pos", s.pos).kv("target", s.target).kv("homed", homed_ ? 1 : 0);
  b.kv("en", enabled() ? 1 : 0).kv("hall", s.hall ? 1 : 0);
  b.kv("line", s.lines_done).kv("lines", s.lines);
  const char* drv = drv_state_ == kDrvOk ? "ok" : drv_state_ == kDrvBypass ? "bypass" : "noresp";
  b.kvStr("drv", drv).kvStr("fault", fault_code_);
  b.kv("dropped", core_.droppedEvents()).kv("t", port_.nowUs());
  replyOk(b);
}

void Controller::cmdEnable() {
  int32_t force = 0;
  if (!args_.getI32("force", &force, 0, 1)) return replyErr("bad_arg", args_.error());
  if (!argsDone()) return;
  if (busy()) {
    LineBuf b;
    b.word("OK").word("ENABLE");
    return replyOk(b);
  }
  if (!applyDriverConfig()) {
    if (!force) {
      return replyErr("driver",
                      "TMC2209 did not answer on UART: check 12 V, VIO and the PDN_UART wire "
                      "(ENABLE force=1 runs without it)");
    }
    drv_state_ = kDrvBypass;
  }
  applyCoreConfig();
  drv_.enable(true);
  state_ = kIdle;
  fault_code_ = "none";
  otpw_warned_ = false;
  last_drv_check_ = port_.nowUs();
  LineBuf b;
  b.word("OK").word("ENABLE").kvStr("drv", drv_state_ == kDrvOk ? "ok" : "bypass");
  replyOk(b);
}

void Controller::stopEverything(const char* why) {
  const bool was_moving = busy();
  if (state_ == kHoming) homeFail(why);
  {
    Locked l(port_);
    core_.halt();
  }
  restoreHold();
  if (was_moving) {
    const CoreSnapshot s = snapshot();
    LineBuf b;
    b.word("EV").word("STOPPED").kv("pos", s.pos).kv("t", port_.nowUs());
    send(b);
  }
}

void Controller::cmdDisable() {
  if (!argsDone()) return;
  LineBuf b;
  b.word("OK").word("DISABLE");
  replyOk(b);
  stopEverything("disabled");
  drv_.enable(false);
  if (state_ != kFault) state_ = kDisabled;
  homed_ = false;
}

void Controller::cmdHome() {
  if (!argsDone()) return;
  if (!requireState(state_ == kIdle)) return;
  LineBuf b;
  b.word("OK").word("HOME");
  replyOk(b);
  homeStart();
}

void Controller::cmdMove() {
  int64_t pos = 0, rel = 0;
  int32_t v = cfg_.vmax, a = cfg_.accel;
  const bool has_pos = args_.has("pos"), has_rel = args_.has("rel");
  if (!args_.getI64("pos", &pos, -1000000, 1000000) || !args_.getI64("rel", &rel, -2000000, 2000000) ||
      !args_.getI32("v", &v, 1, kMaxStepsPerSec) || !args_.getI32("a", &a, 100, kMaxAccel)) {
    return replyErr("bad_arg", args_.error());
  }
  if (!argsDone()) return;
  if (has_pos == has_rel) return replyErr("bad_arg", "give exactly one of pos= or rel=");
  if (!requireState(state_ == kIdle || state_ == kMoving)) return;

  const CoreSnapshot s = snapshot();
  const int64_t target = has_pos ? pos : static_cast<int64_t>(s.target) + rel;
  if (target < cfg_.lim_min || target > cfg_.lim_max) {
    char m[80];
    snprintf(m, sizeof(m), "target %ld is outside min..max %ld..%ld", static_cast<long>(target),
             static_cast<long>(cfg_.lim_min), static_cast<long>(cfg_.lim_max));
    return replyErr("range", m);
  }
  move_tag_ = newTag();
  {
    Locked l(port_);
    core_.move(static_cast<int32_t>(target), profile(v, cfg_.vstart, a), move_tag_);
  }
  state_ = kMoving;
  LineBuf b;
  b.word("OK").word("MOVE").kv("pos", target);
  replyOk(b);
}

void Controller::cmdStop() {
  if (!argsDone()) return;
  LineBuf b;
  b.word("OK").word("STOP");
  replyOk(b);
  if (state_ == kHoming) homeFail("stopped");
  if (state_ == kDisabled || state_ == kFault) {
    // Nothing can be moving; answer at once so STOP always ends in STOPPED.
    const CoreSnapshot s = snapshot();
    LineBuf e;
    e.word("EV").word("STOPPED").kv("pos", s.pos).kv("t", port_.nowUs());
    return send(e);
  }
  stop_tag_ = newTag();
  move_tag_ = 0;
  {
    Locked l(port_);
    core_.stop(stop_tag_);
  }
  state_ = kStopping;
}

void Controller::cmdScan() {
  const char* mode = "stare";
  int64_t start = 0, t0 = 0, delay = 0;
  int32_t step = 1, lines = 0, settle = cfg_.settle;
  uint64_t period = 0;
  const bool has_start = args_.has("start"), has_t0 = args_.has("t0"), has_delay = args_.has("delay");
  if (!args_.getStr("mode", &mode) || !args_.getI64("start", &start, -1000000, 1000000) ||
      !args_.getI32("step", &step, -100000, 100000) ||
      !args_.getI32("lines", &lines, 1, static_cast<int32_t>(kMaxLines)) ||
      !args_.getQ16("period", &period, 1000, 60000000) ||
      !args_.getI64("t0", &t0, 0, INT64_C(1) << 52) || !args_.getI64("delay", &delay, 0, kMaxScanDelayUs) ||
      !args_.getI32("settle", &settle, 0, 1000000)) {
    return replyErr("bad_arg", args_.error());
  }
  if (!argsDone()) return;
  const bool sweep = strcmp(mode, "sweep") == 0;
  if (!sweep && strcmp(mode, "stare") != 0) return replyErr("bad_arg", "mode must be stare or sweep");
  if (!args_.has("lines") || period == 0) return replyErr("bad_arg", "lines= and period= are required");
  if (has_t0 && has_delay) return replyErr("bad_arg", "give t0= or delay=, not both");
  if (!requireState(state_ == kIdle)) return;

  const CoreSnapshot s = snapshot();
  if (!has_start) start = s.pos;
  // Stare holds lines 0..N-1; a sweep runs on to the end of line N-1.
  const int64_t last = start + static_cast<int64_t>(step) * (sweep ? lines : lines - 1);
  const int64_t lo = start < last ? start : last, hi = start < last ? last : start;
  if (lo < cfg_.lim_min || hi > cfg_.lim_max) {
    char m[96];
    snprintf(m, sizeof(m), "scan covers %ld..%ld, outside min..max %ld..%ld", static_cast<long>(lo),
             static_cast<long>(hi), static_cast<long>(cfg_.lim_min), static_cast<long>(cfg_.lim_max));
    return replyErr("range", m);
  }

  // Sweep speed |step| / period, in microsteps/s. It must not need a ramp,
  // or the mirror would lag the line clock at the start.
  const double rate = (step < 0 ? -step : step) * 1e6 * 65536.0 / static_cast<double>(period);
  if (sweep && rate > cfg_.vstart) {
    char m[96];
    snprintf(m, sizeof(m), "sweep speed %ld/s is above vstart %ld; lower step or raise period or vstart",
             static_cast<long>(rate + 0.5), static_cast<long>(cfg_.vstart));
    return replyErr("range", m);
  }

  int64_t now = port_.nowUs();
  int64_t first = has_t0 ? t0 : has_delay ? now + delay : now;
  if (first > now + kMaxScanDelayUs) return replyErr("range", "t0 is more than 10 minutes ahead");

  // Full current at standstill for the whole scan. Setting it takes the UART
  // a few ms, so the start time is checked against the clock after it.
  if (cfg_.scan_hold && drv_state_ == kDrvOk) hold_full_ = drv_.setFullHold(true, cfg_);
  now = port_.nowUs();
  if (first < now + kScanLeadUs) {
    // Too soon (or in the past): move on by whole periods, keeping the phase.
    const uint64_t behind_q16 = static_cast<uint64_t>(now + kScanLeadUs - first) * kQ16;
    const uint64_t k = (behind_q16 + period - 1) / period;
    first += static_cast<int64_t>((k * period + kQ16 / 2) / kQ16);
  }

  ScanParams p;
  p.mode = sweep ? ScanMode::kSweep : ScanMode::kStare;
  p.start = static_cast<int32_t>(start);
  p.step = step;
  p.lines = static_cast<uint32_t>(lines);
  p.t0_us = first;
  p.period_q16 = period;
  p.settle_us = static_cast<uint32_t>(settle);
  p.sweep_v = sweep && step != 0 ? velocityQ32(rate) : 0;
  p.slew = slewProfile();
  p.line = slewProfile();

  bool started;
  {
    Locked l(port_);
    started = core_.startScan(p);
  }
  if (!started) {
    restoreHold();
    return replyErr("busy", "a scan is already running");
  }
  state_ = kScanning;
  scan_sweep_ = sweep;
  scan_step_ = step;
  LineBuf b;
  b.word("OK").word("SCAN").kvStr("mode", sweep ? "sweep" : "stare").kv("t0", first);
  b.kvQ16("period", period).kv("start", start).kv("step", step).kv("lines", lines);
  replyOk(b);
}

void Controller::cmdNudge() {
  int32_t dt = 0;
  if (!args_.getI32("dt", &dt, -1000000, 1000000)) return replyErr("bad_arg", args_.error());
  if (!argsDone()) return;
  if (!args_.has("dt")) return replyErr("bad_arg", "dt= is required");
  if (state_ != kScanning) return replyErr("no_scan", "no scan is running");
  CoreSnapshot s;
  {
    Locked l(port_);
    core_.nudge(dt);
    s = core_.snapshot();
  }
  LineBuf b;
  b.word("OK").word("NUDGE").kv("next", s.next_tick_us);
  replyOk(b);
}

void Controller::cmdPeriod() {
  uint64_t period = 0;
  if (!args_.getQ16("us", &period, 1000, 60000000)) return replyErr("bad_arg", args_.error());
  if (!argsDone()) return;
  if (period == 0) return replyErr("bad_arg", "us= is required");
  if (state_ != kScanning) return replyErr("no_scan", "no scan is running");
  uint32_t v = 0;
  if (scan_sweep_ && scan_step_ != 0) {
    const double rate = (scan_step_ < 0 ? -scan_step_ : scan_step_) * 1e6 * 65536.0 / static_cast<double>(period);
    if (rate > cfg_.vstart) return replyErr("range", "that period makes the sweep faster than vstart");
    v = velocityQ32(rate);
  }
  bool ok;
  {
    Locked l(port_);
    ok = core_.setPeriod(period, v);
  }
  if (!ok) return replyErr("no_scan", "no scan is running");
  LineBuf b;
  b.word("OK").word("PERIOD").kvQ16("us", period);
  replyOk(b);
}

void Controller::cmdZero() {
  int32_t pos = 0;
  if (!args_.getI32("pos", &pos, -1000000, 1000000)) return replyErr("bad_arg", args_.error());
  if (!argsDone()) return;
  if (!requireState(!busy())) return;
  bool ok;
  {
    Locked l(port_);
    ok = core_.setPosition(pos);
  }
  if (!ok) return replyErr("busy", "the mirror is still moving");
  homed_ = false;
  LineBuf b;
  b.word("OK").word("ZERO").kv("pos", pos);
  replyOk(b);
}

void Controller::cmdCfg() {
  if (args_.count() == 0) {
    LineBuf b;
    b.word("OK").word("CFG");
    for (size_t i = 0; i < configKeyCount(); ++i) b.kv(configKey(i).name, cfg_.*(configKey(i).field));
    return replyOk(b);
  }
  if (!requireState(!busy())) return;
  Config next = cfg_;
  uint8_t flags = 0;
  for (int i = 0; i < args_.count(); ++i) {
    const ConfigKey* k = findConfigKey(args_.key(i));
    if (k == nullptr) continue;  // argsDone() reports it
    int32_t v = 0;
    if (!args_.getI32(k->name, &v, k->min, k->max)) return replyErr("bad_arg", args_.error());
    next.*(k->field) = v;
    flags |= k->flags;
  }
  if (!argsDone()) return;
  const ConfigKey* bad = nullptr;
  const char* problem = configProblem(next, &bad);
  if (problem) {
    char m[64];
    snprintf(m, sizeof(m), "%s%s%s", bad ? bad->name : "", bad ? " " : "", problem);
    return replyErr("bad_arg", m);
  }
  cfg_ = next;
  if (flags & kCfgCore) applyCoreConfig();
  if (flags & kCfgUnhome) homed_ = false;
  if ((flags & kCfgDriver) && drv_state_ != kDrvBypass) {
    if (!applyDriverConfig() && enabled()) {
      // The reply comes first; the fault event follows it.
      replyErr("driver", "settings kept, but the TMC2209 did not answer");
      return fault("drv_noresp", "TMC2209 did not take the new settings");
    }
  }
  LineBuf b;
  b.word("OK").word("CFG");
  for (int i = 0; i < args_.count(); ++i) {
    const ConfigKey* k = findConfigKey(args_.key(i));
    if (k) b.kv(k->name, cfg_.*(k->field));
  }
  replyOk(b);
}

void Controller::cmdSave() {
  if (!argsDone()) return;
  // Writing flash stalls the interrupt's cache for a few ms: not mid-scan.
  if (!requireState(!busy())) return;
  if (!port_.saveConfig(cfg_)) return replyErr("flash", "could not write the settings");
  LineBuf b;
  b.word("OK").word("SAVE");
  replyOk(b);
}

void Controller::cmdDefaults() {
  if (!argsDone()) return;
  if (!requireState(!busy())) return;
  configDefaults(&cfg_);
  applyCoreConfig();
  homed_ = false;
  const bool drv_ok = drv_state_ == kDrvBypass || applyDriverConfig();
  LineBuf b;
  b.word("OK").word("DEFAULTS");
  replyOk(b);
  if (!drv_ok && enabled()) fault("drv_noresp", "TMC2209 did not take the default settings");
}

void Controller::cmdDrv() {
  if (!argsDone()) return;
  DriverStatus st;
  if (!drv_.read(&st)) return replyErr("driver", "no answer on UART");
  const uint32_t d = st.drv_status;
  LineBuf b;
  b.word("OK").word("DRV").kvHex("ver", ioinVersion(st.ioin)).kv("enn", (st.ioin & kIoinEnn) ? 1 : 0);
  b.kv("cs", drvCsActual(d));
  b.kv("stst", (d & kDrvStst) ? 1 : 0).kv("stealth", (d & kDrvStealth) ? 1 : 0);
  b.kv("ot", (d & kDrvOt) ? 1 : 0).kv("otpw", (d & kDrvOtpw) ? 1 : 0);
  b.kv("s2g", (d & kDrvS2g) ? 1 : 0).kv("s2vs", (d & kDrvS2vs) ? 1 : 0).kv("ol", (d & kDrvOl) ? 1 : 0);
  b.kv("reset", (st.gstat & kGstatReset) ? 1 : 0).kv("drv_err", (st.gstat & kGstatDrvErr) ? 1 : 0);
  b.kv("uv_cp", (st.gstat & kGstatUvCp) ? 1 : 0).kv("mscnt", st.mscnt).kvHex("raw", d);
  replyOk(b);
}

void Controller::cmdReboot() {
  if (!argsDone()) return;
  LineBuf b;
  b.word("OK").word("REBOOT");
  replyOk(b);
  drv_.enable(false);
  port_.reboot();
}

// ---------------------------------------------------------------- events

void Controller::poll() {
  Event e;
  while (core_.popEvent(&e)) onEvent(e);

  const int64_t now = port_.nowUs();
  const uint32_t dropped = core_.droppedEvents();
  if (dropped != dropped_reported_) {
    dropped_reported_ = dropped;
    LineBuf b;
    b.word("EV").word("WARN").kvStr("code", "events_dropped").kv("n", dropped).kv("t", now);
    b.msg("event queue overflowed; read the port faster");
    send(b);
  }
  if (state_ == kHoming && now > home_deadline_) {
    homeStop();
    homeFail("timeout");
  }
  checkDriver(now);
}

void Controller::onEvent(const Event& e) {
  LineBuf b;
  switch (e.type) {
    case kEvLine:
      b.word("EV").word("LINE").kv("n", e.n).kv("t", e.t).kv("pos", e.pos);
      if (!scan_sweep_) b.kv("ready", e.t2);
      send(b);
      break;

    case kEvScanDone:
      b.word("EV").word("SCAN_DONE").kv("lines", e.n).kv("t", e.t).kv("pos", e.pos);
      b.kv("aborted", e.flag);
      send(b);
      restoreHold();
      if (state_ == kScanning) state_ = kIdle;
      break;

    case kEvMoveDone:
      if (e.tag == move_tag_ && state_ == kMoving) {
        move_tag_ = 0;
        state_ = kIdle;
        b.word("EV").word("MOVED").kv("pos", e.pos).kv("t", e.t);
        send(b);
      } else if (e.tag == stop_tag_ && state_ == kStopping) {
        stop_tag_ = 0;
        state_ = kIdle;
        b.word("EV").word("STOPPED").kv("pos", e.pos).kv("t", e.t);
        send(b);
      } else if (e.tag == home_tag_ && state_ == kHoming) {
        homeOnMoveDone(e);
      }
      break;

    case kEvHall:
      b.word("EV").word("HALL").kv("on", e.flag).kv("pos", e.pos).kv("t", e.t);
      send(b);
      if (state_ == kHoming) homeOnHall(e);
      break;
  }
}

// ---------------------------------------------------------------- homing
//
// 1. If the magnet is already at the sensor, move off it in + first.
// 2. Search in - at home_fast until the sensor trips, then stop.
// 3. Back off in + to home_clear past that edge.
// 4. Cross the window in - at home_slow, noting where the sensor turns on
//    (edge a) and off again (edge b), then stop.
// 5. The middle of the window becomes home_pos; park at home_park.
// Both edges are taken in the same direction at the same slow speed, so the
// sensor's hysteresis shifts them alike and the middle repeats.

void Controller::homeMove(int32_t target, const Profile& p) {
  home_tag_ = newTag();
  Locked l(port_);
  core_.move(target, p, home_tag_);
}

void Controller::homeStop() {
  home_tag_ = newTag();
  Locked l(port_);
  core_.stop(home_tag_);
}

void Controller::homeStart() {
  const CoreSnapshot s = snapshot();
  state_ = kHoming;
  homed_ = false;
  home_tries_ = 0;
  home_seen_ = false;
  home_have_b_ = false;
  home_deadline_ = port_.nowUs() + kHomeTimeoutUs;
  const Profile fast = profile(cfg_.home_fast, cfg_.vstart, cfg_.accel);
  if (s.hall) {
    home_step_ = kHomeLeave;
    homeMove(s.pos + cfg_.home_range, fast);
  } else {
    home_step_ = kHomeSeek;
    homeMove(s.pos - cfg_.home_range, fast);
  }
}

void Controller::homeOnHall(const Event& e) {
  const bool on = e.flag != 0;
  const Profile fast = profile(cfg_.home_fast, cfg_.vstart, cfg_.accel);
  switch (home_step_) {
    case kHomeLeave:
      if (!on) homeMove(e.pos + cfg_.home_clear, fast);
      break;
    case kHomeSeek:
      if (on && !home_seen_) {
        home_seen_ = true;
        home_seek_edge_ = e.pos;
        homeStop();
      }
      break;
    case kHomeApproach:
      if (on) {
        home_edge_a_ = e.pos;
        home_step_ = kHomeCross;
      }
      break;
    case kHomeCross:
      if (!on && !home_have_b_) {
        home_edge_b_ = e.pos;
        home_have_b_ = true;
        homeStop();
      }
      break;
    default:
      break;
  }
}

void Controller::homeOnMoveDone(const Event& e) {
  const CoreSnapshot s = snapshot();
  const Profile fast = profile(cfg_.home_fast, cfg_.vstart, cfg_.accel);
  switch (home_step_) {
    case kHomeLeave:
      if (s.hall) return homeFail("stuck");
      home_step_ = kHomeSeek;
      homeMove(s.pos - cfg_.home_range, fast);
      break;

    case kHomeSeek:
      if (!home_seen_) return homeFail("not_found");
      home_step_ = kHomeBackoff;
      homeMove(home_seek_edge_ + cfg_.home_clear, fast);
      break;

    case kHomeBackoff:
      if (s.hall) {
        // Still on the magnet: the window reaches further than home_clear.
        if (++home_tries_ > 8) return homeFail("too_wide");
        homeMove(s.pos + cfg_.home_clear, fast);
        break;
      }
      home_step_ = kHomeApproach;
      homeMove(s.pos - cfg_.home_clear - cfg_.home_width,
               profile(cfg_.home_slow, cfg_.home_slow, cfg_.accel));
      break;

    case kHomeApproach:
      return homeFail("lost");  // the slow pass never reached the magnet

    case kHomeCross: {
      if (!home_have_b_) return homeFail("too_wide");
      const int32_t center = static_cast<int32_t>(floorHalf(static_cast<int64_t>(home_edge_a_) + home_edge_b_));
      home_shift_ = cfg_.home_pos - center;
      bool ok;
      {
        Locked l(port_);
        ok = core_.setPosition(s.pos + home_shift_);
      }
      if (!ok) return homeFail("busy");
      home_step_ = kHomePark;
      homeMove(cfg_.home_park, slewProfile());
      break;
    }

    case kHomePark: {
      home_step_ = kHomeOff;
      homed_ = true;
      state_ = kIdle;
      LineBuf b;
      b.word("EV").word("HOMED").kv("pos", e.pos).kv("t", e.t);
      b.kv("width", home_edge_a_ - home_edge_b_).kv("shift", home_shift_);
      send(b);
      break;
    }

    case kHomeOff:
      break;
  }
}

void Controller::homeFail(const char* reason) {
  home_step_ = kHomeOff;
  if (state_ == kHoming) state_ = kIdle;
  const CoreSnapshot s = snapshot();
  LineBuf b;
  b.word("EV").word("HOME_FAILED").kvStr("reason", reason).kv("pos", s.pos).kv("t", port_.nowUs());
  send(b);
}

// ---------------------------------------------------------------- driver watchdog

void Controller::fault(const char* code, const char* msg) {
  stopEverything("fault");
  drv_.enable(false);
  state_ = kFault;
  homed_ = false;
  fault_code_ = code;
  LineBuf b;
  b.word("EV").word("FAULT").kvStr("code", code).kv("t", port_.nowUs()).msg(msg);
  send(b);
}

void Controller::checkDriver(int64_t now) {
  if (now - last_drv_check_ < kDriverCheckUs) return;
  last_drv_check_ = now;
  if (!enabled() || drv_state_ != kDrvOk) return;

  DriverStatus st;
  if (!drv_.read(&st)) return fault("drv_noresp", "TMC2209 stopped answering on UART");
  const uint32_t d = st.drv_status;
  // A reset means the driver lost power and is back on its power-up
  // settings, where the current comes from the VREF pot.
  if (st.gstat & kGstatReset) return fault("drv_reset", "TMC2209 reset (lost 12 V?); ENABLE to set it up again");
  if (d & kDrvOt) return fault("drv_ot", "TMC2209 overtemperature shutdown");
  if (d & (kDrvS2g | kDrvS2vs)) return fault("drv_short", "short circuit on a motor coil");
  if (st.gstat & kGstatUvCp) return fault("drv_uv", "charge pump undervoltage (12 V too low?)");
  if (st.gstat & kGstatDrvErr) return fault("drv_err", "TMC2209 shut its outputs off");
  if ((d & kDrvOtpw) && !otpw_warned_) {
    otpw_warned_ = true;
    LineBuf b;
    b.word("EV").word("WARN").kvStr("code", "drv_otpw").kv("t", now).msg("TMC2209 is getting hot");
    send(b);
  }
}

}  // namespace sm
