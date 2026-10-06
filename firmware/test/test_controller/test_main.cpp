// The whole firmware minus the ESP32 glue, on a simulated rig: commands and
// replies, moves, homing against a hall sensor with hysteresis, stare and
// sweep scans on a camera clock, and the TMC2209 watchdog.
#include <unity.h>

#include <stdio.h>

#include "../sim_rig.h"

void setUp() {}
void tearDown() {}

#define ASSERT_LINE(expected, actual) TEST_ASSERT_EQUAL_STRING(expected, (actual).c_str())
#define ASSERT_FIELD(expected, line, key) TEST_ASSERT_EQUAL_INT64(expected, field(line, key))

static void enable(SimRig& r) {
  r.boot();
  ASSERT_LINE("OK ENABLE drv=ok", r.ask("ENABLE"));
  r.run(1000);  // a few ticks, so the hall sensor has been read
}

static int32_t wrap(int32_t v, int32_t n) {
  const int32_t m = v % n;
  return m < 0 ? m + n : m;
}

static bool hasEdge(const SimRig& r, int64_t t, bool level) {
  for (size_t i = 0; i < r.moving_edges.size(); ++i) {
    if (r.moving_edges[i].first == t && r.moving_edges[i].second == level) return true;
  }
  return false;
}

static std::string askf(SimRig& r, const char* fmt, long long a) {
  char line[200];
  snprintf(line, sizeof(line), fmt, a);
  return r.ask(line);
}

// ---------------------------------------------------------------- basics

void test_boot_announces_itself_and_stays_disabled() {
  SimRig r;
  r.boot();
  TEST_ASSERT_EQUAL(1, (int)r.port.out.size());
  ASSERT_LINE("EV BOOT fw=0.1.0 proto=1 reset=poweron drv=ok t=1000000", r.port.out[0]);
  TEST_ASSERT_FALSE(r.drv.enabled);
  ASSERT_LINE(
      "OK STATUS state=disabled pos=0 target=0 homed=0 en=0 hall=0 line=0 lines=0 drv=ok "
      "fault=none dropped=0 t=1000000",
      r.ask("STATUS"));
  ASSERT_LINE("OK INFO fw=0.1.0 proto=1 usteps=32 full_steps=200 tick_us=50 max_rate=10000", r.ask("INFO"));
  ASSERT_LINE("OK PING t=1000000 id=abc", r.ask("ping id=abc"));
}

void test_motion_needs_enable() {
  SimRig r;
  r.boot();
  ASSERT_LINE("ERR MOVE code=disabled msg=not allowed while disabled", r.ask("MOVE pos=10"));
  ASSERT_LINE("ERR HOME code=disabled msg=not allowed while disabled", r.ask("HOME"));
  ASSERT_LINE("ERR SCAN code=disabled msg=not allowed while disabled", r.ask("SCAN lines=5 period=10000"));
  ASSERT_LINE("ERR NUDGE code=no_scan msg=no scan is running", r.ask("NUDGE dt=5"));
  r.run(100000);
  TEST_ASSERT_EQUAL(0, (int)r.step_times.size());
}

void test_errors_name_the_verb_and_echo_the_id() {
  SimRig r;
  enable(r);
  ASSERT_LINE("ERR FOO code=unknown_cmd id=7 msg=try HELP", r.ask("foo id=7"));
  ASSERT_LINE("ERR MOVE code=syntax id=8 msg=expected key=value, got 100", r.ask("move 100 id=8"));
  ASSERT_LINE("ERR MOVE code=bad_arg msg=unknown key 'speed'", r.ask("MOVE pos=1 speed=3"));
  ASSERT_LINE("ERR MOVE code=bad_arg msg=give exactly one of pos= or rel=", r.ask("MOVE pos=1 rel=2"));
  ASSERT_LINE("ERR MOVE code=bad_arg msg=not an integer: pos", r.ask("MOVE pos=abc"));
  ASSERT_LINE("ERR MOVE code=bad_arg msg=out of range: v", r.ask("MOVE pos=10 v=20000"));
  // Blank lines and comments get no reply at all.
  const size_t m = r.mark();
  r.cmd("");
  r.cmd("   # a comment");
  TEST_ASSERT_EQUAL(m, r.mark());
  TEST_ASSERT_EQUAL(0, (int)r.step_times.size());
}

void test_help_drv_and_reboot() {
  SimRig r;
  r.boot();
  TEST_ASSERT_EQUAL(0, r.ask("HELP").find("OK HELP verbs=INFO,PING,STATUS,ENABLE,DISABLE,HOME,MOVE,STOP,SCAN"));
  ASSERT_LINE(
      "OK DRV ver=0x21 enn=1 cs=0 stst=1 stealth=0 ot=0 otpw=0 s2g=0 s2vs=0 ol=0 reset=0 drv_err=0 "
      "uv_cp=0 mscnt=0 raw=0x80000000",
      r.ask("DRV"));
  r.drv.present = false;
  ASSERT_LINE("ERR DRV code=driver msg=no answer on UART", r.ask("DRV"));
  r.drv.enabled = true;
  ASSERT_LINE("OK REBOOT", r.ask("REBOOT"));
  TEST_ASSERT_FALSE(r.drv.enabled);
  TEST_ASSERT_EQUAL(1, r.port.reboots);
}

// ---------------------------------------------------------------- moves

void test_move_arrives_settles_and_reports() {
  SimRig r;
  enable(r);
  ASSERT_LINE("OK MOVE pos=100", r.ask("MOVE pos=100"));
  const std::string moved = r.runUntil("EV MOVED", 1000000);
  ASSERT_FIELD(100, moved, "pos");
  TEST_ASSERT_EQUAL(100, (int)r.step_times.size());
  for (size_t i = 0; i < r.step_dirs.size(); ++i) TEST_ASSERT_EQUAL_INT8(1, r.step_dirs[i]);
  TEST_ASSERT_EQUAL_INT32(100, r.rotor);
  // MOVED (and the MOVING pin's fall) come one settle time after the last step.
  const int64_t last_step = r.step_times.back();
  ASSERT_FIELD(last_step + 3000, moved, "t");
  TEST_ASSERT_TRUE(hasEdge(r, r.step_times.front(), true));
  TEST_ASSERT_TRUE(hasEdge(r, last_step + 3000, false));

  ASSERT_LINE("OK MOVE pos=50", r.ask("MOVE rel=-50"));
  ASSERT_FIELD(50, r.runUntil("EV MOVED", 1000000), "pos");
  TEST_ASSERT_EQUAL_INT32(50, r.rotor);
  ASSERT_LINE("ERR MOVE code=range msg=target 3201 is outside min..max -3200..3200", r.ask("MOVE pos=3201"));
  const std::string st = r.ask("STATUS");
  TEST_ASSERT_EQUAL_STRING("idle", fieldStr(st, "state").c_str());
  ASSERT_FIELD(50, st, "pos");
  ASSERT_FIELD(1, st, "en");
}

void test_new_move_replaces_the_old_one() {
  SimRig r;
  enable(r);
  const size_t m = r.mark();
  r.ask("MOVE pos=3000");
  r.run(200000);
  const int32_t at = r.pos();
  TEST_ASSERT_TRUE(at > 200 && at < 3000);
  ASSERT_LINE("OK MOVE pos=-100", r.ask("MOVE pos=-100"));
  ASSERT_FIELD(-100, r.runUntil("EV MOVED", 3000000), "pos");
  TEST_ASSERT_EQUAL(1, (int)r.linesWith("EV MOVED", m).size());
  TEST_ASSERT_EQUAL_INT32(-100, r.rotor);
}

void test_stop_slows_down_and_reports() {
  SimRig r;
  enable(r);
  r.ask("MOVE pos=3000");
  r.run(300000);
  const int32_t at = r.pos();
  ASSERT_LINE("OK STOP", r.ask("STOP"));
  const std::string stopped = r.runUntil("EV STOPPED", 1000000);
  const long long p = field(stopped, "pos");
  TEST_ASSERT_TRUE(p > at && p < 3000);
  TEST_ASSERT_EQUAL_INT64(p, r.pos());
  TEST_ASSERT_EQUAL_STRING("idle", fieldStr(r.ask("STATUS"), "state").c_str());

  // STOP always ends in STOPPED, even with nothing moving.
  ASSERT_LINE("OK STOP", r.ask("STOP"));
  TEST_ASSERT_TRUE(r.runUntil("EV STOPPED", 10000).size() > 0);
  r.ask("DISABLE");
  const size_t m = r.mark();
  ASSERT_LINE("OK STOP", r.ask("STOP"));
  TEST_ASSERT_EQUAL(1, (int)r.linesWith("EV STOPPED", m).size());
}

void test_disable_stops_dead() {
  SimRig r;
  enable(r);
  r.ask("MOVE pos=3000");
  r.run(200000);
  const size_t m = r.mark();
  r.cmd("DISABLE");
  TEST_ASSERT_EQUAL(m + 2, r.mark());
  ASSERT_LINE("OK DISABLE", r.port.out[m]);
  TEST_ASSERT_EQUAL(0, (int)r.port.out[m + 1].find("EV STOPPED pos="));
  TEST_ASSERT_FALSE(r.drv.enabled);
  const size_t steps = r.step_times.size();
  r.run(100000);
  TEST_ASSERT_EQUAL(steps, r.step_times.size());
  TEST_ASSERT_EQUAL_STRING("disabled", fieldStr(r.ask("STATUS"), "state").c_str());
  ASSERT_LINE("OK ENABLE drv=ok", r.ask("ENABLE"));
  ASSERT_LINE("OK MOVE pos=0", r.ask("MOVE pos=0"));
}

void test_zero_and_dir_invert() {
  SimRig r;
  enable(r);
  ASSERT_LINE("OK ZERO pos=500", r.ask("ZERO pos=500"));
  const std::string st = r.ask("STATUS");
  ASSERT_FIELD(500, st, "pos");
  ASSERT_FIELD(500, st, "target");
  ASSERT_FIELD(0, st, "homed");
  r.ask("MOVE pos=600");
  ASSERT_LINE("ERR ZERO code=busy msg=not allowed while moving", r.ask("ZERO pos=0"));
  r.runUntil("EV MOVED", 1000000);
  TEST_ASSERT_EQUAL_INT32(100, r.rotor);

  ASSERT_LINE("OK CFG dir_inv=1", r.ask("CFG dir_inv=1"));
  TEST_ASSERT_TRUE(r.port.dir_inv);
  r.ask("MOVE pos=500");
  r.runUntil("EV MOVED", 1000000);
  TEST_ASSERT_EQUAL_INT32(500, r.pos());
  TEST_ASSERT_EQUAL_INT32(200, r.rotor);  // the DIR pin was flipped
}

// ---------------------------------------------------------------- homing

// Home from a given rotor angle and firmware count. The window's middle
// (less half the sensor's 10 microstep hysteresis, since both edges are
// taken moving the same way) must always come out as home_pos, -711.
static void homeFrom(int32_t rotor, int32_t fw) {
  SimRig r;
  r.rotor = rotor;
  enable(r);
  askf(r, "ZERO pos=%lld", fw);
  const size_t m = r.mark();
  ASSERT_LINE("OK HOME", r.ask("HOME"));
  const std::string homed = r.runUntil("EV HOMED", 30000000, m);
  TEST_ASSERT_TRUE_MESSAGE(homed.size() > 0, "never homed");
  ASSERT_FIELD(0, homed, "pos");
  ASSERT_FIELD(131, homed, "width");
  TEST_ASSERT_EQUAL_INT32(wrap(-711 + 6, 6400), wrap(r.pos() - r.rotor, 6400));
  const std::string st = r.ask("STATUS");
  ASSERT_FIELD(1, st, "homed");
  TEST_ASSERT_EQUAL_STRING("idle", fieldStr(st, "state").c_str());
  TEST_ASSERT_EQUAL(0, (int)r.linesWith("EV HOME_FAILED", m).size());
}

void test_home_from_the_near_side() { homeFrom(1500, 0); }
void test_home_from_the_far_side() { homeFrom(-2500, 300); }
void test_home_starting_on_the_magnet() { homeFrom(10, -40); }

void test_home_without_a_magnet_fails() {
  SimRig r;
  r.magnet = false;
  enable(r);
  r.ask("HOME");
  const std::string failed = r.runUntil("EV HOME_FAILED", 10000000);
  TEST_ASSERT_EQUAL_STRING("not_found", fieldStr(failed, "reason").c_str());
  const std::string st = r.ask("STATUS");
  ASSERT_FIELD(0, st, "homed");
  TEST_ASSERT_EQUAL_STRING("idle", fieldStr(st, "state").c_str());
}

void test_home_rejects_a_window_that_is_too_wide() {
  SimRig r;
  r.rotor = 3000;
  r.half_on = 1000;
  r.half_off = 1010;
  enable(r);
  r.ask("HOME");
  const std::string failed = r.runUntil("EV HOME_FAILED", 40000000);
  TEST_ASSERT_EQUAL_STRING("too_wide", fieldStr(failed, "reason").c_str());
}

void test_stop_during_home() {
  SimRig r;
  r.rotor = 3000;
  enable(r);
  r.ask("HOME");
  r.run(300000);
  ASSERT_LINE("ERR MOVE code=busy msg=not allowed while homing", r.ask("MOVE pos=0"));
  const size_t m = r.mark();
  ASSERT_LINE("OK STOP", r.ask("STOP"));
  TEST_ASSERT_EQUAL_STRING("stopped", fieldStr(r.port.out[m + 1], "reason").c_str());
  TEST_ASSERT_TRUE(r.runUntil("EV STOPPED", 1000000).size() > 0);
  ASSERT_FIELD(0, r.ask("STATUS"), "homed");
}

// ---------------------------------------------------------------- scans

void test_stare_scan_lines_land_on_the_camera_clock() {
  SimRig r;
  enable(r);
  r.ask("MOVE pos=-100");
  r.runUntil("EV MOVED", 1000000);
  const long long t0 = r.port.now + 100000;
  const size_t m = r.mark();
  const size_t trig0 = r.trig_rises.size();
  const std::string ok =
      askf(r, "SCAN mode=stare start=-100 step=2 lines=50 period=33333.333 t0=%lld", t0);
  char expect[160];
  snprintf(expect, sizeof(expect), "OK SCAN mode=stare t0=%lld period=33333.333 start=-100 step=2 lines=50", t0);
  ASSERT_LINE(expect, ok);
  TEST_ASSERT_TRUE(r.drv.full_hold);  // full current at standstill while scanning

  const std::string done = r.runUntil("EV SCAN_DONE", 3000000, m);
  ASSERT_FIELD(50, done, "lines");
  ASSERT_FIELD(-2, done, "pos");
  ASSERT_FIELD(0, done, "aborted");
  TEST_ASSERT_FALSE(r.drv.full_hold);

  const std::vector<std::string> lines = r.linesWith("EV LINE", m);
  TEST_ASSERT_EQUAL(50, (int)lines.size());
  TEST_ASSERT_EQUAL(50, (int)(r.trig_rises.size() - trig0));
  for (int k = 0; k < 50; ++k) {
    const std::string& l = lines[k];
    ASSERT_FIELD(k, l, "n");
    ASSERT_FIELD(-100 + 2 * k, l, "pos");
    // The tick is on the first 50 us grid point at or after t0 + k * period.
    const double ideal = static_cast<double>(t0) + k * 33333.333;
    const long long t = field(l, "t");
    TEST_ASSERT_TRUE(t >= ideal - 0.5 && t < ideal + 50);
    // Line 0 is ready at once; later lines after a 2 microstep move plus the
    // 3 ms settle. TRIG pulses then, as MOVING falls.
    const long long ready = field(l, "ready");
    if (k == 0) {
      TEST_ASSERT_EQUAL_INT64(t, ready);
    } else {
      TEST_ASSERT_TRUE(ready - t >= 3000 && ready - t <= 4000);
      TEST_ASSERT_TRUE(hasEdge(r, ready, false));
    }
    TEST_ASSERT_EQUAL_INT64(ready, r.trig_rises[trig0 + k]);
  }
  TEST_ASSERT_EQUAL_STRING("idle", fieldStr(r.ask("STATUS"), "state").c_str());
}

void test_scan_start_in_the_past_keeps_the_phase() {
  SimRig r;
  enable(r);
  const long long now = r.port.now;
  const long long t0 = now - 1000000 + 123;
  const std::string ok = askf(r, "SCAN step=0 lines=3 period=33333.333 t0=%lld", t0);
  const long long first = field(ok, "t0");
  // Moved on by whole periods to at least 2 ms from now.
  TEST_ASSERT_TRUE(first >= now + 2000 && first < now + 2000 + 33334);
  const double periods = (first - t0) / 33333.333;
  TEST_ASSERT_DOUBLE_WITHIN(0.5 / 33333.333, static_cast<double>(static_cast<long long>(periods + 0.5)), periods);
  const long long t = field(r.runUntil("EV LINE", 1000000), "t");
  TEST_ASSERT_TRUE(t >= first && t < first + 50);
}

void test_scan_waits_for_the_mirror_without_losing_phase() {
  SimRig r;
  enable(r);
  const size_t m = r.mark();
  const std::string ok = r.ask("SCAN start=3000 step=-10 lines=20 period=10000");
  const long long first = field(ok, "t0");
  TEST_ASSERT_EQUAL_INT64(r.port.now + 10000, first);  // "now" is too soon: one period on
  r.runUntil("EV SCAN_DONE", 3000000, m);
  const std::vector<std::string> lines = r.linesWith("EV LINE", m);
  TEST_ASSERT_EQUAL(20, (int)lines.size());
  // The slew to 3000 takes about a second, so line 0 comes many periods
  // late, but still on the original grid.
  const long long t_line0 = field(lines[0], "t");
  TEST_ASSERT_TRUE(t_line0 - first > 900000);
  TEST_ASSERT_EQUAL_INT64(0, (t_line0 - first) % 10000);
  ASSERT_FIELD(3000, lines[0], "pos");
  for (int k = 0; k < 20; ++k) ASSERT_FIELD(k, lines[k], "n");
  ASSERT_FIELD(2810, lines[19], "pos");
}

void test_sweep_scan_runs_at_constant_speed() {
  SimRig r;
  enable(r);
  ASSERT_LINE(
      "ERR SCAN code=range msg=sweep speed 4000/s is above vstart 1600; lower step or raise period or vstart",
      r.ask("SCAN mode=sweep step=40 lines=10 period=10000"));
  const size_t m = r.mark();
  const std::string ok = r.ask("SCAN mode=sweep start=0 step=4 lines=100 period=10000 delay=5000");
  const long long t0 = field(ok, "t0");
  TEST_ASSERT_EQUAL_INT64(r.port.now + 5000, t0);
  const size_t s0 = r.step_times.size();
  const std::string done = r.runUntil("EV SCAN_DONE", 2000000, m);
  ASSERT_FIELD(100, done, "lines");
  ASSERT_FIELD(400, done, "pos");

  const std::vector<std::string> lines = r.linesWith("EV LINE", m);
  TEST_ASSERT_EQUAL(100, (int)lines.size());
  for (int k = 0; k < 100; ++k) {
    ASSERT_FIELD(k, lines[k], "n");
    ASSERT_FIELD(t0 + 10000LL * k, lines[k], "t");
    ASSERT_FIELD(4 * k, lines[k], "pos");  // the mirror is exactly on its line
    TEST_ASSERT_FALSE(hasField(lines[k], "ready"));
  }
  // 400 microsteps/s, evenly: one every 2.5 ms from half a step in.
  TEST_ASSERT_EQUAL(400, (int)(r.step_times.size() - s0));
  TEST_ASSERT_TRUE(r.step_times[s0] - t0 >= 1150 && r.step_times[s0] - t0 <= 1300);
  for (size_t i = s0 + 1; i < r.step_times.size(); ++i) {
    const int64_t gap = r.step_times[i] - r.step_times[i - 1];
    TEST_ASSERT_TRUE(gap >= 2450 && gap <= 2550);
  }
  TEST_ASSERT_TRUE(hasEdge(r, t0, true));  // MOVING is high for the whole sweep
}

void test_stop_aborts_a_scan() {
  SimRig r;
  enable(r);
  const size_t m = r.mark();
  r.ask("SCAN start=0 step=2 lines=100 period=20000 delay=10000");
  r.runUntil("EV LINE n=9 ", 1000000, m);
  const size_t m2 = r.mark();
  ASSERT_LINE("OK STOP", r.ask("STOP"));
  TEST_ASSERT_TRUE(r.runUntil("EV STOPPED", 1000000, m2).size() > 0);
  const std::vector<std::string> done = r.linesWith("EV SCAN_DONE", m2);
  TEST_ASSERT_EQUAL(1, (int)done.size());
  ASSERT_FIELD(1, done[0], "aborted");
  ASSERT_FIELD(10, done[0], "lines");
  TEST_ASSERT_FALSE(r.drv.full_hold);
  TEST_ASSERT_EQUAL_STRING("idle", fieldStr(r.ask("STATUS"), "state").c_str());
}

void test_nudge_and_period_retime_a_running_scan() {
  SimRig r;
  enable(r);
  const long long t0 = r.port.now + 10000;
  const size_t m = r.mark();
  askf(r, "SCAN step=0 lines=8 period=20000 t0=%lld", t0);
  r.runUntil("EV LINE n=2 ", 1000000, m);
  char expect[64];
  snprintf(expect, sizeof(expect), "OK NUDGE next=%lld", t0 + 60000 - 500);
  ASSERT_LINE(expect, r.ask("NUDGE dt=-500"));
  ASSERT_LINE("OK PERIOD us=25000.000", r.ask("PERIOD us=25000"));
  ASSERT_LINE("ERR PERIOD code=bad_arg msg=out of range: us", r.ask("PERIOD us=100"));
  r.runUntil("EV SCAN_DONE", 1000000, m);
  const std::vector<std::string> lines = r.linesWith("EV LINE", m);
  TEST_ASSERT_EQUAL(8, (int)lines.size());
  ASSERT_FIELD(t0 + 40000, lines[2], "t");
  ASSERT_FIELD(t0 + 59500, lines[3], "t");  // the nudge moves the next tick ...
  ASSERT_FIELD(t0 + 84500, lines[4], "t");  // ... and the new period applies after it
  ASSERT_FIELD(t0 + 159500, lines[7], "t");
  ASSERT_LINE("ERR NUDGE code=no_scan msg=no scan is running", r.ask("NUDGE dt=5"));
}

void test_sweep_period_change_is_checked_against_vstart() {
  SimRig r;
  enable(r);
  r.ask("SCAN mode=sweep step=4 lines=50 period=10000 delay=5000");
  r.run(20000);
  ASSERT_LINE("ERR PERIOD code=range msg=that period makes the sweep faster than vstart", r.ask("PERIOD us=1000"));
  ASSERT_LINE("OK PERIOD us=5000.000", r.ask("PERIOD us=5000"));
}

void test_slow_lines_are_reported_as_never_ready() {
  // 200 microsteps cannot be stepped and settled in a 5 ms line.
  SimRig r;
  enable(r);
  const size_t m = r.mark();
  r.ask("SCAN start=0 step=200 lines=5 period=5000 delay=5000");
  r.runUntil("EV SCAN_DONE", 1000000, m);
  const std::vector<std::string> lines = r.linesWith("EV LINE", m);
  TEST_ASSERT_EQUAL(5, (int)lines.size());
  ASSERT_FIELD(field(lines[0], "t"), lines[0], "ready");
  for (int k = 1; k < 5; ++k) ASSERT_FIELD(-1, lines[k], "ready");
}

void test_a_stuck_main_loop_is_reported() {
  SimRig r;
  enable(r);
  r.ask("SCAN step=0 lines=500 period=1000 delay=5000");
  for (int i = 0; i < 4000; ++i) r.tickCore();  // 200 ms with loop() stuck
  const size_t m = r.mark();
  r.tick();
  const std::vector<std::string> warn = r.linesWith("EV WARN", m);
  TEST_ASSERT_EQUAL(1, (int)warn.size());
  TEST_ASSERT_EQUAL_STRING("events_dropped", fieldStr(warn[0], "code").c_str());
  const long long n = field(warn[0], "n");
  TEST_ASSERT_TRUE(n > 100);
  ASSERT_FIELD(n, r.ask("STATUS"), "dropped");
}

void test_scan_arguments_are_checked() {
  SimRig r;
  enable(r);
  ASSERT_LINE("ERR SCAN code=bad_arg msg=lines= and period= are required", r.ask("SCAN lines=5"));
  ASSERT_LINE("ERR SCAN code=bad_arg msg=mode must be stare or sweep", r.ask("SCAN mode=zigzag lines=5 period=10000"));
  ASSERT_LINE("ERR SCAN code=bad_arg msg=give t0= or delay=, not both",
              r.ask("SCAN lines=5 period=10000 t0=5 delay=5"));
  ASSERT_LINE("ERR SCAN code=range msg=scan covers 3000..3400, outside min..max -3200..3200",
              r.ask("SCAN start=3000 step=100 lines=5 period=10000"));
  ASSERT_LINE("ERR SCAN code=bad_arg msg=unknown key 'perod'", r.ask("SCAN lines=5 perod=10000"));
  r.ask("SCAN lines=5 period=10000");
  ASSERT_LINE("ERR SCAN code=busy msg=not allowed while scanning", r.ask("SCAN lines=5 period=10000"));
  ASSERT_LINE("ERR HOME code=busy msg=not allowed while scanning", r.ask("HOME"));
}

// ---------------------------------------------------------------- settings

void test_cfg_get_set_save_and_defaults() {
  SimRig r;
  enable(r);
  ASSERT_LINE(
      "OK CFG usteps=32 irun=200 ihold=50 stealth=1 dir_inv=0 vmax=3200 vstart=1600 accel=20000 "
      "settle=3000 min=-3200 max=3200 home_pos=-711 home_park=0 home_fast=1600 home_slow=100 "
      "home_range=7200 home_clear=160 home_width=1600 hall_high=0 debounce=4 trig_us=100 scan_hold=1",
      r.ask("CFG"));
  ASSERT_LINE("OK CFG vmax=2000 accel=5000", r.ask("CFG vmax=2000 accel=5000"));
  ASSERT_LINE("ERR CFG code=bad_arg msg=usteps not a power of two", r.ask("CFG usteps=24"));
  ASSERT_LINE("ERR CFG code=bad_arg msg=vstart must not exceed vmax", r.ask("CFG vstart=2500"));
  ASSERT_LINE("ERR CFG code=bad_arg msg=unknown key 'bogus'", r.ask("CFG bogus=1"));
  ASSERT_LINE("ERR CFG code=bad_arg msg=out of range: irun", r.ask("CFG irun=5000"));
  TEST_ASSERT_EQUAL_INT32(2000, r.ctl.config().vmax);  // failed CFGs change nothing

  const int configures = r.drv.configures;
  ASSERT_LINE("OK CFG irun=300", r.ask("CFG irun=300"));
  TEST_ASSERT_EQUAL(configures + 1, r.drv.configures);  // driver settings go straight to the TMC2209

  r.ask("MOVE pos=1000");
  ASSERT_LINE("ERR CFG code=busy msg=not allowed while moving", r.ask("CFG vmax=100"));
  ASSERT_LINE("ERR SAVE code=busy msg=not allowed while moving", r.ask("SAVE"));
  r.runUntil("EV MOVED", 2000000);

  ASSERT_LINE("OK SAVE", r.ask("SAVE"));
  TEST_ASSERT_TRUE(r.port.has_stored);
  TEST_ASSERT_EQUAL_INT32(2000, r.port.stored.vmax);
  ASSERT_LINE("OK DEFAULTS", r.ask("DEFAULTS"));
  TEST_ASSERT_EQUAL_INT32(3200, r.ctl.config().vmax);

  // After a reboot the saved settings are back; a bad saved copy is ignored.
  SimRig r2;
  r2.port.stored = r.port.stored;
  r2.port.has_stored = true;
  r2.boot();
  TEST_ASSERT_EQUAL_INT32(2000, r2.ctl.config().vmax);
  TEST_ASSERT_EQUAL_INT32(300, r2.ctl.config().irun);
  SimRig r3;
  r3.port.stored = r.port.stored;
  r3.port.stored.version = 99;
  r3.port.has_stored = true;
  r3.boot();
  TEST_ASSERT_EQUAL_INT32(3200, r3.ctl.config().vmax);
}

void test_cfg_reply_comes_before_the_fault_it_causes() {
  SimRig r;
  enable(r);
  r.drv.present = false;
  const size_t m = r.mark();
  r.cmd("CFG irun=150 id=4");
  TEST_ASSERT_EQUAL(m + 2, r.mark());
  ASSERT_LINE("ERR CFG code=driver id=4 msg=settings kept, but the TMC2209 did not answer", r.port.out[m]);
  TEST_ASSERT_EQUAL(0, (int)r.port.out[m + 1].find("EV FAULT code=drv_noresp"));
  TEST_ASSERT_EQUAL_INT32(150, r.ctl.config().irun);
  TEST_ASSERT_FALSE(r.drv.enabled);
}

// ---------------------------------------------------------------- driver

void test_driver_faults_switch_the_motor_off() {
  SimRig r;
  enable(r);
  r.drv.st.drv_status |= sm::kDrvOt;
  const std::string f = r.runUntil("EV FAULT", 1000000);
  TEST_ASSERT_EQUAL_STRING("drv_ot", fieldStr(f, "code").c_str());
  TEST_ASSERT_FALSE(r.drv.enabled);
  const std::string st = r.ask("STATUS");
  TEST_ASSERT_EQUAL_STRING("fault", fieldStr(st, "state").c_str());
  TEST_ASSERT_EQUAL_STRING("drv_ot", fieldStr(st, "fault").c_str());
  ASSERT_LINE("ERR MOVE code=disabled msg=not allowed while fault", r.ask("MOVE pos=10"));

  r.drv.st.drv_status &= ~sm::kDrvOt;
  ASSERT_LINE("OK ENABLE drv=ok", r.ask("ENABLE"));
  TEST_ASSERT_EQUAL_STRING("none", fieldStr(r.ask("STATUS"), "fault").c_str());

  // A driver reset means it lost power and forgot its settings.
  r.drv.st.gstat = sm::kGstatReset;
  TEST_ASSERT_EQUAL_STRING("drv_reset", fieldStr(r.runUntil("EV FAULT", 1000000), "code").c_str());
  ASSERT_LINE("OK ENABLE drv=ok", r.ask("ENABLE"));
  r.run(1000000);
  TEST_ASSERT_TRUE(r.drv.enabled);

  // Overheating first only warns, once.
  r.drv.st.drv_status |= sm::kDrvOtpw;
  const size_t m = r.mark();
  r.run(2000000);
  const std::vector<std::string> warn = r.linesWith("EV WARN", m);
  TEST_ASSERT_EQUAL(1, (int)warn.size());
  TEST_ASSERT_EQUAL_STRING("drv_otpw", fieldStr(warn[0], "code").c_str());
  TEST_ASSERT_TRUE(r.drv.enabled);
}

void test_lost_driver_and_bypass() {
  SimRig r;
  enable(r);
  r.drv.present = false;
  TEST_ASSERT_EQUAL_STRING("drv_noresp", fieldStr(r.runUntil("EV FAULT", 1000000), "code").c_str());
  TEST_ASSERT_EQUAL(0, r.ask("ENABLE").find("ERR ENABLE code=driver msg=TMC2209 did not answer"));
  // force=1 runs on the VREF current setting without the UART.
  ASSERT_LINE("OK ENABLE drv=bypass", r.ask("ENABLE force=1"));
  TEST_ASSERT_TRUE(r.drv.enabled);
  r.ask("MOVE pos=20");
  ASSERT_FIELD(20, r.runUntil("EV MOVED", 1000000), "pos");
  TEST_ASSERT_EQUAL_STRING("bypass", fieldStr(r.ask("STATUS"), "drv").c_str());
}

void test_fault_during_a_scan() {
  SimRig r;
  enable(r);
  const size_t m = r.mark();
  r.ask("SCAN start=0 step=1 lines=100 period=20000 delay=5000");
  r.runUntil("EV LINE n=3 ", 1000000, m);
  r.drv.st.drv_status |= sm::kDrvS2g;
  const size_t m2 = r.mark();
  TEST_ASSERT_TRUE(r.runUntil("EV FAULT", 1000000, m2).size() > 0);
  r.run(100000);
  TEST_ASSERT_EQUAL(1, (int)r.linesWith("EV STOPPED", m2).size());
  const std::vector<std::string> done = r.linesWith("EV SCAN_DONE", m2);
  TEST_ASSERT_EQUAL(1, (int)done.size());
  ASSERT_FIELD(1, done[0], "aborted");
  TEST_ASSERT_FALSE(r.drv.enabled);
  TEST_ASSERT_EQUAL_STRING("fault", fieldStr(r.ask("STATUS"), "state").c_str());
}

// ---------------------------------------------------------------- protocol

// Random command lines, good and bad, at random moments: every one gets
// exactly one reply, and the ids come back on the right replies.
void test_every_command_gets_exactly_one_reply() {
  static const char* const kVerbs[] = {"INFO", "PING", "STATUS", "ENABLE", "DISABLE", "HOME", "MOVE",
                                       "STOP", "SCAN", "NUDGE", "PERIOD", "ZERO", "CFG", "DRV",
                                       "DEFAULTS", "HELP", "FROB", "move", "scan"};
  static const char* const kKeys[] = {"pos", "rel", "v", "a", "lines", "period", "step", "start", "mode",
                                      "t0", "delay", "settle", "dt", "us", "force", "vmax", "accel",
                                      "settle", "min", "max", "usteps", "irun", "bogus"};
  static const char* const kValues[] = {"0", "1", "-1", "5", "40", "-300", "3200", "99999", "10000",
                                        "33333.333", "1.5", "stare", "sweep", "x", "", "-9999999999999"};
  uint32_t seed = 12345;
  const auto rnd = [&seed](uint32_t n) {
    seed ^= seed << 13;
    seed ^= seed >> 17;
    seed ^= seed << 5;
    return seed % n;
  };
  SimRig r;
  r.rotor = 900;
  enable(r);
  for (int i = 0; i < 1500; ++i) {
    std::string line = kVerbs[rnd(sizeof(kVerbs) / sizeof(kVerbs[0]))];
    const uint32_t n = rnd(4);
    for (uint32_t k = 0; k < n; ++k) {
      line += std::string(" ") + kKeys[rnd(sizeof(kKeys) / sizeof(kKeys[0]))] + "=" +
              kValues[rnd(sizeof(kValues) / sizeof(kValues[0]))];
    }
    if (rnd(8) == 0) line += " oops";
    char id[16];
    snprintf(id, sizeof(id), "%d", i);
    line += std::string(" id=") + id;
    const size_t m = r.mark();
    r.cmd(line.c_str());
    int replies = 0;
    for (size_t k = m; k < r.mark(); ++k) {
      const std::string& l = r.port.out[k];
      if (l.compare(0, 3, "OK ") == 0 || l.compare(0, 4, "ERR ") == 0) {
        ++replies;
        TEST_ASSERT_EQUAL_STRING_MESSAGE(id, fieldStr(l, "id").c_str(), l.c_str());
      }
    }
    TEST_ASSERT_EQUAL_MESSAGE(1, replies, line.c_str());
    r.run(static_cast<int64_t>(rnd(4000)) * sm::kTickUs);
    if (rnd(50) == 0) r.ask("ENABLE");
  }
  // However that left it, STOP brings it to rest.
  const size_t m = r.mark();
  r.ask("STOP");
  TEST_ASSERT_TRUE(r.runUntil("EV STOPPED", 2000000, m).size() > 0);
  const std::string state = fieldStr(r.ask("STATUS"), "state");
  TEST_ASSERT_TRUE(state == "idle" || state == "disabled" || state == "fault");
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_boot_announces_itself_and_stays_disabled);
  RUN_TEST(test_motion_needs_enable);
  RUN_TEST(test_errors_name_the_verb_and_echo_the_id);
  RUN_TEST(test_help_drv_and_reboot);
  RUN_TEST(test_move_arrives_settles_and_reports);
  RUN_TEST(test_new_move_replaces_the_old_one);
  RUN_TEST(test_stop_slows_down_and_reports);
  RUN_TEST(test_disable_stops_dead);
  RUN_TEST(test_zero_and_dir_invert);
  RUN_TEST(test_home_from_the_near_side);
  RUN_TEST(test_home_from_the_far_side);
  RUN_TEST(test_home_starting_on_the_magnet);
  RUN_TEST(test_home_without_a_magnet_fails);
  RUN_TEST(test_home_rejects_a_window_that_is_too_wide);
  RUN_TEST(test_stop_during_home);
  RUN_TEST(test_stare_scan_lines_land_on_the_camera_clock);
  RUN_TEST(test_scan_start_in_the_past_keeps_the_phase);
  RUN_TEST(test_scan_waits_for_the_mirror_without_losing_phase);
  RUN_TEST(test_sweep_scan_runs_at_constant_speed);
  RUN_TEST(test_stop_aborts_a_scan);
  RUN_TEST(test_nudge_and_period_retime_a_running_scan);
  RUN_TEST(test_sweep_period_change_is_checked_against_vstart);
  RUN_TEST(test_slow_lines_are_reported_as_never_ready);
  RUN_TEST(test_a_stuck_main_loop_is_reported);
  RUN_TEST(test_scan_arguments_are_checked);
  RUN_TEST(test_cfg_get_set_save_and_defaults);
  RUN_TEST(test_cfg_reply_comes_before_the_fault_it_causes);
  RUN_TEST(test_driver_faults_switch_the_motor_off);
  RUN_TEST(test_lost_driver_and_bypass);
  RUN_TEST(test_fault_during_a_scan);
  RUN_TEST(test_every_command_gets_exactly_one_reply);
  return UNITY_END();
}
