// Step generator and line clock: exact arrival, speed and acceleration limits,
// replanning, constant-velocity sweeps, and drift-free fractional periods.
#include <unity.h>

#include <vector>

#include "sm_common.h"
#include "sm_line_clock.h"
#include "sm_stepgen.h"

using sm::Profile;
using sm::Stepgen;

void setUp() {}
void tearDown() {}

static Profile prof(double vmax, double vstart, double accel) {
  Profile p;
  p.v_max = sm::velocityQ32(vmax);
  p.v_start = sm::velocityQ32(vstart);
  p.accel = sm::accelQ32(accel);
  return p;
}

// Run until idle (or max_ticks); returns the tick index of every step.
static std::vector<long> runSteps(Stepgen& g, long max_ticks, std::vector<int>* dirs = nullptr) {
  std::vector<long> t;
  for (long i = 0; i < max_ticks && g.busy(); ++i) {
    const int8_t s = g.tick();
    if (s) {
      t.push_back(i);
      if (dirs) dirs->push_back(s);
    }
  }
  return t;
}

void test_reaches_targets_exactly() {
  const int32_t targets[] = {1, 2, 3, 17, 200, 1000, 6400, -1, -50, -3200};
  for (size_t i = 0; i < sizeof(targets) / sizeof(targets[0]); ++i) {
    Stepgen g;
    g.moveTo(targets[i], prof(3200, 1600, 20000));
    runSteps(g, 2000000);
    TEST_ASSERT_FALSE(g.busy());
    TEST_ASSERT_EQUAL_INT32(targets[i], g.position());
  }
}

void test_first_step_is_immediate() {
  Stepgen g;
  g.moveTo(1, prof(3200, 1600, 20000));
  TEST_ASSERT_EQUAL_INT8(1, g.tick());  // a one-microstep line move happens on the line tick itself
  TEST_ASSERT_FALSE(g.busy());
  TEST_ASSERT_EQUAL_INT32(1, g.position());
}

void test_short_moves_run_at_start_speed() {
  // 4 microsteps at vstart 1600/s: one every 12.5 ticks, no ramp.
  Stepgen g;
  g.moveTo(4, prof(3200, 1600, 20000));
  const std::vector<long> t = runSteps(g, 1000);
  TEST_ASSERT_EQUAL(4, (int)t.size());
  TEST_ASSERT_EQUAL(0, (int)t[0]);
  TEST_ASSERT_TRUE(t[3] <= 38);
}

void test_speed_never_exceeds_vmax() {
  Stepgen g;
  g.moveTo(5000, prof(3200, 400, 20000));
  const std::vector<long> t = runSteps(g, 2000000);
  TEST_ASSERT_EQUAL(5000, (int)t.size());
  // 3200 steps/s = one per 6.25 ticks; the DDA spreads steps to within a tick.
  for (size_t i = 1; i < t.size(); ++i) TEST_ASSERT_TRUE(t[i] - t[i - 1] >= 6);
  // Average over the cruise phase is right on 6.25 ticks/step.
  const double cruise = (t[3000] - t[2000]) / 1000.0;
  TEST_ASSERT_DOUBLE_WITHIN(0.02, 6.25, cruise);
}

void test_ramp_takes_expected_time() {
  // 0 -> 3200/s at 20000/s^2 takes 0.16 s; a 5000 step move then takes
  // 2 * 0.16 + (5000 - 512) / 3200 = 1.7225 s = 34450 ticks. The first step
  // is taken at once, so the last one lands as the ideal profile passes 4999,
  // sqrt(2 / 20000) s = 200 ticks before its end.
  Stepgen g;
  g.moveTo(5000, prof(3200, 1, 20000));
  const std::vector<long> t = runSteps(g, 2000000);
  TEST_ASSERT_INT_WITHIN(20, 34250, (int)t.back());
}

void test_reverse_mid_move() {
  Stepgen g;
  const Profile p = prof(3200, 400, 20000);
  g.moveTo(3000, p);
  for (int i = 0; i < 4000; ++i) g.tick();  // well into the move
  const int32_t at = g.position();
  TEST_ASSERT_TRUE(at > 100 && at < 3000);
  g.moveTo(-200, p);
  std::vector<int> dirs;
  runSteps(g, 2000000, &dirs);
  TEST_ASSERT_EQUAL_INT32(-200, g.position());
  // Braking overshoot past `at` stays under the stopping distance at 3200/s (256 steps).
  int32_t pos = at, peak = at;
  for (size_t i = 0; i < dirs.size(); ++i) {
    pos += dirs[i];
    if (pos > peak) peak = pos;
  }
  TEST_ASSERT_TRUE(peak - at <= 260);
}

void test_stop_brakes_and_halt_is_instant() {
  Stepgen g;
  g.moveTo(100000, prof(3200, 400, 20000));
  for (int i = 0; i < 20000; ++i) g.tick();
  const int32_t at = g.position();
  g.stop();
  runSteps(g, 2000000);
  TEST_ASSERT_FALSE(g.busy());
  TEST_ASSERT_TRUE(g.position() - at <= 260);
  TEST_ASSERT_TRUE(g.position() - at >= 100);  // it really slowed down rather than stopping dead

  Stepgen h;
  h.moveTo(100000, prof(3200, 400, 20000));
  for (int i = 0; i < 20000; ++i) h.tick();
  const int32_t h_at = h.position();
  h.halt();
  TEST_ASSERT_FALSE(h.busy());
  for (int i = 0; i < 100; ++i) TEST_ASSERT_EQUAL_INT8(0, h.tick());
  TEST_ASSERT_EQUAL_INT32(h_at, h.position());
}

void test_velocity_mode_is_a_straight_line() {
  // 30 microsteps/s, preloaded half a step: position = round(v * t).
  Stepgen g;
  const double rate = 30.0;
  g.runVelocity(1, sm::velocityQ32(rate), 0x80000000u, prof(3200, 1600, 20000));
  for (long i = 1; i <= 400000; ++i) {  // 20 s
    g.tick();
    if (i % 997 == 0) {
      const double ideal = rate * i * sm::kTickUs * 1e-6;
      TEST_ASSERT_DOUBLE_WITHIN(0.51, ideal, g.position());
    }
  }
  g.stop();
  runSteps(g, 10);
  TEST_ASSERT_FALSE(g.busy());
}

void test_set_position_only_when_idle() {
  Stepgen g;
  TEST_ASSERT_TRUE(g.setPosition(-711));
  TEST_ASSERT_EQUAL_INT32(-711, g.position());
  g.moveTo(0, prof(3200, 1600, 20000));
  g.tick();
  TEST_ASSERT_FALSE(g.setPosition(5));
}

void test_zero_length_move_finishes() {
  Stepgen g;
  g.moveTo(0, prof(3200, 1600, 20000));
  TEST_ASSERT_TRUE(g.busy());
  TEST_ASSERT_EQUAL_INT8(0, g.tick());
  TEST_ASSERT_FALSE(g.busy());
}

void test_line_clock_fractional_period_has_no_drift() {
  // 30 fps camera period: 33333.333 us. After 10000 ticks the schedule is
  // within a microsecond of t0 + k * period.
  sm::LineClock lc;
  const uint64_t period = 33333ull * 65536 + (333ull * 65536 + 500) / 1000;
  const int64_t t0 = 5000000;
  lc.start(t0, period, 10001);
  int64_t now = t0 - 1000;
  uint32_t seen = 0;
  while (lc.active()) {
    now += sm::kTickUs;
    if (lc.due(now)) {
      const uint32_t k = lc.take();
      TEST_ASSERT_EQUAL_UINT32(seen, k);
      ++seen;
      const double ideal = t0 + k * 33333.333;
      TEST_ASSERT_TRUE(now >= ideal - 1.0);
      TEST_ASSERT_TRUE(now < ideal + sm::kTickUs + 1.0);
    }
  }
  TEST_ASSERT_EQUAL_UINT32(10001, seen);
}

void test_line_clock_nudge_defer_and_period() {
  sm::LineClock lc;
  lc.start(1000, 10000ull * 65536, 5);
  TEST_ASSERT_TRUE(lc.due(1000));
  lc.defer();  // skip a tick without counting it
  TEST_ASSERT_EQUAL_INT64(11000, lc.nextUs());
  TEST_ASSERT_EQUAL_UINT32(0, lc.take());
  lc.nudge(-2500);
  TEST_ASSERT_EQUAL_INT64(18500, lc.nextUs());
  lc.setPeriod(20000ull * 65536);
  TEST_ASSERT_EQUAL_UINT32(1, lc.take());
  TEST_ASSERT_EQUAL_INT64(38500, lc.nextUs());
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_reaches_targets_exactly);
  RUN_TEST(test_first_step_is_immediate);
  RUN_TEST(test_short_moves_run_at_start_speed);
  RUN_TEST(test_speed_never_exceeds_vmax);
  RUN_TEST(test_ramp_takes_expected_time);
  RUN_TEST(test_reverse_mid_move);
  RUN_TEST(test_stop_brakes_and_halt_is_instant);
  RUN_TEST(test_velocity_mode_is_a_straight_line);
  RUN_TEST(test_set_position_only_when_idle);
  RUN_TEST(test_zero_length_move_finishes);
  RUN_TEST(test_line_clock_fractional_period_has_no_drift);
  RUN_TEST(test_line_clock_nudge_defer_and_period);
  return UNITY_END();
}
