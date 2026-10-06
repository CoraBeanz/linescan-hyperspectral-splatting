// MirrorCore: everything that runs in the timer interrupt. It owns the step
// generator, the line clock and the hall-sensor debouncer, and reports what
// happened through an event queue the main loop drains.
#pragma once

#include <stdint.h>

#include "sm_common.h"
#include "sm_event_queue.h"
#include "sm_line_clock.h"
#include "sm_stepgen.h"

namespace sm {

enum class ScanMode : uint8_t { kStare = 0, kSweep = 1 };

struct ScanParams {
  ScanMode mode;
  int32_t start;        // mirror position of line 0, microsteps
  int32_t step;         // microsteps per line (may be 0 or negative)
  uint32_t lines;       // number of lines
  int64_t t0_us;        // device time of line 0's tick
  uint64_t period_q16;  // line period, Q16 microseconds
  uint32_t settle_us;   // stare: last step to "ready"
  uint32_t sweep_v;     // sweep: |step| / period as Q32 microsteps per tick
  Profile slew;         // move to `start` before line 0
  Profile line;         // stare: moves between lines; sweep: start speed
};

enum EventType : uint8_t {
  kEvLine = 1,      // n, pos, t = line tick, t2 = ready (stare; -1 = never settled)
  kEvScanDone = 2,  // n = lines reported, flag = 1 if aborted, pos, t
  kEvMoveDone = 3,  // tag, pos, t = settled
  kEvHall = 4,      // flag = 1 magnet present, pos and t of the first sample in the new state
};

struct Event {
  uint8_t type;
  uint8_t flag;
  uint16_t tag;
  int32_t n;
  int32_t pos;
  int64_t t;
  int64_t t2;
};

struct TickOut {
  int8_t step;  // +1 or -1: pulse STEP in that direction now; 0: no step
  bool moving;  // MOVING output: high while the mirror moves or settles
  bool trig;    // TRIG output: a short pulse when a stare line is ready
};

struct CoreSnapshot {
  int32_t pos;
  int32_t target;
  bool busy;        // step generator running
  bool settled;     // stopped for at least the settle time
  bool hall;        // debounced: true = magnet at the sensor
  bool hall_valid;  // false until the first sample
  bool scanning;    // a scan is running (including the move to its start)
  bool started;     // line 0 has happened
  bool sweep;
  uint32_t lines_done;
  uint32_t lines;
  int64_t next_tick_us;
  int64_t now_us;  // time of the latest tick
};

class MirrorCore {
 public:
  // ---- main loop, with the interrupt locked out ----
  void setSettle(uint32_t settle_us) { settle_us_ = settle_us; }
  void setHall(bool active_high, uint16_t debounce_ticks);
  void setTrigWidth(uint32_t trig_us) { trig_us_ = trig_us; }

  // Move to target. When it has stopped and settled, a kEvMoveDone event
  // carries `tag` (non-zero). A new move replaces the old one and its tag.
  void move(int32_t target, const Profile& p, uint16_t tag);
  // Slow down and stop, aborting any scan; kEvMoveDone(tag) follows.
  void stop(uint16_t tag);
  // Stop dead (motor disable, faults). Aborts a scan; no kEvMoveDone.
  void halt();
  bool setPosition(int32_t pos);

  // Start a scan: move to s.start, then one line per tick from s.t0_us on
  // (ticks that come before the mirror has settled at the start are skipped
  // in whole periods, which keeps the phase). Fails if a scan is running.
  bool startScan(const ScanParams& s);
  bool nudge(int32_t dt_us);
  bool setPeriod(uint64_t period_q16, uint32_t sweep_v);

  CoreSnapshot snapshot() const;

  // ---- timer interrupt ----
  TickOut tick(int64_t now_us, bool hall_raw);

  // ---- main loop, no lock needed ----
  bool popEvent(Event* e) { return events_.pop(e); }
  uint32_t droppedEvents() const { return events_.dropped(); }

 private:
  void sampleHall(int64_t now, bool raw);
  void lineTick(uint32_t idx, int64_t now);
  void finishScan(int64_t now, bool aborted);
  void reportPending(int64_t ready);
  bool settled(int64_t now, uint32_t settle) const {
    return now - last_step_us_ >= static_cast<int64_t>(settle);
  }
  void post(uint8_t type, uint8_t flag, uint16_t tag, int32_t n, int32_t pos, int64_t t, int64_t t2);

  Stepgen sg_;
  LineClock lc_;
  EventQueue<Event, 64> events_;

  uint32_t settle_us_ = 3000;
  uint32_t trig_us_ = 100;
  int64_t now_ = 0;
  int64_t last_step_us_ = -1000000000LL;  // "long ago", so the mirror starts out settled
  int64_t trig_until_ = 0;
  uint16_t move_tag_ = 0;

  bool hall_active_high_ = false;
  uint16_t hall_debounce_ = 4;
  bool hall_valid_ = false;
  bool hall_ = false;
  uint16_t hall_count_ = 0;
  int32_t hall_pos_ = 0;
  int64_t hall_t_ = 0;

  ScanParams scan_ = {};
  bool scanning_ = false;
  bool started_ = false;
  bool abort_ = false;
  uint32_t lines_done_ = 0;
  bool pending_ = false;  // a stare line waiting to settle
  uint32_t pend_n_ = 0;
  int64_t pend_t_ = 0;
};

}  // namespace sm
