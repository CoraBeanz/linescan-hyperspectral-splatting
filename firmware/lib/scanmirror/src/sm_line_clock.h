// Line clock: the schedule of scan-line ticks, in device microseconds.
#pragma once

#include <stdint.h>

#include "sm_common.h"

namespace sm {

// Times are kept in Q16 microseconds so a fractional period (a camera's
// 33333.333 us) accumulates without drift. The clock delivers `ticks` ticks,
// t0, t0 + period, ... ; MirrorCore uses N + 1 ticks for an N-line scan, the
// last one marking the end of the final line.
class LineClock {
 public:
  void start(int64_t t0_us, uint64_t period_q16, uint32_t ticks) {
    next_q16_ = t0_us * static_cast<int64_t>(kQ16);
    period_q16_ = period_q16;
    idx_ = 0;
    ticks_ = ticks;
    active_ = ticks > 0;
  }
  void stop() { active_ = false; }
  bool active() const { return active_; }

  bool due(int64_t now_us) const {
    return active_ && now_us * static_cast<int64_t>(kQ16) >= next_q16_;
  }
  // Consume the due tick and return its index.
  uint32_t take() {
    const uint32_t i = idx_++;
    next_q16_ += static_cast<int64_t>(period_q16_);
    if (idx_ >= ticks_) active_ = false;
    return i;
  }
  // Skip the due tick by one period without counting it (used while the
  // mirror is still getting to the start of a scan; keeps the phase).
  void defer() { next_q16_ += static_cast<int64_t>(period_q16_); }
  // Shift every remaining tick by dt_us (positive = later).
  void nudge(int32_t dt_us) { next_q16_ += static_cast<int64_t>(dt_us) * static_cast<int64_t>(kQ16); }
  // New period from the next tick on; the next tick itself doesn't move.
  void setPeriod(uint64_t period_q16) { period_q16_ = period_q16; }

  uint32_t index() const { return idx_; }
  uint32_t ticks() const { return ticks_; }
  uint64_t period() const { return period_q16_; }
  int64_t nextUs() const { return next_q16_ / static_cast<int64_t>(kQ16); }

 private:
  int64_t next_q16_ = 0;
  uint64_t period_q16_ = 0;
  uint32_t idx_ = 0;
  uint32_t ticks_ = 0;
  bool active_ = false;
};

}  // namespace sm
