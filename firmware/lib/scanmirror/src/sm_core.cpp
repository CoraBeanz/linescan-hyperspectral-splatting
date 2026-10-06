#include "sm_core.h"

namespace sm {

void MirrorCore::setHall(bool active_high, uint16_t debounce_ticks) {
  if (active_high != hall_active_high_) hall_valid_ = false;  // re-read the level
  hall_active_high_ = active_high;
  hall_debounce_ = debounce_ticks == 0 ? 1 : debounce_ticks;
}

void MirrorCore::move(int32_t target, const Profile& p, uint16_t tag) {
  sg_.moveTo(target, p);
  move_tag_ = tag;
}

void MirrorCore::stop(uint16_t tag) {
  sg_.stop();
  move_tag_ = tag;
  if (scanning_) abort_ = true;
}

void MirrorCore::halt() {
  sg_.halt();
  move_tag_ = 0;
  if (scanning_) abort_ = true;
}

bool MirrorCore::setPosition(int32_t pos) {
  if (scanning_) return false;
  return sg_.setPosition(pos);
}

bool MirrorCore::startScan(const ScanParams& s) {
  if (scanning_) return false;
  scan_ = s;
  scanning_ = true;
  started_ = false;
  abort_ = false;
  pending_ = false;
  lines_done_ = 0;
  move_tag_ = 0;
  lc_.start(s.t0_us, s.period_q16, s.lines + 1);
  if (sg_.busy() || sg_.position() != s.start) sg_.moveTo(s.start, s.slew);
  return true;
}

bool MirrorCore::nudge(int32_t dt_us) {
  if (!scanning_) return false;
  lc_.nudge(dt_us);
  return true;
}

bool MirrorCore::setPeriod(uint64_t period_q16, uint32_t sweep_v) {
  if (!scanning_) return false;
  lc_.setPeriod(period_q16);
  scan_.period_q16 = period_q16;
  if (scan_.mode == ScanMode::kSweep) {
    scan_.sweep_v = sweep_v;
    if (started_) sg_.setVelocity(sweep_v);
  }
  return true;
}

CoreSnapshot MirrorCore::snapshot() const {
  CoreSnapshot s;
  s.pos = sg_.position();
  s.target = sg_.target();
  s.busy = sg_.busy();
  s.settled = !s.busy && settled(now_, settle_us_);
  s.hall = hall_;
  s.hall_valid = hall_valid_;
  s.scanning = scanning_;
  s.started = started_;
  s.sweep = scan_.mode == ScanMode::kSweep;
  s.lines_done = lines_done_;
  s.lines = scan_.lines;  // of the running or the last scan
  s.next_tick_us = scanning_ ? lc_.nextUs() : -1;
  s.now_us = now_;
  return s;
}

void SM_ISR MirrorCore::post(uint8_t type, uint8_t flag, uint16_t tag, int32_t n, int32_t pos,
                             int64_t t, int64_t t2) {
  Event e;
  e.type = type;
  e.flag = flag;
  e.tag = tag;
  e.n = n;
  e.pos = pos;
  e.t = t;
  e.t2 = t2;
  events_.push(e);
}

// The reading belongs to the position before this tick's step. On a change,
// remember where the first sample of the new level was taken, and report
// that once the level has held for hall_debounce_ ticks.
void SM_ISR MirrorCore::sampleHall(int64_t now, bool raw) {
  const bool active = hall_active_high_ ? raw : !raw;
  if (!hall_valid_) {
    hall_ = active;
    hall_valid_ = true;
    hall_count_ = 0;
    return;
  }
  if (active == hall_) {
    hall_count_ = 0;
    return;
  }
  if (hall_count_ == 0) {
    hall_pos_ = sg_.position();
    hall_t_ = now;
  }
  if (++hall_count_ >= hall_debounce_) {
    hall_ = active;
    hall_count_ = 0;
    post(kEvHall, active ? 1 : 0, 0, 0, hall_pos_, hall_t_, 0);
  }
}

void SM_ISR MirrorCore::reportPending(int64_t ready) {
  post(kEvLine, ready >= 0 ? 1 : 0, 0, static_cast<int32_t>(pend_n_), sg_.position(), pend_t_, ready);
  pending_ = false;
  lines_done_++;
  if (ready >= 0 && trig_us_ > 0) trig_until_ = ready + trig_us_;
}

void SM_ISR MirrorCore::finishScan(int64_t now, bool aborted) {
  if (pending_) reportPending(-1);
  if (scan_.mode == ScanMode::kSweep && started_) sg_.stop();
  lc_.stop();
  scanning_ = false;
  started_ = false;
  abort_ = false;
  post(kEvScanDone, aborted ? 1 : 0, 0, static_cast<int32_t>(lines_done_), sg_.position(), now, 0);
}

void SM_ISR MirrorCore::lineTick(uint32_t idx, int64_t now) {
  if (scan_.mode == ScanMode::kSweep) {
    if (idx >= scan_.lines) {
      finishScan(now, false);
      return;
    }
    if (idx == 0) {
      // Half a step of preloaded phase rounds the position to the nearest
      // microstep of the ideal line start + v * (t - t_line0).
      sg_.runVelocity(scan_.step < 0 ? -1 : 1, scan_.sweep_v, 0x80000000u, scan_.sweep_limit, scan_.line);
    }
    post(kEvLine, 0, 0, static_cast<int32_t>(idx), sg_.position(), now, -1);
    lines_done_++;
    if (trig_us_ > 0) trig_until_ = now + trig_us_;
    return;
  }

  // Stare: a line that never settled before the next tick is reported now.
  if (pending_) reportPending(-1);
  if (idx >= scan_.lines) {
    finishScan(now, false);
    return;
  }
  if (idx > 0 && scan_.step != 0) {
    const int64_t target = static_cast<int64_t>(scan_.start) + static_cast<int64_t>(idx) * scan_.step;
    sg_.moveTo(static_cast<int32_t>(target), scan_.line);
  }
  pending_ = true;
  pend_n_ = idx;
  pend_t_ = now;
}

TickOut SM_ISR MirrorCore::tick(int64_t now, bool hall_raw) {
  now_ = now;
  sampleHall(now, hall_raw);

  if (scanning_) {
    if (abort_) {
      finishScan(now, true);
    } else if (lc_.due(now)) {
      if (started_) {
        lineTick(lc_.take(), now);
      } else if (sg_.busy() || sg_.position() != scan_.start || !settled(now, scan_.settle_us)) {
        // Not at the start yet: let this tick pass, a whole period later.
        if (!sg_.busy() && sg_.position() != scan_.start) sg_.moveTo(scan_.start, scan_.slew);
        lc_.defer();
      } else {
        started_ = true;
        lineTick(lc_.take(), now);
      }
    }
  }

  TickOut out;
  out.step = sg_.tick();
  if (out.step != 0) last_step_us_ = now;
  // During a sweep the generator stops by itself only on its limit, where
  // NUDGE or PERIOD stretched the sweep to the edge of min..max.
  if (scanning_ && started_ && scan_.mode == ScanMode::kSweep && !sg_.busy()) finishScan(now, true);

  const bool stare = scanning_ && scan_.mode == ScanMode::kStare;
  const uint32_t settle = stare ? scan_.settle_us : settle_us_;
  if (pending_ && !sg_.busy() && settled(now, settle)) reportPending(now);

  if (move_tag_ != 0 && !sg_.busy() && settled(now, settle_us_)) {
    post(kEvMoveDone, 0, move_tag_, 0, sg_.position(), now, 0);
    move_tag_ = 0;
  }

  const bool sweeping = scanning_ && started_ && scan_.mode == ScanMode::kSweep;
  out.moving = sweeping || sg_.busy() || !settled(now, settle);
  out.trig = now < trig_until_;
  return out;
}

}  // namespace sm
