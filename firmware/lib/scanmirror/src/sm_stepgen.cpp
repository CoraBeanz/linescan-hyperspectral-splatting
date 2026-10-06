#include "sm_stepgen.h"

namespace sm {

namespace {
// A move stops dead on its target only if braking to the start speed would
// take fewer steps than this.
constexpr uint32_t kStopDeadSteps = 4;
}  // namespace

void Stepgen::setProfile(const Profile& p) {
  prof_ = p;
  if (prof_.v_max == 0) prof_.v_max = 1;
  if (prof_.v_start == 0) prof_.v_start = 1;
  if (prof_.v_start > prof_.v_max) prof_.v_start = prof_.v_max;
  if (prof_.accel == 0) prof_.accel = 1;
}

void Stepgen::moveTo(int32_t target, const Profile& p) {
  setProfile(p);
  target_ = target;
  // Under way, keep braking at least as hard as the move being replaced
  // would have. From rest, tick() takes this move's own rate.
  if (v_ != 0) {
    if (brake_a_ < prof_.accel) brake_a_ = prof_.accel;
    if (brake_vs_ < prof_.v_start) brake_vs_ = prof_.v_start;
  }
  // From any mode: tick() brakes first if we are heading the wrong way.
  mode_ = kPosition;
}

void Stepgen::runVelocity(int8_t dir, uint32_t v, uint32_t phase, int32_t limit, const Profile& p) {
  setProfile(p);
  brake_a_ = prof_.accel;
  brake_vs_ = prof_.v_start;
  dir_ = dir < 0 ? -1 : 1;
  limit_ = limit;
  v_cmd_ = v < prof_.v_start ? v : prof_.v_start;
  v_ = v_cmd_;
  acc_ = phase;
  mode_ = kVelocity;
}

void Stepgen::setVelocity(uint32_t v) {
  if (mode_ == kVelocity) v_cmd_ = v < prof_.v_start ? v : prof_.v_start;
}

void Stepgen::stop() {
  if (mode_ != kIdle) mode_ = kStopping;
}

void Stepgen::halt() {
  v_ = 0;
  acc_ = 0;
  target_ = pos_;
  mode_ = kIdle;
}

bool Stepgen::setPosition(int32_t pos) {
  if (mode_ != kIdle) return false;
  pos_ = pos;
  target_ = pos;
  return true;
}

// True when the distance left is no more than what it takes to slow from the
// current speed down to the start speed: (v^2 - vs^2) / (2a) >= dist. Every
// term is Q32, hence the shift by 32.
bool SM_ISR Stepgen::needDecel(uint32_t dist) const {
  if (v_ <= brake_vs_) return false;
  const uint64_t vv = static_cast<uint64_t>(v_) * v_ - static_cast<uint64_t>(brake_vs_) * brake_vs_;
  return (vv >> 32) >= 2ull * brake_a_ * dist;
}

// Slow down by one acceleration step. Returns false (and leaves the generator
// at rest) once the speed is down to the start speed, which a stepper can
// stop from at once.
bool SM_ISR Stepgen::brake() {
  if (v_ <= brake_vs_ || v_ - brake_vs_ <= brake_a_) {
    v_ = 0;
    acc_ = 0;
    return false;
  }
  v_ -= brake_a_;
  return true;
}

int8_t SM_ISR Stepgen::tick() {
  switch (mode_) {
    case kIdle:
      return 0;

    case kPosition: {
      const int32_t rem = target_ - pos_;
      if (v_ == 0) {
        if (rem == 0) {
          mode_ = kIdle;
          return 0;
        }
        brake_a_ = prof_.accel;
        brake_vs_ = prof_.v_start;
        dir_ = rem > 0 ? 1 : -1;
        v_ = prof_.v_start;
        acc_ = 0u - v_;  // the add below carries, so the first step is now
      } else {
        const int32_t dist = dir_ > 0 ? rem : -rem;
        if (dist <= 0) {
          // The target moved behind us: brake, then restart towards it.
          if (!brake()) return 0;
        } else if (needDecel(static_cast<uint32_t>(dist))) {
          v_ = (v_ - brake_vs_ > brake_a_) ? v_ - brake_a_ : brake_vs_;
        } else if (v_ < prof_.v_max) {
          v_ = (prof_.v_max - v_ > prof_.accel) ? v_ + prof_.accel : prof_.v_max;
        } else if (v_ > prof_.v_max) {
          v_ = (v_ - prof_.v_max > brake_a_) ? v_ - brake_a_ : prof_.v_max;
        }
      }
      break;
    }

    case kVelocity:
      if (v_ < v_cmd_) {
        v_ = (v_cmd_ - v_ > prof_.accel) ? v_ + prof_.accel : v_cmd_;
      } else if (v_ > v_cmd_) {
        v_ = (v_ - v_cmd_ > prof_.accel) ? v_ - prof_.accel : v_cmd_;
      }
      if (v_ == 0) return 0;
      break;

    case kStopping:
      if (!brake()) {
        mode_ = kIdle;
        target_ = pos_;
        return 0;
      }
      break;
  }

  const uint32_t before = acc_;
  acc_ += v_;
  if (acc_ >= before) return 0;  // no carry, no step

  if (mode_ == kVelocity && (dir_ > 0 ? pos_ >= limit_ : pos_ <= limit_)) {
    // This step would pass the limit. The run is no faster than the start
    // speed, so it can stop dead here instead.
    v_ = 0;
    acc_ = 0;
    target_ = pos_;
    mode_ = kIdle;
    return 0;
  }
  pos_ += dir_;
  // A ramp reaches its target near the start speed (braking from there would
  // take under 3 steps). Only a target moved closer than the braking distance
  // mid-move is reached faster: then carry on, brake and come back to it,
  // rather than stop dead from speed and lose steps.
  if (mode_ == kPosition && pos_ == target_ && !needDecel(kStopDeadSteps)) {
    v_ = 0;
    acc_ = 0;
    mode_ = kIdle;
  }
  return dir_;
}

}  // namespace sm
