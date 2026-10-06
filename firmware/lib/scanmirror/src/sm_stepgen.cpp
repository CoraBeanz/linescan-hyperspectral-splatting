#include "sm_stepgen.h"

namespace sm {

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
  // From any mode: tick() brakes first if we are heading the wrong way.
  mode_ = kPosition;
}

void Stepgen::runVelocity(int8_t dir, uint32_t v, uint32_t phase, const Profile& p) {
  setProfile(p);
  dir_ = dir < 0 ? -1 : 1;
  v_cmd_ = v;
  v_ = v < prof_.v_start ? v : prof_.v_start;
  acc_ = phase;
  mode_ = kVelocity;
}

void Stepgen::setVelocity(uint32_t v) {
  if (mode_ == kVelocity) v_cmd_ = v;
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
  if (v_ <= prof_.v_start) return false;
  const uint64_t vv = static_cast<uint64_t>(v_) * v_ -
                      static_cast<uint64_t>(prof_.v_start) * prof_.v_start;
  return (vv >> 32) >= 2ull * prof_.accel * dist;
}

// Slow down by one acceleration step. Returns false (and leaves the generator
// at rest) once the speed is down to the start speed, which a stepper can
// stop from at once.
bool SM_ISR Stepgen::brake() {
  if (v_ <= prof_.v_start || v_ - prof_.v_start <= prof_.accel) {
    v_ = 0;
    acc_ = 0;
    return false;
  }
  v_ -= prof_.accel;
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
        dir_ = rem > 0 ? 1 : -1;
        v_ = prof_.v_start;
        acc_ = 0u - v_;  // the add below carries, so the first step is now
      } else {
        const int32_t dist = dir_ > 0 ? rem : -rem;
        if (dist <= 0) {
          // The target moved behind us: brake, then restart towards it.
          if (!brake()) return 0;
        } else if (needDecel(static_cast<uint32_t>(dist))) {
          v_ = (v_ - prof_.v_start > prof_.accel) ? v_ - prof_.accel : prof_.v_start;
        } else if (v_ < prof_.v_max) {
          v_ = (prof_.v_max - v_ > prof_.accel) ? v_ + prof_.accel : prof_.v_max;
        } else if (v_ > prof_.v_max) {
          v_ = (v_ - prof_.v_max > prof_.accel) ? v_ - prof_.accel : prof_.v_max;
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

  pos_ += dir_;
  if (mode_ == kPosition && pos_ == target_) {
    v_ = 0;
    acc_ = 0;
    mode_ = kIdle;
  }
  return dir_;
}

}  // namespace sm
