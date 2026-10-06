// Step generator: decides, once per timer tick, whether to issue a step pulse.
#pragma once

#include <stdint.h>

#include "sm_common.h"

namespace sm {

// A motion profile in tick units; build one with velocityQ32/accelQ32.
// Moves start at v_start (a stepper can jump straight to a modest speed from
// rest, so short moves need no ramp), accelerate to v_max, and slow back to
// v_start before the target.
struct Profile {
  uint32_t v_start;  // Q32 microsteps per tick
  uint32_t v_max;    // Q32 microsteps per tick
  uint32_t accel;    // Q32 microsteps per tick^2
};

// A fixed-rate DDA (digital differential analyzer). Each tick adds the
// current speed to a 32-bit phase accumulator, and every carry out of it is
// one step. Acceleration changes the speed by a constant amount per tick, and
// the stopping distance check uses 64-bit integer maths only.
//
// tick() runs in the timer interrupt; the other methods run in the main loop
// with the interrupt locked out (see MirrorCore).
class Stepgen {
 public:
  // Go to target, replanning from the current speed and direction.
  void moveTo(int32_t target, const Profile& p);
  // Run at constant speed v in direction dir until stop(); from rest only.
  // phase preloads the accumulator (2^31 rounds the position to the nearest
  // step of the ideal straight line).
  void runVelocity(int8_t dir, uint32_t v, uint32_t phase, const Profile& p);
  void setVelocity(uint32_t v);
  // Slow down at the profile's acceleration and stop.
  void stop();
  // Stop at once, without slowing down.
  void halt();
  // Redefine the current position; refused while moving.
  bool setPosition(int32_t pos);

  // Advance one tick. Returns +1 or -1 when a step in that direction is due
  // now, otherwise 0. The position already counts the returned step.
  int8_t tick();

  bool busy() const { return mode_ != kIdle; }
  int32_t position() const { return pos_; }
  int32_t target() const { return target_; }
  uint32_t speed() const { return v_; }
  int8_t direction() const { return dir_; }

 private:
  enum Mode : uint8_t { kIdle, kPosition, kVelocity, kStopping };

  void setProfile(const Profile& p);
  bool needDecel(uint32_t dist) const;
  bool brake();

  Profile prof_ = {1, 1, 1};
  Mode mode_ = kIdle;
  int8_t dir_ = 1;
  int32_t pos_ = 0;
  int32_t target_ = 0;
  uint32_t v_ = 0;      // current speed
  uint32_t v_cmd_ = 0;  // set point in velocity mode
  uint32_t acc_ = 0;    // phase accumulator
};

}  // namespace sm
