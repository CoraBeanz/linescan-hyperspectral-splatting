// Shared constants and fixed-point helpers for the scan-mirror firmware.
//
// Everything in lib/scanmirror is plain C++11 with no Arduino dependency, so
// the same code runs in the ESP32 timer interrupt and in the native unit tests.
#pragma once

#include <stdint.h>

#if defined(ESP_PLATFORM) || defined(ARDUINO_ARCH_ESP32)
#include <esp_attr.h>
// Keep the code the timer interrupt runs in IRAM, so a flash cache miss can't
// stall a step.
#define SM_ISR IRAM_ATTR
#else
#define SM_ISR
#endif

namespace sm {

// The timer interrupt runs every kTickUs. Step pulses and line ticks land on
// this grid, so it sets the timing resolution (50 us is under 0.2% of a 30 fps
// frame) and the top step rate (one step per tick).
constexpr uint32_t kTickUs = 50;
constexpr uint32_t kTicksPerSec = 1000000 / kTickUs;

// Speeds above this are refused: half the tick rate keeps step pulses evenly
// spread and the Q32 maths below away from overflow.
constexpr int32_t kMaxStepsPerSec = 10000;
constexpr int32_t kMaxAccel = 1000000;  // microsteps/s^2

// Velocities are microsteps per tick in Q32 (2^32 = one step per tick) and
// accelerations microsteps per tick^2 in Q32. Only the main loop converts;
// the ESP32 interrupt must not touch the FPU.
inline uint32_t velocityQ32(double steps_per_s) {
  if (steps_per_s <= 0) return 0;
  if (steps_per_s > kMaxStepsPerSec) steps_per_s = kMaxStepsPerSec;
  const double q = steps_per_s * kTickUs * 1e-6 * 4294967296.0;
  return q < 1.0 ? 1u : static_cast<uint32_t>(q + 0.5);
}

inline uint32_t accelQ32(double steps_per_s2) {
  if (steps_per_s2 > kMaxAccel) steps_per_s2 = kMaxAccel;
  const double t = kTickUs * 1e-6;
  const double q = steps_per_s2 * t * t * 4294967296.0;
  return q < 1.0 ? 1u : static_cast<uint32_t>(q + 0.5);
}

// Line periods are microseconds in Q16 (65536 = 1 us), so a 30 fps camera
// period of 33333.333 us can be matched without drifting a whole microsecond
// every three frames.
constexpr uint64_t kQ16 = 65536;

}  // namespace sm
