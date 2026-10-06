// Fixed-size queue that carries events from the timer interrupt to the main loop.
#pragma once

#include <stdint.h>

#include <atomic>

namespace sm {

// Single producer, single consumer ring buffer. On the ESP32 the producer is
// the timer interrupt on core 0 and the consumer is loop() on core 1; the
// acquire/release pairs make a slot's contents visible before its index.
// N must be a power of two. When full, new events are dropped and counted.
template <typename T, uint32_t N>
class EventQueue {
  static_assert((N & (N - 1)) == 0, "N must be a power of two");

 public:
  bool push(const T& item) {
    const uint32_t head = head_.load(std::memory_order_relaxed);
    const uint32_t tail = tail_.load(std::memory_order_acquire);
    if (head - tail >= N) {
      dropped_.store(dropped_.load(std::memory_order_relaxed) + 1, std::memory_order_relaxed);
      return false;
    }
    buf_[head & (N - 1)] = item;
    head_.store(head + 1, std::memory_order_release);
    return true;
  }

  bool pop(T* out) {
    const uint32_t tail = tail_.load(std::memory_order_relaxed);
    const uint32_t head = head_.load(std::memory_order_acquire);
    if (tail == head) return false;
    *out = buf_[tail & (N - 1)];
    tail_.store(tail + 1, std::memory_order_release);
    return true;
  }

  uint32_t dropped() const { return dropped_.load(std::memory_order_relaxed); }

 private:
  T buf_[N];
  std::atomic<uint32_t> head_{0};
  std::atomic<uint32_t> tail_{0};
  std::atomic<uint32_t> dropped_{0};
};

}  // namespace sm
