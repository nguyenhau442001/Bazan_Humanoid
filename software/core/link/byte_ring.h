// Lock-free single-producer / single-consumer byte ring for passing data between two tasks
// (or a task and an ISR). One writer calls write(), one reader calls read(). No allocation, no locks.
#pragma once
#include <atomic>
#include <stddef.h>
#include <stdint.h>

namespace mico {

template <size_t N>
class ByteRing {
  static_assert(N >= 2 && (N & (N - 1)) == 0, "N must be a power of two");

 public:
  // All or nothing: false (and nothing written) if fewer than n bytes are free. Writer side only.
  bool write(const uint8_t* data, size_t n) {
    uint32_t head = head_.load(std::memory_order_relaxed);
    uint32_t tail = tail_.load(std::memory_order_acquire);
    if (N - (head - tail) < n) return false;
    for (size_t i = 0; i < n; ++i) buf_[(head + i) & (N - 1)] = data[i];
    head_.store(head + static_cast<uint32_t>(n), std::memory_order_release);
    return true;
  }

  // Up to max bytes. Reader side only.
  size_t read(uint8_t* out, size_t max) {
    uint32_t tail = tail_.load(std::memory_order_relaxed);
    uint32_t head = head_.load(std::memory_order_acquire);
    size_t n = head - tail;
    if (n > max) n = max;
    for (size_t i = 0; i < n; ++i) out[i] = buf_[(tail + i) & (N - 1)];
    tail_.store(tail + static_cast<uint32_t>(n), std::memory_order_release);
    return n;
  }

  size_t used() const { return head_.load(std::memory_order_acquire) - tail_.load(std::memory_order_acquire); }
  size_t free() const { return N - used(); }

 private:
  uint8_t buf_[N];
  std::atomic<uint32_t> head_{0};   // free-running; only the writer stores it
  std::atomic<uint32_t> tail_{0};   // free-running; only the reader stores it
};

}  // namespace mico
