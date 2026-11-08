// Single-writer snapshot: the writer publishes a whole struct, readers get a consistent copy or retry.
// Used to hand timing statistics from the control task to the diagnostics task without a mutex.
// The payload is stored as relaxed 32-bit atomics, so the copy is race-free in the C++ memory model
// (on a single-core Cortex-M these compile to plain word loads and stores plus barriers).
#pragma once
#include <atomic>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

namespace mico {

template <typename T>
class Seqlock {
  static constexpr size_t W = (sizeof(T) + 3) / 4;

 public:
  void write(const T& v) {
    uint32_t tmp[W] = {};
    memcpy(tmp, &v, sizeof(T));
    uint32_t s = seq_.load(std::memory_order_relaxed);
    seq_.store(s + 1, std::memory_order_relaxed);          // odd: write in progress
    // Release on each word keeps the odd sequence ahead of the data (no fences: TSan-checkable).
    for (size_t i = 0; i < W; ++i) words_[i].store(tmp[i], std::memory_order_release);
    seq_.store(s + 2, std::memory_order_release);
  }

  // False if the writer kept interrupting for `tries` attempts.
  bool read(T& out, int tries = 8) const {
    for (int i = 0; i < tries; ++i) {
      uint32_t s0 = seq_.load(std::memory_order_acquire);
      if (s0 & 1) continue;
      uint32_t tmp[W];
      // Acquire on each word keeps the re-check of the sequence after the data loads.
      for (size_t k = 0; k < W; ++k) tmp[k] = words_[k].load(std::memory_order_acquire);
      if (seq_.load(std::memory_order_relaxed) == s0) {
        memcpy(static_cast<void*>(&out), tmp, sizeof(T));
        return true;
      }
    }
    return false;
  }

 private:
  std::atomic<uint32_t> words_[W] = {};
  std::atomic<uint32_t> seq_{0};
};

}  // namespace mico
