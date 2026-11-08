// Timing statistics for a periodic task: release lateness (jitter), execution time, missed deadlines.
// Fixed-size histograms (10 us bins) give p99 without storing samples; maxima are exact.
#pragma once
#include <stdint.h>

namespace mico {

class LoopStats {
 public:
  static constexpr uint32_t BIN_US = 10;
  static constexpr int BINS = 500;                 // 0 .. 5 ms; later samples go to the last bin

  explicit LoopStats(uint32_t deadline_us = 5000) : deadline_us_(deadline_us) {}

  // One cycle: release_us is when it should have started, start_us when it did, end_us when it finished.
  // Returns true if the cycle missed its deadline (end later than release + deadline).
  bool record(uint32_t release_us, uint32_t start_us, uint32_t end_us);
  void reset();

  uint32_t cycles() const { return cycles_; }
  uint32_t missed() const { return missed_; }
  uint32_t exec_max_us() const { return exec_max_; }
  uint32_t exec_avg_us() const { return cycles_ ? static_cast<uint32_t>(exec_sum_ / cycles_) : 0; }
  uint32_t jitter_max_us() const { return jitter_max_; }
  uint32_t exec_p99_us() const { return percentile(exec_hist_, 0.99f); }
  uint32_t jitter_p99_us() const { return percentile(jitter_hist_, 0.99f); }

 private:
  uint32_t percentile(const uint32_t* hist, float p) const;
  static int bin(uint32_t us) { return us / BIN_US >= BINS ? BINS - 1 : static_cast<int>(us / BIN_US); }

  uint32_t deadline_us_;
  uint32_t cycles_ = 0, missed_ = 0, exec_max_ = 0, jitter_max_ = 0;
  uint64_t exec_sum_ = 0;
  uint32_t exec_hist_[BINS] = {};
  uint32_t jitter_hist_[BINS] = {};
};

}  // namespace mico
