#include "loop_stats.h"

#include <string.h>

namespace mico {

bool LoopStats::record(uint32_t release_us, uint32_t start_us, uint32_t end_us) {
  int32_t late = static_cast<int32_t>(start_us - release_us);
  uint32_t jitter = late > 0 ? static_cast<uint32_t>(late) : 0;   // starting early is not jitter we care about
  uint32_t exec = end_us - start_us;
  bool miss = static_cast<int32_t>(end_us - release_us) > static_cast<int32_t>(deadline_us_);
  ++cycles_;
  if (miss) ++missed_;
  exec_sum_ += exec;
  if (exec > exec_max_) exec_max_ = exec;
  if (jitter > jitter_max_) jitter_max_ = jitter;
  ++exec_hist_[bin(exec)];
  ++jitter_hist_[bin(jitter)];
  return miss;
}

void LoopStats::reset() {
  cycles_ = missed_ = exec_max_ = jitter_max_ = 0;
  exec_sum_ = 0;
  memset(exec_hist_, 0, sizeof(exec_hist_));
  memset(jitter_hist_, 0, sizeof(jitter_hist_));
}

uint32_t LoopStats::percentile(const uint32_t* hist, float p) const {
  if (!cycles_) return 0;
  uint32_t target = static_cast<uint32_t>(p * cycles_ + 0.999f);   // ceil: p99 of 100 samples is the 99th
  uint32_t seen = 0;
  for (int i = 0; i < BINS; ++i) {
    seen += hist[i];
    if (seen >= target) return (i + 1) * BIN_US;             // upper edge: never under-reports
  }
  return BINS * BIN_US;
}

}  // namespace mico
