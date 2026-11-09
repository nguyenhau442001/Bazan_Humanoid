// Builds the 1 Hz Diag message: control timing (from the control task's snapshot), CPU load per task,
// stack headroom, heap, allocation and link counters, supervisor state and the fault history.
// Runs in the diagnostics task; uses only FreeRTOS's read-only introspection calls.
#pragma once
#include <stdint.h>

#include "app.h"
#include "protocol.h"
#include "supervisor.h"

namespace mico {
namespace app {

struct DiagSources {
  const ControlSnapshot* snap;
  const Supervisor* sup;
  uint8_t reset_cause;
  uint32_t rx_overruns;
  uint32_t allocs_after_start;
  uint32_t sup_jitter_us_max;      // supervisor wake-time spread
  uint32_t wdt_gap_us_max;         // longest interval between IWDG refreshes
};

// Diag.supervisor bits
constexpr uint8_t SUP_HEALTHY = 1, SUP_LATCHED = 2, SUP_CLEAR_PENDING = 4, SUP_OE_LEGS = 8, SUP_OE_ARMS = 16;

class Diagnostics {
 public:
  Diag build(const DiagSources& src, uint32_t now_ms);

 private:
  // Run-time counters at the previous call, per Diag slot (4 tasks + idle), for 1 s CPU deltas.
  uint32_t prev_task_[N_TASKS + 1] = {};
  uint32_t prev_total_ = 0;
  bool have_prev_ = false;
};

}  // namespace app
}  // namespace mico
