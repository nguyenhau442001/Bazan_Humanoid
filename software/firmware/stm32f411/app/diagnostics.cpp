#include "diagnostics.h"

#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

namespace mico {
namespace app {

namespace {
uint16_t sat16(uint32_t v) { return v > 0xFFFF ? 0xFFFF : static_cast<uint16_t>(v); }
}  // namespace

Diag Diagnostics::build(const DiagSources& src, uint32_t now_ms) {
  Diag d;
  memset(&d, 0, sizeof(d));
  const ControlSnapshot& s = *src.snap;
  d.t_ms = now_ms;
  d.cycles = s.cycles;
  d.missed_deadlines = s.missed;
  d.exec_us_max = sat16(s.exec_max_us);
  d.exec_us_p99 = sat16(s.exec_p99_us);
  d.exec_us_avg = sat16(s.exec_avg_us);
  d.jitter_us_max = sat16(s.jitter_max_us);
  d.jitter_us_p99 = sat16(s.jitter_p99_us);

  // CPU: run-time counter deltas over the last period. Slot N_TASKS is the idle task.
  static TaskStatus_t st[N_TASKS + 3];
  uint32_t total = 0;
  UBaseType_t n = uxTaskGetSystemState(st, N_TASKS + 3, &total);
  uint32_t now_cnt[N_TASKS + 1] = {};
  for (UBaseType_t i = 0; i < n; ++i) {
    int slot = -1;
    for (int k = 0; k < N_TASKS; ++k)
      if (st[i].xHandle == task_handles[k]) slot = k;
    if (slot < 0 && strcmp(st[i].pcTaskName, configIDLE_TASK_NAME) == 0) slot = N_TASKS;
    if (slot < 0) continue;  // e.g. the host test's driver task
    now_cnt[slot] = st[i].ulRunTimeCounter;
    if (slot < N_TASKS) d.stack_free_words[slot] = sat16(st[i].usStackHighWaterMark);
  }
  uint32_t dt = total - prev_total_;  // unsigned: wrap-safe
  for (int k = 0; k <= N_TASKS; ++k) {
    uint32_t dk = now_cnt[k] - prev_task_[k];
    d.cpu_permille[k] = (have_prev_ && dt) ? sat16(static_cast<uint32_t>((uint64_t)dk * 1000u / dt)) : 0;
    prev_task_[k] = now_cnt[k];
  }
  prev_total_ = total;
  have_prev_ = true;

  d.heap_free = xPortGetFreeHeapSize();
  d.heap_min_free = xPortGetMinimumEverFreeHeapSize();
  d.allocs_after_start = sat16(src.allocs_after_start);
  d.rx_overruns = sat16(src.rx_overruns);
  d.tx_drops = sat16(s.tx_drops);
  uint32_t oe = src.sup->oe_applied();
  d.supervisor = (src.sup->healthy() ? SUP_HEALTHY : 0) | (src.sup->latched() ? SUP_LATCHED : 0) |
                 (src.sup->clear_pending() ? SUP_CLEAR_PENDING : 0) | ((oe & 1u) ? SUP_OE_LEGS : 0) |
                 ((oe & 2u) ? SUP_OE_ARMS : 0);
  d.sup_jitter_us_max = sat16(src.sup_jitter_us_max);
  d.wdt_gap_us_max = sat16(src.wdt_gap_us_max);
  d.latch_delay_us = sat16(src.sup->last_latch_delay_us());
  d.reset_cause = src.reset_cause;
  for (int k = 0; k < FAULT_HISTORY; ++k) {
    d.fault_t_ms[k] = s.fault_t_ms[k];
    d.fault_bits[k] = static_cast<uint16_t>(s.fault_bits[k]);
  }
  return d;
}

}  // namespace app
}  // namespace mico
