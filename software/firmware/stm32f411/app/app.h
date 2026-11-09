// The FreeRTOS application: four static tasks around the hardware-independent control core.
//
//   task         priority  period  owns
//   supervisor   5 (top)   1 ms    OE pins, IWDG refresh, deadline checks of the control task
//   control      4         5 ms    sensors -> Controller (safety, trajectory) -> PWM; never blocks on the link
//   comm         3         1 ms    UART: RX DMA ring -> control, control/diagnostics frames -> TX DMA
//   diagnostics  1         1 s     timing, CPU, stacks, heap, fault history -> Diag message
//
// Tasks talk only through lock-free single-producer/single-consumer rings, a seqlock snapshot and the
// supervisor's atomics. Nothing allocates after the scheduler starts.
#pragma once
#include <atomic>
#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"
#include "protocol.h"

namespace mico {
namespace app {

constexpr UBaseType_t PRIO_SUPERVISOR = 5, PRIO_CONTROL = 4, PRIO_COMM = 3, PRIO_DIAG = 1;
constexpr uint32_t CONTROL_PERIOD_MS = 5;
constexpr uint32_t CONTROL_DEADLINE_US = 5000;

// What the control task publishes for the diagnostics task (seqlock; 32-bit fields only).
struct ControlSnapshot {
  uint32_t cycles, missed;
  uint32_t exec_max_us, exec_p99_us, exec_avg_us, jitter_max_us, jitter_p99_us;
  uint32_t tx_drops;               // control frames dropped because the TX ring was full
  uint32_t fault_t_ms[FAULT_HISTORY];
  uint32_t fault_bits[FAULT_HISTORY];
};

// Task handles in Diag order (protocol.h TASKS).
extern TaskHandle_t task_handles[N_TASKS];

// Create the four tasks (static memory). The caller starts the scheduler: app_start() on the target,
// the integration test's main() on the host, after adding its own driver task.
void create_tasks(bool test_hooks);

// Bytes from the UART the control task could not take (its ring was full).
uint32_t rx_ring_drops();

// Allocation counter fed by FreeRTOS's traceMALLOC (and newlib's _sbrk on the target).
uint32_t allocations_after_start();

}  // namespace app
}  // namespace mico
