// FreeRTOS tasks of the Mico MCU. See app.h for the task table.
#include <string.h>

#include "app.h"
#include "board.h"
#include "byte_ring.h"
#include "config.h"
#include "control_cycle.h"
#include "diagnostics.h"
#include "frame.h"
#include "loop_stats.h"
#include "seqlock.h"
#include "supervisor.h"

namespace mico {
namespace app {

TaskHandle_t task_handles[N_TASKS];

namespace {

// ---------------------------------------------------------------- shared state
Controller ctrl(0, 2);
SupervisorConfig sup_cfg() {
  SupervisorConfig c;
  c.period_us = CONTROL_PERIOD_MS * 1000;
  c.deadline_us = CONTROL_DEADLINE_US;
  return c;
}
Supervisor sup(sup_cfg());
ControlCycle cycle(ctrl, sup);
LoopStats stats(CONTROL_DEADLINE_US);
Seqlock<ControlSnapshot> snapshot;
ByteRing<1024> rx_ring;      // comm -> control: host bytes
ByteRing<4096> tx_ring;      // control -> comm: State (100 Hz), Info, Log frames
ByteRing<512> diag_ring;     // diagnostics -> comm: Diag frames
std::atomic<uint32_t> alloc_after_start{0};
std::atomic<uint32_t> rx_drops{0};

// ---------------------------------------------------------------- static task memory
constexpr uint32_t STACK_WORDS[N_TASKS] = {256, 1024, 384, 512};   // supervisor, control, comm, diagnostics
StaticTask_t tcb[N_TASKS];
StackType_t stack_sup[STACK_WORDS[0]], stack_ctrl[STACK_WORDS[1]], stack_comm[STACK_WORDS[2]],
    stack_diag[STACK_WORDS[3]];

// ---------------------------------------------------------------- supervisor: highest priority, 1 ms
// Measured here and reported in Diag: the spread of the supervisor's wake time against the tick (its scheduling
// latency variation) and the longest gap between two IWDG refreshes while control is healthy.
std::atomic<uint32_t> sup_jitter_max{0}, wdt_gap_max{0};

void supervisor_task(void*) {
  board::watchdog_start();
  uint32_t now = board::micros();
  sup.begin(now);
  uint32_t last_refresh = now;
  int32_t off_min = INT32_MAX, off_max = INT32_MIN;
  TickType_t wake = xTaskGetTickCount();
  for (bool first = true;; first = false) {
    now = board::micros();
    // TIM5 and the tick share one clock, so micros - tick * 1000 is constant up to the wake latency.
    // The first pass runs at scheduler start, not on a tick edge: measure from the second.
    if (!first) {
      int32_t off = static_cast<int32_t>(now - xTaskGetTickCount() * 1000u);
      if (off < off_min) off_min = off;
      if (off > off_max) off_max = off;
      sup_jitter_max.store(static_cast<uint32_t>(off_max - off_min));
    }

    Supervisor::Decision d = sup.step(now);
    board::set_oe(d.oe_legs, d.oe_arms);           // the only place OE is driven
    if (d.refresh_watchdog) {
      board::watchdog_refresh();
      if (now - last_refresh > wdt_gap_max.load()) wdt_gap_max.store(now - last_refresh);
      last_refresh = now;
    }
    xTaskDelayUntil(&wake, 1);
  }
}

// ---------------------------------------------------------------- control: 200 Hz
void control_task(void*) {
  ctrl.begin(xTaskGetTickCount(), board::reset_by_watchdog());
  ControlSnapshot snap = {};
  Outputs out = {};
  bool pwm_ok = true;
  uint16_t last_faults = ctrl.faults();
  int hist = 0;

  board::devices_init();                           // I2C device set-up: before the first timed cycle
  vTaskDelay(1);                                   // start on a tick edge, so releases line up with ticks
  TickType_t wake = xTaskGetTickCount();
  uint32_t release = board::micros();
  for (uint32_t k = 0;; ++k) {
    uint32_t start = board::micros();

    Inputs in = {};
    in.now_ms = xTaskGetTickCount();               // 1 kHz monotonic, wraps after 49 days
    board::read_inputs(in, k);
    in.pwm_ok = pwm_ok;
    uint8_t rx[256];
    size_t n = rx_ring.read(rx, sizeof(rx));       // never waits: whatever arrived since the last cycle
    cycle.run(in, rx, n, out);

    TestHook hook;
    if (ctrl.take_test_hook(hook)) {
      if (hook.action == 1) board::busy_wait_us(hook.arg_ms * 1000u);   // overrun this cycle
      if (hook.action == 2) {                      // hang: the supervisor must catch it
        volatile bool hung = true;                 // volatile: a side-effect-free endless loop is UB in C++
        while (hung) {
        }
      }
    }

    // Servo frames are 20 ms: legs on cycle 0 mod 4, arms on 2 mod 4, so no cycle writes both boards.
    // A failed write on either board counts until that board's next successful write.
    static bool ok_legs = true, ok_arms = true;
    if (k % PWM_EVERY == 0) ok_legs = board::write_pwm(0, out.pulse_us, NLEG);
    if (k % PWM_EVERY == 2) ok_arms = board::write_pwm(1, out.pulse_us + NLEG, NJ - NLEG);
    pwm_ok = ok_legs && ok_arms;

    uint8_t frame[256];
    size_t m;
    while ((m = ctrl.take_tx(frame, sizeof(frame))) > 0)
      if (!tx_ring.write(frame, m)) ++snap.tx_drops;
    board::set_led(ctrl.mode() == Mode::FAULT ? (in.now_ms / 100) & 1 : ctrl.mode() == Mode::IDLE);

    if (ctrl.faults() != last_faults) {            // fault history: the last FAULT_HISTORY changes
      last_faults = ctrl.faults();
      snap.fault_t_ms[hist % FAULT_HISTORY] = in.now_ms;
      snap.fault_bits[hist % FAULT_HISTORY] = last_faults;
      ++hist;
    }

    uint32_t end = board::micros();
    stats.record(release, start, end);
    sup.control_completed(release, end);           // the supervisor's evidence that this cycle finished
    if (k % 20 == 0) {                             // every 100 ms: Diag (1 Hz) is at most 100 ms stale
      snap.cycles = stats.cycles();
      snap.missed = stats.missed();
      snap.exec_max_us = stats.exec_max_us();
      snap.exec_p99_us = stats.exec_p99_us();
      snap.exec_avg_us = stats.exec_avg_us();
      snap.jitter_max_us = stats.jitter_max_us();
      snap.jitter_p99_us = stats.jitter_p99_us();
      snapshot.write(snap);
      ctrl.set_loop_us_max(static_cast<uint16_t>(snap.exec_max_us > 65535 ? 65535 : snap.exec_max_us));
    }

    xTaskDelayUntil(&wake, pdMS_TO_TICKS(CONTROL_PERIOD_MS));
    release += CONTROL_PERIOD_MS * 1000;
  }
}

// ---------------------------------------------------------------- comm: UART <-> rings
void comm_task(void*) {
  board::uart_start();
  static uint8_t tx[512];
  size_t pending = 0;
  int source = 0;                                  // 0 control frames, 1 diagnostics frames
  for (;;) {
    uint8_t rx[256];
    size_t n;
    while ((n = board::uart_read(rx, sizeof(rx))) > 0)
      if (!rx_ring.write(rx, n)) rx_drops.fetch_add(n);   // the parser resynchronises on the next 0x00

    if (!pending) {
      // Frames are 0x00-terminated. Switch source only on a frame boundary, so frames never interleave.
      for (int tries = 0; tries < 2 && !pending; ++tries) {
        pending = source == 0 ? tx_ring.read(tx, sizeof(tx)) : diag_ring.read(tx, sizeof(tx));
        if (!pending || tx[pending - 1] == 0) source ^= 1;
      }
    }
    if (pending && board::uart_write(tx, pending)) pending = 0;
    vTaskDelay(1);
  }
}

// ---------------------------------------------------------------- diagnostics: 1 Hz
void diagnostics_task(void*) {
  Diagnostics diag;
  uint8_t seq = 0;
  TickType_t wake = xTaskGetTickCount();
  for (;;) {
    xTaskDelayUntil(&wake, pdMS_TO_TICKS(1000));
    ControlSnapshot snap;
    if (!snapshot.read(snap)) continue;
    DiagSources src;
    src.snap = &snap;
    src.sup = &sup;
    src.reset_cause = board::reset_cause();
    src.rx_overruns = board::uart_rx_overruns() + rx_drops.load();
    src.allocs_after_start = alloc_after_start.load();
    src.sup_jitter_us_max = sup_jitter_max.load();
    src.wdt_gap_us_max = wdt_gap_max.load();
    Diag d = diag.build(src, xTaskGetTickCount());
    uint8_t frame[2 * sizeof(Diag)];
    size_t m = frame_build(MSG_DIAG, seq++, &d, sizeof(d), frame, sizeof(frame));
    if (m) diag_ring.write(frame, m);
  }
}

}  // namespace

uint32_t allocations_after_start() { return alloc_after_start.load(); }
uint32_t rx_ring_drops() { return rx_drops.load(); }

void create_tasks(bool test_hooks) {
  ctrl.enable_test_hooks(test_hooks);
  // heap_4 initialises itself on first use. Touch it once before the scheduler so its 24 KB (kept for
  // micro-ROS in M1) is linked, counted in the RAM budget and reported in Diag. Not a steady-state allocation.
  vPortFree(pvPortMalloc(8));
  task_handles[0] = xTaskCreateStatic(supervisor_task, "supervisor", STACK_WORDS[0], nullptr, PRIO_SUPERVISOR,
                                      stack_sup, &tcb[0]);
  task_handles[1] = xTaskCreateStatic(control_task, "control", STACK_WORDS[1], nullptr, PRIO_CONTROL, stack_ctrl,
                                      &tcb[1]);
  task_handles[2] = xTaskCreateStatic(comm_task, "comm", STACK_WORDS[2], nullptr, PRIO_COMM, stack_comm, &tcb[2]);
  task_handles[3] = xTaskCreateStatic(diagnostics_task, "diagnostics", STACK_WORDS[3], nullptr, PRIO_DIAG,
                                      stack_diag, &tcb[3]);
}

}  // namespace app
}  // namespace mico

// ---------------------------------------------------------------- FreeRTOS hooks
extern "C" {

void mico_trace_malloc(void) {
  if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) mico::app::alloc_after_start.fetch_add(1);
}

void vApplicationGetIdleTaskMemory(StaticTask_t** tcb, StackType_t** stack, configSTACK_DEPTH_TYPE* words) {
  static StaticTask_t idle_tcb;
  static StackType_t idle_stack[configMINIMAL_STACK_SIZE];
  *tcb = &idle_tcb;
  *stack = idle_stack;
  *words = configMINIMAL_STACK_SIZE;
}

// A task overran its stack: memory is no longer trusted. Outputs off, stop, and let the IWDG reset the MCU.
void vApplicationStackOverflowHook(TaskHandle_t, char*) {
  mico::board::emergency_outputs_off();
  taskDISABLE_INTERRUPTS();
  for (;;) {
  }
}
void vApplicationMallocFailedHook(void) { mico::app::alloc_after_start.fetch_add(1); }

}  // extern "C"
