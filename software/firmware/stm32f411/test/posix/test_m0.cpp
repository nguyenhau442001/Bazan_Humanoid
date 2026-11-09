// M0 integration test on the host: the real FreeRTOS tasks (app/tasks.cpp) on the POSIX port with a
// simulated board. A driver task at the top priority plays the host and checks the M0 safety behaviour:
//   start-up: watchdog started and refreshed, OE off, mode IDLE
//   Enable: OE on through the supervisor
//   overrun (TestHook 1): deadline latch, OE off, watchdog still refreshed; ClearFault recovers
//   hang (TestHook 2): watchdog refresh stops, OE off
//   no allocation after start, Diag counters consistent
// Simulated time follows the host's scheduling, so timing values (latch delay, refresh gaps) are checked by the
// deterministic unit tests in lib/core and measured on the board, not here.
// Run with --watchdog-reset to start as after an IWDG reset: servos must never be enabled.
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "FreeRTOS.h"
#include "app.h"
#include "board_sim.h"
#include "frame.h"
#include "protocol.h"
#include "task.h"

using namespace mico;

namespace {

int checks = 0, failures = 0;
#define CHECK(c)                                                    \
  do {                                                              \
    ++checks;                                                       \
    if (!(c)) {                                                     \
      ++failures;                                                   \
      printf("  FAIL %s:%d  %s\n", __FILE__, __LINE__, #c);         \
      fflush(stdout);                                               \
    }                                                               \
  } while (0)

// ---- host side of the link
FrameParser parser;
State last_state = {};
Diag last_diag = {};
int states = 0, diags = 0;
uint8_t seq = 0;

void pump() {
  uint8_t buf[512];
  size_t n;
  while ((n = sim::mcu_to_host.read(buf, sizeof(buf))) > 0)
    for (size_t i = 0; i < n; ++i) {
      Frame f;
      if (!parser.push(buf[i], f)) continue;
      if (f.type == MSG_STATE && f.len == sizeof(State)) {
        memcpy(&last_state, f.payload, sizeof(State));
        ++states;
      } else if (f.type == MSG_DIAG && f.len == sizeof(Diag)) {
        memcpy(&last_diag, f.payload, sizeof(Diag));
        ++diags;
      } else if (f.type == MSG_LOG && f.len > 1) {
        printf("  mcu log: %.*s\n", static_cast<int>(f.len - 1), reinterpret_cast<const char*>(f.payload + 1));
      }
    }
}

void send(uint8_t type, const void* p, size_t n) {
  uint8_t frame[64];
  size_t m = frame_build(type, seq++, p, n, frame, sizeof(frame));
  while (!sim::host_to_mcu.write(frame, m)) vTaskDelay(1);
}

// Wait ms while reading the link; Hello every 50 ms keeps the controller out of comm-loss HOLD.
void wait_ms(uint32_t ms) {
  for (uint32_t t = 0; t < ms; t += 10) {
    if (t % 50 == 0) send(MSG_HELLO, nullptr, 0);
    vTaskDelay(10);
    pump();
  }
}

Mode mode() { return static_cast<Mode>(last_state.mode); }

// Wait up to ms for cond (polled every 10 ms); a loaded CI host may delay the simulated tasks.
template <typename F>
bool wait_for(F cond, uint32_t ms) {
  for (uint32_t t = 0; t < ms; t += 10) {
    if (cond()) return true;
    wait_ms(10);
  }
  return cond();
}

void normal_run() {
  // ---- start-up
  wait_ms(300);
  CHECK(sim::watchdog_started);
  CHECK(sim::refreshes > 200);                         // 1 kHz supervisor, refreshing
  CHECK(!sim::oe_ever_on);                             // nothing enabled the servos
  CHECK(states > 10 && mode() == Mode::IDLE && last_state.faults == 0);

  // ---- Enable: the supervisor drives OE because control asks and is healthy
  uint8_t on = 1;
  send(MSG_ENABLE, &on, 1);
  CHECK(wait_for([] { return mode() == Mode::RUN; }, 500));
  CHECK(wait_for([] { return sim::oe_legs && sim::oe_arms; }, 100));
  wait_ms(1100);
  // Diag reports both OE lines on (unless a host stall made a cycle late in simulated time: then off is right)
  CHECK((last_diag.supervisor & (8 | 16)) == (8 | 16) || (last_state.faults & F_CONTROL_DEADLINE));

  // ---- overrun one cycle by 12 ms
  TestHook h = {1, 12};
  send(MSG_TESTHOOK, &h, sizeof(h));
  uint32_t r0 = sim::refreshes;
  wait_ms(100);
  CHECK(!sim::oe_legs && !sim::oe_arms);
  CHECK(mode() == Mode::FAULT && (last_state.faults & F_CONTROL_DEADLINE));
  CHECK(sim::refreshes - r0 > 50);                     // control recovered: the MCU keeps running
  wait_ms(1100);                                       // let a Diag through (1 Hz, snapshot every 100 ms)
  CHECK(diags >= 1);
  CHECK(last_diag.missed_deadlines >= 1);
  CHECK(last_diag.cycles > 150);                       // ~200 per s, snapshot up to 100 ms old
  CHECK(last_diag.supervisor & 2);                     // latched
  CHECK(last_diag.allocs_after_start == 0);
  CHECK(last_diag.heap_free > 20000 && last_diag.heap_min_free > 20000);   // 24 KB kept for micro-ROS
  CHECK(last_diag.fault_bits[0] & F_CONTROL_DEADLINE);
  CHECK((last_diag.supervisor & (8 | 16)) == 0);        // OE reported off

  // ---- ClearFault: accepted (healthy, nothing present) and clears the supervisor latch too
  send(MSG_CLEARFAULT, nullptr, 0);
  CHECK(wait_for([] { return mode() == Mode::IDLE && last_state.faults == 0; }, 500));
  CHECK(!sim::oe_legs);                                // clearing never enables by itself
  send(MSG_ENABLE, &on, 1);
  CHECK(wait_for([] { return mode() == Mode::RUN && sim::oe_legs; }, 500));

  // ---- hang: the control task never completes again
  h = {2, 0};
  send(MSG_TESTHOOK, &h, sizeof(h));
  vTaskDelay(30);
  CHECK(!sim::oe_legs && !sim::oe_arms);
  uint32_t r1 = sim::refreshes;
  vTaskDelay(200);
  CHECK(sim::refreshes == r1);                         // no refresh: the IWDG would reset the MCU
}

void watchdog_reset_run() {
  wait_ms(300);
  CHECK(mode() == Mode::FAULT && (last_state.faults & F_WATCHDOG_RESET));
  uint8_t on = 1;
  send(MSG_ENABLE, &on, 1);                            // refused in FAULT
  wait_ms(100);
  CHECK(!sim::oe_ever_on);
  wait_ms(1100);
  CHECK(diags >= 1 && last_diag.reset_cause == 4);
  CHECK((last_diag.supervisor & (8 | 16)) == 0);        // outputs off after the reset
  send(MSG_CLEARFAULT, nullptr, 0);                    // operator clears: IDLE, still not enabled
  CHECK(wait_for([] { return mode() == Mode::IDLE; }, 500));
  CHECK(!sim::oe_ever_on);
}

void driver(void*) {
  if (sim::watchdog_reset)
    watchdog_reset_run();
  else
    normal_run();
  printf("test_m0%s: %d checks, %d failures\n", sim::watchdog_reset ? " (watchdog reset)" : "", checks,
         failures);
  fflush(stdout);
  _Exit(failures ? 1 : 0);   // the hung control task cannot be joined: leave without running destructors
}

StaticTask_t driver_tcb;
StackType_t driver_stack[65536 / sizeof(StackType_t)];

}  // namespace

extern "C" void vAssertCalled(const char* file, int line) {
  printf("configASSERT failed %s:%d\n", file, line);
  fflush(stdout);
  _Exit(2);
}

void start_pty_bridge(char* argv0, int fd);

// test_m0                     scripted checks (CI)
// test_m0 --watchdog-reset    scripted checks after a simulated IWDG reset
// test_m0 --pty               prints a pty name; run tools/mico_link.py --port <it> against the simulated MCU
int main(int argc, char** argv) {
  int pty_fd = -1;
  bool pty = false;
  for (int i = 1; i < argc; ++i) {
    if (strcmp(argv[i], "--watchdog-reset") == 0) sim::watchdog_reset = true;
    if (strcmp(argv[i], "--pty") == 0) pty = true;
    if (strcmp(argv[i], "--pty-fd") == 0 && i + 1 < argc) {
      pty = true;
      pty_fd = atoi(argv[++i]);
    }
  }
  app::create_tasks(true);
  if (pty)
    start_pty_bridge(argv[0], pty_fd);
  else
    xTaskCreateStatic(driver, "driver", sizeof(driver_stack) / sizeof(StackType_t), nullptr,
                      configMAX_PRIORITIES - 1, driver_stack, &driver_tcb);
  vTaskStartScheduler();
  return 3;
}
