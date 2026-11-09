// Host tests for the M0 safety layer: supervisor deadlines, watchdog gating, OE ownership, start-up,
// fault latching and clearing through the controller, loop statistics. Run: make -C lib/core test
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "byte_ring.h"
#include "control_cycle.h"
#include "controller.h"
#include "frame.h"
#include "loop_stats.h"
#include "seqlock.h"
#include "supervisor.h"

using namespace mico;

static int failures = 0, checks = 0;
#define CHECK(c)                                                       \
  do {                                                                 \
    ++checks;                                                          \
    if (!(c)) {                                                        \
      ++failures;                                                      \
      printf("  FAIL %s:%d  %s\n", __FILE__, __LINE__, #c);            \
    }                                                                  \
  } while (0)

constexpr uint32_t P = 5000;   // control period, us

// ---------------------------------------------------------------- supervisor alone
static void test_startup() {
  Supervisor s;
  s.begin(1000);
  s.set_oe_request(true, true);
  Supervisor::Decision d = s.step(2000);
  CHECK(!d.oe_legs && !d.oe_arms);              // no completed cycle yet: outputs off whatever is requested
  CHECK(d.refresh_watchdog && !d.latched);      // inside the boot grace
  d = s.step(1000 + 499000);
  CHECK(d.refresh_watchdog);
  d = s.step(1000 + 501000);                    // the control task never ran
  CHECK(!d.refresh_watchdog && d.latched && !d.oe_legs);
}

static void test_normal_cycles() {
  Supervisor s;
  s.begin(0);
  s.set_oe_request(false, false);
  uint32_t t = 10000;
  for (int k = 0; k < 1000; ++k, t += P) {
    s.control_completed(t, t + 1800);
    for (uint32_t u = t + 1800; u < t + P; u += 1000) {
      Supervisor::Decision d = s.step(u);
      CHECK(d.refresh_watchdog && !d.latched && (k > 500 || !d.oe_legs));   // outputs only on request
    }
    if (k == 500) s.set_oe_request(true, false);
  }
  Supervisor::Decision d = s.step(t - P + 2500);
  CHECK(d.oe_legs && !d.oe_arms);               // request honoured per board
  s.set_oe_request(false, false);
  CHECK(!s.step(t - P + 2600).oe_legs);
  CHECK(s.completions() == 1000);
}

static void test_deadline_miss() {
  Supervisor s;
  s.begin(0);
  s.set_oe_request(true, true);
  s.control_completed(10000, 12000);
  CHECK(s.step(12500).oe_legs);
  s.control_completed(15000, 15000 + P + 1);    // finished 1 us after its deadline
  Supervisor::Decision d = s.step(20100);
  CHECK(d.latched && !d.oe_legs && !d.oe_arms);
  CHECK(d.refresh_watchdog);                    // control is still running: no reset, just outputs off
  CHECK(s.latch_count() == 1);
  s.control_completed(20000, 21000);            // back on time: still latched until cleared
  d = s.step(21500);
  CHECK(d.latched && !d.oe_legs && d.refresh_watchdog);
  s.request_clear();
  d = s.step(21600);
  CHECK(!d.latched && d.oe_legs && !s.clear_pending());
  CHECK(s.latch_count() == 1);
}

static void test_stall() {
  Supervisor s;
  s.begin(0);
  s.set_oe_request(true, true);
  s.control_completed(10000, 11000);
  CHECK(s.step(10000 + P + P).refresh_watchdog);           // exactly at the next cycle's deadline
  Supervisor::Decision d = s.step(10000 + P + P + 1);      // next cycle not done in time: hung
  CHECK(!d.refresh_watchdog && d.latched && !d.oe_legs);
  CHECK(!s.healthy());
  s.request_clear();                                       // refused while stalled
  d = s.step(30000);
  CHECK(d.latched && !d.refresh_watchdog && !s.clear_pending());
  s.control_completed(30000, 31000);                       // control recovers
  d = s.step(31500);
  CHECK(d.refresh_watchdog && d.latched && !d.oe_legs);    // still latched: it was not cleared while healthy
  CHECK(s.latch_count() == 1);
}

// A late cycle followed by an on-time one inside the same supervisor step must still latch.
static void test_late_cycle_hidden_by_next() {
  Supervisor s;
  s.begin(0);
  s.set_oe_request(true, true);
  s.control_completed(10000, 11000);
  CHECK(s.step(11500).oe_legs);
  s.control_completed(15000, 23000);             // overran by 3 ms (deadline 20000)
  s.control_completed(20000, 23800);             // caught up, on time
  Supervisor::Decision d = s.step(24000);        // the supervisor only sees the second completion
  CHECK(d.latched && !d.oe_legs && d.refresh_watchdog);
  CHECK(s.bad_completions() == 1);
  CHECK(s.last_latch_delay_us() == 4000);        // latched 4 ms after the broken 20000 deadline
  CHECK(s.oe_applied() == 0);
}

// Completions must follow the 5 ms release grid: repeated, backwards or off-grid releases are not valid cycles.
static void test_release_grid() {
  const uint32_t bad_next[3] = {10000, 12000, 5000};   // repeated, off-grid, backwards
  for (uint32_t r : bad_next) {
    Supervisor s;
    s.begin(0);
    s.set_oe_request(true, true);
    s.control_completed(10000, 11000);
    CHECK(s.step(11500).oe_legs && s.oe_applied() == 3);
    s.control_completed(r, 12500);
    Supervisor::Decision d = s.step(12600);
    CHECK(d.latched && !d.oe_legs);
  }
  Supervisor s;                                  // skipped cycles are still on the grid: valid
  s.begin(0);
  s.control_completed(10000, 11000);
  s.control_completed(25000, 26000);
  CHECK(!s.step(26500).latched && s.bad_completions() == 0);
}

// Persistently late cycles: latched at once, and after max_bad_run in a row the watchdog is no longer fed.
static void test_persistent_overrun_stops_watchdog() {
  Supervisor s;
  s.begin(0);
  uint32_t t = 10000;
  s.control_completed(t, t + 1000);
  for (int k = 1; k <= 25; ++k) {
    t += P;
    s.control_completed(t, t + P + 100);         // every cycle 100 us late, but control keeps running
    Supervisor::Decision d = s.step(t + P + 200);
    CHECK(d.latched && !d.oe_legs);
    CHECK(d.refresh_watchdog == (k < 20));
  }
  t += P;
  s.control_completed(t, t + 1000);              // one valid cycle: fed again, still latched
  Supervisor::Decision d = s.step(t + 1500);
  CHECK(d.refresh_watchdog && d.latched);
}

static void test_stall_latch_delay() {
  Supervisor s;
  s.begin(0);
  s.control_completed(10000, 11000);
  s.step(20000);
  Supervisor::Decision d = s.step(20700);        // next deadline was 20000
  CHECK(d.latched && !d.refresh_watchdog && s.last_latch_delay_us() == 700);
}

static void test_clock_wrap() {
  Supervisor s;
  uint32_t t0 = 0xFFFFFFFFu - 7000;                        // the 32-bit us clock wraps every 71 minutes
  s.begin(t0);
  s.set_oe_request(true, true);
  s.control_completed(t0 + 1000, t0 + 3000);
  s.control_completed(t0 + 1000 + P, t0 + 3000 + P);      // crosses zero
  Supervisor::Decision d = s.step(t0 + 4000 + P);
  CHECK(d.refresh_watchdog && !d.latched && d.oe_legs);
  d = s.step(t0 + 1000 + 3 * P + 1);
  CHECK(!d.refresh_watchdog && d.latched);
}

// ---------------------------------------------------------------- controller + supervisor
static Inputs ok_inputs(uint32_t now_ms) {
  Inputs in = {};
  in.now_ms = now_ms;
  for (int k = 0; k < NNTC; ++k) {
    in.ntc_adc[k] = 2048;
    in.ntc_present[k] = true;
  }
  in.rail_v[0] = in.rail_v[1] = 6.0f;
  in.rail_valid = true;
  in.pwm_ok = in.sensors_ok = true;
  return in;
}

struct Rig {
  Controller ctrl;
  Supervisor sup;
  ControlCycle cycle{ctrl, sup};
  Outputs out = {};
  uint32_t t_us = 1000;
  Supervisor::Decision last = {};
  uint8_t rx[256];
  size_t rx_n = 0;

  Rig(bool wdt_reset = false) {
    ctrl.begin(0, wdt_reset);
    sup.begin(t_us);
  }
  void queue(uint8_t type, const void* p, size_t n) { rx_n += frame_build(type, 0, p, n, rx + rx_n, sizeof(rx) - rx_n); }
  // One control period: the control task runs (or not), the supervisor steps every 1 ms.
  void period(bool control_runs = true, uint32_t exec_us = 1500, Inputs* in_override = nullptr) {
    uint32_t release = t_us;
    if (control_runs) {
      Inputs in = in_override ? *in_override : ok_inputs(release / 1000);
      in.now_ms = release / 1000;
      cycle.run(in, rx, rx_n, out);
      rx_n = 0;
      sup.control_completed(release, release + exec_us);
    }
    for (uint32_t u = release + 1000; u <= release + P; u += 1000) last = sup.step(u);
    t_us += P;
  }
};

static void test_reset_never_enables() {
  Rig r(true);                                  // came back from a watchdog reset
  for (int k = 0; k < 10; ++k) r.period();
  CHECK(r.ctrl.mode() == Mode::FAULT && (r.ctrl.faults() & F_WATCHDOG_RESET));
  CHECK(!r.last.oe_legs && !r.out.oe_legs);
  Rig n;                                        // normal power-up
  for (int k = 0; k < 10; ++k) n.period();
  CHECK(n.ctrl.mode() == Mode::IDLE && !n.last.oe_legs && n.last.refresh_watchdog);
}

static void test_oe_follows_controller_and_supervisor() {
  Rig r;
  r.period();
  uint8_t on = 1;
  r.queue(MSG_ENABLE, &on, 1);
  r.period();
  CHECK(r.ctrl.mode() == Mode::RUN && r.out.oe_legs);
  CHECK(r.last.oe_legs && r.last.oe_arms);      // the supervisor drives OE because control asked and is healthy
  uint8_t off = 0;
  r.queue(MSG_ENABLE, &off, 1);
  r.period();
  CHECK(!r.last.oe_legs);
}

static void test_overrun_latches_fault() {
  Rig r;
  uint8_t on = 1;
  r.queue(MSG_ENABLE, &on, 1);
  r.period();
  r.period();
  CHECK(r.last.oe_legs);
  r.period(true, P + 200);                      // one cycle overruns its 5 ms deadline
  CHECK(r.last.latched && !r.last.oe_legs && r.last.refresh_watchdog);
  r.period();
  CHECK(r.ctrl.mode() == Mode::FAULT && (r.ctrl.faults() & F_CONTROL_DEADLINE));
  CHECK(!r.out.oe_legs && !r.last.oe_legs);

  r.queue(MSG_CLEARFAULT, nullptr, 0);          // healthy again: the clear goes through both layers
  r.period();
  r.period();
  CHECK(r.ctrl.mode() == Mode::IDLE && r.ctrl.faults() == 0 && !r.sup.latched());
  r.queue(MSG_ENABLE, &on, 1);
  r.period();
  r.period();
  CHECK(r.ctrl.mode() == Mode::RUN && r.last.oe_legs);
}

static void test_hang_stops_watchdog() {
  Rig r;
  uint8_t on = 1;
  r.queue(MSG_ENABLE, &on, 1);
  r.period();
  r.period();
  CHECK(r.last.oe_legs && r.last.refresh_watchdog);
  r.period(false);                              // control task hangs: no completion this period
  CHECK(r.last.refresh_watchdog);               // still within the next cycle's deadline
  r.period(false);
  CHECK(!r.last.refresh_watchdog && !r.last.oe_legs && r.last.latched);
  for (int k = 0; k < 5; ++k) {
    r.period(false);
    CHECK(!r.last.refresh_watchdog);            // never refreshed again: the IWDG resets the MCU
  }
}

static void test_clear_refused_while_unhealthy() {
  Rig r;
  r.period();
  r.period(false);
  r.period(false);                              // stalled long enough to latch
  CHECK(r.sup.latched() && !r.sup.healthy());
  r.queue(MSG_CLEARFAULT, nullptr, 0);
  r.period();                                   // control runs, sees supervisor unhealthy at the start of the cycle
  CHECK(r.ctrl.mode() == Mode::FAULT && (r.ctrl.faults() & F_CONTROL_DEADLINE));
  CHECK(r.sup.latched());
  r.period();
  r.queue(MSG_CLEARFAULT, nullptr, 0);          // healthy now
  r.period();
  r.period();
  CHECK(r.ctrl.mode() == Mode::IDLE && !r.sup.latched());
}

static void test_test_hooks_gated() {
  Rig r;
  TestHook h = {2, 0};
  r.queue(MSG_TESTHOOK, &h, sizeof(h));
  r.period();
  TestHook got;
  CHECK(!r.ctrl.take_test_hook(got));          // disabled by default
  r.ctrl.enable_test_hooks(true);
  h = {1, 7};
  r.queue(MSG_TESTHOOK, &h, sizeof(h));
  r.period();
  CHECK(r.ctrl.take_test_hook(got) && got.action == 1 && got.arg_ms == 7);
  CHECK(!r.ctrl.take_test_hook(got));
}

// ---------------------------------------------------------------- statistics and plumbing
static void test_loop_stats() {
  LoopStats s(5000);
  for (int i = 0; i < 1000; ++i) {
    uint32_t rel = 100000 + i * 5000;
    uint32_t late = (i % 100 == 0) ? 400 : 20;  // 1% of cycles start 400 us late
    s.record(rel, rel + late, rel + late + 1500);
  }
  CHECK(s.cycles() == 1000 && s.missed() == 0);
  CHECK(s.jitter_max_us() == 400 && s.exec_max_us() == 1500);
  CHECK(s.jitter_p99_us() == 30);               // 99% at 20 us: upper edge of the 20..30 us bin
  CHECK(s.exec_p99_us() == 1510);
  CHECK(s.exec_avg_us() == 1500);
  CHECK(s.record(0, 100, 5001));                // finishes 1 us after the deadline
  CHECK(s.missed() == 1);
  CHECK(!s.record(0xFFFFF000u, 0xFFFFF010u, 0x00000300u));   // across the clock wrap, within deadline
  s.reset();
  CHECK(s.cycles() == 0 && s.jitter_p99_us() == 0);
}

static void test_ring_and_seqlock() {
  ByteRing<16> r;
  uint8_t a[10] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10}, b[16];
  CHECK(r.write(a, 10) && !r.write(a, 7) && r.free() == 6);   // all or nothing
  CHECK(r.read(b, 4) == 4 && b[3] == 4);
  CHECK(r.write(a, 10));                         // wraps
  CHECK(r.read(b, 16) == 16 && b[0] == 5 && b[5] == 10 && b[6] == 1 && b[15] == 10);
  CHECK(r.used() == 0);
  struct S {
    uint32_t a, b;
  };
  Seqlock<S> q;
  S v{};
  CHECK(q.read(v) && v.a == 0);
  q.write({3, 4});
  CHECK(q.read(v) && v.a == 3 && v.b == 4);
}

int main() {
  test_startup();
  test_normal_cycles();
  test_deadline_miss();
  test_stall();
  test_clock_wrap();
  test_late_cycle_hidden_by_next();
  test_release_grid();
  test_stall_latch_delay();
  test_persistent_overrun_stops_watchdog();
  test_reset_never_enables();
  test_oe_follows_controller_and_supervisor();
  test_overrun_latches_fault();
  test_hang_stops_watchdog();
  test_clear_refused_while_unhealthy();
  test_test_hooks_gated();
  test_loop_stats();
  test_ring_and_seqlock();
  printf("%d checks, %d failed\n", checks, failures);
  return failures ? 1 : 0;
}
