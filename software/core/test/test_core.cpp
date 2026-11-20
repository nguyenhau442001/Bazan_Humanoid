// Host unit tests for lib/core (controller, trajectory, conversions, link). Run: make -C lib/core test
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "controller.h"
#include "conversions.h"
#include "frame.h"
#include "trajectory.h"

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
#define NEAR(a, b, tol) CHECK(fabs((double)(a) - (double)(b)) <= (tol))

// ---------------------------------------------------------------- helpers
static Inputs idle_inputs(uint32_t now) {
  Inputs in = {};
  in.now_ms = now;
  for (int k = 0; k < NNTC; ++k) {
    in.ntc_adc[k] = 2048;                 // 10k against 10k: 25 C
    in.ntc_present[k] = true;
  }
  in.rail_v[0] = in.rail_v[1] = 6.0f;
  in.rail_valid = true;
  in.pwm_ok = in.sensors_ok = true;
  in.supervisor_healthy = true;
  return in;
}

// Decode every frame the controller wants to send; return the last frame of `type` (payload copied).
static bool last_frame(Controller& c, uint8_t type, uint8_t* payload, size_t* len) {
  uint8_t buf[256];
  bool found = false;
  FrameParser p;
  size_t n;
  while ((n = c.take_tx(buf, sizeof(buf))) > 0) {
    Frame f;
    for (size_t i = 0; i < n; ++i)
      if (p.push(buf[i], f) && f.type == type) {
        memcpy(payload, f.payload, f.len);
        *len = f.len;
        found = true;
      }
  }
  return found;
}

static void send(Controller& c, uint8_t type, const void* payload, size_t n, uint32_t now) {
  uint8_t buf[600];
  size_t m = frame_build(type, 0, payload, n, buf, sizeof(buf));
  c.on_bytes(buf, m, now);
}

static void send_traj(Controller& c, const float* q, uint16_t dt_ms, bool replace, uint32_t now) {
  uint8_t buf[sizeof(Traj) + sizeof(TrajPoint)];
  Traj h = {static_cast<uint8_t>(replace ? 1 : 0), 1};
  TrajPoint p;
  p.dt_ms = dt_ms;
  for (int j = 0; j < NJ; ++j) p.q[j] = q_to_wire(q[j]);
  memcpy(buf, &h, sizeof(h));
  memcpy(buf + sizeof(h), &p, sizeof(p));
  send(c, MSG_TRAJ, buf, sizeof(buf), now);
}

// Run the 200 Hz loop from t0 to t1 (exclusive) with the same inputs.
static void run(Controller& c, Inputs in, uint32_t t0, uint32_t t1, Outputs& out) {
  for (uint32_t t = t0; t < t1; t += 1000 / LOOP_HZ) {
    in.now_ms = t;
    c.tick(in, out);
  }
}

// ---------------------------------------------------------------- tests
static void test_frame() {
  uint8_t payload[300];
  for (int i = 0; i < 300; ++i) payload[i] = static_cast<uint8_t>(i % 7 == 0 ? 0 : i);   // zeros and long runs
  uint8_t buf[700];
  size_t n = frame_build(0x42, 9, payload, sizeof(payload), buf, sizeof(buf));
  CHECK(n > 0);
  for (size_t i = 0; i + 1 < n; ++i) CHECK(buf[i] != 0);
  CHECK(buf[n - 1] == 0);
  FrameParser p;
  Frame f;
  bool got = false;
  for (size_t i = 0; i < n; ++i) got = p.push(buf[i], f) || got;
  CHECK(got && f.type == 0x42 && f.seq == 9 && f.len == sizeof(payload));
  CHECK(memcmp(f.payload, payload, sizeof(payload)) == 0);

  buf[5] ^= 0x10;                                           // corrupt one byte: dropped, counted
  got = false;
  for (size_t i = 0; i < n; ++i) got = p.push(buf[i], f) || got;
  CHECK(!got && p.errors() == 1);

  uint8_t ones[254];
  memset(ones, 1, sizeof(ones));                            // exactly one full COBS block
  n = frame_build(1, 0, ones, sizeof(ones), buf, sizeof(buf));
  got = false;
  for (size_t i = 0; i < n; ++i) got = p.push(buf[i], f) || got;
  CHECK(got && f.len == sizeof(ones));
  NEAR(crc16(reinterpret_cast<const uint8_t*>("123456789"), 9), 0x29B1, 0);    // CRC-16/CCITT-FALSE check value
}

static void test_conversions() {
  NEAR(ntc_celsius(2048, 10000, 3950, 10000), 25.0, 0.05);
  CHECK(ntc_celsius(2400, 10000, 3950, 10000) < 25);       // higher ADC: higher resistance: colder
  CHECK(isnan(ntc_celsius(0, 10000, 3950, 10000)));
  CHECK(joint_to_pulse_us(0, 0) == 1500);
  CHECK(joint_to_pulse_us(0, 10.0f) == joint_to_pulse_us(0, Q_MAX[0]));      // clamped to the joint limit
  NEAR(pulse_to_pca_counts(1500, 25000000, 50), 1500 * 4096.0 / 20000, 2);
  NEAR(ina226_current_a(4000, 0.010f), 1.0, 1e-6);           // 4000 * 2.5 uV / 10 mOhm
  CHECK(q_to_wire(0.12345f) == 1235 && fabs(q_from_wire(1235) - 0.1235f) < 1e-6);
  float q;
  CHECK(!pot_to_angle(0, 2000, q));                          // not calibrated in calibration.json
}

static void test_trajectory() {
  Trajectory t;
  float a[NJ] = {}, b[NJ] = {}, out[NJ];
  b[3] = 1.0f;                                               // l_knee
  t.reset(a, 1000);
  t.push(100, b, 1000);
  t.sample(1050, out);
  NEAR(out[3], 0.5, 1e-5);
  CHECK(t.active(1050));
  t.sample(1200, out);
  NEAR(out[3], 1.0, 1e-6);
  CHECK(!t.active(1200));
  b[3] = 0.0f;                                               // queue ran dry: timing restarts at push time
  t.push(100, b, 5000);
  t.sample(5050, out);
  NEAR(out[3], 0.5, 1e-5);
  float far[NJ] = {};
  far[3] = 99.0f;
  t.reset(far, 0);
  t.sample(0, out);
  NEAR(out[3], Q_MAX[3], 1e-6);
}

static void test_modes_and_comm_loss() {
  Controller c;
  Outputs out;
  c.begin(0, false);
  CHECK(c.mode() == Mode::IDLE);
  run(c, idle_inputs(0), 0, 50, out);
  CHECK(!out.oe_legs && out.pulse_us[0] == 0);              // limp until enabled

  uint8_t on = 1;
  send(c, MSG_ENABLE, &on, 1, 50);
  CHECK(c.mode() == Mode::RUN);
  run(c, idle_inputs(50), 50, 60, out);
  CHECK(out.oe_legs && out.pulse_us[3] == joint_to_pulse_us(3, Q_STAND[3]));   // starts at the stand pose

  float q[NJ];
  memcpy(q, Q_STAND, sizeof(q));
  q[3] += 0.2f;
  send_traj(c, q, 200, true, 60);
  run(c, idle_inputs(60), 60, 160, out);
  NEAR(c.q_cmd()[3], Q_STAND[3] + 0.1f, 0.01);
  run(c, idle_inputs(160), 160, 255, out);
  CHECK(c.mode() == Mode::RUN);                              // still moving: no HOLD while points remain
  run(c, idle_inputs(255), 255, 300, out);
  NEAR(c.q_cmd()[3], q[3], 1e-4);                            // reached the target (wire unit is 1e-4 rad)
  // Trajectory done at 260 and nothing heard since 60: HOLD on the last point, then crouch.
  CHECK(c.mode() == Mode::HOLD);
  run(c, idle_inputs(300), 300, 300 + HOLD_TO_CROUCH_MS + 10, out);
  CHECK(c.mode() == Mode::CROUCH);
  run(c, idle_inputs(1310), 1310, 1310 + CROUCH_MS + 20, out);
  NEAR(c.q_cmd()[3], Q_CROUCH[3], 1e-4);
  CHECK(out.oe_legs);                                        // crouch keeps the servos powered

  send_traj(c, Q_STAND, 500, false, 3400);                  // the host is back: RUN from the crouch
  CHECK(c.mode() == Mode::RUN);
  run(c, idle_inputs(3400), 3400, 3650, out);
  CHECK(c.q_cmd()[3] < Q_CROUCH[3] && c.q_cmd()[3] > Q_STAND[3]);

  uint8_t off = 0;
  send(c, MSG_ENABLE, &off, 1, 3650);
  run(c, idle_inputs(3650), 3650, 3660, out);
  CHECK(c.mode() == Mode::IDLE && !out.oe_legs);
}

static void test_faults() {
  Controller c;
  Outputs out;
  c.begin(0, false);
  uint8_t on = 1;
  send(c, MSG_ENABLE, &on, 1, 0);
  Inputs in = idle_inputs(0);
  in.current_a[0] = I_TRIP_A + 1;                            // short spike: ignored
  run(c, in, 0, I_TRIP_MS - 50, out);
  CHECK(c.mode() != Mode::FAULT);
  in.current_a[0] = 0;
  run(c, in, 150, 200, out);
  in.current_a[0] = I_TRIP_A + 1;                            // sustained: latched fault, outputs off
  run(c, in, 200, 200 + I_TRIP_MS + 20, out);
  CHECK(c.mode() == Mode::FAULT && (c.faults() & F_OVERCURRENT_L) && !out.oe_legs);
  send(c, MSG_ENABLE, &on, 1, 500);
  CHECK(c.mode() == Mode::FAULT);                            // enable does not clear a fault
  send(c, MSG_CLEARFAULT, nullptr, 0, 510);
  CHECK(c.mode() == Mode::FAULT && (c.faults() & F_OVERCURRENT_L));   // still over-current: refused
  uint8_t p[256];
  size_t plen = 0;
  CHECK(last_frame(c, MSG_LOG, p, &plen) && plen > 1 && strstr(reinterpret_cast<char*>(p + 1), "over-current"));
  CHECK(!c.take_clear_request());
  in.current_a[0] = 0;
  run(c, in, 520, 530, out);
  send(c, MSG_CLEARFAULT, nullptr, 0, 530);
  CHECK(c.mode() == Mode::IDLE && c.faults() == 0);
  CHECK(c.take_clear_request() && !c.take_clear_request());  // reported once

  Controller h;
  h.begin(0, false);
  send(h, MSG_ENABLE, &on, 1, 0);
  in = idle_inputs(0);
  in.ntc_adc[1] = 600;                                       // about 74 C
  run(h, in, 0, T_TRIP_MS + 20, out);
  CHECK(h.mode() == Mode::FAULT && (h.faults() & F_OVERTEMP));

  Controller u;                                              // under-voltage only counts with outputs on
  u.begin(0, false);
  in = idle_inputs(0);
  in.rail_v[1] = 4.0f;
  run(u, in, 0, 400, out);
  CHECK(u.mode() == Mode::IDLE);
  send(u, MSG_ENABLE, &on, 1, 400);
  run(u, in, 400, 400 + V_TRIP_MS + 20, out);
  CHECK(u.mode() == Mode::FAULT && (u.faults() & F_UNDERVOLT));

  Controller n;                                              // NTC not fitted: no over-temperature fault
  n.begin(0, false);
  send(n, MSG_ENABLE, &on, 1, 0);
  in = idle_inputs(0);
  in.ntc_adc[0] = in.ntc_adc[1] = 0;
  in.ntc_present[0] = in.ntc_present[1] = false;
  run(n, in, 0, 1000, out);
  CHECK(n.mode() != Mode::FAULT);

  Controller w;
  w.begin(0, true);
  CHECK(w.mode() == Mode::FAULT && (w.faults() & F_WATCHDOG_RESET));
}

static void test_link_messages() {
  Controller c;
  Outputs out;
  c.begin(0, false);
  send(c, MSG_HELLO, nullptr, 0, 0);
  uint8_t p[256];
  size_t n = 0;
  CHECK(last_frame(c, MSG_INFO, p, &n) && n == sizeof(Info));
  Info info;
  memcpy(&info, p, sizeof(info));
  CHECK(info.n_joints == NJ && info.table_hash == TABLE_HASH && info.proto == PROTO_VERSION);

  run(c, idle_inputs(0), 0, 20, out);
  CHECK(last_frame(c, MSG_STATE, p, &n) && n == sizeof(State));
  State s;
  memcpy(&s, p, sizeof(s));
  CHECK(s.mode == static_cast<uint8_t>(Mode::IDLE) && s.temp_c10[0] == 250 && s.q_meas[0] == INT16_MIN);
  CHECK(s.batt_mv == 0 && s.batt_ma == 0 && s.soc_pct == 255);  // no battery reading: unknown, not 0 %

  Inputs b = idle_inputs(20);                                // battery lead: full pack, electronics only
  b.batt_v = 8.35f;
  b.batt_a = 0.30f;
  b.batt_v_valid = b.batt_i_valid = true;
  run(c, b, 20, 40, out);
  CHECK(last_frame(c, MSG_STATE, p, &n) && n == sizeof(State));
  memcpy(&s, p, sizeof(s));
  CHECK(s.batt_mv == 8350 && s.batt_ma == 300 && s.soc_pct >= 97 && s.soc_pct <= 100);

  RawPulse r = {5, 1600};                                    // calibration: one servo pulsed, others limp
  send(c, MSG_RAWPULSE, &r, sizeof(r), 30);
  run(c, idle_inputs(30), 30, 40, out);
  CHECK(c.mode() == Mode::CALIB && out.pulse_us[5] == 1600 && out.pulse_us[4] == 0 && out.oe_legs);
  run(c, idle_inputs(40), 40, 40 + CALIB_TIMEOUT_MS + 20, out);
  CHECK(c.mode() == Mode::IDLE);                             // host went quiet: back to limp

  uint8_t bad[4] = {1, 9, 0, 0};                             // Traj claiming 9 points
  send(c, MSG_TRAJ, bad, sizeof(bad), 600);
  CHECK(last_frame(c, MSG_LOG, p, &n));
}

static Inputs batt_inputs(uint32_t now, float v, float a) {
  Inputs in = idle_inputs(now);
  in.batt_v = v;
  in.batt_a = a;
  in.batt_v_valid = in.batt_i_valid = true;
  return in;
}

static bool log_seen(Controller& c, const char* text) {
  uint8_t p[256] = {};
  size_t n = 0;
  return last_frame(c, MSG_LOG, p, &n) && n > 1 && strstr(reinterpret_cast<const char*>(p + 1), text);
}

static void test_battery_empty() {
  // Walking when the pack reaches 0 %: crouch under control, then outputs off and latched.
  Controller c;
  Outputs out;
  c.begin(0, false);
  uint8_t on = 1;
  send(c, MSG_ENABLE, &on, 1, 0);
  send_traj(c, Q_STAND, 5000, true, 0);                      // the host is streaming a motion
  const float v_empty = 2 * 3.28f - 3.0f * 0.080f;           // OCV below 3.30 V/cell while drawing 3 A
  run(c, batt_inputs(0, v_empty, 3.0f), 0, BATT_EMPTY_MS - 50, out);
  CHECK(c.mode() == Mode::RUN && out.oe_legs);              // debounced: not yet
  run(c, batt_inputs(BATT_EMPTY_MS - 50, v_empty, 3.0f), BATT_EMPTY_MS - 50, BATT_EMPTY_MS + 20, out);
  CHECK(c.mode() == Mode::CROUCH && out.oe_legs);           // sitting down, servos still powered
  CHECK(log_seen(c, "battery empty"));
  uint32_t t = BATT_EMPTY_MS + 20;
  send_traj(c, Q_STAND, 200, true, t);                       // the host keeps walking: refused
  CHECK(c.mode() == Mode::CROUCH);
  run(c, batt_inputs(t, v_empty, 1.0f), t, t + CROUCH_MS, out);
  NEAR(c.q_cmd()[3], Q_CROUCH[3], 1e-3);
  CHECK(c.mode() == Mode::CROUCH && out.oe_legs);           // crouched, settling
  t += CROUCH_MS;
  run(c, batt_inputs(t, v_empty, 0.3f), t, t + BATT_SETTLE_MS + 20, out);
  CHECK(c.mode() == Mode::FAULT && (c.faults() & F_BATT_EMPTY) && !out.oe_legs);
  t += BATT_SETTLE_MS + 20;
  send(c, MSG_CLEARFAULT, nullptr, 0, t);                    // still empty: clearing is refused
  run(c, batt_inputs(t, v_empty, 0.3f), t, t + 10, out);
  CHECK(c.mode() == Mode::FAULT);
  CHECK(log_seen(c, "battery empty"));

  // Idle on an empty pack: fault, so it cannot be enabled.
  Controller idle;
  idle.begin(0, false);
  run(idle, batt_inputs(0, 2 * 3.25f, 0.3f), 0, BATT_EMPTY_MS + 20, out);
  CHECK(idle.mode() == Mode::FAULT && (idle.faults() & F_BATT_EMPTY));
  send(idle, MSG_ENABLE, &on, 1, BATT_EMPTY_MS + 20);
  CHECK(idle.mode() == Mode::FAULT);

  // Terminal voltage under the 3.0 V/cell damage limit on a pack that still has charge: off at once.
  Controller cut;
  cut.begin(0, false);
  send(cut, MSG_ENABLE, &on, 1, 0);
  send_traj(cut, Q_STAND, 5000, true, 0);
  run(cut, batt_inputs(0, 7.9f, 1.0f), 0, 1000, out);       // healthy pack, about 75 %
  CHECK(cut.mode() == Mode::RUN && cut.battery().soc_pct() > 50);
  run(cut, batt_inputs(1000, 5.9f, 9.0f), 1000, 1000 + BATT_CUT_MS - 20, out);
  CHECK(cut.mode() == Mode::RUN);                            // a stance-phase dip shorter than the debounce
  run(cut, batt_inputs(1480, 7.6f, 2.0f), 1480, 1500, out);
  run(cut, batt_inputs(1500, 5.9f, 9.0f), 1500, 1500 + BATT_CUT_MS + 10, out);
  CHECK(cut.mode() == Mode::FAULT && (cut.faults() & F_BATT_EMPTY) && !out.oe_legs);

  // Low warning, once.
  Controller low;
  low.begin(0, false);
  run(low, batt_inputs(0, 2 * 3.70f, 0.3f), 0, 10, out);    // about 12 %
  CHECK(log_seen(low, "battery low"));
  run(low, batt_inputs(10, 2 * 3.70f, 0.3f), 10, 200, out);
  CHECK(!log_seen(low, "battery low"));
  CHECK(low.mode() == Mode::IDLE);

  // No battery reading: no battery protection (the servo-rail under-voltage still applies).
  Controller none;
  none.begin(0, false);
  run(none, idle_inputs(0), 0, BATT_EMPTY_MS + 100, out);
  CHECK(none.mode() == Mode::IDLE);
}

int main() {
  test_frame();
  test_conversions();
  test_trajectory();
  test_modes_and_comm_loss();
  test_faults();
  test_link_messages();
  test_battery_empty();
  printf("%d checks, %d failed\n", checks, failures);
  return failures ? 1 : 0;
}
