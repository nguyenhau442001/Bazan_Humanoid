// Everything the MCU decides, with no hardware access: link messages, modes, trajectory, safety, pulses.
// main.cpp reads the sensors into Inputs, calls tick() at LOOP_HZ, and writes Outputs to the hardware.
// The same class runs in the host unit tests.
#pragma once
#include <stddef.h>
#include <stdint.h>

#include "config.h"
#include "frame.h"
#include "joint_table.h"
#include "protocol.h"
#include "trajectory.h"

namespace mico {

struct Inputs {
  uint32_t now_ms;
  uint16_t mux_adc[16];       // 0..11 leg pots in wire order, 12..15 FSR (l_heel, l_toe, r_heel, r_toe)
  uint16_t ntc_adc[NNTC];
  bool ntc_present[NNTC];     // fitted; a fitted NTC that reads open or shorted counts as over-temperature
  float current_a[2];         // left, right leg branch
  float rail_v[2];
  bool rail_valid;            // INA226 fitted: enables the over-current and under-voltage checks
  int16_t imu[6];             // MPU6050 raw: ax, ay, az, gx, gy, gz
  bool pwm_ok;                // last PCA9685 writes succeeded
  bool sensors_ok;            // last INA226 / MPU6050 reads succeeded
  bool supervisor_healthy;    // the supervisor sees the control task on time (a fault cannot be cleared otherwise)
};

struct Outputs {
  bool oe_legs;               // true: PCA9685 #1 outputs enabled
  bool oe_arms;               // true: PCA9685 #2 outputs enabled
  uint16_t pulse_us[NJ];      // 0: no pulse (servo limp)
};

class Controller {
 public:
  explicit Controller(uint8_t fw_major = 0, uint8_t fw_minor = 1) : fw_major_(fw_major), fw_minor_(fw_minor) {}

  void begin(uint32_t now_ms, bool watchdog_reset);
  // Bytes from the link (USB or UART). Replies are queued; collect them with take_tx().
  void on_bytes(const uint8_t* data, size_t n, uint32_t now_ms);
  void tick(const Inputs& in, Outputs& out);
  // Next pending frame to send (replies, logs, State). Returns its length, 0 if nothing is pending.
  size_t take_tx(uint8_t* out, size_t cap);

  void set_loop_us_max(uint16_t us) { loop_us_max_ = us; }
  // A fault found outside the controller (the supervisor). Latches FAULT like any internal fault.
  void raise_fault(uint16_t bits, uint32_t now_ms) { fault(bits, now_ms); }
  // True once after the host's ClearFault was accepted, so the caller can clear the supervisor's latch.
  bool take_clear_request() {
    bool r = clear_accepted_;
    clear_accepted_ = false;
    return r;
  }
  // TestHook messages are honoured only when enabled (M0 acceptance builds). Returns the pending request.
  void enable_test_hooks(bool on) { test_hooks_ = on; }
  bool take_test_hook(TestHook& h) {
    if (!test_hook_pending_) return false;
    h = test_hook_;
    test_hook_pending_ = false;
    return true;
  }
  Mode mode() const { return mode_; }
  uint16_t faults() const { return faults_; }
  const float* q_cmd() const { return q_cmd_; }

 private:
  void on_frame(const Frame& f, uint32_t now_ms);
  void enter(Mode m, uint32_t now_ms);
  void fault(uint16_t bits, uint32_t now_ms);
  void check_safety(const Inputs& in, uint32_t dt_ms);
  void queue(uint8_t type, const void* payload, size_t n);
  void log(uint8_t level, const char* text);
  const char* active_fault_condition() const;
  bool debounce(bool cond, uint32_t& timer, uint32_t dt_ms, uint32_t limit_ms);

  uint8_t fw_major_, fw_minor_;
  FrameParser parser_;
  Trajectory traj_;
  Mode mode_ = Mode::BOOT;
  uint16_t faults_ = 0;
  uint32_t mode_since_ = 0, last_rx_ = 0, last_tick_ = 0;
  bool have_tick_ = false;
  float q_cmd_[NJ] = {};
  float q_meas_[NLEG] = {};
  bool q_meas_ok_[NLEG] = {};
  int calib_joint_ = -1;
  uint16_t calib_us_ = 0;
  uint32_t t_oc_[2] = {}, t_ot_ = 0, t_uv_ = 0, t_jam_[NLEG] = {};
  uint16_t pwm_errs_ = 0, sensor_errs_ = 0;
  uint32_t ticks_ = 0;
  uint16_t loop_us_max_ = 0;
  uint8_t tx_seq_ = 0;
  bool clear_accepted_ = false;
  bool test_hooks_ = false, test_hook_pending_ = false;
  TestHook test_hook_ = {};
  Inputs last_in_ = {};

  // A small queue of encoded frames waiting to go out.
  static constexpr int TXQ = 6;
  static constexpr int TXLEN = 200;
  uint8_t txbuf_[TXQ][TXLEN];
  size_t txlen_[TXQ] = {};
  int txhead_ = 0, txcount_ = 0;
};

}  // namespace mico
