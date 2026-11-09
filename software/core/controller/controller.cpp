#include "controller.h"

#include <math.h>
#include <string.h>

#include "conversions.h"

namespace mico {

namespace {
constexpr uint8_t TRAJ_REPLACE = 0x01;
bool outputs_on(Mode m) { return m == Mode::RUN || m == Mode::HOLD || m == Mode::CROUCH || m == Mode::CALIB; }
}  // namespace

void Controller::begin(uint32_t now_ms, bool watchdog_reset) {
  for (int j = 0; j < NJ; ++j) q_cmd_[j] = Q_STAND[j];
  traj_.reset(q_cmd_, now_ms);
  last_rx_ = now_ms;
  enter(Mode::IDLE, now_ms);
  if (watchdog_reset) fault(F_WATCHDOG_RESET, now_ms);
}

void Controller::enter(Mode m, uint32_t now_ms) {
  if (m == mode_) return;
  mode_ = m;
  mode_since_ = now_ms;
  if (m != Mode::CALIB) calib_joint_ = -1;
}

void Controller::fault(uint16_t bits, uint32_t now_ms) {
  bool fresh = (faults_ & bits) != bits;
  faults_ |= bits;
  enter(Mode::FAULT, now_ms);
  if (fresh) log(2, "fault: outputs off");
}

void Controller::on_bytes(const uint8_t* data, size_t n, uint32_t now_ms) {
  Frame f;
  for (size_t i = 0; i < n; ++i)
    if (parser_.push(data[i], f)) on_frame(f, now_ms);
}

void Controller::on_frame(const Frame& f, uint32_t now_ms) {
  last_rx_ = now_ms;
  switch (f.type) {
    case MSG_HELLO: {
      Info info = {fw_major_, fw_minor_, PROTO_VERSION, static_cast<uint8_t>(NJ), TABLE_HASH};
      queue(MSG_INFO, &info, sizeof(info));
      break;
    }
    case MSG_ENABLE: {
      if (f.len != sizeof(Enable)) return log(1, "bad Enable");
      if (f.payload[0] == 0) {
        if (mode_ != Mode::FAULT) enter(Mode::IDLE, now_ms);
      } else if (mode_ == Mode::IDLE) {
        // Start from where the legs are if the pots are calibrated, otherwise from the stand pose.
        for (int j = 0; j < NJ; ++j) q_cmd_[j] = (j < NLEG && q_meas_ok_[j]) ? q_meas_[j] : Q_STAND[j];
        traj_.reset(q_cmd_, now_ms);
        enter(Mode::RUN, now_ms);
      } else if (mode_ == Mode::FAULT) {
        log(1, "in FAULT: send ClearFault first");
      }
      break;
    }
    case MSG_TRAJ: {
      if (f.len < sizeof(Traj)) return log(1, "bad Traj");
      Traj h;
      memcpy(&h, f.payload, sizeof(h));
      if (h.n > TRAJ_MAX_POINTS || f.len != sizeof(Traj) + h.n * sizeof(TrajPoint)) return log(1, "bad Traj length");
      if (mode_ != Mode::RUN && mode_ != Mode::HOLD && mode_ != Mode::CROUCH) return log(1, "Traj ignored: not enabled");
      if (h.flags & TRAJ_REPLACE || mode_ != Mode::RUN) traj_.reset(q_cmd_, now_ms);
      enter(Mode::RUN, now_ms);
      for (int k = 0; k < h.n; ++k) {
        TrajPoint p;
        memcpy(&p, f.payload + sizeof(Traj) + k * sizeof(TrajPoint), sizeof(p));
        float q[NJ];
        for (int j = 0; j < NJ; ++j) q[j] = q_from_wire(p.q[j]);
        if (!traj_.push(p.dt_ms, q, now_ms)) return log(1, "trajectory queue full");
      }
      break;
    }
    case MSG_CLEARFAULT: {
      if (mode_ == Mode::FAULT) {
        // Clearing never overrides a condition that is still there: the fault would only come back,
        // or worse, not be re-detected (e.g. an over-temperature that is now debouncing from zero).
        if (const char* why = active_fault_condition()) {
          char msg[64] = "fault still present: ";
          strncat(msg, why, sizeof(msg) - strlen(msg) - 1);
          return log(1, msg);
        }
        clear_accepted_ = true;
        faults_ = 0;
        t_oc_[0] = t_oc_[1] = t_ot_ = t_uv_ = 0;
        memset(t_jam_, 0, sizeof(t_jam_));
        pwm_errs_ = sensor_errs_ = 0;
        enter(Mode::IDLE, now_ms);
      }
      break;
    }
    case MSG_RAWPULSE: {
      if (f.len != sizeof(RawPulse)) return log(1, "bad RawPulse");
      RawPulse r;
      memcpy(&r, f.payload, sizeof(r));
      if (mode_ != Mode::IDLE && mode_ != Mode::CALIB) return log(1, "RawPulse only from IDLE");
      if (r.joint >= NJ || r.pulse_us < MIN_US[r.joint] || r.pulse_us > MAX_US[r.joint]) return log(1, "RawPulse out of range");
      enter(Mode::CALIB, now_ms);
      calib_joint_ = r.joint;
      calib_us_ = r.pulse_us;
      break;
    }
    case MSG_TESTHOOK: {
      if (f.len != sizeof(TestHook)) return log(1, "bad TestHook");
      if (!test_hooks_) return log(1, "test hooks disabled in this build");
      memcpy(&test_hook_, f.payload, sizeof(test_hook_));
      test_hook_pending_ = true;
      break;
    }
    default:
      log(1, "unknown message");
  }
}

const char* Controller::active_fault_condition() const {
  const Inputs& in = last_in_;
  if (!in.supervisor_healthy) return "control deadline";
  if (in.rail_valid && (in.current_a[0] > I_TRIP_A || in.current_a[1] > I_TRIP_A)) return "over-current";
  for (int k = 0; k < NNTC; ++k) {
    if (!in.ntc_present[k]) continue;
    float t = ntc_celsius(in.ntc_adc[k], NTC_R0[k], NTC_BETA[k], NTC_R_FIXED);
    if (isnan(t) || t > T_TRIP_C) return "over-temperature or NTC open";
  }
  if (!in.pwm_ok) return "PWM I2C";
  if (!in.sensors_ok) return "sensor I2C";
  return nullptr;
}

bool Controller::debounce(bool cond, uint32_t& timer, uint32_t dt_ms, uint32_t limit_ms) {
  timer = cond ? timer + dt_ms : 0;
  return timer >= limit_ms;
}

void Controller::check_safety(const Inputs& in, uint32_t dt) {
  pwm_errs_ = in.pwm_ok ? 0 : pwm_errs_ + 1;
  sensor_errs_ = in.sensors_ok ? 0 : sensor_errs_ + 1;
  uint16_t f = 0;
  if (pwm_errs_ >= I2C_MAX_ERRORS) f |= F_I2C_PWM;
  if (sensor_errs_ >= I2C_MAX_ERRORS) f |= F_I2C_SENSOR;
  if (debounce(in.rail_valid && in.current_a[0] > I_TRIP_A, t_oc_[0], dt, I_TRIP_MS)) f |= F_OVERCURRENT_L;
  if (debounce(in.rail_valid && in.current_a[1] > I_TRIP_A, t_oc_[1], dt, I_TRIP_MS)) f |= F_OVERCURRENT_R;
  bool hot = false;
  for (int k = 0; k < NNTC; ++k) {
    if (!in.ntc_present[k]) continue;
    float t = ntc_celsius(in.ntc_adc[k], NTC_R0[k], NTC_BETA[k], NTC_R_FIXED);
    if (isnan(t) || t > T_TRIP_C) hot = true;          // a broken sensor is treated as hot
  }
  if (debounce(hot, t_ot_, dt, T_TRIP_MS)) f |= F_OVERTEMP;
  bool low = in.rail_valid && outputs_on(mode_) && (in.rail_v[0] < V_MIN_V || in.rail_v[1] < V_MIN_V);
  if (debounce(low, t_uv_, dt, V_TRIP_MS)) f |= F_UNDERVOLT;
  if (mode_ == Mode::RUN || mode_ == Mode::HOLD || mode_ == Mode::CROUCH) {
    for (int j = 0; j < NLEG; ++j) {
      bool jam = q_meas_ok_[j] && fabsf(q_meas_[j] - q_cmd_[j]) > JAM_ERR_RAD;
      if (debounce(jam, t_jam_[j], dt, JAM_MS)) f |= F_JOINT_JAM;
    }
  }
  if (f) fault(f, in.now_ms);
}

void Controller::tick(const Inputs& in, Outputs& out) {
  uint32_t now = in.now_ms;
  uint32_t dt = have_tick_ ? now - last_tick_ : 0;
  have_tick_ = true;
  last_tick_ = now;
  last_in_ = in;
  ++ticks_;

  for (int j = 0; j < NLEG; ++j) q_meas_ok_[j] = pot_to_angle(j, in.mux_adc[j], q_meas_[j]);

  if (mode_ != Mode::FAULT) check_safety(in, dt);

  uint32_t quiet = now - last_rx_;
  if (mode_ == Mode::RUN || mode_ == Mode::CROUCH) traj_.sample(now, q_cmd_);
  switch (mode_) {
    case Mode::RUN:
      if (!traj_.active(now) && quiet > COMM_HOLD_MS) enter(Mode::HOLD, now);
      break;
    case Mode::HOLD:
      if (now - mode_since_ >= HOLD_TO_CROUCH_MS) {
        traj_.reset(q_cmd_, now);
        traj_.push(CROUCH_MS, Q_CROUCH, now);
        enter(Mode::CROUCH, now);
      }
      break;
    case Mode::CALIB:
      if (quiet > CALIB_TIMEOUT_MS) enter(Mode::IDLE, now);
      break;
    default:
      break;
  }
  bool on = outputs_on(mode_);
  out.oe_legs = out.oe_arms = on;
  for (int j = 0; j < NJ; ++j) {
    if (mode_ == Mode::CALIB) {
      out.pulse_us[j] = (j == calib_joint_) ? calib_us_ : 0;
    } else {
      out.pulse_us[j] = on ? joint_to_pulse_us(j, q_cmd_[j]) : 0;
    }
  }

  if (ticks_ % STATE_EVERY == 0) {
    State s = {};
    s.t_ms = now;
    s.mode = static_cast<uint8_t>(mode_);
    s.faults = faults_;
    for (int j = 0; j < NJ; ++j) s.q_cmd[j] = q_to_wire(q_cmd_[j]);
    for (int j = 0; j < NLEG; ++j) s.q_meas[j] = q_meas_ok_[j] ? q_to_wire(q_meas_[j]) : INT16_MIN;
    memcpy(s.mux_adc, in.mux_adc, sizeof(s.mux_adc));
    for (int k = 0; k < NNTC; ++k) {
      float t = in.ntc_present[k] ? ntc_celsius(in.ntc_adc[k], NTC_R0[k], NTC_BETA[k], NTC_R_FIXED) : NAN;
      s.temp_c10[k] = isnan(t) ? INT16_MIN : static_cast<int16_t>(lroundf(t * 10));
    }
    for (int k = 0; k < 2; ++k) {
      s.i_ma[k] = static_cast<int16_t>(lroundf(clampf(in.current_a[k] * 1000, -32767, 32767)));
      s.v_mv[k] = static_cast<uint16_t>(lroundf(clampf(in.rail_v[k] * 1000, 0, 65535)));
    }
    memcpy(s.imu, in.imu, sizeof(s.imu));
    s.loop_us_max = loop_us_max_;
    s.rx_errors = parser_.errors();
    queue(MSG_STATE, &s, sizeof(s));
  }
}

void Controller::queue(uint8_t type, const void* payload, size_t n) {
  if (txcount_ == TXQ) {                     // drop the oldest: State is resent every 10 ms anyway
    txhead_ = (txhead_ + 1) % TXQ;
    --txcount_;
  }
  int slot = (txhead_ + txcount_) % TXQ;
  txlen_[slot] = frame_build(type, tx_seq_++, payload, n, txbuf_[slot], TXLEN);
  if (txlen_[slot]) ++txcount_;
}

void Controller::log(uint8_t level, const char* text) {
  uint8_t buf[1 + 64];
  size_t n = strlen(text);
  if (n > 64) n = 64;
  buf[0] = level;
  memcpy(buf + 1, text, n);
  queue(MSG_LOG, buf, n + 1);
}

size_t Controller::take_tx(uint8_t* out, size_t cap) {
  if (!txcount_) return 0;
  size_t n = txlen_[txhead_];
  if (n > cap) n = 0;
  else memcpy(out, txbuf_[txhead_], n);
  txhead_ = (txhead_ + 1) % TXQ;
  --txcount_;
  return n;
}

}  // namespace mico
