// Battery fuel gauge in software: state of charge (SoC) of the 2S pack from its terminal voltage and current.
// No hardware access: the control task feeds it the battery-lead INA226 (or the VBAT divider alone) every cycle.
//
// Method (the one used by fuel-gauge ICs without a learned cell model):
//   1. Start-up: SoC from the open-circuit voltage (OCV), the terminal voltage corrected for the IR drop.
//   2. Every cycle: coulomb counting, SoC -= I dt / capacity. Accurate over minutes, drifts over hours
//      (sensor offset, capacity error).
//   3. Drift correction: pull SoC towards the voltage-based estimate. Weak while the pack is loaded (the
//      voltage then carries polarisation the IR term does not model), stronger once the current has stayed
//      low long enough for the voltage to relax, and never on the flat middle of the curve more than its
//      slope justifies.
//   4. Booted under load: the first SoC carries the IR-model error, so it is corrected fast until the first rest.
// Without a current reading (i_valid false) the OCV is taken as the upper envelope of the terminal voltage
// (the voltage during the rests), which may only fall as fast as a 2 C discharge: about +-15 %.
#pragma once
#include <stdint.h>

namespace mico {

struct BatteryConfig {
  float capacity_mah = 1000.0f;   // rated capacity of the pack in the CAD cradle (2S LiPo ~1000 mAh)
  int cells = 2;
  float r_int_ohm = 0.080f;       // pack + leads + fuse + shunt, measured once: (V_rest - V_load) / I_load
  float rest_a = 0.40f;           // below this the robot is standing still (electronics only)
  uint32_t rest_settle_ms = 5000; // the voltage needs this long at rest to relax towards the OCV
  float tau_rest_s = 60.0f;       // correction time constant at rest
  float tau_load_s = 900.0f;      // correction time constant under load
  float tau_v_only_s = 30.0f;     // SoC filter when there is no current reading
  float env_fall_v_s = 0.0005f;   // per cell: the OCV envelope falls at most this fast (2 C on the curve)
  float empty_cell_v = 3.30f;     // OCV per cell counted as 0 % (leaves margin above the 3.0 V damage limit)
};

// OCV of one LiPo cell at rest (V) <-> SoC (0..1), piecewise linear on a 5 % grid.
float lipo_ocv_to_soc(float cell_v);
float lipo_soc_to_ocv(float soc);

class BatteryEstimator {
 public:
  explicit BatteryEstimator(const BatteryConfig& cfg = BatteryConfig()) : cfg_(cfg) {}

  // v_pack: terminal voltage (V). i_a: battery current, positive = discharge (A). i_valid: current is usable.
  // Call at a fixed rate; dt_ms is the time since the previous call.
  void update(uint32_t dt_ms, float v_pack, float i_a, bool i_valid);

  bool valid() const { return init_; }
  float soc() const { return soc_; }                      // 0..1
  uint8_t soc_pct() const { return (uint8_t)(soc_ * 100.0f + 0.5f); }
  float ocv_cell_v() const { return ocv_cell_; }          // IR-compensated, per cell (V)
  bool resting() const { return rest_ms_ >= cfg_.rest_settle_ms; }
  int cells() const { return cfg_.cells; }

 private:
  BatteryConfig cfg_;
  bool init_ = false;
  float soc_ = 0.0f;
  float ocv_cell_ = 0.0f;
  float v_filt_ = 0.0f;
  uint32_t rest_ms_ = 0;
  bool provisional_ = false;      // first SoC taken under load: not yet confirmed at rest
};

}  // namespace mico
