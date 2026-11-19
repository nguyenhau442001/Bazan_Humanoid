#include "battery.h"

namespace mico {

// Typical LiPo cell OCV after a rest, from 0 % to 100 % in 5 % steps (manufacturer curves at 0.2 C, 25 C).
static const float OCV[21] = {3.27f, 3.61f, 3.69f, 3.71f, 3.73f, 3.75f, 3.77f, 3.79f, 3.80f, 3.82f, 3.84f,
                              3.85f, 3.87f, 3.91f, 3.95f, 3.98f, 4.02f, 4.08f, 4.11f, 4.15f, 4.20f};

static float clamp01(float x) { return x < 0.0f ? 0.0f : (x > 1.0f ? 1.0f : x); }

float lipo_ocv_to_soc(float v) {
  if (v <= OCV[0]) return 0.0f;
  if (v >= OCV[20]) return 1.0f;
  int k = 0;
  while (v > OCV[k + 1]) ++k;
  return 0.05f * ((float)k + (v - OCV[k]) / (OCV[k + 1] - OCV[k]));
}

float lipo_soc_to_ocv(float soc) {
  float x = clamp01(soc) * 20.0f;
  int k = (int)x;
  if (k >= 20) return OCV[20];
  return OCV[k] + (x - (float)k) * (OCV[k + 1] - OCV[k]);
}

void BatteryEstimator::update(uint32_t dt_ms, float v_pack, float i_a, bool i_valid) {
  if (v_pack <= 0.5f * (float)cfg_.cells) return;            // no battery reading: keep the last estimate
  const float dt = (float)dt_ms * 1e-3f;
  const float i = i_valid ? i_a : 0.0f;
  ocv_cell_ = (v_pack + i * cfg_.r_int_ohm) / (float)cfg_.cells;
  // 0 % sits at empty_cell_v, 100 % at 4.20 V: rescale the table so the robot stops with a margin left.
  const float s0 = lipo_ocv_to_soc(cfg_.empty_cell_v);

  if (!init_) {
    v_filt_ = ocv_cell_;
    soc_ = clamp01((lipo_ocv_to_soc(ocv_cell_) - s0) / (1.0f - s0));
    provisional_ = i_valid && i >= cfg_.rest_a;
    init_ = true;
    return;
  }

  float tau;
  if (i_valid) {
    soc_ -= i * dt / (cfg_.capacity_mah * 3.6f * (1.0f - s0));
    rest_ms_ = (i < cfg_.rest_a) ? rest_ms_ + dt_ms : 0;
    if (rest_ms_ > 3600000u) rest_ms_ = 3600000u;
    v_filt_ = ocv_cell_;
    if (resting()) provisional_ = false;
    tau = (resting() || provisional_) ? cfg_.tau_rest_s : cfg_.tau_load_s;
  } else {
    if (ocv_cell_ > v_filt_) v_filt_ += (ocv_cell_ - v_filt_) * dt / (2.0f + dt);   // rests: rise in ~2 s
    else v_filt_ -= cfg_.env_fall_v_s * dt;                                           // load sag: ignored
    tau = cfg_.tau_v_only_s;
  }

  // Trust the voltage in proportion to how steep the curve is here: on the flat middle a 10 mV error is
  // several percent, at the ends it is less than one. Slope relative to the mean slope of the table.
  const float s_v = clamp01((lipo_ocv_to_soc(v_filt_) - s0) / (1.0f - s0));
  const float s_raw = s0 + soc_ * (1.0f - s0);
  const float slope = (lipo_soc_to_ocv(s_raw + 0.025f) - lipo_soc_to_ocv(s_raw - 0.025f)) / 0.05f;
  float w = slope / ((OCV[20] - OCV[0]) / 1.0f);
  if (w > 1.0f) w = 1.0f;
  if (!i_valid || provisional_) w = 1.0f;
  soc_ = clamp01(soc_ + (s_v - soc_) * w * dt / (tau + dt));
}

}  // namespace mico
