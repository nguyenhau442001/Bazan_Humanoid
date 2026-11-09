#include "conversions.h"

#include <math.h>

#include "joint_table.h"

namespace mico {

float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

uint16_t joint_to_pulse_us(int j, float q) {
  q = clampf(q, Q_MIN[j], Q_MAX[j]);
  float us = CENTER_US[j] + DIR[j] * US_PER_RAD[j] * q;
  return static_cast<uint16_t>(lroundf(clampf(us, MIN_US[j], MAX_US[j])));
}

uint16_t pulse_to_pca_counts(uint16_t us, uint32_t osc_hz, uint32_t frame_hz) {
  // The PCA9685 prescaler is round(osc / (4096 * f)) - 1, so the real frame is osc / (4096 * (pre + 1)).
  uint32_t pre = static_cast<uint32_t>(lroundf(static_cast<float>(osc_hz) / (4096.0f * frame_hz))) - 1;
  float frame_us = 4096.0f * (pre + 1) * 1e6f / osc_hz;
  float counts = us * 4096.0f / frame_us;
  return static_cast<uint16_t>(clampf(lroundf(counts), 0, 4095));
}

bool pot_to_angle(int j, uint16_t adc, float& q) {
  if (!POT_VALID[j] || POT_ADC1[j] == POT_ADC0[j]) return false;
  q = POT_Q0[j] + (static_cast<float>(adc) - POT_ADC0[j]) * (POT_Q1[j] - POT_Q0[j]) / (POT_ADC1[j] - POT_ADC0[j]);
  return true;
}

float ntc_celsius(uint16_t adc, float r0, float beta, float r_fixed) {
  if (adc <= 8 || adc >= 4087) return NAN;
  float r = r_fixed * adc / (4095.0f - adc);
  float inv_t = 1.0f / 298.15f + logf(r / r0) / beta;
  return 1.0f / inv_t - 273.15f;
}

int16_t q_to_wire(float q) {
  float w = q * Q_SCALE;
  return static_cast<int16_t>(lroundf(clampf(w, -32767.0f, 32767.0f)));
}

float q_from_wire(int16_t w) { return w / Q_SCALE; }

}  // namespace mico
