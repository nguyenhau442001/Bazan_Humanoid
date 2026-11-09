// Unit conversions: joint angle <-> servo pulse, pot ADC -> angle, NTC, INA226, wire format.
#pragma once
#include <stdint.h>

namespace mico {

float clampf(float v, float lo, float hi);

// Joint angle (rad) to servo pulse width (us), clamped to the joint limits and the servo's pulse range.
uint16_t joint_to_pulse_us(int j, float q);
// Pulse width to PCA9685 counts (12-bit, per servo frame), corrected for the measured oscillator.
uint16_t pulse_to_pca_counts(uint16_t us, uint32_t osc_hz, uint32_t frame_hz);

// Leg pot ADC (12-bit) to joint angle by the joint's two-point calibration. False if not calibrated.
bool pot_to_angle(int leg_j, uint16_t adc, float& q);

// NTC on the low side of a divider from 3.3 V: ADC (12-bit) to deg C with the Beta equation.
// Returns NaN for an open or shorted sensor.
float ntc_celsius(uint16_t adc, float r0, float beta, float r_fixed);

// INA226 registers to SI units.
inline float ina226_current_a(int16_t shunt_raw, float shunt_ohm) { return shunt_raw * 2.5e-6f / shunt_ohm; }
inline float ina226_bus_v(uint16_t bus_raw) { return bus_raw * 1.25e-3f; }

// Joint angle on the wire: int16 in 1/Q_SCALE rad.
int16_t q_to_wire(float q);
float q_from_wire(int16_t w);

// First-order low-pass, alpha in (0, 1].
struct Lowpass {
  float alpha;
  float y;
  bool init;
  float step(float x) {
    if (!init) {
      y = x;
      init = true;
    } else {
      y += alpha * (x - y);
    }
    return y;
  }
};

}  // namespace mico
