// Timing and safety thresholds. Values tied to docs/engineering/hardware_architecture.md.
#pragma once
#include <stdint.h>

namespace mico {

constexpr uint32_t LOOP_HZ = 200;            // sensing, safety and trajectory interpolation
constexpr uint32_t STATE_EVERY = 2;          // send State every 2nd loop: 100 Hz
constexpr uint32_t PWM_EVERY = 4;            // write servo pulses every 4th loop: 50 Hz servo frame

constexpr uint32_t COMM_HOLD_MS = 100;       // trajectory finished and nothing heard for this long: HOLD
constexpr uint32_t HOLD_TO_CROUCH_MS = 1000; // then crouch slowly
constexpr uint32_t CROUCH_MS = 2000;         // duration of the move into the crouch
constexpr uint32_t CALIB_TIMEOUT_MS = 500;   // calibration pulses stop when the host goes quiet

constexpr float I_TRIP_A = 6.0f;             // per leg branch (6 servos)
constexpr uint32_t I_TRIP_MS = 200;
constexpr float T_TRIP_C = 60.0f;            // servo case, the P0 stop condition
constexpr uint32_t T_TRIP_MS = 500;
constexpr float V_MIN_V = 5.2f;              // servo rail, only checked while outputs are on
constexpr uint32_t V_TRIP_MS = 200;
// Battery (software fuel gauge, battery-lead INA226). Empty: crouch under control, then outputs off.
constexpr uint8_t BATT_LOW_PCT = 15;         // host warning (logged once)
constexpr uint32_t BATT_EMPTY_MS = 1000;     // 0 % (rest voltage 3.30 V/cell) held this long: crouch and stop
constexpr float BATT_CUT_CELL_V = 3.00f;     // terminal voltage per cell, the damage limit: outputs off now
constexpr uint32_t BATT_CUT_MS = 500;        // longer than a stance-phase current peak
constexpr uint32_t BATT_SETTLE_MS = 500;     // after the crouch, before the outputs go off
constexpr uint8_t BATT_RESUME_PCT = 5;       // ClearFault accepted again from here (a fresh or charged pack)
constexpr float JAM_ERR_RAD = 0.2618f;       // 15 deg between command and pot
constexpr uint32_t JAM_MS = 500;
constexpr uint16_t I2C_MAX_ERRORS = 10;      // consecutive failed transfers before a fault

constexpr float NTC_R_FIXED = 10000.0f;      // pull-up from 3.3 V to the NTC, NTC to GND
constexpr float INA226_SHUNT_OHM = 0.010f;   // CJMCU-226 "R010", leg branches: 8.19 A range
constexpr float BATT_SHUNT_OHM = 0.005f;     // battery-lead INA226, shunt changed to R005: 16.4 A range (power_calc.md)

constexpr int TRAJ_QUEUE = 64;               // points buffered on the MCU
constexpr int MAX_FRAME = 512;               // bytes before COBS

}  // namespace mico
