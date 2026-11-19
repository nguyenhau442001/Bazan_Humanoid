// Battery fuel gauge against a simulated 2S LiPo that is deliberately NOT the estimator's model:
// aged capacity, different internal resistance, an RC polarisation branch, a perturbed OCV curve and an
// INA226 with gain, offset and noise. Load: walking bursts with rests, at the 200 Hz control rate.
// Run: make -C lib/core test
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "battery.h"

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

static uint32_t rng = 12345;
static float noise() {                       // uniform in [-1, 1], deterministic
  rng = rng * 1664525u + 1013904223u;
  return (float)(rng >> 8) / 8388608.0f - 1.0f;
}

struct Pack {                                // the "real" battery
  float cap_mah = 900.0f;                    // 10 % below the 1000 mAh the estimator assumes
  float r0 = 0.060f;                         // the estimator assumes 0.080
  float r1 = 0.035f, tau1 = 25.0f;           // polarisation the estimator does not model
  float soc = 1.0f;                          // raw table SoC
  float v1 = 0.0f;                           // voltage across the RC branch
  float ocv_cell() const { return lipo_soc_to_ocv(soc) + 0.008f * sinf(soc * 17.0f); }  // not the table
  float step(float i, float dt) {            // returns terminal voltage
    soc -= i * dt / (cap_mah * 3.6f);
    v1 += (i * r1 - v1) * dt / tau1;
    return 2.0f * ocv_cell() - i * r0 - v1;
  }
};

// Robot load at time t (s): 50 s walking (gait ripple and steps), 20 s standing, every 5 min a 2 min rest.
static float load(float t) {
  float m = fmodf(t, 300.0f);
  if (m >= 180.0f) return 0.30f;
  float c = fmodf(m, 70.0f);
  if (c >= 50.0f) return 0.35f;
  return 3.0f + 1.5f * sinf(t * 2.0f * 3.14159f * 1.6f) + (fmodf(t, 0.625f) < 0.08f ? 2.0f : 0.0f);
}

// True SoC on the estimator's scale (0 % at its empty voltage).
static float true_soc(const Pack& p, const BatteryConfig& cfg) {
  float s0 = lipo_ocv_to_soc(cfg.empty_cell_v);
  float s = (p.soc - s0) / (1.0f - s0);
  return s < 0 ? 0 : (s > 1 ? 1 : s);
}

// Discharge from soc_start until the true SoC reaches 3 %; returns the worst error after the first minute.
static float run(float soc_start, bool with_current, float start_rest_s, float* worst_low) {
  BatteryConfig cfg;
  BatteryEstimator est(cfg);
  Pack p;
  p.soc = soc_start;
  const float dt = 0.005f;
  float worst = 0, worst_lo = 0, t = 0;
  for (int k = 0; true_soc(p, cfg) > 0.03f && k < 2000000; ++k, t += dt) {
    float i = t < start_rest_s ? 0.30f : load(t);
    float v = p.step(i, dt);
    float v_meas = v + 0.005f * noise();                       // INA226 bus LSB 1.25 mV + ripple
    float i_meas = 1.02f * i + 0.015f + 0.05f * noise();       // 2 % gain, 15 mA offset, 50 mA noise
    est.update(5, v_meas, i_meas, with_current);
    if (t < 60.0f) continue;
    float e = fabsf(est.soc() - true_soc(p, cfg));
    if (e > worst) worst = e;
    if (true_soc(p, cfg) < 0.20f && e > worst_lo) worst_lo = e;
  }
  if (worst_low) *worst_low = worst_lo;
  printf("  start %3.0f %%, %-12s, %4.1f min: worst error %4.1f %%, below 20 %%: %4.1f %%\n", soc_start * 100,
         with_current ? "V + I" : "voltage only", t / 60.0f, worst * 100, worst_lo * 100);
  return worst;
}

static void test_table() {
  CHECK(lipo_ocv_to_soc(4.25f) == 1.0f);
  CHECK(lipo_ocv_to_soc(3.0f) == 0.0f);
  NEAR(lipo_ocv_to_soc(3.84f), 0.50f, 1e-4);
  float prev = -1;
  for (int k = 0; k <= 100; ++k) {
    float s = k / 100.0f, v = lipo_soc_to_ocv(s);
    CHECK(v > prev);                                           // strictly increasing
    NEAR(lipo_ocv_to_soc(v), s, 1e-4);                         // round trip
    prev = v;
  }
}

static void test_start_up() {
  BatteryEstimator est;
  CHECK(!est.valid());
  est.update(5, 0.0f, 0.0f, true);                             // no reading: still invalid
  CHECK(!est.valid());
  est.update(5, 8.40f, 0.0f, true);                            // full pack at rest
  CHECK(est.valid());
  CHECK(est.soc_pct() == 100);
  BatteryEstimator loaded;
  loaded.update(5, 2 * 3.84f - 4.0f * 0.080f, 4.0f, true);     // half full, booted while drawing 4 A
  NEAR(loaded.soc(), (0.50f - lipo_ocv_to_soc(3.30f)) / (1.0f - lipo_ocv_to_soc(3.30f)), 0.01);
}

static void test_rest_detection() {
  BatteryEstimator est;
  for (int k = 0; k < 200; ++k) est.update(5, 7.8f, 3.0f, true);
  CHECK(!est.resting());
  for (int k = 0; k < 999; ++k) est.update(5, 7.8f, 0.2f, true);
  CHECK(!est.resting());                                       // 4.995 s: not yet relaxed
  est.update(5, 7.8f, 0.2f, true);
  CHECK(est.resting());
  est.update(5, 7.8f, 1.0f, true);
  CHECK(!est.resting());
}

int main() {
  test_table();
  test_start_up();
  test_rest_detection();
  float lo;
  CHECK(run(1.00f, true, 10.0f, &lo) <= 0.05f);                // full pack, the normal case
  CHECK(lo <= 0.04f);                                          // and tighter near empty, where it matters
  CHECK(run(0.60f, true, 10.0f, nullptr) <= 0.05f);            // part-used pack
  CHECK(run(0.85f, true, 0.0f, nullptr) <= 0.05f);             // booted straight into walking
  CHECK(run(1.00f, false, 10.0f, nullptr) <= 0.15f);           // divider only, no current sensor
  CHECK(run(0.85f, false, 0.0f, nullptr) <= 0.15f);            // divider only, booted while walking
  printf("test_battery: %d checks, %d failures\n", checks, failures);
  return failures ? 1 : 0;
}
