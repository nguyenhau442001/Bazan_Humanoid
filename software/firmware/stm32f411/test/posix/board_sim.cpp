// board.h on the host. The clock is the FreeRTOS tick (1 ms) in microseconds, so time only advances with
// the simulated scheduler and the test does not depend on how loaded the host is.
#include "board_sim.h"

#include "FreeRTOS.h"
#include "board.h"
#include "task.h"

namespace sim {
std::atomic<int> oe_legs{0}, oe_arms{0}, oe_ever_on{0};
std::atomic<uint32_t> refreshes{0};
std::atomic<int> watchdog_started{0};
bool watchdog_reset = false;
mico::ByteRing<4096> host_to_mcu;
mico::ByteRing<65536> mcu_to_host;
}  // namespace sim

namespace mico {
namespace board {

uint32_t micros() { return xTaskGetTickCount() * 1000u; }
uint8_t reset_cause() { return sim::watchdog_reset ? RESET_IWDG : RESET_POWER_ON; }
bool reset_by_watchdog() { return sim::watchdog_reset; }
void emergency_outputs_off() { sim::oe_legs = sim::oe_arms = 0; }

void devices_init() {}

void read_inputs(Inputs& in, uint32_t) {
  // Healthy readings; pots at 0 read as "not fitted", so the jam check stays out of the way.
  for (int k = 0; k < NNTC; ++k) {
    in.ntc_present[k] = true;
    in.ntc_adc[k] = 2048;                          // about 25 C
  }
  in.rail_v[0] = in.rail_v[1] = 6.0f;
  in.current_a[0] = in.current_a[1] = 0.5f;
  in.rail_valid = true;
  in.sensors_ok = true;
}

bool write_pwm(int, const uint16_t*, int) { return true; }
void set_led(bool) {}

void busy_wait_us(uint32_t us) {
  uint32_t t0 = micros();
  while (micros() - t0 < us) {
  }
}

void set_oe(bool legs, bool arms) {
  sim::oe_legs = legs;
  sim::oe_arms = arms;
  if (legs || arms) sim::oe_ever_on = 1;
}
void watchdog_start() { sim::watchdog_started = 1; }
void watchdog_refresh() { sim::refreshes.fetch_add(1); }

void uart_start() {}
size_t uart_read(uint8_t* buf, size_t max) { return sim::host_to_mcu.read(buf, max); }
bool uart_write(const uint8_t* buf, size_t n) { return sim::mcu_to_host.write(buf, n); }
uint32_t uart_rx_overruns() { return 0; }

}  // namespace board
}  // namespace mico
