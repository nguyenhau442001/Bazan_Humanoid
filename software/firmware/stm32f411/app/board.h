// Hardware abstraction for the FreeRTOS application. hardware/board_stm32.cpp implements it with the
// STM32 HAL; test/posix/board_sim.cpp implements it for the host integration tests. Each function says
// which task may call it: the tasks never share a peripheral.
#pragma once
#include <stddef.h>
#include <stdint.h>

#include "controller.h"

namespace mico {
namespace board {

// ---- any task
uint32_t micros();                 // monotonic 32-bit microseconds (TIM5), wraps every 71.6 min
bool reset_by_watchdog();          // the last reset came from the IWDG
// Cause of the last reset, for Diag.reset_cause.
enum ResetCause : uint8_t { RESET_UNKNOWN = 0, RESET_POWER_ON = 1, RESET_PIN = 2, RESET_SOFTWARE = 3,
                            RESET_IWDG = 4, RESET_WWDG = 5, RESET_LOW_POWER = 6 };
uint8_t reset_cause();
// Servo outputs off now (OE high). Only ever disables, so any context may call it: fault handlers,
// FreeRTOS hooks, Error_Handler. Turning outputs on stays the supervisor's job.
void emergency_outputs_off();

// ---- control task only
// Configure the PCA9685s, INA226s and MPU6050 once, before the first control cycle (outputs are off).
void devices_init();
// Fill sensor inputs for cycle `cycle` (INA226 every 2nd cycle). Leaves in.pwm_ok and in.now_ms alone.
void read_inputs(Inputs& in, uint32_t cycle);
// Write servo pulses to one PCA9685 (0 legs, 1 arms). 0 us = channel off. False on an I2C error.
bool write_pwm(int pca, const uint16_t* pulse_us, int n);
void set_led(bool on);
void busy_wait_us(uint32_t us);    // TestHook only

// ---- supervisor task only
void set_oe(bool legs, bool arms);  // true = outputs enabled (pin low)
void watchdog_start();              // IWDG, about 50 ms; cannot be stopped once started
void watchdog_refresh();

// ---- comm task only
void uart_start();
size_t uart_read(uint8_t* buf, size_t max);    // bytes the RX DMA ring has received since the last call
bool uart_write(const uint8_t* buf, size_t n); // starts a DMA transfer; false while the previous one runs
uint32_t uart_rx_overruns();                    // RX losses: DMA ring lapped the reader, or a UART error

}  // namespace board
}  // namespace mico
