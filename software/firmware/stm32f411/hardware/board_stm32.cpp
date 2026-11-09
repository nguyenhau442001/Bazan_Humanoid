// board.h on the STM32F411 with the ST HAL. Peripherals are initialised by Core/Src/main.c (CubeMX layout);
// this file only uses them. Each peripheral belongs to one task (see board.h), so nothing here locks.
#include <atomic>
#include <math.h>
#include <string.h>

#include "board.h"
#include "config.h"
#include "conversions.h"
#include "joint_table.h"
#include "main.h"

#ifndef MICO_HAS_INA226
#define MICO_HAS_INA226 1
#endif
#ifndef MICO_NTC_MASK
#define MICO_NTC_MASK 0x3          // bit k: NTC k fitted
#endif
#ifndef MICO_HAS_IMU
#define MICO_HAS_IMU 1
#endif
#ifndef MICO_HAS_ARMS_PCA
#define MICO_HAS_ARMS_PCA 1        // PCA9685 #2 fitted
#endif

namespace mico {
namespace board {

namespace {

constexpr uint16_t PCA_ADDR[2] = {0x40 << 1, 0x41 << 1};   // HAL takes the 8-bit address
constexpr uint16_t INA_ADDR[2] = {0x44 << 1, 0x45 << 1};
constexpr uint16_t MPU_ADDR = 0x68 << 1;
constexpr uint32_t I2C_TIMEOUT_MS = 2;                     // HAL tick is 1 ms: 2 means 1..2 ms

// ---------------------------------------------------------------- I2C
// A failed transfer can leave the peripheral BUSY; re-initialising it is the standard F4 recovery.
bool i2c_check(I2C_HandleTypeDef* h, HAL_StatusTypeDef st) {
  if (st == HAL_OK) return true;
  HAL_I2C_DeInit(h);
  HAL_I2C_Init(h);
  return false;
}

bool reg_write(I2C_HandleTypeDef* h, uint16_t addr, uint8_t reg, const uint8_t* d, uint16_t n) {
  return i2c_check(h, HAL_I2C_Mem_Write(h, addr, reg, I2C_MEMADD_SIZE_8BIT, const_cast<uint8_t*>(d), n,
                                        I2C_TIMEOUT_MS));
}

bool reg_read(I2C_HandleTypeDef* h, uint16_t addr, uint8_t reg, uint8_t* d, uint16_t n) {
  return i2c_check(h, HAL_I2C_Mem_Read(h, addr, reg, I2C_MEMADD_SIZE_8BIT, d, n, I2C_TIMEOUT_MS));
}

bool reg_write16(I2C_HandleTypeDef* h, uint16_t addr, uint8_t reg, uint16_t v) {
  uint8_t d[2] = {static_cast<uint8_t>(v >> 8), static_cast<uint8_t>(v & 0xFF)};
  return reg_write(h, addr, reg, d, 2);
}

// ---------------------------------------------------------------- PCA9685 (I2C1)
bool pca_init(uint16_t addr) {
  uint8_t pre = static_cast<uint8_t>(lroundf(static_cast<float>(PCA_OSC_HZ) / (4096.0f * SERVO_FRAME_HZ)) - 1);
  uint8_t sleep = 0x10, ai = 0x20 | 0x01 /* auto-increment, all-call */, outdrv = 0x04;
  bool ok = reg_write(&hi2c1, addr, 0x00, &sleep, 1);      // MODE1: sleep before changing the prescaler
  ok = ok && reg_write(&hi2c1, addr, 0xFE, &pre, 1);
  ok = ok && reg_write(&hi2c1, addr, 0x01, &outdrv, 1);    // MODE2: totem-pole outputs
  ok = ok && reg_write(&hi2c1, addr, 0x00, &ai, 1);        // wake
  busy_wait_us(600);                                       // oscillator start-up
  uint8_t restart = ai | 0x80;
  return ok && reg_write(&hi2c1, addr, 0x00, &restart, 1);
}

// ---------------------------------------------------------------- ADC1
uint16_t adc_read(uint32_t channel) {
  ADC_ChannelConfTypeDef c = {};
  c.Channel = channel;
  c.Rank = 1;
  c.SamplingTime = ADC_SAMPLETIME_84CYCLES;                // 3.5 us at 24 MHz: pots and NTCs are 10k sources
  HAL_ADC_ConfigChannel(&hadc1, &c);
  uint16_t v = 0;
  for (int k = 0; k < 2; ++k) {                            // first conversion after a switch: discarded
    HAL_ADC_Start(&hadc1);
    if (HAL_ADC_PollForConversion(&hadc1, 1) == HAL_OK) v = static_cast<uint16_t>(HAL_ADC_GetValue(&hadc1));
  }
  return v;
}

void read_mux(uint16_t* adc) {
  for (int ch = 0; ch < 16; ++ch) {
    // S0..S3 = PB12..PB15 in one BSRR write: set the 1 bits, reset the 0 bits.
    uint32_t sel = static_cast<uint32_t>(ch) << 12;
    MUX_GPIO_Port->BSRR = sel | ((~sel & (0xFu << 12)) << 16);
    busy_wait_us(5);                                       // mux + RC settle
    adc[ch] = adc_read(ADC_CH_MUX);
  }
}

bool read_ina(Inputs& in, int k) {
  uint8_t d[2];
  if (!reg_read(&hi2c2, INA_ADDR[k], 0x01, d, 2)) return false;
  in.current_a[k] = ina226_current_a(static_cast<int16_t>((d[0] << 8) | d[1]), INA226_SHUNT_OHM);
  if (!reg_read(&hi2c2, INA_ADDR[k], 0x02, d, 2)) return false;
  in.rail_v[k] = ina226_bus_v(static_cast<uint16_t>((d[0] << 8) | d[1]));
  return true;
}

bool read_imu(Inputs& in) {
  uint8_t d[14];
  if (!reg_read(&hi2c2, MPU_ADDR, 0x3B, d, 14)) return false;
  const int idx[6] = {0, 2, 4, 8, 10, 12};                // skip the temperature word
  for (int k = 0; k < 6; ++k) in.imu[k] = static_cast<int16_t>((d[idx[k]] << 8) | d[idx[k] + 1]);
  return true;
}

bool pca_ready[2] = {false, false};

void sensors_init() {
  if (MICO_HAS_INA226)  // AVG 4, bus and shunt conversion 1.1 ms, continuous: a new value every 8.8 ms
    for (int k = 0; k < 2; ++k) reg_write16(&hi2c2, INA_ADDR[k], 0x00, 0x4327);
  if (MICO_HAS_IMU) {
    uint8_t wake = 0x00, dlpf = 0x03;                      // clock on, 44 Hz low-pass
    reg_write(&hi2c2, MPU_ADDR, 0x6B, &wake, 1);
    reg_write(&hi2c2, MPU_ADDR, 0x1A, &dlpf, 1);
  }
}

// ---------------------------------------------------------------- USART1 DMA
constexpr size_t RX_N = 512;     // 5.5 ms of bytes at 921600 baud; the comm task drains every 1 ms
constexpr size_t TX_N = 512;
uint8_t rx_dma[RX_N];
uint8_t tx_dma[TX_N];
std::atomic<uint32_t> rx_laps{0};        // DMA wrapped to the start (TC interrupt)
std::atomic<uint32_t> rx_restart{0};     // UART error: reception was aborted, restart in the comm task
std::atomic<uint32_t> tx_busy{0};
uint32_t rx_consumed = 0;                // bytes taken since the last (re)start, free-running
uint32_t rx_lost = 0;

void rx_begin() {
  rx_laps.store(0);
  rx_consumed = 0;
  HAL_UART_Receive_DMA(&huart1, rx_dma, RX_N);
  __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);       // only full laps are counted
}

}  // namespace

// ---------------------------------------------------------------- any task
uint32_t micros() { return TIM5->CNT; }

uint8_t reset_cause() {
  uint32_t f = mico_reset_flags();
  if (f & RCC_CSR_IWDGRSTF) return RESET_IWDG;
  if (f & RCC_CSR_WWDGRSTF) return RESET_WWDG;
  if (f & RCC_CSR_SFTRSTF) return RESET_SOFTWARE;
  if (f & RCC_CSR_LPWRRSTF) return RESET_LOW_POWER;
  if (f & (RCC_CSR_PORRSTF | RCC_CSR_BORRSTF)) return RESET_POWER_ON;
  if (f & RCC_CSR_PINRSTF) return RESET_PIN;               // set by every reset: checked last
  return RESET_UNKNOWN;
}

bool reset_by_watchdog() { return reset_cause() == RESET_IWDG; }

void emergency_outputs_off() { OE_GPIO_Port->BSRR = OE_LEGS_Pin | OE_ARMS_Pin; }

// ---------------------------------------------------------------- control task
void devices_init() {
  sensors_init();
  pca_ready[0] = pca_init(PCA_ADDR[0]);
  pca_ready[1] = !MICO_HAS_ARMS_PCA || pca_init(PCA_ADDR[1]);
}

void read_inputs(Inputs& in, uint32_t cycle) {
  read_mux(in.mux_adc);
  const uint32_t ntc_ch[NNTC] = {ADC_CH_NTC_L, ADC_CH_NTC_R};
  for (int k = 0; k < NNTC; ++k) {
    in.ntc_present[k] = (MICO_NTC_MASK >> k) & 1;
    in.ntc_adc[k] = in.ntc_present[k] ? adc_read(ntc_ch[k]) : 0;
  }
  static float i_last[2], v_last[2];                       // INA226 every 2nd cycle: hold between reads
  bool ok = true;
  in.rail_valid = MICO_HAS_INA226;
  if (MICO_HAS_INA226) {
    if (cycle % 2 == 0) {
      ok = read_ina(in, 0) && read_ina(in, 1);
      memcpy(i_last, in.current_a, sizeof(i_last));
      memcpy(v_last, in.rail_v, sizeof(v_last));
    } else {
      memcpy(in.current_a, i_last, sizeof(i_last));
      memcpy(in.rail_v, v_last, sizeof(v_last));
    }
  }
  if (MICO_HAS_IMU) ok = read_imu(in) && ok;
  in.sensors_ok = ok;
}

bool write_pwm(int pca, const uint16_t* pulse_us, int n) {
  if (pca == 1 && !MICO_HAS_ARMS_PCA) return true;
  if (!pca_ready[pca]) pca_ready[pca] = pca_init(PCA_ADDR[pca]);   // retry a board that failed to start
  if (!pca_ready[pca]) return false;
  // All channels of one board in a single auto-increment transfer (no 32-byte Wire limit with the HAL).
  uint8_t buf[4 * 16];
  for (int k = 0; k < n; ++k) {
    uint16_t v = pulse_us[k];
    uint16_t off = v ? pulse_to_pca_counts(v, PCA_OSC_HZ, SERVO_FRAME_HZ) : 0;
    buf[4 * k + 0] = 0;                                    // ON at count 0
    buf[4 * k + 1] = 0;
    buf[4 * k + 2] = static_cast<uint8_t>(off & 0xFF);
    buf[4 * k + 3] = static_cast<uint8_t>(v ? (off >> 8) : 0x10);   // 0x10: channel fully off (limp)
  }
  return reg_write(&hi2c1, PCA_ADDR[pca], 0x06, buf, static_cast<uint16_t>(4 * n));
}

void set_led(bool on) { HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, on ? GPIO_PIN_RESET : GPIO_PIN_SET); }

void busy_wait_us(uint32_t us) {
  uint32_t t0 = micros();
  while (micros() - t0 < us) {
  }
}

// ---------------------------------------------------------------- supervisor task
void set_oe(bool legs, bool arms) {
  // One BSRR write: high (off) bits in the low half, low (on) bits in the high half.
  uint32_t off = (legs ? 0 : OE_LEGS_Pin) | (arms ? 0 : OE_ARMS_Pin);
  uint32_t on = (legs ? OE_LEGS_Pin : 0) | (arms ? OE_ARMS_Pin : 0);
  OE_GPIO_Port->BSRR = off | (on << 16);
}

void watchdog_start() {
  __HAL_DBGMCU_FREEZE_IWDG();                              // halted in the debugger: no reset mid-session
  MX_IWDG_Init(50);                                        // ~50 ms (34..94 ms over the LSI range)
}

void watchdog_refresh() { HAL_IWDG_Refresh(&hiwdg); }

// ---------------------------------------------------------------- comm task
void uart_start() { rx_begin(); }

size_t uart_read(uint8_t* buf, size_t max) {
  if (rx_restart.exchange(0)) {
    ++rx_lost;
    HAL_UART_AbortReceive(&huart1);
    rx_begin();
  }
  // Bytes produced = full laps * N + DMA position. Read laps around the position so they agree.
  uint32_t laps, pos;
  do {
    laps = rx_laps.load();
    pos = RX_N - __HAL_DMA_GET_COUNTER(&hdma_usart1_rx);
  } while (laps != rx_laps.load());
  if (pos >= RX_N) pos = 0;
  uint32_t produced = laps * RX_N + pos;
  if (static_cast<int32_t>(produced - rx_consumed) < 0) produced += RX_N;   // wrapped, TC not yet handled
  uint32_t avail = produced - rx_consumed;
  // Lapped (or about to be, while we copy): drop the oldest bytes, keep a margin from the DMA write point.
  constexpr uint32_t KEEP = RX_N - 32;
  if (avail > KEEP) {
    rx_lost += avail - KEEP;
    rx_consumed = produced - KEEP;
    avail = KEEP;
  }
  size_t n = avail < max ? avail : max;
  for (size_t i = 0; i < n; ++i) buf[i] = rx_dma[(rx_consumed + i) % RX_N];
  rx_consumed += static_cast<uint32_t>(n);
  return n;
}

bool uart_write(const uint8_t* buf, size_t n) {
  if (tx_busy.load() || n > TX_N) return false;
  memcpy(tx_dma, buf, n);                                  // the caller's buffer is free once we return
  tx_busy.store(1);
  if (HAL_UART_Transmit_DMA(&huart1, tx_dma, static_cast<uint16_t>(n)) != HAL_OK) {
    tx_busy.store(0);
    return false;
  }
  return true;
}

uint32_t uart_rx_overruns() { return rx_lost; }

}  // namespace board
}  // namespace mico

// ---------------------------------------------------------------- HAL callbacks (USART1 interrupts)
extern "C" {

void HAL_UART_RxCpltCallback(UART_HandleTypeDef* h) {
  if (h == &huart1) mico::board::rx_laps.fetch_add(1);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef* h) {
  if (h == &huart1) mico::board::tx_busy.store(0);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef* h) {
  if (h != &huart1) return;
  // Overrun, noise or framing error: the HAL has aborted reception (and maybe transmission).
  if (h->gState == HAL_UART_STATE_READY) mico::board::tx_busy.store(0);
  mico::board::rx_restart.store(1);
}

uint32_t mico_runtime_counter(void) { return TIM5->CNT; }

}  // extern "C"
