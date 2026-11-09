/* Board configuration for Mico on the STM32F411CEU6 Black Pill. Written in STM32CubeMX's layout so the
 * project can be regenerated from CubeMX later; peripheral settings are listed in firmware/stm32f411/README.md. */
#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

/* Pins */
#define OE_LEGS_Pin       GPIO_PIN_0   /* PB0  -> PCA9685 #1 OE, active low, 10k pull-up on the board */
#define OE_ARMS_Pin       GPIO_PIN_1   /* PB1  -> PCA9685 #2 OE, active low, 10k pull-up on the board */
#define OE_GPIO_Port      GPIOB
#define MUX_S0_Pin        GPIO_PIN_12  /* PB12..PB15 -> CD74HC4067 S0..S3 */
#define MUX_GPIO_Port     GPIOB
#define LED_Pin           GPIO_PIN_13  /* PC13, active low */
#define LED_GPIO_Port     GPIOC
#define ADC_CH_MUX        ADC_CHANNEL_0  /* PA0: mux SIG */
#define ADC_CH_NTC_L      ADC_CHANNEL_1  /* PA1 */
#define ADC_CH_NTC_R      ADC_CHANNEL_2  /* PA2 */

extern ADC_HandleTypeDef hadc1;
extern I2C_HandleTypeDef hi2c1;   /* PB6 SCL, PB7 SDA: PCA9685 #1 (0x40), #2 (0x41) */
extern I2C_HandleTypeDef hi2c2;   /* PB10 SCL, PB3 SDA: INA226 0x44, 0x45, MPU6050 0x68 */
extern IWDG_HandleTypeDef hiwdg;
extern TIM_HandleTypeDef htim5;   /* 32-bit free-running 1 MHz: the monotonic clock */
extern UART_HandleTypeDef huart1; /* PA9 TX, PA10 RX, 921600 8N1, DMA2 stream 2 RX (circular), stream 7 TX */
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;

void Error_Handler(void);
void MX_IWDG_Init(uint32_t reload);   /* started by the supervisor task, never by main() */
uint32_t mico_reset_flags(void);      /* RCC->CSR captured at boot, before the flags are cleared */
void app_start(void);                 /* app/tasks.cpp: creates the tasks and starts the scheduler */

#ifdef __cplusplus
}
#endif

#endif /* MAIN_H */
