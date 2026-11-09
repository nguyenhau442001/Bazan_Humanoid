/* Interrupt handlers. SVC, PendSV and SysTick are FreeRTOS's (mapped in FreeRTOSConfig.h). */
#include "main.h"

/* A hard fault cuts the servo outputs at once; the watchdog then resets the MCU. */
static void outputs_off_and_wait(void) {
  OE_GPIO_Port->BSRR = OE_LEGS_Pin | OE_ARMS_Pin;
  for (;;) {
  }
}

void NMI_Handler(void) { outputs_off_and_wait(); }
void HardFault_Handler(void) { outputs_off_and_wait(); }
void MemManage_Handler(void) { outputs_off_and_wait(); }
void BusFault_Handler(void) { outputs_off_and_wait(); }
void UsageFault_Handler(void) { outputs_off_and_wait(); }
void DebugMon_Handler(void) {}

void DMA2_Stream2_IRQHandler(void) { HAL_DMA_IRQHandler(&hdma_usart1_rx); }
void DMA2_Stream7_IRQHandler(void) { HAL_DMA_IRQHandler(&hdma_usart1_tx); }
void USART1_IRQHandler(void) { HAL_UART_IRQHandler(&huart1); }
