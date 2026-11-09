/* Peripheral pin and DMA set-up (STM32CubeMX's MSP layer). */
#include "main.h"

void HAL_MspInit(void) {
  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_RCC_PWR_CLK_ENABLE();
  HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);   /* all bits preempt priority: FreeRTOS requirement */
  HAL_NVIC_SetPriority(PendSV_IRQn, 15, 0);
}

void HAL_UART_MspInit(UART_HandleTypeDef* h) {
  if (h->Instance != USART1) return;
  GPIO_InitTypeDef g = {0};
  __HAL_RCC_USART1_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  g.Pin = GPIO_PIN_9 | GPIO_PIN_10;          /* PA9 TX, PA10 RX */
  g.Mode = GPIO_MODE_AF_PP;
  g.Pull = GPIO_PULLUP;
  g.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  g.Alternate = GPIO_AF7_USART1;
  HAL_GPIO_Init(GPIOA, &g);

  /* RX: DMA2 stream 2 channel 4, circular into the receive ring; the comm task follows NDTR. */
  hdma_usart1_rx.Instance = DMA2_Stream2;
  hdma_usart1_rx.Init.Channel = DMA_CHANNEL_4;
  hdma_usart1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
  hdma_usart1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_usart1_rx.Init.MemInc = DMA_MINC_ENABLE;
  hdma_usart1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  hdma_usart1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
  hdma_usart1_rx.Init.Mode = DMA_CIRCULAR;
  hdma_usart1_rx.Init.Priority = DMA_PRIORITY_VERY_HIGH;
  hdma_usart1_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
  if (HAL_DMA_Init(&hdma_usart1_rx) != HAL_OK) Error_Handler();
  __HAL_LINKDMA(h, hdmarx, hdma_usart1_rx);

  /* TX: DMA2 stream 7 channel 4, one transfer per batch of frames. */
  hdma_usart1_tx.Instance = DMA2_Stream7;
  hdma_usart1_tx.Init.Channel = DMA_CHANNEL_4;
  hdma_usart1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
  hdma_usart1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_usart1_tx.Init.MemInc = DMA_MINC_ENABLE;
  hdma_usart1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  hdma_usart1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
  hdma_usart1_tx.Init.Mode = DMA_NORMAL;
  hdma_usart1_tx.Init.Priority = DMA_PRIORITY_HIGH;
  hdma_usart1_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
  if (HAL_DMA_Init(&hdma_usart1_tx) != HAL_OK) Error_Handler();
  __HAL_LINKDMA(h, hdmatx, hdma_usart1_tx);

  HAL_NVIC_SetPriority(USART1_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
}

void HAL_I2C_MspInit(I2C_HandleTypeDef* h) {
  GPIO_InitTypeDef g = {0};
  g.Mode = GPIO_MODE_AF_OD;
  g.Pull = GPIO_PULLUP;                      /* boards carry 4.7k pull-ups; the internal ones only help */
  g.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  __HAL_RCC_GPIOB_CLK_ENABLE();
  if (h->Instance == I2C1) {
    g.Pin = GPIO_PIN_6 | GPIO_PIN_7;         /* PB6 SCL, PB7 SDA */
    g.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &g);
    __HAL_RCC_I2C1_CLK_ENABLE();
  } else if (h->Instance == I2C2) {
    g.Pin = GPIO_PIN_10;                     /* PB10 SCL */
    g.Alternate = GPIO_AF4_I2C2;
    HAL_GPIO_Init(GPIOB, &g);
    g.Pin = GPIO_PIN_3;                      /* PB3 SDA */
    g.Alternate = GPIO_AF9_I2C2;
    HAL_GPIO_Init(GPIOB, &g);
    __HAL_RCC_I2C2_CLK_ENABLE();
  }
}

void HAL_ADC_MspInit(ADC_HandleTypeDef* h) {
  if (h->Instance != ADC1) return;
  GPIO_InitTypeDef g = {0};
  __HAL_RCC_ADC1_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  g.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2;   /* mux SIG, NTC left, NTC right */
  g.Mode = GPIO_MODE_ANALOG;
  g.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &g);
}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* h) {
  if (h->Instance == TIM5) __HAL_RCC_TIM5_CLK_ENABLE();
}
