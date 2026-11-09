/* HAL time base on TIM11 (1 kHz), because SysTick belongs to FreeRTOS. */
#include "main.h"

static TIM_HandleTypeDef htim11;

HAL_StatusTypeDef HAL_InitTick(uint32_t TickPriority) {
  __HAL_RCC_TIM11_CLK_ENABLE();
  uint32_t clk = HAL_RCC_GetPCLK2Freq();     /* APB2 prescaler 1: timer clock = PCLK2 */
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = clk / 1000000U - 1U;
  htim11.Init.Period = 1000U - 1U;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK) return HAL_ERROR;
  HAL_NVIC_SetPriority(TIM1_TRG_COM_TIM11_IRQn, TickPriority, 0U);
  HAL_NVIC_EnableIRQ(TIM1_TRG_COM_TIM11_IRQn);
  uwTickPrio = TickPriority;
  return HAL_TIM_Base_Start_IT(&htim11);
}

void HAL_SuspendTick(void) { __HAL_TIM_DISABLE_IT(&htim11, TIM_IT_UPDATE); }
void HAL_ResumeTick(void) { __HAL_TIM_ENABLE_IT(&htim11, TIM_IT_UPDATE); }

void TIM1_TRG_COM_TIM11_IRQHandler(void) { HAL_TIM_IRQHandler(&htim11); }

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* h) {
  if (h->Instance == TIM11) HAL_IncTick();
}
