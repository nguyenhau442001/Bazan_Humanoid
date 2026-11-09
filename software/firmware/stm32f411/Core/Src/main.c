/* Mico MCU: clock and peripheral initialisation in STM32CubeMX's layout, then the FreeRTOS application.
 * The servo outputs are made safe first: both PCA9685 OE lines are driven high (outputs off) before any
 * other peripheral starts, and nothing here enables them. Only the supervisor task drives OE. */
#include "main.h"

ADC_HandleTypeDef hadc1;
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
IWDG_HandleTypeDef hiwdg;
TIM_HandleTypeDef htim5;
UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

static uint32_t reset_flags;

static void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM5_Init(void);

int main(void) {
  /* USER CODE BEGIN 0: keep the reset cause (watchdog or power-on) before clearing the flags */
  reset_flags = RCC->CSR;
  __HAL_RCC_CLEAR_RESET_FLAGS();
  /* USER CODE END 0 */

  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();          /* OE high first */
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_ADC1_Init();
  MX_TIM5_Init();

  /* USER CODE BEGIN 2 */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;   /* keep DWT available for the debugger's profiling */
  HAL_TIM_Base_Start(&htim5);
  app_start();             /* never returns */
  /* USER CODE END 2 */

  for (;;) {
  }
}

uint32_t mico_reset_flags(void) { return reset_flags; }

/* HSE 25 MHz -> PLL (M 25, N 192, P 2, Q 4): SYSCLK 96 MHz, USB 48 MHz for later. APB1 48 MHz, APB2 96 MHz. */
static void SystemClock_Config(void) {
  RCC_OscInitTypeDef osc = {0};
  RCC_ClkInitTypeDef clk = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  osc.OscillatorType = RCC_OSCILLATORTYPE_HSE | RCC_OSCILLATORTYPE_LSI;
  osc.HSEState = RCC_HSE_ON;
  osc.LSIState = RCC_LSI_ON;   /* IWDG clock */
  osc.PLL.PLLState = RCC_PLL_ON;
  osc.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  osc.PLL.PLLM = 25;
  osc.PLL.PLLN = 192;
  osc.PLL.PLLP = RCC_PLLP_DIV2;
  osc.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&osc) != HAL_OK) Error_Handler();

  clk.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
  clk.APB1CLKDivider = RCC_HCLK_DIV2;
  clk.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_3) != HAL_OK) Error_Handler();
}

static void MX_GPIO_Init(void) {
  GPIO_InitTypeDef g = {0};
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();

  /* Output levels before the pins become outputs: OE high (servo outputs off), LED off. */
  HAL_GPIO_WritePin(OE_GPIO_Port, OE_LEGS_Pin | OE_ARMS_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(MUX_GPIO_Port, GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15, GPIO_PIN_RESET);

  g.Pin = OE_LEGS_Pin | OE_ARMS_Pin;
  g.Mode = GPIO_MODE_OUTPUT_PP;
  g.Pull = GPIO_NOPULL;
  g.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OE_GPIO_Port, &g);

  g.Pin = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
  HAL_GPIO_Init(MUX_GPIO_Port, &g);

  g.Pin = LED_Pin;
  HAL_GPIO_Init(LED_GPIO_Port, &g);
}

static void MX_DMA_Init(void) {
  __HAL_RCC_DMA2_CLK_ENABLE();
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);
}

static void MX_USART1_UART_Init(void) {
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 921600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_8;   /* 96 MHz / 921600 = 104.2: 0.16% error at x8 */
  if (HAL_UART_Init(&huart1) != HAL_OK) Error_Handler();
}

static void i2c_init(I2C_HandleTypeDef* h, I2C_TypeDef* inst) {
  h->Instance = inst;
  h->Init.ClockSpeed = 400000;
  h->Init.DutyCycle = I2C_DUTYCYCLE_2;
  h->Init.OwnAddress1 = 0;
  h->Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  h->Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  h->Init.OwnAddress2 = 0;
  h->Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  h->Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(h) != HAL_OK) Error_Handler();
}

static void MX_I2C1_Init(void) { i2c_init(&hi2c1, I2C1); }
static void MX_I2C2_Init(void) { i2c_init(&hi2c2, I2C2); }

static void MX_ADC1_Init(void) {
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;   /* 96 MHz / 4 = 24 MHz (max 36) */
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK) Error_Handler();
}

static void MX_TIM5_Init(void) {
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 96 - 1;               /* APB1 timer clock 96 MHz -> 1 MHz */
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 0xFFFFFFFFu;             /* 32-bit: wraps every 71.6 minutes */
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK) Error_Handler();
}

/* LSI ~32 kHz / 32 = 1 kHz; reload in ms. The LSI varies 17..47 kHz, so 50 means 34..94 ms. */
void MX_IWDG_Init(uint32_t reload) {
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_32;
  hiwdg.Init.Reload = reload;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK) Error_Handler();
}

void Error_Handler(void) {
  /* Outputs off and wait for the watchdog (if started) or a manual reset. */
  OE_GPIO_Port->BSRR = OE_LEGS_Pin | OE_ARMS_Pin;
  __disable_irq();
  for (;;) {
  }
}
