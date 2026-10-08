#include "boot_init.h"

#include "M2006.h"
#include "robot_config.h"
#include "usart.h"

#include <stdarg.h>
#include <stdio.h>

/* Kept in SRAM so an attached debugger can inspect a silent startup failure. */
static const char * volatile boot_stage = "EARLY_RESET";
static volatile uint32_t boot_attempt;
static volatile HAL_StatusTypeDef boot_last_status;
static bool boot_uart_ready;

typedef struct
{
  RCC_ClkInitTypeDef *config;
  uint32_t flash_latency;
} BootClockContext;

typedef struct
{
  CAN_HandleTypeDef *hcan;
  bool start_motor_can;
} BootCanContext;

void BootInit_Begin(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0U;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  boot_stage = "HAL_INIT";
  boot_attempt = 0U;
  boot_last_status = HAL_OK;
  boot_uart_ready = false;
}

void BootInit_SetUartReady(bool ready)
{
  boot_uart_ready = ready;
}

void BootInit_SetStage(const char *stage)
{
  boot_stage = stage;
  boot_attempt = 0U;
  boot_last_status = HAL_OK;
  BootInit_Logf("[BOOT-INIT] BEGIN %s tick=%lu\r\n", stage,
                (unsigned long)HAL_GetTick());
}

void BootInit_DelayMs(uint32_t milliseconds)
{
  /* CPU cycle counter: independent of TIM4 interrupts and FreeRTOS. Keep
     each comparison below one CYCCNT wrap, even for a long fault indication. */
  while (milliseconds != 0U)
  {
    uint32_t chunk = (milliseconds > 1000U) ? 1000U : milliseconds;
    uint32_t cycles = (SystemCoreClock / 1000U) * chunk;
    uint32_t start = DWT->CYCCNT;
    while ((uint32_t)(DWT->CYCCNT - start) < cycles)
    {
      __NOP();
    }
    milliseconds -= chunk;
  }
}

void BootInit_Logf(const char *format, ...)
{
  char text[128];
  va_list args;
  uint32_t start;
  uint32_t cycles;

  if (!boot_uart_ready || (huart2.Instance != USART2))
  {
    return;
  }
  va_start(args, format);
  (void)vsnprintf(text, sizeof(text), format, args);
  va_end(args);

  /* Raw, bounded polling also works inside Error_Handler with IRQs disabled.
     HAL_UART_Transmit's HAL tick timeout would not work in that situation. */
  start = DWT->CYCCNT;
  cycles = (SystemCoreClock / 1000U) * ROBOT_BOOT_UART_TIMEOUT_MS;
  for (uint32_t index = 0U; text[index] != '\0'; ++index)
  {
    while ((USART2->SR & USART_SR_TXE) == 0U)
    {
      if ((uint32_t)(DWT->CYCCNT - start) >= cycles) return;
    }
    USART2->DR = (uint8_t)text[index];
  }
  while ((USART2->SR & USART_SR_TC) == 0U)
  {
    if ((uint32_t)(DWT->CYCCNT - start) >= cycles) return;
  }
}

HAL_StatusTypeDef BootInit_CheckHalTick(void)
{
  uint32_t start = HAL_GetTick();
  uint32_t elapsed;

  BootInit_DelayMs(20U);
  elapsed = (uint32_t)(HAL_GetTick() - start);
  BootInit_Logf("[BOOT-INIT] HAL_TICK delta=%lu expected=20ms\r\n",
                (unsigned long)elapsed);
  if ((elapsed < 10U) || (elapsed > 40U))
  {
    boot_stage = "HAL_TICK";
    boot_last_status = HAL_TIMEOUT;
    return HAL_TIMEOUT;
  }
  return HAL_OK;
}

HAL_StatusTypeDef BootInit_Run(const char *stage, BootInitAttempt function,
                             void *context)
{
  HAL_StatusTypeDef status = HAL_ERROR;

  BootInit_SetStage(stage);
  if (function == NULL)
  {
    boot_last_status = HAL_ERROR;
    return HAL_ERROR;
  }
  for (uint32_t attempt = 1U; attempt <= ROBOT_BOOT_INIT_MAX_ATTEMPTS; ++attempt)
  {
    boot_attempt = attempt;
    BootInit_Logf("[BOOT-INIT] %s attempt=%lu/%lu\r\n", stage,
                  (unsigned long)attempt,
                  (unsigned long)ROBOT_BOOT_INIT_MAX_ATTEMPTS);
    /* HAL clock/CAN timeouts rely on TIM4; a stopped tick is a fault, not
       something another HAL call can safely wait or retry through. */
    if (BootInit_CheckHalTick() != HAL_OK) return HAL_TIMEOUT;
    status = function(context, attempt);
    boot_last_status = status;
    BootInit_Logf("[BOOT-INIT] %s result=%u tick=%lu\r\n", stage,
                  (unsigned int)status, (unsigned long)HAL_GetTick());
    if (status == HAL_OK) return HAL_OK;
    if (attempt < ROBOT_BOOT_INIT_MAX_ATTEMPTS)
    {
      BootInit_DelayMs(ROBOT_BOOT_INIT_RETRY_DELAY_MS);
    }
  }
  return status;
}

static HAL_StatusTypeDef BootInit_OscillatorAttempt(void *context,
                                                    uint32_t attempt)
{
  HAL_StatusTypeDef status;

  if (attempt > 1U)
  {
    /* Only used before timers/CAN/tasks are created. Return to HSI and clear
       incomplete PLL/HSE state; never run the mission on the fallback clock. */
    status = HAL_RCC_DeInit();
    MX_USART2_UART_Init();
    if (status != HAL_OK) return status;
    if (BootInit_CheckHalTick() != HAL_OK) return HAL_TIMEOUT;
  }
  return HAL_RCC_OscConfig((RCC_OscInitTypeDef *)context);
}

static HAL_StatusTypeDef BootInit_ClockAttempt(void *context, uint32_t attempt)
{
  BootClockContext *clock = context;
  HAL_StatusTypeDef status;

  (void)attempt;
  status = HAL_RCC_ClockConfig(clock->config, clock->flash_latency);
  /* APB1 changes from HSI to 42MHz. Correct UART2 BRR before any next log. */
  MX_USART2_UART_Init();
  return status;
}

HAL_StatusTypeDef BootInit_ConfigOscillators(RCC_OscInitTypeDef *config)
{
  if (config == NULL) return HAL_ERROR;
  return BootInit_Run("HSE_PLL", BootInit_OscillatorAttempt, config);
}

HAL_StatusTypeDef BootInit_ConfigClock(RCC_ClkInitTypeDef *config,
                                      uint32_t flash_latency)
{
  BootClockContext clock = {config, flash_latency};
  if (config == NULL) return HAL_ERROR;
  return BootInit_Run("CLOCK_SWITCH", BootInit_ClockAttempt, &clock);
}

static HAL_StatusTypeDef BootInit_CanAttempt(void *context, uint32_t attempt)
{
  BootCanContext *can = context;
  CAN_HandleTypeDef *hcan = can->hcan;
  HAL_StatusTypeDef status;

  if (attempt > 1U)
  {
    /* A failed HAL_CAN_Start leaves State=ERROR; calling Start again alone
       cannot recover. Reset hardware and handle, then reinstall the filter. */
    HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
    __HAL_RCC_CAN1_CLK_ENABLE();
    __HAL_RCC_CAN1_FORCE_RESET();
    __HAL_RCC_CAN1_RELEASE_RESET();
    HAL_NVIC_ClearPendingIRQ(CAN1_RX0_IRQn);
    hcan->State = HAL_CAN_STATE_RESET;
    hcan->ErrorCode = HAL_CAN_ERROR_NONE;
  }
  if (!can->start_motor_can || (attempt > 1U))
  {
    status = HAL_CAN_Init(hcan);
    if (status != HAL_OK) return status;
  }
  return can->start_motor_can ? M2006_Init(hcan) : HAL_OK;
}

HAL_StatusTypeDef BootInit_InitCan(CAN_HandleTypeDef *hcan, bool start_motor_can)
{
  BootCanContext can = {hcan, start_motor_can};
  if ((hcan == NULL) || (hcan->Instance != CAN1)) return HAL_ERROR;
  return BootInit_Run(start_motor_can ? "CAN_START" : "CAN_INIT",
                       BootInit_CanAttempt, &can);
}

void BootInit_FaultLoop(void)
{
  /* No reboot loop and no scheduler start. Keep SWD pins untouched. */
  if (boot_last_status == HAL_OK) boot_last_status = HAL_ERROR;
  for (;;)
  {
    BootInit_Logf("[BOOT-FAULT] stage=%s attempt=%lu hal=%u; STOPPED, RESET REQUIRED\r\n",
                  boot_stage, (unsigned long)boot_attempt,
                  (unsigned int)boot_last_status);
    for (uint32_t pulse = 0U; pulse < 3U; ++pulse)
    {
      HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
      BootInit_DelayMs(100U);
      HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
      BootInit_DelayMs(100U);
    }
    BootInit_DelayMs(1000U);
  }
}
