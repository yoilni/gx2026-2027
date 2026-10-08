#ifndef BOOT_INIT_H
#define BOOT_INIT_H

#include "stm32f4xx_hal.h"
#include <stdbool.h>

typedef HAL_StatusTypeDef (*BootInitAttempt)(void *context, uint32_t attempt);

/* Pre-scheduler only: UART2 is handed to debug_uart_task after kernel start. */
void BootInit_Begin(void);
void BootInit_SetUartReady(bool ready);
void BootInit_SetStage(const char *stage);
void BootInit_Logf(const char *format, ...);
void BootInit_DelayMs(uint32_t milliseconds);
HAL_StatusTypeDef BootInit_CheckHalTick(void);
HAL_StatusTypeDef BootInit_Run(const char *stage, BootInitAttempt function,
                             void *context);
HAL_StatusTypeDef BootInit_ConfigOscillators(RCC_OscInitTypeDef *config);
HAL_StatusTypeDef BootInit_ConfigClock(RCC_ClkInitTypeDef *config,
                                      uint32_t flash_latency);
HAL_StatusTypeDef BootInit_InitCan(CAN_HandleTypeDef *hcan, bool start_motor_can);
void BootInit_FaultLoop(void);

#endif
