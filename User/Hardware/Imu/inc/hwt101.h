#ifndef HWT101_H
#define HWT101_H

#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

#define HWT101_BAUD_RATE              115200U
#define HWT101_YAW_TIMEOUT_MS            500U
#define HWT101_POWER_SETTLE_MS           500U
#define HWT101_UNLOCK_WAIT_MS            200U
#define HWT101_ZERO_WAIT_MS              500U
#define HWT101_SAVE_WAIT_MS              200U

typedef struct
{
  int32_t yaw_cdeg; /* Signed centidegrees, -18000..17999, left-positive. */
  uint32_t sequence;
  uint32_t update_tick;
  uint16_t version;
} HWT101_Yaw;

typedef enum
{
  HWT101_UNINITIALIZED = 0,
  HWT101_WAIT_POWER,
  HWT101_WAIT_UNLOCK,
  HWT101_WAIT_ZERO,
  HWT101_WAIT_SAVE,
  HWT101_READY,
  HWT101_INIT_FAILED
} HWT101_StartupState;

/* Call once after CubeMX UART4 initialization (115200, 8N1). RX starts
   immediately; Service sends the boot-zero sequence from task context. */
HAL_StatusTypeDef HWT101_Init(UART_HandleTypeDef *huart);
void HWT101_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void HWT101_UART_ErrorCallback(UART_HandleTypeDef *huart);
/* Call from one task every20ms. Never call Service or DebugYaw from an ISR. */
void HWT101_Service(uint32_t now);
void HWT101_DebugYaw(uint32_t now);
HWT101_StartupState HWT101_GetStartupState(void);
/* False until zero/save commands finish AND a new valid angle frame arrives.
   A successful TX is not a sensor acknowledgement or calibration proof. */
bool HWT101_GetYaw(HWT101_Yaw *yaw);

#endif
