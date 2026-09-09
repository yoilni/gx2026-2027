#ifndef JY901S_H
#define JY901S_H

#include "stm32f4xx_hal.h"

#include <stdbool.h>
#include <stdint.h>

#define JY901S_ATTITUDE_TIMEOUT_MS 500U

typedef struct
{
  int32_t roll_cdeg;
  int32_t pitch_cdeg;
  int32_t yaw_cdeg;
  uint32_t sequence;
  uint32_t update_tick;
} JY901S_Attitude;

/* UART4 is configured by CubeMX as 115200 baud, 8 data bits, no parity, 1 stop bit. */
HAL_StatusTypeDef JY901S_Init(UART_HandleTypeDef *huart);
void JY901S_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void JY901S_UART_ErrorCallback(UART_HandleTypeDef *huart);
bool JY901S_GetAttitude(JY901S_Attitude *attitude);

#endif /* JY901S_H */
