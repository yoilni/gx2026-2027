#ifndef IMU_TASK_H
#define IMU_TASK_H

#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

typedef struct
{
  int32_t yaw_cdeg;
  uint32_t sequence;
  uint32_t update_tick;
} IMU_Yaw;

typedef enum
{
  IMU_UNINITIALIZED = 0,
  IMU_PROBING,
  IMU_HWT101,
  IMU_JY901S,
  IMU_INIT_FAILED
} IMU_Source;

/* Exactly one TTL sensor on UART4 at115200/8N1. Only this interface owns
   receive callbacks; drivers are initialized exclusively after boot probing. */
HAL_StatusTypeDef IMU_Init(UART_HandleTypeDef *huart);
void IMU_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void IMU_UART_ErrorCallback(UART_HandleTypeDef *huart);
void IMU_Service(uint32_t now);
void IMU_DebugYaw(uint32_t now);
IMU_Source IMU_GetSource(void);
bool IMU_GetYaw(IMU_Yaw *yaw);

#endif
