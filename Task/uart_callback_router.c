#include "hwt101.h"
#include "maixcam_task.h"
#include "stm32f4xx_hal.h"

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  HWT101_UART_RxCpltCallback(huart);
  MaixCam_UART_RxCpltCallback(huart);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  HWT101_UART_ErrorCallback(huart);
  MaixCam_UART_ErrorCallback(huart);
}
