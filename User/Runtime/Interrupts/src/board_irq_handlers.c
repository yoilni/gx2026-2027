#include "can.h"
#include "usart.h"

/* These vectors are not emitted by the current CubeMX NVIC configuration. */
void UART4_IRQHandler(void)
{
  HAL_UART_IRQHandler(&huart4);
}

void USART3_IRQHandler(void)
{
  HAL_UART_IRQHandler(&huart3);
}

void CAN1_RX0_IRQHandler(void)
{
  HAL_CAN_IRQHandler(&hcan1);
}
