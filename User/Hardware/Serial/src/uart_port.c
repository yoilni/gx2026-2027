#include "uart_port.h"

void UartPort_WriteBootText(UART_HandleTypeDef *port, const char *text,
                           uint32_t start_cycle, uint32_t budget_cycles)
{
  USART_TypeDef *const uart = port->Instance;
  for (uint32_t index = 0U; text[index] != '\0'; ++index)
  {
    while ((uart->SR & USART_SR_TXE) == 0U)
    {
      if ((uint32_t)(DWT->CYCCNT - start_cycle) >= budget_cycles) return;
    }
    uart->DR = (uint8_t)text[index];
  }
  while ((uart->SR & USART_SR_TC) == 0U)
  {
    if ((uint32_t)(DWT->CYCCNT - start_cycle) >= budget_cycles) return;
  }
}
