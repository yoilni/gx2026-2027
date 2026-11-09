#ifndef UART_PORT_H
#define UART_PORT_H

#include "stm32f4xx_hal.h"

/* No protocol, queue, locking or retries. Callers retain each port's RX
   buffer, timeout, and UART callback ownership. */
static inline HAL_StatusTypeDef UartPort_ReceiveIT(UART_HandleTypeDef *port,
                                                 uint8_t *data, uint16_t size)
{
  return HAL_UART_Receive_IT(port, data, size);
}

static inline HAL_StatusTypeDef UartPort_AbortReceive(UART_HandleTypeDef *port)
{
  return HAL_UART_AbortReceive(port);
}

static inline HAL_StatusTypeDef UartPort_Transmit(UART_HandleTypeDef *port,
                                                const uint8_t *data, uint16_t size,
                                                uint32_t timeout_ms)
{
  return HAL_UART_Transmit(port, data, size, timeout_ms);
}

/* Early boot only: initialized UART and DWT cycle counter required.
   Uses one deadline for the entire string, even with IRQs disabled. */
void UartPort_WriteBootText(UART_HandleTypeDef *port, const char *text,
                           uint32_t start_cycle, uint32_t budget_cycles);

#endif
