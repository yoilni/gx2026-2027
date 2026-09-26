#ifndef BLUETOOTH_H
#define BLUETOOTH_H

#include "stm32f4xx_hal.h"

#include <stdbool.h>
#include <stdint.h>

#define BLUETOOTH_RX_BUFFER_SIZE 128U
#define BLUETOOTH_RECENT_BYTE_COUNT 5U

typedef struct
{
  uint32_t received_bytes;
  uint32_t dropped_bytes;
  uint32_t last_rx_tick;
  uint8_t recent_bytes[BLUETOOTH_RECENT_BYTE_COUNT];
  uint8_t recent_count;
  bool initialized;
} BluetoothStatus;

/* UART2 byte transport. The task-context frame parser is defined separately in
   bluetooth_frame.h so the ISR remains short and never controls a motor. */
HAL_StatusTypeDef Bluetooth_Init(UART_HandleTypeDef *huart);
void Bluetooth_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void Bluetooth_UART_ErrorCallback(UART_HandleTypeDef *huart);

/* Task-context raw transport API. Only one task should own TX. */
uint16_t Bluetooth_ReadRaw(uint8_t *data, uint16_t capacity);
HAL_StatusTypeDef Bluetooth_SendRaw(const uint8_t *data, uint16_t length,
                                    uint32_t timeout_ms);
bool Bluetooth_GetStatus(BluetoothStatus *status);

#endif /* BLUETOOTH_H */
