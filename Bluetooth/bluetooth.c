#include "bluetooth.h"

static UART_HandleTypeDef *bluetooth_uart;
static uint8_t bluetooth_rx_byte;
static volatile uint8_t bluetooth_rx_buffer[BLUETOOTH_RX_BUFFER_SIZE];
static volatile uint16_t bluetooth_rx_head;
static volatile uint16_t bluetooth_rx_tail;
static volatile uint32_t bluetooth_received_bytes;
static volatile uint32_t bluetooth_dropped_bytes;
static volatile uint32_t bluetooth_last_rx_tick;
static volatile uint8_t bluetooth_recent_bytes[BLUETOOTH_RECENT_BYTE_COUNT];
static volatile uint8_t bluetooth_recent_count;

static uint16_t Bluetooth_NextIndex(uint16_t index)
{
  ++index;
  if (index >= BLUETOOTH_RX_BUFFER_SIZE)
  {
    index = 0U;
  }
  return index;
}

HAL_StatusTypeDef Bluetooth_Init(UART_HandleTypeDef *huart)
{
  if (huart == NULL)
  {
    return HAL_ERROR;
  }

  bluetooth_uart = huart;
  bluetooth_rx_head = 0U;
  bluetooth_rx_tail = 0U;
  bluetooth_received_bytes = 0U;
  bluetooth_dropped_bytes = 0U;
  bluetooth_last_rx_tick = 0U;
  bluetooth_recent_count = 0U;
  return HAL_UART_Receive_IT(bluetooth_uart, &bluetooth_rx_byte, 1U);
}

void Bluetooth_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  uint16_t next_head;

  if ((huart == NULL) || (huart != bluetooth_uart))
  {
    return;
  }

  ++bluetooth_received_bytes;
  bluetooth_last_rx_tick = HAL_GetTick();
  if (bluetooth_recent_count < BLUETOOTH_RECENT_BYTE_COUNT)
  {
    bluetooth_recent_bytes[bluetooth_recent_count++] = bluetooth_rx_byte;
  }
  else
  {
    for (uint8_t i = 0U; i < (BLUETOOTH_RECENT_BYTE_COUNT - 1U); ++i)
    {
      bluetooth_recent_bytes[i] = bluetooth_recent_bytes[i + 1U];
    }
    bluetooth_recent_bytes[BLUETOOTH_RECENT_BYTE_COUNT - 1U] =
        bluetooth_rx_byte;
  }
  next_head = Bluetooth_NextIndex(bluetooth_rx_head);
  if (next_head == bluetooth_rx_tail)
  {
    ++bluetooth_dropped_bytes;
  }
  else
  {
    bluetooth_rx_buffer[bluetooth_rx_head] = bluetooth_rx_byte;
    bluetooth_rx_head = next_head;
  }

  (void)HAL_UART_Receive_IT(bluetooth_uart, &bluetooth_rx_byte, 1U);
}

void Bluetooth_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if ((huart != NULL) && (huart == bluetooth_uart))
  {
    (void)HAL_UART_Receive_IT(bluetooth_uart, &bluetooth_rx_byte, 1U);
  }
}

uint16_t Bluetooth_ReadRaw(uint8_t *data, uint16_t capacity)
{
  uint16_t count = 0U;

  if (data == NULL)
  {
    return 0U;
  }

  while ((count < capacity) && (bluetooth_rx_tail != bluetooth_rx_head))
  {
    data[count++] = bluetooth_rx_buffer[bluetooth_rx_tail];
    bluetooth_rx_tail = Bluetooth_NextIndex(bluetooth_rx_tail);
  }

  return count;
}

HAL_StatusTypeDef Bluetooth_SendRaw(const uint8_t *data, uint16_t length,
                                    uint32_t timeout_ms)
{
  if ((bluetooth_uart == NULL) || (data == NULL) || (length == 0U))
  {
    return HAL_ERROR;
  }

  return HAL_UART_Transmit(bluetooth_uart, (uint8_t *)data, length, timeout_ms);
}

bool Bluetooth_GetStatus(BluetoothStatus *status)
{
  uint32_t primask;

  if (status == NULL)
  {
    return false;
  }

  primask = __get_PRIMASK();
  __disable_irq();
  status->received_bytes = bluetooth_received_bytes;
  status->dropped_bytes = bluetooth_dropped_bytes;
  status->last_rx_tick = bluetooth_last_rx_tick;
  status->recent_count = bluetooth_recent_count;
  for (uint8_t i = 0U; i < BLUETOOTH_RECENT_BYTE_COUNT; ++i)
  {
    status->recent_bytes[i] = bluetooth_recent_bytes[i];
  }
  status->initialized = bluetooth_uart != NULL;
  if (primask == 0U)
  {
    __enable_irq();
  }

  return status->initialized;
}
