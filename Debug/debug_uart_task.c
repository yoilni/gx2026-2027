#include "debug_uart_task.h"

#include "cmsis_os.h"
#include "reset_reason.h"

#include <stdarg.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>

#define DEBUG_UART_QUEUE_DEPTH       8U
#define DEBUG_UART_MESSAGE_CAPACITY 128U
#define DEBUG_UART_TX_TIMEOUT_MS    100U
#define DEBUG_UART_TASK_STACK_BYTES (256U * 4U)

typedef struct
{
  char text[DEBUG_UART_MESSAGE_CAPACITY];
} DebugUartMessage;

static UART_HandleTypeDef *debug_uart;
static osMessageQueueId_t debug_uart_queue;
static osThreadId_t debug_uart_task_handle;

static const osThreadAttr_t debug_uart_task_attributes = {
  .name = "debugUartTask",
  .stack_size = DEBUG_UART_TASK_STACK_BYTES,
  .priority = (osPriority_t)osPriorityLow,
};

static void DebugUart_Transmit(const char *text)
{
  size_t length;

  if ((debug_uart == NULL) || (text == NULL))
  {
    return;
  }

  length = strnlen(text, DEBUG_UART_MESSAGE_CAPACITY);
  if (length != 0U)
  {
    (void)HAL_UART_Transmit(debug_uart, (uint8_t *)text,
                           (uint16_t)length, DEBUG_UART_TX_TIMEOUT_MS);
  }
}

static const char *DebugUart_ResetReasonText(ResetReason reason)
{
  switch (reason)
  {
    case RESET_REASON_POWER_ON:     return "POR";
    case RESET_REASON_BROWNOUT:     return "BOR";
    case RESET_REASON_EXTERNAL_PIN: return "PIN";
    case RESET_REASON_SOFTWARE:     return "SW";
    case RESET_REASON_IWDG:         return "IWDG";
    case RESET_REASON_WWDG:         return "WWDG";
    case RESET_REASON_LOW_POWER:    return "LP";
    case RESET_REASON_UNKNOWN:
    default:                        return "?";
  }
}

static void DebugUart_TransmitResetReason(void)
{
  ResetReasonSnapshot snapshot;
  char text[DEBUG_UART_MESSAGE_CAPACITY];

  if (!ResetReason_GetSnapshot(&snapshot))
  {
    DebugUart_Transmit("[BOOT] rst=NA\r\n");
    return;
  }

  (void)snprintf(text, sizeof(text),
                 "[BOOT] rst=%s f=%08lX\r\n",
                 DebugUart_ResetReasonText(snapshot.reason),
                 (unsigned long)snapshot.raw_flags);
  DebugUart_Transmit(text);
}

bool DebugUartTask_Create(UART_HandleTypeDef *huart)
{
  if ((huart == NULL) || (huart->Instance != USART2))
  {
    return false;
  }
  if (debug_uart_task_handle != NULL)
  {
    return true;
  }

  debug_uart = huart;
  debug_uart_queue = osMessageQueueNew(DEBUG_UART_QUEUE_DEPTH,
                                       sizeof(DebugUartMessage), NULL);
  if (debug_uart_queue == NULL)
  {
    return false;
  }

  debug_uart_task_handle = osThreadNew(StartDebugUartTask, NULL,
                                       &debug_uart_task_attributes);
  return debug_uart_task_handle != NULL;
}

bool DebugUart_Log(const char *text)
{
  DebugUartMessage message = {0};

  if ((debug_uart_queue == NULL) || (text == NULL))
  {
    return false;
  }

  (void)strncpy(message.text, text, sizeof(message.text) - 1U);
  return osMessageQueuePut(debug_uart_queue, &message, 0U, 0U) == osOK;
}

bool DebugUart_Logf(const char *format, ...)
{
  DebugUartMessage message = {0};
  va_list args;

  if ((debug_uart_queue == NULL) || (format == NULL))
  {
    return false;
  }

  va_start(args, format);
  (void)vsnprintf(message.text, sizeof(message.text), format, args);
  va_end(args);
  return osMessageQueuePut(debug_uart_queue, &message, 0U, 0U) == osOK;
}

void StartDebugUartTask(void *argument)
{
  DebugUartMessage message;

  (void)argument;
  DebugUart_Transmit("[BOOT] UART2=115200 READY\r\n");
  DebugUart_TransmitResetReason();

  /* Some wireless UART adapters reconnect just after an MCU reset and can
     miss the first bytes, so publish one delayed boot marker as well. */
  osDelay(500U);
  DebugUart_Transmit("[BOOT] ALIVE500\r\n");
  DebugUart_TransmitResetReason();

  for (;;)
  {
    if (osMessageQueueGet(debug_uart_queue, &message, NULL,
                          osWaitForever) == osOK)
    {
      DebugUart_Transmit(message.text);
    }
  }
}
