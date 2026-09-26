#ifndef DEBUG_UART_TASK_H
#define DEBUG_UART_TASK_H

#include "stm32f4xx_hal.h"

#include <stdbool.h>

/* Creates the single UART2 debug-output owner and its fixed-size queue. */
bool DebugUartTask_Create(UART_HandleTypeDef *huart);

/* Task-context, non-blocking enqueue. Returns false if the queue is full. */
bool DebugUart_Log(const char *text);

/* Task-context formatted enqueue. Keep high-rate control loops free of logs;
   mission telemetry currently calls this at 10 Hz. */
bool DebugUart_Logf(const char *format, ...);

void StartDebugUartTask(void *argument);

#endif /* DEBUG_UART_TASK_H */
