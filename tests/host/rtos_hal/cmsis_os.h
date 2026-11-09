#ifndef HOST_RTOS_CMSIS_OS_H
#define HOST_RTOS_CMSIS_OS_H

#include <stddef.h>
#include <stdint.h>

typedef void *osThreadId_t;
typedef void (*osThreadFunc_t)(void *);
typedef enum {
  osPriorityLow = 8,
  osPriorityBelowNormal = 16,
  osPriorityNormal = 24,
  osPriorityAboveNormal = 32,
  osPriorityHigh = 40,
} osPriority_t;

typedef struct {
  const char *name;
  uint32_t stack_size;
  osPriority_t priority;
} osThreadAttr_t;

osThreadId_t osThreadNew(osThreadFunc_t func, void *arg, const osThreadAttr_t *attr);
int osDelay(uint32_t ticks);

#endif
