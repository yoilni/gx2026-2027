#ifndef HOST_RTOS_HAL_H
#define HOST_RTOS_HAL_H

#include <stdint.h>

typedef struct { void *Instance; } UART_HandleTypeDef;
typedef struct { void *Instance; } I2C_HandleTypeDef;
typedef enum { HAL_OK, HAL_ERROR } HAL_StatusTypeDef;

uint32_t HAL_GetTick(void);

#endif
