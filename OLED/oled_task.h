#ifndef OLED_TASK_H
#define OLED_TASK_H

#include "oled.h"

extern HAL_StatusTypeDef OLED_TaskInit(I2C_HandleTypeDef *hi2c);
extern void StartOledTask(void *argument);

#endif /* OLED_TASK_H */
