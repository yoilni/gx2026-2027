#ifndef SERVO_TEST_TASK_H
#define SERVO_TEST_TASK_H

#include <stdbool.h>

/* PE12 high holds the 180-degree MG90 at 90 degrees on PA5/TIM2 CH1. */
bool ServoTestTask_Create(void);
void StartServoTestTask(void *argument);

#endif /* SERVO_TEST_TASK_H */
