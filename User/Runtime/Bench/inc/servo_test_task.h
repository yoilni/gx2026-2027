#ifndef SERVO_TEST_TASK_H
#define SERVO_TEST_TASK_H

#include <stdbool.h>
#include <stdint.h>

/* Hold the configured position-test pulse on PA5; PE12 is unused. */
bool ServoTestTask_Create(void);
void StartServoTestTask(void *argument);
uint16_t ServoTest_GetPulseUs(void);

#endif /* SERVO_TEST_TASK_H */
