#ifndef MOTOR_TEST_TASK_H
#define MOTOR_TEST_TASK_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
  MOTOR_TEST_WAIT_FEEDBACK = 0,
  MOTOR_TEST_RUNNING,
  MOTOR_TEST_DONE,
  MOTOR_TEST_FEEDBACK_FAULT
} MotorTestStatus;

bool MotorTestTask_Create(void);
MotorTestStatus MotorTest_GetStatus(void);

#endif /* MOTOR_TEST_TASK_H */
