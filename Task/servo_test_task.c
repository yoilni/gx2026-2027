#include "servo_test_task.h"

#include "cmsis_os.h"
#include "debug_uart_task.h"
#include "main.h"
#include "robot_config.h"
#include "tim.h"

#include <stdint.h>

#define SERVO_TEST_TASK_PERIOD_TICKS  10U
#define SERVO_TEST_TASK_STACK_BYTES   (256U * 4U)

#define SERVO_TEST_TIMER_PSC          83U
#define SERVO_TEST_TIMER_ARR          19999U

static osThreadId_t servo_test_task_handle;
static volatile uint16_t servo_test_pulse_us = ROBOT_MG90_TEST_PULSE_US;

static const osThreadAttr_t servo_test_task_attributes = {
  .name = "servoTestTask",
  .stack_size = SERVO_TEST_TASK_STACK_BYTES,
  .priority = (osPriority_t)osPriorityLow,
};

static bool ServoTest_TimerConfigurationIsSafe(void)
{
  return (htim2.Instance == TIM2) &&
         (htim2.Init.Prescaler == SERVO_TEST_TIMER_PSC) &&
         (htim2.Init.Period == SERVO_TEST_TIMER_ARR);
}

uint16_t ServoTest_GetPulseUs(void)
{
  return servo_test_pulse_us;
}

static void ServoTest_SetMg90Pulse(uint16_t pulse_us)
{
  /* PA5 is TIM2 CH1 and TIM2 runs at one count per microsecond. */
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pulse_us);
  if (servo_test_pulse_us != pulse_us)
  {
    servo_test_pulse_us = pulse_us;
    DebugUart_Logf("[MG90 TEST] PWM=%u us\r\n", (unsigned)pulse_us);
  }
}

bool ServoTestTask_Create(void)
{
  if (servo_test_task_handle != NULL)
  {
    return true;
  }

  servo_test_task_handle = osThreadNew(StartServoTestTask, NULL,
                                       &servo_test_task_attributes);
  return servo_test_task_handle != NULL;
}

void StartServoTestTask(void *argument)
{
  uint32_t next_wake;

  (void)argument;
  if (!ServoTest_TimerConfigurationIsSafe())
  {
    DebugUart_Logf("[MG90 TEST] invalid TIM2 configuration\r\n");
    osThreadExit();
    return;
  }
  /* Actuator_Init already starts CH1; frame PWM and mission tasks stay off. */
  ServoTest_SetMg90Pulse(ROBOT_MG90_TEST_PULSE_US);
  DebugUart_Logf("[MG90 TEST] hold 40 degrees, PWM=%u us; PE12 unused\r\n",
                (unsigned)ROBOT_MG90_TEST_PULSE_US);
  next_wake = osKernelGetTickCount();

  for (;;)
  {
    ServoTest_SetMg90Pulse(ROBOT_MG90_TEST_PULSE_US);

    next_wake += SERVO_TEST_TASK_PERIOD_TICKS;
    (void)osDelayUntil(next_wake);
  }
}
