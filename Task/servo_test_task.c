#include "servo_test_task.h"

#include "app_rtos_task_config.h"
#include "cmsis_os.h"
#include "debug_uart_task.h"
#include "main.h"
#include "robot_config.h"
#include "servo_pwm.h"

#include <stdint.h>

#define SERVO_TEST_TASK_PERIOD_TICKS  10U

static osThreadId_t servo_test_task_handle;
static volatile uint16_t servo_test_pulse_us = ROBOT_MG90_TEST_START_PULSE_US;

uint16_t ServoTest_GetPulseUs(void)
{
  return servo_test_pulse_us;
}

static void ServoTest_SetMg90Pulse(uint16_t pulse_us)
{
  ServoPwm_WritePulseUs(SERVO_PWM_CAMERA, pulse_us);
  if (servo_test_pulse_us != pulse_us)
  {
    servo_test_pulse_us = pulse_us;
    DebugUart_Logf("[TEST] MG90 pwm=%u\r\n", (unsigned)pulse_us);
  }
}

bool ServoTestTask_Create(void)
{
  if (servo_test_task_handle != NULL)
  {
    return true;
  }

  servo_test_task_handle = osThreadNew(StartServoTestTask, NULL,
                                       &AppRtos_ServoTestTaskAttr);
  return servo_test_task_handle != NULL;
}

void StartServoTestTask(void *argument)
{
  uint32_t next_wake;

  (void)argument;
  if (!ServoPwm_IsTimerValid())
  {
    DebugUart_Logf("[TEST] MG90 TIM2 ERR\r\n");
    osThreadExit();
    return;
  }
  /* Actuator_Init starts CH1 and the independently enabled PA2/PA3 tests.
     The mission stays off; an optional one-shot wheel test runs separately.
     TIM2 keeps enabled PWM outputs running. */
#if ROBOT_MG90_TEST_WIDE_HOLD_MS > 0U
  DebugUart_Logf("[TEST] MG90 wide=%u hold=%u\r\n",
                (unsigned)ROBOT_CAMERA_WIDE_ANGLE_DEG,
                (unsigned)ROBOT_MG90_TEST_WIDE_HOLD_MS);
  (void)osDelay(ROBOT_MG90_TEST_WIDE_HOLD_MS);
#endif
  ServoTest_SetMg90Pulse(ROBOT_MG90_TEST_PULSE_US);
  DebugUart_Logf("[TEST] MG90 angle=%u pwm=%u\r\n",
                (unsigned)ROBOT_MG90_TEST_ANGLE_DEG,
                (unsigned)ROBOT_MG90_TEST_PULSE_US);
#if ROBOT_BAFFLE_TEST_ENABLED
  DebugUart_Logf("[TEST] PA2 baffle=%u\r\n",
                (unsigned)ROBOT_BAFFLE_TEST_ANGLE_DEG);
#endif
#if ROBOT_CLAW_TEST_ENABLED
  DebugUart_Logf("[TEST] PA3 claw=%u\r\n",
                (unsigned)ROBOT_CLAW_TEST_ANGLE_DEG);
#endif
  next_wake = osKernelGetTickCount();

  for (;;)
  {
    ServoTest_SetMg90Pulse(ROBOT_MG90_TEST_PULSE_US);

    next_wake += SERVO_TEST_TASK_PERIOD_TICKS;
    (void)osDelayUntil(next_wake);
  }
}
