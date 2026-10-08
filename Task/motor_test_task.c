#include "motor_test_task.h"

#include "M2006_Speed.h"
#include "cmsis_os.h"
#include "debug_uart_task.h"
#include "main.h"
#include "robot_config.h"

#define MOTOR_TEST_PERIOD_TICKS 10U
#define MOTOR_TEST_STACK_BYTES (256U * 4U)
#define MOTOR_TEST_FEEDBACK_WAIT_MS 2000U

static osThreadId_t motor_test_task_handle;
static volatile MotorTestStatus motor_test_status = MOTOR_TEST_WAIT_FEEDBACK;

static const osThreadAttr_t motor_test_task_attributes = {
  .name = "motorTestTask",
  .stack_size = MOTOR_TEST_STACK_BYTES,
  .priority = (osPriority_t)osPriorityNormal,
};

static bool MotorTest_FeedbackFresh(uint8_t motor_id, uint32_t now)
{
  const moto_measure_t *motor = &moto_chassis[motor_id - 1U];
  return motor->msg_cnt != 0U &&
         (uint32_t)(now - motor->last_rx_tick) <= M2006_FEEDBACK_TIMEOUT_MS;
}

static bool MotorTest_BothFeedbackFresh(uint32_t now)
{
  return MotorTest_FeedbackFresh(ROBOT_LEFT_WHEEL_MOTOR_ID, now) &&
         MotorTest_FeedbackFresh(ROBOT_RIGHT_WHEEL_MOTOR_ID, now);
}

static void MotorTest_Stop(void)
{
  brake();
}

static void StartMotorTestTask(void *argument)
{
  uint32_t next_wake = osKernelGetTickCount();
  uint32_t wait_started = HAL_GetTick();
  uint32_t drive_started = 0U;

  (void)argument;
  MotorTest_Stop();
  (void)DebugUart_Log("[TEST] MOTOR WAIT FB\r\n");

  for (;;)
  {
    uint32_t now = HAL_GetTick();
    if (motor_test_status == MOTOR_TEST_WAIT_FEEDBACK)
    {
      if (MotorTest_BothFeedbackFresh(now))
      {
        SpeedLoop_SetMotorTarget(ROBOT_LEFT_WHEEL_MOTOR_ID,
            (float)ROBOT_STRAIGHT_TEST_RPM * ROBOT_LEFT_WHEEL_FORWARD_SIGN,
            ROBOT_S1_CURRENT_LIMIT);
        SpeedLoop_SetMotorTarget(ROBOT_RIGHT_WHEEL_MOTOR_ID,
            (float)ROBOT_STRAIGHT_TEST_RPM * ROBOT_RIGHT_WHEEL_FORWARD_SIGN,
            ROBOT_S1_CURRENT_LIMIT);
        drive_started = now;
        motor_test_status = MOTOR_TEST_RUNNING;
        (void)DebugUart_Logf(
            "[TEST] MOTOR FWD=%d/%lu\r\n",
            ROBOT_STRAIGHT_TEST_RPM, (unsigned long)ROBOT_STRAIGHT_TEST_MS);
      }
      else if ((uint32_t)(now - wait_started) >= MOTOR_TEST_FEEDBACK_WAIT_MS)
      {
        SpeedLoop_EmergencyStop();
        motor_test_status = MOTOR_TEST_FEEDBACK_FAULT;
        (void)DebugUart_Log("[TEST] MOTOR FB TIMEOUT\r\n");
      }
    }
    else if (motor_test_status == MOTOR_TEST_RUNNING)
    {
      if (!MotorTest_BothFeedbackFresh(now))
      {
        SpeedLoop_EmergencyStop();
        motor_test_status = MOTOR_TEST_FEEDBACK_FAULT;
        (void)DebugUart_Log("[TEST] MOTOR FB LOST STOP\r\n");
      }
      else if ((uint32_t)(now - drive_started) >= ROBOT_STRAIGHT_TEST_MS)
      {
        MotorTest_Stop();
        motor_test_status = MOTOR_TEST_DONE;
        (void)DebugUart_Log("[TEST] MOTOR DONE STOP\r\n");
      }
    }

    next_wake += MOTOR_TEST_PERIOD_TICKS;
    (void)osDelayUntil(next_wake);
  }
}

bool MotorTestTask_Create(void)
{
  if (motor_test_task_handle != NULL)
  {
    return true;
  }
  motor_test_task_handle = osThreadNew(StartMotorTestTask, NULL,
                                      &motor_test_task_attributes);
  return motor_test_task_handle != NULL;
}

MotorTestStatus MotorTest_GetStatus(void)
{
  return motor_test_status;
}
