#include "app_rtos.h"
#include "app_rtos_task_config.h"

#include "M2006_task.h"
#include "debug_uart_task.h"
#include "imu_task.h"
#include "led_task.h"
#include "mission_task.h"
#include "motor_test_task.h"
#include "oled_task.h"
#include "robot_config.h"
#include "servo_test_task.h"
#include "start_button_task.h"
#include "usart.h"

#include <stddef.h>

static void StartImuTask(void *argument)
{
  (void)argument;
  for (;;)
  {
    uint32_t now = HAL_GetTick();
    IMU_Service(now);
    IMU_DebugYaw(HAL_GetTick());
    (void)osDelay(20U);
  }
}

bool AppRtos_CreateTasks(bool default_task_ready)
{
#if ROBOT_STRAIGHT_TEST_ENABLED || (!ROBOT_MG90_SPEED_TEST_ENABLED && !ROBOT_FRAME_DOWN_TEST_ENABLED)
  osThreadId_t motor_task = osThreadNew(StartMotorTask, NULL, &AppRtos_MotorTaskAttr);
#endif
#if ROBOT_OLED_ENABLED
  if (osThreadNew(StartOledTask, NULL, &AppRtos_OledTaskAttr) == NULL)
  {
    return false;
  }
#endif
  if (!default_task_ready || !DebugUartTask_Create(&huart2))
  {
    return false;
  }
  if (osThreadNew(StartImuTask, NULL, &AppRtos_ImuTaskAttr) == NULL)
  {
    return false;
  }
#if ROBOT_MG90_SPEED_TEST_ENABLED
  if (!ServoTestTask_Create())
  {
    return false;
  }
#endif
#if ROBOT_STRAIGHT_TEST_ENABLED
  if ((motor_task == NULL) || !MotorTestTask_Create())
  {
    return false;
  }
#elif !ROBOT_MG90_SPEED_TEST_ENABLED && !ROBOT_FRAME_DOWN_TEST_ENABLED
  if ((motor_task == NULL) || !MissionTask_Create() || !StartButtonTask_Create())
  {
    return false;
  }
#endif
  return true;
}

void AppRtos_RunHeartbeat(void *argument)
{
#if ROBOT_FRAME_DOWN_TEST_ENABLED
  (void)DebugUart_Logf(
      "[TEST] FRAME lift=%u cam=%u MOTOR OFF\r\n",
      (unsigned int)ROBOT_FRAME_TEST_LIFT_DEG,
      (unsigned int)ROBOT_FRAME_TEST_CAMERA_ANGLE_DEG);
#endif
  LED_Task(argument);
}
