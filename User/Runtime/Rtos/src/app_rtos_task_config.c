#include "app_rtos_task_config.h"

/* CMSIS-RTOS2 stack sizes are in bytes. defaultTask is CubeMX-owned
   and configured in gx_version1.ioc, not in this table. */
const osThreadAttr_t AppRtos_MotorTaskAttr = {
  .name = "motorTask",
  .stack_size = 1024U,
  .priority = osPriorityHigh,
};

const osThreadAttr_t AppRtos_OledTaskAttr = {
  .name = "oledTask",
  .stack_size = 1024U,
  .priority = osPriorityLow,
};

const osThreadAttr_t AppRtos_DebugUartTaskAttr = {
  .name = "debugUartTask",
  .stack_size = 1024U,
  .priority = osPriorityLow,
};

const osThreadAttr_t AppRtos_ImuTaskAttr = {
  .name = "imuTask",
  .stack_size = 1024U,
  .priority = osPriorityBelowNormal,
};

const osThreadAttr_t AppRtos_ServoTestTaskAttr = {
  .name = "servoTestTask",
  .stack_size = 1024U,
  .priority = osPriorityLow,
};

const osThreadAttr_t AppRtos_MotorTestTaskAttr = {
  .name = "motorTestTask",
  .stack_size = 1024U,
  .priority = osPriorityNormal,
};

const osThreadAttr_t AppRtos_MissionTaskAttr = {
  .name = "missionTask",
  .stack_size = 2048U,
  .priority = osPriorityAboveNormal,
};

const osThreadAttr_t AppRtos_StartButtonTaskAttr = {
  .name = "startButtonTask",
  .stack_size = 1024U,
  .priority = osPriorityNormal,
};
