#ifndef APP_RTOS_TASK_CONFIG_H
#define APP_RTOS_TASK_CONFIG_H

#include "cmsis_os.h"

/* The startup code and resource-owning task factories use the same table. */
extern const osThreadAttr_t AppRtos_MotorTaskAttr;
extern const osThreadAttr_t AppRtos_OledTaskAttr;
extern const osThreadAttr_t AppRtos_DebugUartTaskAttr;
extern const osThreadAttr_t AppRtos_ImuTaskAttr;
extern const osThreadAttr_t AppRtos_ServoTestTaskAttr;
extern const osThreadAttr_t AppRtos_MotorTestTaskAttr;
extern const osThreadAttr_t AppRtos_MissionTaskAttr;
extern const osThreadAttr_t AppRtos_StartButtonTaskAttr;

#endif
