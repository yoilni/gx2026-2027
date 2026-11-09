#ifndef APP_RTOS_H
#define APP_RTOS_H

#include <stdbool.h>

/* Call once, after osKernelInitialize() and the CubeMX defaultTask creation,
   before osKernelStart(). False means startup must stop. */
bool AppRtos_CreateTasks(bool default_task_ready);

/* Application body of the CubeMX-owned defaultTask. */
void AppRtos_RunHeartbeat(void *argument);

#endif
