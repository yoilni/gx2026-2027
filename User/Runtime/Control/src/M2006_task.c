#include "M2006_task.h"

#include "M2006_Speed.h"
#include "can.h"
#include "cmsis_os.h"

#define M2006_TASK_PERIOD_TICKS 2U

void StartMotorTask(void *argument)
{
  uint32_t next_wake;

  (void)argument;
  next_wake = osKernelGetTickCount();

  for (;;)
  {
    SpeedLoop_Update(&hcan1);
    next_wake += M2006_TASK_PERIOD_TICKS;
    (void)osDelayUntil(next_wake);
  }
}
