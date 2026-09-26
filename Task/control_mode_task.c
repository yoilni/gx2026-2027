#include "control_mode_task.h"

#include "cmsis_os.h"
#include "main.h"
#include "mission_task.h"

#define CONTROL_MODE_TASK_PERIOD_TICKS 10U
#define CONTROL_MODE_DEBOUNCE_SAMPLES  5U
#define CONTROL_MODE_TASK_STACK_BYTES  (256U * 4U)

static osThreadId_t control_mode_task_handle;
static volatile ControlModeSnapshot control_mode_snapshot = {
  /* Safe startup default. A stable PE15 high level explicitly enables
     Bluetooth authority after the debounce interval. */
  .mode = CONTROL_MODE_AUTONOMOUS,
  .sequence = 0U,
  .update_tick = 0U,
};

static const osThreadAttr_t control_mode_task_attributes = {
  .name = "controlModeTask",
  .stack_size = CONTROL_MODE_TASK_STACK_BYTES,
  .priority = (osPriority_t)osPriorityNormal,
};

static ControlMode ControlModeTask_ReadPin(void)
{
  return (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_15) == GPIO_PIN_SET)
             ? CONTROL_MODE_BLUETOOTH
             : CONTROL_MODE_AUTONOMOUS;
}

static void ControlModeTask_Publish(ControlMode mode, uint32_t now)
{
  uint32_t primask = __get_PRIMASK();

  __disable_irq();
  control_mode_snapshot.mode = mode;
  ++control_mode_snapshot.sequence;
  control_mode_snapshot.update_tick = now;
  if (primask == 0U)
  {
    __enable_irq();
  }

  /* Switching control authority always stops the previous controller. */
  MissionTask_RequestStop();
}

bool ControlModeTask_Create(void)
{
  if (control_mode_task_handle != NULL)
  {
    return true;
  }

  control_mode_task_handle = osThreadNew(StartControlModeTask, NULL,
                                         &control_mode_task_attributes);
  return control_mode_task_handle != NULL;
}

bool ControlModeTask_GetSnapshot(ControlModeSnapshot *snapshot)
{
  uint32_t primask;

  if (snapshot == NULL)
  {
    return false;
  }

  primask = __get_PRIMASK();
  __disable_irq();
  *snapshot = control_mode_snapshot;
  if (primask == 0U)
  {
    __enable_irq();
  }
  return true;
}

void StartControlModeTask(void *argument)
{
  ControlMode candidate;
  ControlMode last_sample;
  ControlMode published_mode;
  uint8_t stable_samples = 0U;
  uint32_t next_wake;

  (void)argument;
  last_sample = ControlModeTask_ReadPin();
  published_mode = control_mode_snapshot.mode;
  MissionTask_RequestStop();
  next_wake = osKernelGetTickCount();

  for (;;)
  {
    candidate = ControlModeTask_ReadPin();
    if (candidate == last_sample)
    {
      if (stable_samples < CONTROL_MODE_DEBOUNCE_SAMPLES)
      {
        ++stable_samples;
      }
    }
    else
    {
      last_sample = candidate;
      stable_samples = 1U;
    }

    if ((stable_samples >= CONTROL_MODE_DEBOUNCE_SAMPLES) &&
        (candidate != published_mode))
    {
      published_mode = candidate;
      ControlModeTask_Publish(published_mode, HAL_GetTick());
    }

    next_wake += CONTROL_MODE_TASK_PERIOD_TICKS;
    (void)osDelayUntil(next_wake);
  }
}
