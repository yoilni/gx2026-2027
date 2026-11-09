#include "start_button_task.h"

#include "app_rtos_task_config.h"
#include "cmsis_os.h"
#include "debug_uart_task.h"
#include "main.h"
#include "mission_task.h"
#include "switch_input.h"

#include <stdint.h>

#define START_BUTTON_TASK_PERIOD_TICKS 10U
#define START_BUTTON_DEBOUNCE_SAMPLES   5U

static osThreadId_t start_button_task_handle;

bool StartButtonTask_Create(void)
{
  if (start_button_task_handle != NULL)
  {
    return true;
  }

  start_button_task_handle = osThreadNew(StartStartButtonTask, NULL,
                                         &AppRtos_StartButtonTaskAttr);
  return start_button_task_handle != NULL;
}

void StartStartButtonTask(void *argument)
{
  GPIO_PinState candidate;
  GPIO_PinState last_sample;
  GPIO_PinState stable_state;
  uint8_t stable_samples = 1U;
  bool armed;
  uint32_t next_wake;

  (void)argument;
  last_sample = SwitchInput_ReadStart();
  stable_state = last_sample;

  /* A button held during reset cannot start the vehicle. It must first be
     released, then pressed after the scheduler is running. */
  armed = stable_state == GPIO_PIN_SET;
  next_wake = osKernelGetTickCount();

  for (;;)
  {
    candidate = SwitchInput_ReadStart();
    if (candidate == last_sample)
    {
      if (stable_samples < START_BUTTON_DEBOUNCE_SAMPLES)
      {
        ++stable_samples;
      }
    }
    else
    {
      last_sample = candidate;
      stable_samples = 1U;
    }

    if ((stable_samples >= START_BUTTON_DEBOUNCE_SAMPLES) &&
        (candidate != stable_state))
    {
      stable_state = candidate;
      if (stable_state == GPIO_PIN_SET)
      {
        armed = true;
        (void)DebugUart_Log("[START] RELEASE\r\n");
      }
      else if (armed)
      {
        armed = false;
        MissionTask_RequestStart();
        (void)DebugUart_Log("[START] PRESS\r\n");
      }
    }

    next_wake += START_BUTTON_TASK_PERIOD_TICKS;
    (void)osDelayUntil(next_wake);
  }
}
