#include "led_task.h"

#include "board_led.h"
#include "cmsis_os.h"

#define LED_TASK_PERIOD_TICKS 500U

void LED_Task(void *argument)
{
  (void)argument;

  for (;;)
  {
    osDelay(LED_TASK_PERIOD_TICKS);
    BoardLed_Toggle();
  }
}
