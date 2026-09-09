#include "led_task.h"

#include "cmsis_os.h"
#include "main.h"

#define LED_TASK_PERIOD_TICKS 500U

void LED_Task(void *argument)
{
  (void)argument;

  for (;;)
  {
    osDelay(LED_TASK_PERIOD_TICKS);
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
  }
}
