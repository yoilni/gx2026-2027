#include "board_led.h"
#include "stm32f4xx_hal.h"

void BoardLed_Toggle(void)
{
  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
}
