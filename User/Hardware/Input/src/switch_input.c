#include "switch_input.h"

GPIO_PinState SwitchInput_ReadStart(void)
{
  return HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_12);
}

GPIO_PinState SwitchInput_ReadTeam(void)
{
  return HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_13);
}

GPIO_PinState SwitchInput_ReadZoneMsb(void)
{
  return HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_14);
}

GPIO_PinState SwitchInput_ReadZoneLsb(void)
{
  return HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_15);
}
