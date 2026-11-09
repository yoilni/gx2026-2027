#ifndef SWITCH_INPUT_H
#define SWITCH_INPUT_H

#include "stm32f4xx_hal.h"

/* Raw levels only. The caller owns debounce, polarity, and field mapping. */
GPIO_PinState SwitchInput_ReadStart(void);
GPIO_PinState SwitchInput_ReadTeam(void);
GPIO_PinState SwitchInput_ReadZoneMsb(void);
GPIO_PinState SwitchInput_ReadZoneLsb(void);

#endif
