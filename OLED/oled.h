#ifndef OLED_H
#define OLED_H

#include "stm32f4xx_hal.h"

#include <stdint.h>

#define OLED_I2C_ADDRESS 0x78U

HAL_StatusTypeDef OLED_Init(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef OLED_Clear(void);
HAL_StatusTypeDef OLED_WriteString(uint8_t column, uint8_t page, const char *text);

#endif /* OLED_H */
