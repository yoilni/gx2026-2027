#ifndef TEST_SERVO_PWM_TIM_H
#define TEST_SERVO_PWM_TIM_H

#include "stm32f4xx_hal.h"

typedef struct { uint32_t CCER; } TIM_TypeDef;
typedef struct
{
  TIM_TypeDef *Instance;
  struct { uint32_t Prescaler, Period; } Init;
} TIM_HandleTypeDef;

extern TIM_TypeDef test_tim2;
extern TIM_HandleTypeDef htim2;

#define TIM2 (&test_tim2)
#define TIM_CCER_CC1E (1U << 0)
#define TIM_CCER_CC3E (1U << 8)
#define TIM_CCER_CC4E (1U << 12)
#define TIM_CHANNEL_1 0U
#define TIM_CHANNEL_3 2U
#define TIM_CHANNEL_4 3U

void ServoPwmTest_SetCompare(TIM_HandleTypeDef *timer, unsigned channel, uint16_t pulse);
#define __HAL_TIM_SET_COMPARE(timer, channel, pulse) ServoPwmTest_SetCompare(timer, channel, pulse)
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *timer, uint32_t channel);
HAL_StatusTypeDef HAL_TIM_PWM_Stop(TIM_HandleTypeDef *timer, uint32_t channel);

#endif
