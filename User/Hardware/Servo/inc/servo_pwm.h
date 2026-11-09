#ifndef SERVO_PWM_H
#define SERVO_PWM_H

#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

typedef enum
{
  SERVO_PWM_CAMERA,
  SERVO_PWM_BAFFLE,
  SERVO_PWM_CLAW
} ServoPwm_Channel;

/* Check CubeMX's TIM2 setup before an actuator transaction. */
bool ServoPwm_IsTimerValid(void);

/* Only update compare; the pulse is already calibrated by the caller.
   Never start or stop the selected output as a side effect. */
void ServoPwm_WritePulseUs(ServoPwm_Channel channel, uint16_t pulse_us);
bool ServoPwm_IsEnabled(ServoPwm_Channel channel);

/* Raw HAL start/stop: callers own the enable policy and operation order. */
HAL_StatusTypeDef ServoPwm_Start(ServoPwm_Channel channel);
HAL_StatusTypeDef ServoPwm_Stop(ServoPwm_Channel channel);

#endif /* SERVO_PWM_H */
