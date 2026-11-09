#include "servo_pwm.h"
#include "tim.h"
#include <stddef.h>

#define SERVO_PWM_TIM2_PSC 83U
#define SERVO_PWM_TIM2_ARR 19999U

typedef struct
{
  uint32_t timer_channel;
  uint32_t enable_mask;
} ServoPwm_Output;

static const ServoPwm_Output servo_outputs[] = {
  [SERVO_PWM_CAMERA] = { TIM_CHANNEL_1, TIM_CCER_CC1E }, /* PA5 */
  [SERVO_PWM_BAFFLE] = { TIM_CHANNEL_3, TIM_CCER_CC3E }, /* PA2 */
  [SERVO_PWM_CLAW]   = { TIM_CHANNEL_4, TIM_CCER_CC4E }, /* PA3 */
};

bool ServoPwm_IsTimerValid(void)
{
  return htim2.Instance == TIM2 &&
         htim2.Init.Prescaler == SERVO_PWM_TIM2_PSC &&
         htim2.Init.Period == SERVO_PWM_TIM2_ARR;
}

static const ServoPwm_Output *ServoPwm_GetOutput(ServoPwm_Channel channel)
{
  const unsigned int index = (unsigned int)channel;
  if (index >= sizeof(servo_outputs) / sizeof(servo_outputs[0]) ||
      !ServoPwm_IsTimerValid())
  {
    return NULL;
  }
  return &servo_outputs[index];
}

void ServoPwm_WritePulseUs(ServoPwm_Channel channel, uint16_t pulse_us)
{
  const ServoPwm_Output *output = ServoPwm_GetOutput(channel);
  if (output != NULL)
  {
    __HAL_TIM_SET_COMPARE(&htim2, output->timer_channel, pulse_us);
  }
}

bool ServoPwm_IsEnabled(ServoPwm_Channel channel)
{
  const ServoPwm_Output *output = ServoPwm_GetOutput(channel);
  return output != NULL && (htim2.Instance->CCER & output->enable_mask) != 0U;
}

HAL_StatusTypeDef ServoPwm_Start(ServoPwm_Channel channel)
{
  const ServoPwm_Output *output = ServoPwm_GetOutput(channel);
  return output != NULL ? HAL_TIM_PWM_Start(&htim2, output->timer_channel) : HAL_ERROR;
}

HAL_StatusTypeDef ServoPwm_Stop(ServoPwm_Channel channel)
{
  const ServoPwm_Output *output = ServoPwm_GetOutput(channel);
  return output != NULL ? HAL_TIM_PWM_Stop(&htim2, output->timer_channel) : HAL_ERROR;
}
