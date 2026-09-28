#include "actuator_task.h"

#include "robot_config.h"
#include "tim.h"

#define ACTUATOR_TIM2_EXPECTED_PSC 83U
#define ACTUATOR_TIM2_EXPECTED_ARR 19999U

static uint8_t frame_pwm_started;

static HAL_StatusTypeDef Actuator_SetFrameAngles(uint16_t left_angle_deg,
                                                 uint16_t right_angle_deg);

static uint8_t Actuator_TimerIsValid(void)
{
  return (htim2.Instance == TIM2) &&
         (htim2.Init.Prescaler == ACTUATOR_TIM2_EXPECTED_PSC) &&
         (htim2.Init.Period == ACTUATOR_TIM2_EXPECTED_ARR);
}

static uint16_t Actuator_Mg90AngleToPulse(uint16_t angle_deg)
{
  if (angle_deg > 180U)
  {
    angle_deg = 180U;
  }

  return (uint16_t)(ROBOT_CAMERA_SERVO_MIN_PULSE_US +
      (((uint32_t)angle_deg *
        (ROBOT_CAMERA_SERVO_MAX_PULSE_US -
         ROBOT_CAMERA_SERVO_MIN_PULSE_US)) / 180U));
}

static uint16_t Actuator_FrameAngleToPulse(uint16_t angle_deg)
{
  if (angle_deg > ROBOT_FRAME_SERVO_MAX_ANGLE_DEG)
  {
    angle_deg = ROBOT_FRAME_SERVO_MAX_ANGLE_DEG;
  }

  return (uint16_t)(ROBOT_FRAME_SERVO_MIN_PULSE_US +
      (((uint32_t)angle_deg *
        (ROBOT_FRAME_SERVO_MAX_PULSE_US -
         ROBOT_FRAME_SERVO_MIN_PULSE_US)) /
       ROBOT_FRAME_SERVO_MAX_ANGLE_DEG));
}

HAL_StatusTypeDef Actuator_Init(void)
{
  uint16_t pulse_us;

  if (Actuator_TimerIsValid() == 0U)
  {
    return HAL_ERROR;
  }

#if ROBOT_MG90_SPEED_TEST_ENABLED
  pulse_us = ROBOT_MG90_TEST_PULSE_US;
#else
  pulse_us = Actuator_Mg90AngleToPulse(ROBOT_CAMERA_WIDE_ANGLE_DEG);
#endif

  /* PA5 is TIM2 CH1: fixed test pulse in test mode, wide-view otherwise. */
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pulse_us);
  if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1) != HAL_OK)
  {
    return HAL_ERROR;
  }

  HAL_Delay(ROBOT_CAMERA_POWERUP_SETTLE_MS);

#if ROBOT_MG90_SPEED_TEST_ENABLED
  /* Frame channels stay off during the camera-servo-only test. */
  return HAL_OK;
#else
  /* Raise the collection frame after the camera settles so the chassis can
     cross the speed bump. State 3 lowers it before target tracking. */
  return Actuator_SetFrameRaised();
#endif
}

static HAL_StatusTypeDef Actuator_SetFrameAngles(uint16_t left_angle_deg,
                                                 uint16_t right_angle_deg)
{
  uint16_t left_pulse_us;
  uint16_t right_pulse_us;

  if (Actuator_TimerIsValid() == 0U)
  {
    return HAL_ERROR;
  }

  left_pulse_us = Actuator_FrameAngleToPulse(left_angle_deg);
  right_pulse_us = Actuator_FrameAngleToPulse(right_angle_deg);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, left_pulse_us);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, right_pulse_us);

  if (frame_pwm_started != 0U)
  {
    return HAL_OK;
  }

  if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3) != HAL_OK)
  {
    return HAL_ERROR;
  }
  if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4) != HAL_OK)
  {
    return HAL_ERROR;
  }

  frame_pwm_started = 1U;
  return HAL_OK;
}

HAL_StatusTypeDef Actuator_SetFrameRaised(void)
{
  return Actuator_SetFrameAngles(ROBOT_LEFT_FRAME_UP_DEG,
                                 ROBOT_RIGHT_FRAME_UP_DEG);
}

HAL_StatusTypeDef Actuator_SetFrameLowered(void)
{
  return Actuator_SetFrameAngles(ROBOT_LEFT_FRAME_DOWN_DEG,
                                 ROBOT_RIGHT_FRAME_DOWN_DEG);
}

static HAL_StatusTypeDef Actuator_SetCameraAngle(uint16_t angle_deg)
{
  if (Actuator_TimerIsValid() == 0U)
  {
    return HAL_ERROR;
  }

  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1,
      Actuator_Mg90AngleToPulse(angle_deg));
  return HAL_OK;
}

HAL_StatusTypeDef Actuator_SetCameraWideView(void)
{
  return Actuator_SetCameraAngle(ROBOT_CAMERA_WIDE_ANGLE_DEG);
}

HAL_StatusTypeDef Actuator_SetCameraNearView(void)
{
  return Actuator_SetCameraAngle(ROBOT_CAMERA_NEAR_ANGLE_DEG);
}
