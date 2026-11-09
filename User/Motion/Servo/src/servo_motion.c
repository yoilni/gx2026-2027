#include "servo_motion.h"
#include "servo_pwm.h"
#include "robot_config.h"

uint16_t ServoMotion_CameraPulseUs(uint16_t angle_deg)
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

uint16_t ServoMotion_FramePulseUs(uint16_t angle_deg)
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

HAL_StatusTypeDef ServoMotion_SetBaffleAngle(uint16_t angle_deg)
{
  if (!ServoPwm_IsTimerValid()) return HAL_ERROR;
  ServoPwm_WritePulseUs(SERVO_PWM_BAFFLE, ServoMotion_FramePulseUs(angle_deg));
  return ServoPwm_IsEnabled(SERVO_PWM_BAFFLE) ? HAL_OK : ServoPwm_Start(SERVO_PWM_BAFFLE);
}

HAL_StatusTypeDef ServoMotion_StopBafflePwm(void)
{
  if (!ServoPwm_IsTimerValid()) return HAL_ERROR;
  return ServoPwm_IsEnabled(SERVO_PWM_BAFFLE) ? ServoPwm_Stop(SERVO_PWM_BAFFLE) : HAL_OK;
}

HAL_StatusTypeDef ServoMotion_SetClawAngle(uint16_t angle_deg)
{
  if (!ServoPwm_IsEnabled(SERVO_PWM_CLAW)) return HAL_ERROR;
  ServoPwm_WritePulseUs(SERVO_PWM_CLAW, ServoMotion_FramePulseUs(angle_deg));
  return HAL_OK;
}

HAL_StatusTypeDef ServoMotion_SetCameraAngle(uint16_t angle_deg)
{
  if (!ServoPwm_IsTimerValid()) return HAL_ERROR;
  ServoPwm_WritePulseUs(SERVO_PWM_CAMERA, ServoMotion_CameraPulseUs(angle_deg));
  return HAL_OK;
}
