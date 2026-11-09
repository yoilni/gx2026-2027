#include "actuator_task.h"
#include "boot_init.h"

#include "robot_config.h"
#include "servo_pwm.h"

static uint8_t baffle_pwm_suspended;
static uint8_t claw_close_guard_active;
static uint8_t claw_close_pending;
static uint32_t claw_close_guard_tick;

static uint8_t Actuator_DeferClawCommand(uint16_t angle_deg)
{
  if (claw_close_guard_active == 0U) return 0U;
  claw_close_pending = (angle_deg == ROBOT_CLAW_CLOSED_DEG);
  return claw_close_pending;
}

static HAL_StatusTypeDef Actuator_SetFrameAngles(uint16_t left_angle_deg,
                                                 uint16_t right_angle_deg);

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

  if (!ServoPwm_IsTimerValid())
  {
    return HAL_ERROR;
  }

#if ROBOT_MG90_SPEED_TEST_ENABLED
  pulse_us = ROBOT_MG90_TEST_START_PULSE_US;
#elif ROBOT_FRAME_DOWN_TEST_ENABLED
  pulse_us = Actuator_Mg90AngleToPulse(ROBOT_FRAME_TEST_CAMERA_ANGLE_DEG);
#else
  pulse_us = Actuator_Mg90AngleToPulse(ROBOT_CAMERA_WIDE_ANGLE_DEG);
#endif

  /* PA5 is TIM2 CH1: initial bench pose or the configured mission wide view. */
  ServoPwm_WritePulseUs(SERVO_PWM_CAMERA, pulse_us);
  if (ServoPwm_Start(SERVO_PWM_CAMERA) != HAL_OK)
  {
    return HAL_ERROR;
  }

  BootInit_DelayMs(ROBOT_CAMERA_POWERUP_SETTLE_MS);

#if ROBOT_MG90_SPEED_TEST_ENABLED
  /* PA2 baffle and PA3 claw now have independent test angles. Never invoke
     the legacy mirrored-frame setter. Started channels hold in hardware. */
#if ROBOT_BAFFLE_TEST_ENABLED
  ServoPwm_WritePulseUs(SERVO_PWM_BAFFLE,
                        Actuator_FrameAngleToPulse(ROBOT_BAFFLE_TEST_ANGLE_DEG));
  if (ServoPwm_Start(SERVO_PWM_BAFFLE) != HAL_OK)
  {
    return HAL_ERROR;
  }
#endif
#if ROBOT_CLAW_TEST_ENABLED
  ServoPwm_WritePulseUs(SERVO_PWM_CLAW,
                        Actuator_FrameAngleToPulse(ROBOT_CLAW_TEST_ANGLE_DEG));
  if (ServoPwm_Start(SERVO_PWM_CLAW) != HAL_OK)
  {
    return HAL_ERROR;
  }
#endif
  return HAL_OK;
#elif ROBOT_FRAME_DOWN_TEST_ENABLED
  /* Hold the camera angle and the mirrored test lift above fully DOWN. */
  return Actuator_SetFramePartiallyRaised(ROBOT_FRAME_TEST_LIFT_DEG);
#else
  /* The departure turn starts with the baffle down and claw closed. */
  return Actuator_SetFrameAngles(ROBOT_BAFFLE_DOWN_DEG, ROBOT_CLAW_CLOSED_DEG);
#endif
}

static HAL_StatusTypeDef Actuator_SetFrameAngles(uint16_t left_angle_deg,
                                                 uint16_t right_angle_deg)
{
  uint16_t left_pulse_us;
  uint16_t right_pulse_us;

  if (!ServoPwm_IsTimerValid())
  {
    return HAL_ERROR;
  }

  left_pulse_us = Actuator_FrameAngleToPulse(left_angle_deg);
  right_pulse_us = Actuator_FrameAngleToPulse(right_angle_deg);
  if (baffle_pwm_suspended == 0U)
  {
    ServoPwm_WritePulseUs(SERVO_PWM_BAFFLE, left_pulse_us);
  }
  if (Actuator_DeferClawCommand(right_angle_deg) == 0U)
  {
    ServoPwm_WritePulseUs(SERVO_PWM_CLAW, right_pulse_us);
  }

  if (baffle_pwm_suspended == 0U && !ServoPwm_IsEnabled(SERVO_PWM_BAFFLE) &&
      ServoPwm_Start(SERVO_PWM_BAFFLE) != HAL_OK)
  {
    return HAL_ERROR;
  }
  if (!ServoPwm_IsEnabled(SERVO_PWM_CLAW) &&
      ServoPwm_Start(SERVO_PWM_CLAW) != HAL_OK)
  {
    return HAL_ERROR;
  }

  return HAL_OK;
}

HAL_StatusTypeDef Actuator_SetBaffleAngle(uint16_t angle_deg)
{
  if (!ServoPwm_IsTimerValid()) return HAL_ERROR;
  ServoPwm_WritePulseUs(SERVO_PWM_BAFFLE, Actuator_FrameAngleToPulse(angle_deg));
  if (!ServoPwm_IsEnabled(SERVO_PWM_BAFFLE) &&
      ServoPwm_Start(SERVO_PWM_BAFFLE) != HAL_OK)
  {
    return HAL_ERROR;
  }
  baffle_pwm_suspended = 0U;
  return HAL_OK;
}

HAL_StatusTypeDef Actuator_StopBafflePwm(void)
{
  if (!ServoPwm_IsTimerValid()) return HAL_ERROR;
  if (ServoPwm_IsEnabled(SERVO_PWM_BAFFLE) &&
      ServoPwm_Stop(SERVO_PWM_BAFFLE) != HAL_OK)
  {
    return HAL_ERROR;
  }
  /* Paired pose setters must not write or restart PA2 while suspended. */
  baffle_pwm_suspended = 1U;
  return HAL_OK;
}

HAL_StatusTypeDef Actuator_SetBaffleUpClawOpen(uint16_t claw_angle_deg)
{
  return Actuator_SetFrameAngles(ROBOT_BAFFLE_UP_DEG, claw_angle_deg);
}

HAL_StatusTypeDef Actuator_SetFrameLowered(void)
{
  return Actuator_SetFrameAngles(ROBOT_BAFFLE_DOWN_DEG,
                                 ROBOT_CLAW_CLOSED_DEG);
}

HAL_StatusTypeDef Actuator_SetClawAngle(uint16_t angle_deg)
{
  if (!ServoPwm_IsEnabled(SERVO_PWM_CLAW))
  {
    return HAL_ERROR;
  }

  /* CH4 is already running; keep PWM continuous and do not move the baffle. */
  if (Actuator_DeferClawCommand(angle_deg) == 0U)
  {
    ServoPwm_WritePulseUs(SERVO_PWM_CLAW, Actuator_FrameAngleToPulse(angle_deg));
  }
  return HAL_OK;
}

HAL_StatusTypeDef Actuator_SetClawClosed(void)
{
  return Actuator_SetClawAngle(ROBOT_CLAW_CLOSED_DEG);
}

void Actuator_DeferClawClose(uint32_t now)
{
  claw_close_guard_tick = now;
  claw_close_pending = 1U;
  claw_close_guard_active = 1U;
}

uint8_t Actuator_ClawCloseDeferred(void)
{
  return claw_close_guard_active;
}

void Actuator_CancelDeferredClawClose(void)
{
  claw_close_guard_active = 0U;
  claw_close_pending = 0U;
}

HAL_StatusTypeDef Actuator_ServiceDeferredClawClose(uint32_t now)
{
  if (claw_close_guard_active == 0U ||
      (uint32_t)(now - claw_close_guard_tick) < ROBOT_S6_EXIT_BAFFLE_SETTLE_MS)
  {
    return HAL_OK;
  }
  uint8_t pending = claw_close_pending;
  Actuator_CancelDeferredClawClose();
  return pending ? Actuator_SetClawClosed() : HAL_OK;
}

static uint16_t Actuator_PartialFrameAngle(uint16_t down_angle_deg,
                                            uint16_t up_angle_deg,
                                            uint16_t lift_angle_deg)
{
  uint16_t travel_deg;

  if (up_angle_deg >= down_angle_deg)
  {
    travel_deg = up_angle_deg - down_angle_deg;
    if (lift_angle_deg >= travel_deg)
    {
      return up_angle_deg;
    }
    return down_angle_deg + lift_angle_deg;
  }

  travel_deg = down_angle_deg - up_angle_deg;
  if (lift_angle_deg >= travel_deg)
  {
    return up_angle_deg;
  }
  return down_angle_deg - lift_angle_deg;
}

HAL_StatusTypeDef Actuator_SetFramePartiallyRaised(uint16_t lift_angle_deg)
{
  return Actuator_SetFrameAngles(
      Actuator_PartialFrameAngle(ROBOT_LEFT_FRAME_DOWN_DEG,
                                  ROBOT_LEFT_FRAME_UP_DEG, lift_angle_deg),
      Actuator_PartialFrameAngle(ROBOT_RIGHT_FRAME_DOWN_DEG,
                                  ROBOT_RIGHT_FRAME_UP_DEG, lift_angle_deg));
}

static HAL_StatusTypeDef Actuator_SetCameraAngle(uint16_t angle_deg)
{
  if (!ServoPwm_IsTimerValid())
  {
    return HAL_ERROR;
  }

  ServoPwm_WritePulseUs(SERVO_PWM_CAMERA, Actuator_Mg90AngleToPulse(angle_deg));
  return HAL_OK;
}

HAL_StatusTypeDef Actuator_SetCameraWideView(void)
{
  return Actuator_SetCameraAngle(ROBOT_CAMERA_WIDE_ANGLE_DEG);
}

HAL_StatusTypeDef Actuator_SetCameraObjectTrackView(void)
{
  return Actuator_SetCameraAngle(ROBOT_CAMERA_OBJECT_TRACK_ANGLE_DEG);
}

HAL_StatusTypeDef Actuator_SetCameraSafeZoneView(void)
{
  return Actuator_SetCameraAngle(ROBOT_CAMERA_SAFE_ZONE_ANGLE_DEG);
}

HAL_StatusTypeDef Actuator_SetCameraFinalTrackView(void)
{
  return Actuator_SetCameraAngle(ROBOT_CAMERA_FINAL_TRACK_ANGLE_DEG);
}

HAL_StatusTypeDef Actuator_SetCameraNearView(void)
{
  return Actuator_SetCameraAngle(ROBOT_CAMERA_NEAR_ANGLE_DEG);
}
