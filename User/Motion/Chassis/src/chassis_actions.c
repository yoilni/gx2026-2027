#include "chassis_actions.h"

ChassisActionStep ChassisMotion_ResolveTurnStep(HeadingControl_State *controller,
    uint32_t *stable_since, uint32_t now, int32_t absolute_error_cdeg,
    int32_t enter_tolerance_cdeg, int32_t release_tolerance_cdeg,
    uint32_t settle_ms, int16_t turn_rpm)
{
  const HeadingTurnStatus turn_status = HeadingControl_CheckTurn(stable_since, now,
      absolute_error_cdeg, enter_tolerance_cdeg, release_tolerance_cdeg, settle_ms);
  if (turn_status == HEADING_TURN_DRIVE)
  {
    return (ChassisActionStep){ .status = CHASSIS_ACTION_RUNNING,
      .wheels = { .left_rpm = turn_rpm, .right_rpm = (int16_t)-turn_rpm } };
  }
  controller->integral = 0.0f;
  return (ChassisActionStep){ .brake = true,
    .status = turn_status == HEADING_TURN_REACHED ? CHASSIS_ACTION_DONE : CHASSIS_ACTION_RUNNING };
}

ChassisActionStep ChassisMotion_TurnToYawStep(HeadingControl_State *controller,
    uint32_t *stable_since, const ChassisTurnRequest *request,
    uint32_t now, int32_t current_yaw_cdeg)
{
  if (request->timeout_ms != 0U && (uint32_t)(now - request->started_at) >= request->timeout_ms)
  {
    controller->integral = 0.0f;
    controller->initialized = false;
    *stable_since = 0U;
    return (ChassisActionStep){ .status = CHASSIS_ACTION_TIMEOUT, .brake = true };
  }
  const int32_t error = HeadingControl_YawError(request->target_yaw_cdeg, current_yaw_cdeg);
  const int16_t turn_rpm = HeadingControl_Update(controller, &request->controller, now, error);
  const ChassisWheelRpm wheels = ChassisMotion_MixRpm(0, turn_rpm, request->wheel_max_rpm);
  return ChassisMotion_ResolveTurnStep(controller, stable_since, now, error < 0 ? -error : error,
      request->enter_tolerance_cdeg, request->release_tolerance_cdeg,
      request->settle_ms, wheels.left_rpm);
}

ChassisActionStep ChassisMotion_TimedDriveStep(uint32_t started_at, uint32_t now,
    uint32_t duration_ms, int16_t forward_rpm)
{
  if ((uint32_t)(now - started_at) >= duration_ms)
  {
    return (ChassisActionStep){ .status = CHASSIS_ACTION_DONE, .brake = true };
  }
  return (ChassisActionStep){ .status = CHASSIS_ACTION_RUNNING,
    .wheels = { .left_rpm = forward_rpm, .right_rpm = forward_rpm } };
}

float ChassisMotion_SmoothStep(float fraction)
{
  if (fraction <= 0.0f) return 0.0f;
  if (fraction >= 1.0f) return 1.0f;
  return fraction * fraction * (3.0f - 2.0f * fraction);
}

float ChassisMotion_RampScale(uint32_t elapsed, uint32_t duration, uint32_t ramp_ms)
{
  if (elapsed >= duration) return 0.0f;
  if (ramp_ms > duration / 2U) ramp_ms = duration / 2U;
  if (ramp_ms == 0U) return 1.0f;
  const uint32_t edge_time = elapsed < duration - elapsed ? elapsed : duration - elapsed;
  return ChassisMotion_SmoothStep((float)edge_time / (float)ramp_ms);
}
