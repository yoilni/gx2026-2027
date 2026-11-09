#include "departure_flow.h"
#include "robot_config.h"

DepartureTurnResult DepartureFlow_Turn(const FlowMotion *io,
    bool (*expired)(uint32_t now, int32_t target), uint32_t now, int32_t target,
    int32_t yaw_error, bool was_stable, uint32_t *stable_since)
{
  const int32_t magnitude = yaw_error < 0 ? -yaw_error : yaw_error;
  if (magnitude > ROBOT_S1_TURN_ANGLE_CDEG + ROBOT_S1_WRONG_DIRECTION_MARGIN_CDEG)
    return (DepartureTurnResult){ .status = DEPARTURE_WRONG_DIRECTION };
  if (expired(now, target)) return (DepartureTurnResult){ .status = DEPARTURE_OPEN_CLAW };
  if (magnitude <= ROBOT_S1_TURN_TOLERANCE_CDEG)
  {
    io->stop();
    if (!was_stable) *stable_since = now;
    return (DepartureTurnResult){ .newly_stable = !was_stable,
      .status = (uint32_t)(now - *stable_since) >= ROBOT_S1_TURN_STABLE_MS
          ? DEPARTURE_OPEN_CLAW : DEPARTURE_STABLE };
  }
  *stable_since = 0U;
  const int16_t rpm = magnitude > ROBOT_S1_TURN_SLOW_THRESHOLD_CDEG
      ? ROBOT_S1_TURN_FAST_RPM : ROBOT_S1_TURN_SLOW_RPM;
  const bool left = yaw_error * (int32_t)ROBOT_YAW_LEFT_SIGN > 0L;
  io->wheels(left ? -rpm : rpm, left ? rpm : -rpm);
  return (DepartureTurnResult){ .status = DEPARTURE_TURNING };
}

HAL_StatusTypeDef DepartureFlow_OpenClaw(void (*stop)(void), HAL_StatusTypeDef (*claw)(uint16_t))
{
  stop();
  return claw(ROBOT_CLAW_MATERIAL_OPEN_DEG);
}

DepartureStepStatus DepartureFlow_LiftAfterClaw(uint32_t now, uint32_t started_at,
    void (*stop)(void), HAL_StatusTypeDef (*baffle)(uint16_t))
{
  stop();
  if ((uint32_t)(now - started_at) < ROBOT_S1_CLAW_OPEN_SETTLE_MS) return DEPARTURE_WAITING;
  return baffle(ROBOT_BAFFLE_UP_DEG) == HAL_OK ? DEPARTURE_READY : DEPARTURE_ACTUATOR_FAULT;
}

DepartureBumpResult DepartureFlow_CrossBump(const FlowMotion *io,
    HAL_StatusTypeDef (*baffle)(uint16_t), HAL_StatusTypeDef (*close_claw)(void),
    int16_t (*yaw_correction)(void), uint32_t now, uint32_t started_at,
    bool *drive_finished, uint32_t *baffle_down_tick)
{
  if ((uint32_t)(now - started_at) < ROBOT_S2_CROSS_BUMP_DRIVE_MS)
  {
    const int16_t correction = yaw_correction();
    /* Slow only the inner wheel; never exceed the crossing cruise speed. */
    const int16_t left = ROBOT_S2_CROSS_BUMP_RPM - (correction > 0 ? correction : 0);
    const int16_t right = ROBOT_S2_CROSS_BUMP_RPM + (correction < 0 ? correction : 0);
    io->wheels(left, right);
    return (DepartureBumpResult){ .status = DEPARTURE_DRIVING, .correction = correction };
  }

  DepartureBumpResult result = { .status = DEPARTURE_WAITING };
  if (!*drive_finished)
  {
    io->stop();
    if (baffle(ROBOT_BAFFLE_DOWN_DEG) != HAL_OK)
      return (DepartureBumpResult){ .status = DEPARTURE_ACTUATOR_FAULT };
    *drive_finished = true;
    *baffle_down_tick = now;
    result.just_lowered = true;
  }
  if ((uint32_t)(now - *baffle_down_tick) >= ROBOT_S2_CROSS_BUMP_SETTLE_MS)
    result.status = close_claw() == HAL_OK ? DEPARTURE_READY : DEPARTURE_ACTUATOR_FAULT;
  return result;
}
