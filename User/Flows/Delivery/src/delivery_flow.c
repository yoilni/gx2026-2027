#include "delivery_flow.h"
#include "heading_control.h"
#include "robot_config.h"

void DeliveryFlow_ResetProgress(DeliveryFlow_State *flow, uint32_t now)
{
  flow->forward_ms = 0U;
  flow->command_tick = now;
  flow->advancing = false;
}

bool DeliveryFlow_ReleasePwm(DeliveryFlow_State *flow, bool also_active,
    HAL_StatusTypeDef (*stop_pwm)(void))
{
  if (!also_active && !flow->baffle_held) return true;
  if (stop_pwm() != HAL_OK) return false;
  flow->baffle_held = false;
  return true;
}

DeliveryFlow_Result DeliveryFlow_Step(DeliveryFlow_State *flow, const DeliveryFlow_Io *io,
    DeliveryFlow_Phase phase, uint32_t now, uint32_t started_at,
    int32_t yaw_target, int32_t yaw_now, uint16_t release_angle,
    const int16_t *effective_forward_rpm)
{
  if (phase == DELIVERY_ALIGN)
  {
    if (!io->motion.turn(now, yaw_target)) return DELIVERY_RUNNING;
    io->motion.stop();
    return DELIVERY_ALIGNED;
  }

  const uint32_t elapsed = (uint32_t)(now - started_at);
  if (phase == DELIVERY_BACKOFF || phase == DELIVERY_REVERSE)
  {
    const bool retreat = phase == DELIVERY_REVERSE;
    if (elapsed >= ROBOT_S6_REVERSE_TIMEOUT_MS) return DELIVERY_DRIVE_TIMEOUT;
    const FlowLeg leg = { .kind = FLOW_DRIVE_HEADING, .yaw_cdeg = yaw_target,
      .duration_ms = retreat ? ROBOT_S6_REVERSE_DRIVE_MS : ROBOT_S6_BORDER_BACKOFF_MS,
      .rpm = retreat ? -ROBOT_S6_REVERSE_RPM : -ROBOT_S6_BORDER_LOCATE_RPM,
      .wheel_max_rpm = retreat ? ROBOT_S6_REVERSE_WHEEL_MAX_RPM : ROBOT_S6_BORDER_LOCATE_WHEEL_MAX_RPM };
    if (!FlowMotion_RunLeg(&io->motion, leg, now, started_at)) return DELIVERY_RUNNING;
    io->motion.stop();
    if (retreat) return io->close_claw() == HAL_OK ? DELIVERY_RETREATED : DELIVERY_ACTUATOR_FAULT;
    return io->baffle(ROBOT_S6_PUSH_BAFFLE_DEG) == HAL_OK ? DELIVERY_BACKED_OFF : DELIVERY_ACTUATOR_FAULT;
  }

  const bool locating = phase == DELIVERY_LOCATE;
  const uint32_t forward_ms = locating ? ROBOT_S6_BORDER_LOCATE_MS : ROBOT_S6_FINAL_PUSH_MS;
  /* Attribute the previous interval to the command that was actually held.
     Heading correction in place cannot consume a push's forward budget. */
  if (flow->advancing) flow->forward_ms += (uint32_t)(now - flow->command_tick);
  flow->command_tick = now;
  if (elapsed >= ROBOT_S6_BORDER_LOCATE_TIMEOUT_MS)
  {
    io->motion.stop();
    if (io->claw(release_angle) != HAL_OK || io->baffle(ROBOT_BAFFLE_UP_DEG) != HAL_OK)
      return DELIVERY_ACTUATOR_FAULT;
    flow->baffle_held = false;
    flow->advancing = false;
    return DELIVERY_FORCED_RETREAT;
  }
  if (locating && !flow->baffle_held)
  {
    if (io->baffle(ROBOT_BAFFLE_DOWN_DEG) != HAL_OK) return DELIVERY_ACTUATOR_FAULT;
    flow->baffle_held = true;
  }

  const bool forward_done = flow->forward_ms >= forward_ms;
  const int32_t yaw_error = HeadingControl_YawError(yaw_target, yaw_now);
  if (forward_done && yaw_error >= -ROBOT_S6_YAW_FORWARD_ENABLE_CDEG &&
      yaw_error <= ROBOT_S6_YAW_FORWARD_ENABLE_CDEG)
  {
    io->motion.stop();
    if (locating) return io->claw(release_angle) == HAL_OK ? DELIVERY_LOCATED : DELIVERY_ACTUATOR_FAULT;
    if (io->baffle(ROBOT_BAFFLE_UP_DEG) != HAL_OK) return DELIVERY_ACTUATOR_FAULT;
    flow->baffle_held = false;
    return DELIVERY_PLACED;
  }

  io->motion.straight(now, yaw_target, forward_done ? 0 : ROBOT_S6_BORDER_LOCATE_RPM,
                      ROBOT_S6_BORDER_LOCATE_WHEEL_MAX_RPM);
  flow->advancing = *effective_forward_rpm > 0;
  return DELIVERY_RUNNING;
}
