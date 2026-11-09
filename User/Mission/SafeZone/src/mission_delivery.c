#include "mission_delivery.h"
#include "mission_internal.h"
#include "mission_state.h"
#include "mission_safety.h"
#include "mission_capture.h"
#include "mission_safe_zone.h"
#include "mission_strategy.h"

#include "actuator_task.h"
#include "delivery_flow.h"
#include "debug_uart_task.h"
#include "maixcam_task.h"
#include "robot_config.h"
#include "stm32f4xx_hal.h"

void MissionTask_ResetS6BorderProgress(uint32_t now)
{
  DeliveryFlow_ResetProgress(&mission_ctx.delivery_flow, now);
}

bool MissionTask_StopS6DeliveryBafflePwm(void)
{
  if (!DeliveryFlow_ReleasePwm(&mission_ctx.delivery_flow, mission_ctx.s6_found_baffle_pwm_active, Actuator_StopBafflePwm))
  {
    mission_ctx.mission_snapshot.fault_flags |= MISSION_FAULT_ACTUATOR;
    return false;
  }
  mission_ctx.s6_found_baffle_pwm_active = false;
  return true;
}

static void MissionTask_CommandDeliveryStraight(uint32_t now, int32_t yaw_cdeg,
    int16_t rpm, int16_t max_rpm)
{
  if (mission_ctx.mission_snapshot.state != MISSION_STATE_S6_BORDER_BACKOFF)
    mission_ctx.mission_snapshot.safe_zone_fixed_heading_active = true;
  MissionTask_CommandS6Straight(now, yaw_cdeg, rpm, max_rpm);
}

static DeliveryFlow_Result MissionTask_StepDelivery(DeliveryFlow_Phase phase, uint32_t now, uint16_t open_deg)
{
  const DeliveryFlow_Io io = {
    .motion = { .stop = MissionTask_StopWheels, .turn = MissionTask_RunS6TurnToHeading,
                .straight = MissionTask_CommandDeliveryStraight },
    .baffle = Actuator_SetBaffleAngle,
    .claw = Actuator_SetClawAngle,
    .close_claw = Actuator_SetClawClosed,
  };
  const bool held_before = mission_ctx.delivery_flow.baffle_held;
  const DeliveryFlow_Result result = DeliveryFlow_Step(&mission_ctx.delivery_flow, &io, phase, now,
      mission_ctx.mission_snapshot.state_entry_tick, mission_ctx.s6_push_yaw_target_cdeg, mission_ctx.mission_snapshot.yaw_cdeg,
      open_deg, &mission_ctx.mission_snapshot.vision_forward_rpm);
  if ((!held_before && mission_ctx.delivery_flow.baffle_held) || result == DELIVERY_FORCED_RETREAT)
    mission_ctx.s6_found_baffle_pwm_active = false;
  return result;
}

void MissionTask_RunS6FinalAlign(uint32_t now)
{
  if (mission_ctx.mission_snapshot.state != MISSION_STATE_S6_FINAL_ALIGN) return;
  if (MissionTask_StepDelivery(DELIVERY_ALIGN, now, 0U) != DELIVERY_ALIGNED) return;
  MissionTask_ResetS6ControllerState();
  mission_ctx.mission_snapshot.safe_zone_fixed_heading_active = true;
  mission_ctx.s6_next_debug_tick = now;
  MissionTask_EnterState(MISSION_STATE_S6_BORDER_LOCATE, now);
  (void)DebugUart_Logf("[26] TURN OK BORDER=%d/%lu\r\n",
      ROBOT_S6_BORDER_LOCATE_RPM, (unsigned long)ROBOT_S6_BORDER_LOCATE_MS);
}

void MissionTask_RunS6BorderLocate(uint32_t now)
{
  DeliveryFlow_Phase phase;
  switch (mission_ctx.mission_snapshot.state)
  {
    case MISSION_STATE_S6_BORDER_LOCATE: phase = DELIVERY_LOCATE; break;
    case MISSION_STATE_S6_BORDER_BACKOFF: phase = DELIVERY_BACKOFF; break;
    case MISSION_STATE_S6_FINAL_PUSH: phase = DELIVERY_PUSH; break;
    default: return;
  }
  const uint8_t cargo_id = MissionTask_GetCarriedObjectId();
  const uint16_t open_deg = cargo_id == 4U ? ROBOT_CLAW_CASUALTY_RELEASE_DEG
      : (MissionTask_S6IsSupplyTarget() ? ROBOT_CLAW_MATERIAL_RELEASE_DEG
                                        : ROBOT_CLAW_CASUALTY_OPEN_DEG);
  DeliveryFlow_Result result = MissionTask_StepDelivery(phase, now, open_deg);
  if (result == DELIVERY_BACKED_OFF)
  {
    MissionTask_ResetS6ControllerState();
    MissionTask_EnterState(MISSION_STATE_S6_FINAL_PUSH, now);
    if (mission_ctx.mission_snapshot.state != MISSION_STATE_S6_FINAL_PUSH) return;
    /* Preserve the same-tick start of the second push. */
    phase = DELIVERY_PUSH;
    result = MissionTask_StepDelivery(phase, now, open_deg);
  }
  const uint32_t elapsed = now - mission_ctx.mission_snapshot.state_entry_tick;
  if (result == DELIVERY_DRIVE_TIMEOUT || result == DELIVERY_ACTUATOR_FAULT)
  {
    mission_ctx.mission_snapshot.fault_flags |= result == DELIVERY_DRIVE_TIMEOUT
        ? MISSION_FAULT_S6_TIMEOUT : MISSION_FAULT_ACTUATOR;
    MissionTask_EnterState(MISSION_STATE_FAULT, now);
    return;
  }
  if (result == DELIVERY_LOCATED)
  {
    MissionTask_ResetS6ControllerState();
    MissionTask_EnterState(MISSION_STATE_S6_BORDER_BACKOFF, now);
    if (mission_ctx.mission_snapshot.state != MISSION_STATE_S6_BORDER_BACKOFF) return;
    (void)MissionTask_StepDelivery(DELIVERY_BACKOFF, now, open_deg);
    (void)DebugUart_Logf("[26] BACKOFF CLAW=%u\r\n", (unsigned int)open_deg);
    return;
  }
  if (result == DELIVERY_FORCED_RETREAT)
  {
    const bool locating = phase == DELIVERY_LOCATE;
    (void)DebugUart_Logf("[26] %s TMO fwd=%lu/%lu total=%lu FORCE BACK>07\r\n",
        locating ? "BORDER" : "PUSH", (unsigned long)mission_ctx.delivery_flow.forward_ms,
        (unsigned long)(locating ? ROBOT_S6_BORDER_LOCATE_MS : ROBOT_S6_FINAL_PUSH_MS),
        (unsigned long)elapsed);
    mission_ctx.s6_push_yaw_target_cdeg = MissionTask_WrapYaw(mission_ctx.mission_snapshot.yaw_cdeg);
  }
  else if (result == DELIVERY_PLACED)
  {
    /* Mission owns scoring; preserve the count before the final retreat. */
    if (!mission_ctx.s7_delivery_counted)
    {
      mission_ctx.s7_delivery_counted = true;
      if (mission_ctx.s7_target_switch_enabled &&
          mission_ctx.s3_target_select_command == MAIXCAM_COMMAND_SELECT_BLACK_GREEN &&
          (cargo_id == 5U || cargo_id == 6U))
      {
        if (mission_ctx.s7_material_delivery_count < UINT16_MAX) ++mission_ctx.s7_material_delivery_count;
      }
      else if (mission_ctx.s7_target_switch_enabled && mission_ctx.s5_load_class_confirmed &&
          cargo_id == 4U)
      {
        if (mission_ctx.s7_casualty_delivery_count < UINT16_MAX) ++mission_ctx.s7_casualty_delivery_count;
      }
      (void)DebugUart_Logf("[TRIP] 51=%u red=%u onlyM=%u\r\n",
          (unsigned int)mission_ctx.s7_material_delivery_count,
          (unsigned int)mission_ctx.s7_casualty_delivery_count,
          MissionTask_S7MaterialOnly() ? 1U : 0U);
    }
  }
  else return;

  MissionTask_ResetS6ControllerState();
  mission_ctx.mission_snapshot.safe_zone_fixed_heading_active = true;
  mission_ctx.s6_next_debug_tick = now;
  MissionTask_EnterState(MISSION_STATE_S6_FINAL_REVERSE, now);
  if (result == DELIVERY_PLACED)
    (void)DebugUart_Logf("[26] PUSH OK fwd=%lu total=%lu PA2=%u back=%d/%lu\r\n",
        (unsigned long)mission_ctx.delivery_flow.forward_ms, (unsigned long)elapsed,
        (unsigned int)ROBOT_BAFFLE_UP_DEG, ROBOT_S6_REVERSE_RPM,
        (unsigned long)ROBOT_S6_REVERSE_DRIVE_MS);
}

void MissionTask_RunS6FinalReverse(uint32_t now)
{
  if (mission_ctx.mission_snapshot.state != MISSION_STATE_S6_FINAL_REVERSE) return;
  const DeliveryFlow_Result result = MissionTask_StepDelivery(DELIVERY_REVERSE, now, 0U);
  if (result == DELIVERY_DRIVE_TIMEOUT || result == DELIVERY_ACTUATOR_FAULT)
  {
    mission_ctx.mission_snapshot.fault_flags |= result == DELIVERY_DRIVE_TIMEOUT
        ? MISSION_FAULT_S6_TIMEOUT : MISSION_FAULT_ACTUATOR;
    MissionTask_EnterState(MISSION_STATE_FAULT, now);
    return;
  }
  if (result != DELIVERY_RETREATED) return;
  mission_ctx.s6_exit_yaw_target_cdeg = MissionTask_WrapYaw(
      mission_ctx.s6_push_yaw_target_cdeg + (int32_t)ROBOT_YAW_LEFT_SIGN * ROBOT_S6_EXIT_TURN_ANGLE_CDEG);
  MissionTask_ResetS6ControllerState();
  mission_ctx.s6_phase_stable_since = 0U;
  mission_ctx.s6_next_debug_tick = now;
  MissionTask_EnterState(MISSION_STATE_S6_EXIT_TURN_180, now);
  (void)DebugUart_Logf("[26] BACK OK v=%d ms=%lu CLAW CLOSED TURN180 goal=%ld max=%d >07\r\n",
      ROBOT_S6_REVERSE_RPM, (unsigned long)ROBOT_S6_REVERSE_DRIVE_MS,
      (long)mission_ctx.s6_exit_yaw_target_cdeg, ROBOT_S6_EXIT_TURN_MAX_RPM);
}

void MissionTask_RunS6ExitTurn180(uint32_t now)
{
  if (mission_ctx.mission_snapshot.state != MISSION_STATE_S6_EXIT_TURN_180)
  {
    return;
  }

  if (!MissionTask_RunS6TurnToHeading(now, mission_ctx.s6_exit_yaw_target_cdeg))
  {
    return;
  }

  MissionTask_StopWheels();
  uint16_t open_deg = MissionTask_GetCarriedObjectId() == 4U
      ? ROBOT_CLAW_CASUALTY_OPEN_DEG : ROBOT_CLAW_MATERIAL_OPEN_DEG;
  if (Actuator_SetClawAngle(open_deg) != HAL_OK)
  {
    mission_ctx.mission_snapshot.fault_flags |= MISSION_FAULT_ACTUATOR;
    MissionTask_EnterState(MISSION_STATE_FAULT, now);
    return;
  }
  (void)DebugUart_Logf(
      "[26] TURN180 END r=%ld goal=%ld CLAW=%u DOWN >07\r\n",
      (long)MissionTask_WrapYaw(mission_ctx.mission_snapshot.yaw_cdeg),
      (long)mission_ctx.s6_exit_yaw_target_cdeg, (unsigned int)open_deg);
  MissionTask_StartS7Decision(now);
}
