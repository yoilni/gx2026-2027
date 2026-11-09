#include "recovery_flow.h"
#include "heading_control.h"
#include "robot_config.h"

void RecoveryFlow_ObserveSpin(int32_t *previous_yaw, int32_t *progress, int32_t current_yaw,
    int32_t yaw_direction)
{
  *progress += yaw_direction * HeadingControl_YawError(current_yaw, *previous_yaw);
  *previous_yaw = HeadingControl_WrapYaw(current_yaw);
  if (*progress < 0) *progress = 0;
}

int16_t RecoveryFlow_TaperSpin(int32_t target, int32_t progress, int32_t slow_threshold,
    int16_t fast_rpm, int16_t slow_rpm)
{
  return target - progress > slow_threshold ? fast_rpm : slow_rpm;
}

RedSweep_Phase RecoveryFlow_StepRed(const FlowMotion *io, RedSweep_Phase phase,
    bool (*expired)(uint32_t now, int32_t target), uint32_t now, uint32_t started_at,
    uint32_t forward_ms, int16_t forward_rpm, int32_t yaw, int32_t *origin, int32_t *target,
    int16_t *turn_command, int32_t *yaw_error)
{
  if (phase == RECOVERY_RED_FORWARD)
  {
    const FlowLeg leg = { .kind = FLOW_DRIVE_OPEN, .duration_ms = forward_ms, .rpm = forward_rpm };
    if (!FlowMotion_RunLeg(io, leg, now, started_at)) return phase;
    io->stop();
    *origin = HeadingControl_WrapYaw(yaw);
    *target = HeadingControl_WrapYaw(*origin +
        (int32_t)ROBOT_YAW_LEFT_SIGN * ROBOT_S7_E3_SEARCH_SIDE_CDEG);
    return RECOVERY_RED_LEFT;
  }
  if (phase == RECOVERY_RED_HOLD_LEFT || phase == RECOVERY_RED_HOLD_RIGHT)
  {
    io->stop();
    if ((uint32_t)(now - started_at) < ROBOT_S7_E3_SEARCH_HOLD_MS) return phase;
    if (phase == RECOVERY_RED_HOLD_LEFT)
    {
      *target = HeadingControl_WrapYaw(*origin -
          (int32_t)ROBOT_YAW_LEFT_SIGN * ROBOT_S7_E3_SEARCH_SIDE_CDEG);
      return RECOVERY_RED_RIGHT;
    }
    *target = *origin;
    return RECOVERY_RED_HOME;
  }
  if (phase == RECOVERY_RED_DONE) return phase;
  *yaw_error = HeadingControl_YawError(*target, yaw);
  const int32_t error = *yaw_error < 0 ? -*yaw_error : *yaw_error;
  if (expired(now, *target) || error <= ROBOT_S3_SEARCH_TURN_TOLERANCE_CDEG)
  {
    io->stop();
    if (phase == RECOVERY_RED_HOME) return RECOVERY_RED_DONE;
    return phase == RECOVERY_RED_LEFT ? RECOVERY_RED_HOLD_LEFT : RECOVERY_RED_HOLD_RIGHT;
  }
  const int16_t rpm = error > ROBOT_S3_SEARCH_TURN_SLOW_THRESHOLD_CDEG
      ? ROBOT_S7_E3_SEARCH_TURN_FAST_RPM : ROBOT_S7_E3_SEARCH_TURN_SLOW_RPM;
  *turn_command = *yaw_error * (int32_t)ROBOT_YAW_LEFT_SIGN > 0L ? -rpm : rpm;
  io->wheels(*turn_command, (int16_t)-*turn_command);
  return phase;
}

ZoneRecovery_Phase RecoveryFlow_StepZone(const FlowMotion *io, ZoneRecovery_Phase phase,
    uint32_t now, uint32_t started_at, int32_t left_yaw, int32_t right_yaw, int32_t safe_yaw)
{
  static const FlowLeg legs[] = {
    [ZONE_RECOVERY_FIRST_DRIVE] = { .kind = FLOW_DRIVE_OPEN,
      .duration_ms = ROBOT_S6_RECOVERY_FIRST_FORWARD_MS, .rpm = ROBOT_S6_RECOVERY_FIRST_FORWARD_RPM },
    [ZONE_RECOVERY_FACE] = { .kind = FLOW_TURN },
    [ZONE_RECOVERY_LONG_DRIVE] = { .kind = FLOW_DRIVE_OPEN,
      .duration_ms = ROBOT_S6_RECOVERY_LONG_FORWARD_MS, .rpm = ROBOT_S6_RECOVERY_LONG_FORWARD_RPM },
    [ZONE_RECOVERY_LEFT] = { .kind = FLOW_TURN },
    [ZONE_RECOVERY_RIGHT] = { .kind = FLOW_TURN },
    [ZONE_RECOVERY_HOME] = { .kind = FLOW_TURN },
  };
  if ((unsigned)phase >= ZONE_RECOVERY_DONE) return phase;
  FlowLeg leg = legs[phase];
  leg.yaw_cdeg = phase == ZONE_RECOVERY_LEFT ? left_yaw :
      (phase == ZONE_RECOVERY_RIGHT ? right_yaw : safe_yaw);
  if (!FlowMotion_RunLeg(io, leg, now, started_at)) return phase;
  if (phase != ZONE_RECOVERY_LEFT) io->stop();
  if (phase == ZONE_RECOVERY_FIRST_DRIVE) return ZONE_RECOVERY_FACE;
  if (phase == ZONE_RECOVERY_FACE || phase == ZONE_RECOVERY_LONG_DRIVE) return ZONE_RECOVERY_LEFT;
  return (ZoneRecovery_Phase)(phase + 1);
}

RecoveryScan_Phase RecoveryFlow_StepScan(const FlowMotion *io, RecoveryScan_Profile profile,
    RecoveryScan_Phase phase, uint32_t now, uint32_t started_at, int32_t heading)
{
  if (phase == RECOVERY_SCAN_DONE) return phase;
  FlowLeg leg = { .kind = FLOW_TURN, .yaw_cdeg = heading };
  if (phase == RECOVERY_SCAN_BACK)
  {
    leg.kind = FLOW_DRIVE_OPEN;
    leg.duration_ms = profile == RECOVERY_POST22 ? ROBOT_S4_POST22_REVERSE_MS :
        (profile == RECOVERY_CAPTURE_LOSS ? ROBOT_S4_FINAL_RECOVERY_REVERSE_MS : ROBOT_S4_ARRANGE_RECOVERY_REVERSE_MS);
    leg.rpm = profile == RECOVERY_POST22 ? -ROBOT_S4_POST22_REVERSE_RPM :
        (profile == RECOVERY_CAPTURE_LOSS ? -ROBOT_S4_FINAL_RECOVERY_REVERSE_RPM : -ROBOT_S4_ARRANGE_RECOVERY_REVERSE_RPM);
  }
  if (!FlowMotion_RunLeg(io, leg, now, started_at)) return phase;
  if (profile == RECOVERY_CAPTURE_LOSS ||
      (profile == RECOVERY_ARRANGE_LOSS && phase == RECOVERY_SCAN_BACK) ||
      (profile == RECOVERY_POST22 && phase == RECOVERY_SCAN_SECOND)) io->stop();
  return (RecoveryScan_Phase)(phase + 1);
}
