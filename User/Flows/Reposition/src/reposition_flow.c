#include "reposition_flow.h"
#include "robot_config.h"

SideFlow_Phase RepositionFlow_StepSide(const FlowMotion *io, SideFlow_Phase phase,
    uint32_t now, uint32_t started_at, uint32_t forward_ms, int32_t side_yaw, int32_t safe_yaw)
{
  static const FlowLeg legs[] = {
    [SIDE_TURN] = { .kind = FLOW_TURN },
    [SIDE_FORWARD] = { .kind = FLOW_DRIVE_OPEN,
      .rpm = ROBOT_S6_REPOSITION_FORWARD_RPM },
    [SIDE_FACE_SAFE] = { .kind = FLOW_TURN },
  };
  if ((unsigned)phase >= SIDE_DONE) return phase;
  FlowLeg leg = legs[phase];
  leg.yaw_cdeg = phase == SIDE_TURN ? side_yaw : safe_yaw;
  if (phase == SIDE_FORWARD) leg.duration_ms = forward_ms;
  if (!FlowMotion_RunLeg(io, leg, now, started_at)) return phase;
  if (phase != SIDE_FACE_SAFE) io->stop();
  return (SideFlow_Phase)(phase + 1);
}

CornerFlow_Phase RepositionFlow_StepCorner(const FlowMotion *io, CornerFlow_Phase phase,
    uint32_t now, uint32_t started_at, int32_t away_yaw, int32_t side_yaw, int32_t safe_yaw)
{
  static const FlowLeg legs[] = {
    [CORNER_TURN_AWAY] = { .kind = FLOW_TURN },
    [CORNER_FORWARD_AWAY] = { .kind = FLOW_DRIVE_HEADING,
      .duration_ms = ROBOT_S6_CORNER_AWAY_FORWARD_MS, .rpm = ROBOT_S6_CORNER_FORWARD_RPM,
      .wheel_max_rpm = ROBOT_S6_CORNER_WHEEL_MAX_RPM },
    [CORNER_TURN_SIDE] = { .kind = FLOW_TURN },
    [CORNER_FORWARD_SIDE] = { .kind = FLOW_DRIVE_HEADING,
      .duration_ms = ROBOT_S6_CORNER_SIDE_FORWARD_MS, .rpm = ROBOT_S6_CORNER_FORWARD_RPM,
      .wheel_max_rpm = ROBOT_S6_CORNER_WHEEL_MAX_RPM },
    [CORNER_FACE_SAFE] = { .kind = FLOW_TURN },
  };
  if ((unsigned)phase >= CORNER_DONE) return phase;
  FlowLeg leg = legs[phase];
  leg.yaw_cdeg = phase == CORNER_FACE_SAFE ? safe_yaw :
      (phase == CORNER_TURN_AWAY || phase == CORNER_FORWARD_AWAY ? away_yaw : side_yaw);
  if (!FlowMotion_RunLeg(io, leg, now, started_at)) return phase;
  if (phase != CORNER_FACE_SAFE) io->stop();
  return (CornerFlow_Phase)(phase + 1);
}
