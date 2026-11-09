#include "bar_shift_flow.h"
#include "robot_config.h"

BarShiftFlow_Phase BarShiftFlow_Step(const FlowMotion *io, BarShiftFlow_Phase phase,
    uint32_t now, uint32_t started_at, int32_t back_yaw, int32_t side_yaw, int32_t safe_yaw)
{
  static const FlowLeg legs[] = {
    [BAR_SHIFT_ALIGN] = { .kind = FLOW_TURN },
    [BAR_SHIFT_BACK] = { .kind = FLOW_DRIVE_HEADING,
      .duration_ms = ROBOT_S6_BAR_REVERSE_MS, .rpm = -ROBOT_S6_BAR_REVERSE_RPM,
      .wheel_max_rpm = ROBOT_S6_BAR_WHEEL_MAX_RPM },
    [BAR_SHIFT_TURN_SIDE] = { .kind = FLOW_TURN },
    [BAR_SHIFT_FORWARD] = { .kind = FLOW_DRIVE_HEADING,
      .duration_ms = ROBOT_S6_BAR_FORWARD_MS, .rpm = ROBOT_S6_BAR_FORWARD_RPM,
      .wheel_max_rpm = ROBOT_S6_BAR_WHEEL_MAX_RPM },
    [BAR_SHIFT_FACE_SAFE] = { .kind = FLOW_TURN },
  };
  if ((unsigned)phase >= BAR_SHIFT_DONE) return phase;
  FlowLeg leg = legs[phase];
  leg.yaw_cdeg = phase == BAR_SHIFT_BACK ? back_yaw :
      (phase == BAR_SHIFT_TURN_SIDE || phase == BAR_SHIFT_FORWARD ? side_yaw : safe_yaw);
  if (!FlowMotion_RunLeg(io, leg, now, started_at)) return phase;
  io->stop();
  return (BarShiftFlow_Phase)(phase + 1);
}
