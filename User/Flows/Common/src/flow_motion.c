#include "flow_motion.h"
#include "chassis_actions.h"

bool FlowMotion_RunLeg(const FlowMotion *motion, FlowLeg leg, uint32_t now, uint32_t started_at)
{
  if (leg.kind == FLOW_TURN) return motion->turn(now, leg.yaw_cdeg);
  const ChassisActionStep step = ChassisMotion_TimedDriveStep(started_at, now, leg.duration_ms, leg.rpm);
  if (step.status == CHASSIS_ACTION_DONE) return true;
  if (leg.kind == FLOW_DRIVE_HEADING)
    motion->straight(now, leg.yaw_cdeg, leg.rpm, leg.wheel_max_rpm);
  else
    motion->wheels(step.wheels.left_rpm, step.wheels.right_rpm);
  return false;
}
