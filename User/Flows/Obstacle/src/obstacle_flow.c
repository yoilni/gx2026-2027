#include "obstacle_flow.h"
#include "heading_control.h"
#include "robot_config.h"

int32_t ObstacleFlow_RelativeHeading(int32_t yaw_cdeg, bool turn_right, bool outward)
{
  const bool left = outward != turn_right;
  const int32_t delta = left ? ROBOT_S6_OBSTACLE_LEFT_ANGLE_CDEG : -ROBOT_S6_OBSTACLE_RIGHT_ANGLE_CDEG;
  return HeadingControl_WrapYaw(yaw_cdeg + (int32_t)ROBOT_YAW_LEFT_SIGN * delta);
}

ObstacleFlow_Phase ObstacleFlow_Step(const FlowMotion *io, ObstacleFlow_Phase phase,
    uint32_t now, uint32_t started_at, int32_t out_yaw, int32_t back_yaw, int32_t safe_yaw)
{
  static const FlowLeg legs[] = {
    [OBSTACLE_TURN_OUT] = { .kind = FLOW_TURN },
    [OBSTACLE_FORWARD] = { .kind = FLOW_DRIVE_OPEN, .duration_ms = ROBOT_S6_OBSTACLE_FORWARD_MS,
      .rpm = ROBOT_S6_OBSTACLE_DRIVE_RPM },
    [OBSTACLE_TURN_BACK] = { .kind = FLOW_TURN },
    [OBSTACLE_FACE_SAFE] = { .kind = FLOW_TURN },
  };
  if ((unsigned)phase >= OBSTACLE_DONE) return phase;
  FlowLeg leg = legs[phase];
  leg.yaw_cdeg = phase == OBSTACLE_TURN_OUT ? out_yaw :
      (phase == OBSTACLE_TURN_BACK ? back_yaw : safe_yaw);
  if (!FlowMotion_RunLeg(io, leg, now, started_at)) return phase;
  io->stop();
  return (ObstacleFlow_Phase)(phase + 1);
}
