#ifndef OBSTACLE_FLOW_H
#define OBSTACLE_FLOW_H
#include "flow_motion.h"

typedef enum { OBSTACLE_TURN_OUT, OBSTACLE_FORWARD, OBSTACLE_TURN_BACK,
               OBSTACLE_FACE_SAFE, OBSTACLE_DONE } ObstacleFlow_Phase;

int32_t ObstacleFlow_RelativeHeading(int32_t yaw_cdeg, bool turn_right, bool outward);
ObstacleFlow_Phase ObstacleFlow_Step(const FlowMotion *io, ObstacleFlow_Phase phase,
    uint32_t now, uint32_t started_at, int32_t out_yaw, int32_t back_yaw, int32_t safe_yaw);
#endif
