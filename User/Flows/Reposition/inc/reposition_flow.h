#ifndef REPOSITION_FLOW_H
#define REPOSITION_FLOW_H
#include "flow_motion.h"

typedef enum { SIDE_TURN, SIDE_FORWARD, SIDE_FACE_SAFE, SIDE_DONE } SideFlow_Phase;
typedef enum { CORNER_TURN_AWAY, CORNER_FORWARD_AWAY, CORNER_TURN_SIDE,
               CORNER_FORWARD_SIDE, CORNER_FACE_SAFE, CORNER_DONE } CornerFlow_Phase;

SideFlow_Phase RepositionFlow_StepSide(const FlowMotion *io, SideFlow_Phase phase,
    uint32_t now, uint32_t started_at, uint32_t forward_ms, int32_t side_yaw, int32_t safe_yaw);
CornerFlow_Phase RepositionFlow_StepCorner(const FlowMotion *io, CornerFlow_Phase phase,
    uint32_t now, uint32_t started_at, int32_t away_yaw, int32_t side_yaw, int32_t safe_yaw);
#endif
