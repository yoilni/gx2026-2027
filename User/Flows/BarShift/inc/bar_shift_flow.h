#ifndef BAR_SHIFT_FLOW_H
#define BAR_SHIFT_FLOW_H
#include "flow_motion.h"

typedef enum { BAR_SHIFT_ALIGN, BAR_SHIFT_BACK, BAR_SHIFT_TURN_SIDE,
               BAR_SHIFT_FORWARD, BAR_SHIFT_FACE_SAFE, BAR_SHIFT_DONE } BarShiftFlow_Phase;

BarShiftFlow_Phase BarShiftFlow_Step(const FlowMotion *io, BarShiftFlow_Phase phase,
    uint32_t now, uint32_t started_at, int32_t back_yaw, int32_t side_yaw, int32_t safe_yaw);
#endif
