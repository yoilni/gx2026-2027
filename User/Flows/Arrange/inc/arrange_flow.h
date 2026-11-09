#ifndef ARRANGE_FLOW_H
#define ARRANGE_FLOW_H
#include "flow_motion.h"

typedef enum { ARRANGE_OFFSET, ARRANGE_TRACK, ARRANGE_TURN, ARRANGE_PUSH,
               ARRANGE_COUNTER, ARRANGE_FINISH_LEFT, ARRANGE_LEGACY_END,
               ARRANGE_DONE, ARRANGE_TIMEOUT } ArrangeFlow_Phase;

typedef struct
{
  FlowMotion motion;
  bool (*track)(uint32_t now, bool *in_tolerance);
  int16_t (*yaw_pid)(uint32_t now, int32_t heading, float min_rpm, float max_rpm);
} ArrangeFlow_Io;

typedef struct
{
  int32_t turn_yaw;
  int32_t forward_yaw;
  int32_t spin_previous_yaw;
  int32_t spin_progress;
} ArrangeFlow_State;

typedef struct
{
  bool first_green;
  bool side_push;
  bool right_pass;
  bool casualty;
  bool turn_left;
} ArrangeFlow_Profile;

typedef struct
{
  ArrangeFlow_Phase next;
  bool reset_controller;
  bool reset_stable;
  bool fixed_output;
  int16_t forward_rpm;
  int16_t turn_rpm;
} ArrangeFlow_StepResult;

/* Caller has already checked the tracking sample and preempting events. */
ArrangeFlow_StepResult ArrangeFlow_Step(ArrangeFlow_State *flow, const ArrangeFlow_Io *io,
    ArrangeFlow_Phase phase, ArrangeFlow_Profile profile, uint32_t now, uint32_t started_at,
    int32_t yaw_now);

/* Legacy curved paths and their distance-matched reverse replay. */
uint32_t ArrangeFlow_CornerPushMs(bool push_right);
uint32_t ArrangeFlow_CornerReverseMs(bool push_right);
void ArrangeFlow_CornerRpm(uint32_t elapsed, uint32_t duration, bool push_right,
    int16_t *left_rpm, int16_t *right_rpm);
void ArrangeFlow_CornerReverseRpm(uint32_t elapsed, bool push_right,
    int16_t *left_rpm, int16_t *right_rpm);

typedef enum { SCATTER_TURN_IN, SCATTER_FORWARD, SCATTER_COUNTER, SCATTER_FINISH,
               SCATTER_DONE } ScatterFlow_Phase;
ScatterFlow_Phase ArrangeFlow_StepScatter(const FlowMotion *io, ScatterFlow_Phase phase,
    uint32_t now, uint32_t started_at, int32_t yaw_now, int32_t *target_yaw);

#endif
