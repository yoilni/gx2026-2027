#ifndef HEADING_CONTROL_H
#define HEADING_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
  uint32_t update_tick;
  float integral;
  float last_error;
  bool initialized;
} HeadingControl_State;

typedef struct
{
  float kp;
  float ki;
  float kd;
  float integral_limit;
  float output_scale;
  float min_turn_rpm;
  float max_turn_rpm;
  uint32_t initial_period_ms;
  int8_t yaw_left_sign;
} HeadingControl_Config;

static inline int32_t HeadingControl_WrapYaw(int32_t angle_cdeg)
{
  while (angle_cdeg < 0L) angle_cdeg += 36000L;
  while (angle_cdeg >= 36000L) angle_cdeg -= 36000L;
  return angle_cdeg;
}

static inline int32_t HeadingControl_YawError(int32_t target_cdeg, int32_t current_cdeg)
{
  int32_t error = HeadingControl_WrapYaw(target_cdeg) - HeadingControl_WrapYaw(current_cdeg);
  if (error > 18000L) error -= 36000L;
  else if (error < -18000L) error += 36000L;
  return error;
}

static inline int16_t HeadingControl_ProportionalRpm(int32_t error_cdeg,
    int32_t tolerance_cdeg, int8_t yaw_left_sign, float kp, float max_rpm)
{
  if (error_cdeg >= -tolerance_cdeg && error_cdeg <= tolerance_cdeg) return 0;
  float correction = (float)error_cdeg * (float)yaw_left_sign * kp;
  if (correction < -max_rpm) correction = -max_rpm;
  if (correction > max_rpm) correction = max_rpm;
  return (int16_t)(correction >= 0.0f ? correction + 0.5f : correction - 0.5f);
}

/* Updates one controller instance and returns signed differential RPM.
   The caller supplies signed shortest yaw error in centidegrees, selects
   the profile, and decides when to reset, stop, or advance a flow. */
int16_t HeadingControl_Update(HeadingControl_State *state,
                              const HeadingControl_Config *config,
                              uint32_t now, int32_t error_cdeg);

typedef enum
{
  HEADING_TURN_DRIVE,
  HEADING_TURN_SETTLE,
  HEADING_TURN_REACHED
} HeadingTurnStatus;

/* Call once per motion tick after computing the shortest error. A settle
   period of zero means immediate arrival. The caller owns the deadline. */
HeadingTurnStatus HeadingControl_CheckTurn(uint32_t *stable_since, uint32_t now,
    int32_t absolute_error_cdeg, int32_t enter_tolerance_cdeg,
    int32_t release_tolerance_cdeg, uint32_t settle_ms);

/* Keep correcting yaw in place when forward travel must be gated. */
void HeadingControl_LimitStraight(HeadingControl_State *state,
    int32_t absolute_error_cdeg, int32_t correction_deadband_cdeg,
    int32_t forward_stop_error_cdeg, bool stop_on_boundary,
    int16_t *forward_rpm, int16_t *turn_rpm);

#endif
