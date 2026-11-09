#ifndef CHASSIS_ACTIONS_H
#define CHASSIS_ACTIONS_H

#include "chassis_motion.h"
#include "heading_control.h"

typedef enum
{
  CHASSIS_ACTION_RUNNING,
  CHASSIS_ACTION_DONE,
  CHASSIS_ACTION_TIMEOUT
} ChassisActionStatus;

typedef struct
{
  ChassisActionStatus status;
  bool brake;
  ChassisWheelRpm wheels;
} ChassisActionStep;

typedef struct
{
  HeadingControl_Config controller;
  int32_t target_yaw_cdeg;
  int32_t enter_tolerance_cdeg;
  int32_t release_tolerance_cdeg;
  uint32_t settle_ms;
  uint32_t started_at;
  uint32_t timeout_ms; /* Zero leaves the deadline to the caller. */
  int16_t wheel_max_rpm;
} ChassisTurnRequest;

/* A non-blocking step. These APIs return commands; they never write the
   wheels. The caller applies one result, or discards it and stops on cancel. */
ChassisActionStep ChassisMotion_TurnToYawStep(HeadingControl_State *controller,
    uint32_t *stable_since, const ChassisTurnRequest *request,
    uint32_t now, int32_t current_yaw_cdeg);

/* For existing policies that select or taper the turn RPM before settling. */
ChassisActionStep ChassisMotion_ResolveTurnStep(HeadingControl_State *controller,
    uint32_t *stable_since, uint32_t now, int32_t absolute_error_cdeg,
    int32_t enter_tolerance_cdeg, int32_t release_tolerance_cdeg,
    uint32_t settle_ms, int16_t turn_rpm);

/* Pass a fresh tick taken at or after started_at. Signed RPM gives the
   direction. This equal-wheel action does not apply heading correction. */
ChassisActionStep ChassisMotion_TimedDriveStep(uint32_t started_at, uint32_t now,
    uint32_t duration_ms, int16_t forward_rpm);

float ChassisMotion_SmoothStep(float fraction);
float ChassisMotion_RampScale(uint32_t elapsed, uint32_t duration, uint32_t ramp_ms);

#endif
