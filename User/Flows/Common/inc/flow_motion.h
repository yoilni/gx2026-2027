#ifndef FLOW_MOTION_H
#define FLOW_MOTION_H

#include <stdbool.h>
#include <stdint.h>

/* The application supplies its existing single owner of the wheel targets.
   No route owns a task, a UART, or a mission state. */
typedef struct
{
  void (*stop)(void);
  void (*wheels)(int16_t left, int16_t right);
  bool (*turn)(uint32_t now, int32_t yaw_cdeg);
  void (*straight)(uint32_t now, int32_t yaw_cdeg, int16_t rpm, int16_t wheel_max_rpm);
} FlowMotion;

typedef enum { FLOW_TURN, FLOW_DRIVE_OPEN, FLOW_DRIVE_HEADING } FlowLegKind;
typedef struct
{
  FlowLegKind kind;
  int32_t yaw_cdeg;
  uint32_t duration_ms;
  int16_t rpm;
  int16_t wheel_max_rpm;
} FlowLeg;

/* Run one turn or drive leg. True means done, not permission to start the
   next mission state. Stop/reset policy belongs to the owning flow. */
bool FlowMotion_RunLeg(const FlowMotion *motion, FlowLeg leg, uint32_t now, uint32_t started_at);

#endif
