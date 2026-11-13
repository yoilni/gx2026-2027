#ifndef VISION_MOTION_H
#define VISION_MOTION_H

#include <stdbool.h>
#include <stdint.h>
#include "heading_control.h"

typedef enum
{
  VISION_MOTION_GENERAL,
  VISION_MOTION_SEARCH,
  VISION_MOTION_CENTER,
  VISION_MOTION_FINAL,
  VISION_MOTION_SAFE_ZONE
} VisionMotion_Profile;

typedef struct
{
  uint32_t update_tick;
  uint32_t last_sequence;
  float integral;
  float last_error;
  float output;
  bool initialized;
} VisionMotion_Axis;

typedef struct
{
  uint32_t context;
  uint32_t last_sequence;
  uint32_t last_frame_tick;
  int16_t last_y_error;
  uint8_t last_id;
  uint8_t pending_frames;
  int8_t direction;
  int8_t pending_direction;
  bool frame_known;
} VisionMotion_ReverseGate;

typedef struct
{
  uint32_t context;
  uint32_t phase_tick;
  uint32_t resume_after_sequence;
  HeadingControl_State heading_pid;
  int32_t heading_cdeg;
  uint8_t target_id;
  bool near_mode;
  bool aligned;
  bool backoff_used;
  bool heading_latched;
  uint8_t retreat_phase;
  uint8_t last_drive;
} VisionMotion_ClawApproach;

typedef enum
{
  VISION_CLAW_HOLD,
  VISION_CLAW_FAR_TRACK,
  VISION_CLAW_ALIGN,
  VISION_CLAW_STRAIGHT,
  VISION_CLAW_BACKOFF
} VisionMotion_ClawDrive;

typedef struct
{
  VisionMotion_Axis x;
  VisionMotion_Axis y;
  VisionMotion_ReverseGate reverse;
  VisionMotion_ClawApproach claw;
} VisionMotion_State;

typedef struct
{
  uint32_t context;    /* Opaque to the controller: change with tracking/reference. */
  uint32_t sequence;
  uint32_t age_ms;
  uint32_t timeout_ms;
  uint8_t target_id;
  bool area_near;
  bool area_open;
  bool claw_closed;
  bool valid;
} VisionMotion_Frame;

typedef struct
{
  int16_t rpm;
  int16_t previous_y;
  uint8_t confirmation_frames;
  bool y_jump;
  bool confirmation_changed;
  bool accepted;
} VisionMotion_GuardResult;

void VisionMotion_Reset(VisionMotion_State *state);
void VisionMotion_ResetAxis(VisionMotion_Axis *axis);
bool VisionMotion_IsObjectProfile(VisionMotion_Profile profile);
VisionMotion_ClawDrive VisionMotion_ClawApproachStep(VisionMotion_ClawApproach *approach,
    VisionMotion_Profile profile, const VisionMotion_Frame *frame, uint32_t now,
    int16_t x_error, int16_t y_error, bool wheels_stopped);
bool VisionMotion_ClawRetreatActive(const VisionMotion_ClawApproach *approach);
void VisionMotion_ClawResetHeading(VisionMotion_ClawApproach *approach);
int16_t VisionMotion_ClawHoldHeading(VisionMotion_ClawApproach *approach, uint32_t now,
    int32_t yaw_cdeg, uint32_t initial_period_ms);
int16_t VisionMotion_CalculateTurn(VisionMotion_Axis *axis, VisionMotion_Profile profile,
    uint32_t now, uint32_t sequence, int16_t error_px, uint32_t initial_period_ms);
int16_t VisionMotion_CalculateForward(VisionMotion_Axis *axis, VisionMotion_Profile profile,
    uint32_t now, uint32_t sequence, int16_t error_px, uint32_t initial_period_ms);
VisionMotion_GuardResult VisionMotion_GuardForward(VisionMotion_ReverseGate *gate,
    const VisionMotion_Frame *frame, uint32_t now, int16_t error_px, int16_t requested_rpm);
float VisionMotion_AgeScale(uint32_t age_ms, uint32_t fade_start_ms, uint32_t timeout_ms);
int16_t VisionMotion_LimitSafeZoneForward(int16_t rpm, int32_t absolute_y_error, bool aligned);

#endif
