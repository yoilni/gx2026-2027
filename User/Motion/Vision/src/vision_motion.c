#include "vision_motion.h"
#include "robot_config.h"

static float clamp_float(float value, float minimum, float maximum)
{
  if (value < minimum) return minimum;
  if (value > maximum) return maximum;
  return value;
}

static int16_t round_rpm(float rpm)
{
  return (int16_t)(rpm >= 0.0f ? rpm + 0.5f : rpm - 0.5f);
}

static float enforce_nonzero_floor(float rpm, float minimum)
{
  float magnitude = rpm < 0.0f ? -rpm : rpm;
  if (magnitude > 0.0f && magnitude < minimum) return rpm < 0.0f ? -minimum : minimum;
  return rpm;
}

void VisionMotion_ResetAxis(VisionMotion_Axis *axis)
{
  axis->integral = 0.0f;
  axis->output = 0.0f;
  axis->initialized = false;
  axis->last_sequence = UINT32_MAX;
}

void VisionMotion_Reset(VisionMotion_State *state)
{
  *state = (VisionMotion_State){0};
  VisionMotion_ResetAxis(&state->x);
  VisionMotion_ResetAxis(&state->y);
  state->reverse.last_sequence = UINT32_MAX;
}

static void update_axis(VisionMotion_Axis *axis, uint32_t now, uint32_t sequence,
    float error, uint32_t initial_period_ms, float kp, float ki, float kd,
    float integral_limit, float output_limit)
{
  float dt;
  float derivative;
  if (!axis->initialized)
  {
    dt = (float)initial_period_ms / 1000.0f;
    derivative = 0.0f;
    axis->initialized = true;
  }
  else
  {
    dt = clamp_float((float)(now - axis->update_tick) / 1000.0f, 0.01f, 0.20f);
    derivative = (error - axis->last_error) / dt;
  }
  axis->integral = clamp_float(axis->integral + error * dt, -integral_limit, integral_limit);
  axis->output = clamp_float(kp * error + ki * axis->integral + kd * derivative,
                             -output_limit, output_limit);
  axis->last_error = error;
  axis->update_tick = now;
  axis->last_sequence = sequence;
}

bool VisionMotion_IsObjectProfile(VisionMotion_Profile profile)
{
  return profile == VISION_MOTION_SEARCH || profile == VISION_MOTION_CENTER ||
         profile == VISION_MOTION_FINAL;
}

enum { CLAW_RETREAT_IDLE, CLAW_RETREAT_DRIVE, CLAW_RETREAT_BRAKE, CLAW_RETREAT_FRESH };

bool VisionMotion_ClawRetreatActive(const VisionMotion_ClawApproach *approach)
{
  return approach->retreat_phase != CLAW_RETREAT_IDLE;
}

VisionMotion_ClawDrive VisionMotion_ClawApproachStep(VisionMotion_ClawApproach *approach,
    VisionMotion_Profile profile, const VisionMotion_Frame *frame, uint32_t now,
    int16_t x_error, int16_t y_error, bool wheels_stopped)
{
  const bool valid = frame->valid && frame->age_ms < frame->timeout_ms;
  if (approach->context != frame->context ||
      (valid && approach->target_id != frame->target_id))
  {
    *approach = (VisionMotion_ClawApproach){.context = frame->context, .target_id = frame->target_id};
  }
  if (!valid)
  {
    approach->aligned = false;
    approach->last_drive = VISION_CLAW_HOLD;
    if (approach->retreat_phase == CLAW_RETREAT_DRIVE)
      approach->retreat_phase = CLAW_RETREAT_BRAKE;
    return VISION_CLAW_HOLD;
  }

  if (approach->retreat_phase == CLAW_RETREAT_DRIVE)
  {
    if ((uint32_t)(now - approach->phase_tick) < ROBOT_VISION_CLAW_BACKOFF_MS)
      return VISION_CLAW_BACKOFF;
    approach->retreat_phase = CLAW_RETREAT_BRAKE;
    approach->last_drive = VISION_CLAW_HOLD;
    return VISION_CLAW_HOLD;
  }
  if (approach->retreat_phase == CLAW_RETREAT_BRAKE)
  {
    if (wheels_stopped)
    {
      approach->phase_tick = now;
      approach->resume_after_sequence = frame->sequence;
      approach->retreat_phase = CLAW_RETREAT_FRESH;
    }
    return VISION_CLAW_HOLD;
  }
  if (approach->retreat_phase == CLAW_RETREAT_FRESH)
  {
    if (!wheels_stopped || frame->sequence == approach->resume_after_sequence ||
        (int32_t)((now - frame->age_ms) - approach->phase_tick) < 0)
      return VISION_CLAW_HOLD;
    approach->retreat_phase = CLAW_RETREAT_IDLE;
  }

  const int32_t x = x_error < 0 ? -(int32_t)x_error : x_error;
  const int32_t forward_error = (int32_t)y_error * ROBOT_VISION_Y_FORWARD_SIGN;
  const int32_t align_limit = profile == VISION_MOTION_SEARCH ? ROBOT_VISION_CLAW_ALIGN_03_PX :
      profile == VISION_MOTION_CENTER ? ROBOT_VISION_CLAW_ALIGN_04_PX : ROBOT_VISION_CLAW_ALIGN_24_PX;
  /* Once near, stay in the guarded approach for this target and phase.
     The short backoff itself must not reactivate the XY steering mode. */
  if (!approach->near_mode && !frame->area_near && !frame->area_open && forward_error > align_limit)
  {
    approach->last_drive = VISION_CLAW_FAR_TRACK;
    return VISION_CLAW_FAR_TRACK;
  }
  approach->near_mode = true;
  const int32_t near_limit = profile == VISION_MOTION_SEARCH ? ROBOT_VISION_CLAW_NEAR_03_PX :
      profile == VISION_MOTION_CENTER ? ROBOT_VISION_CLAW_NEAR_04_PX : ROBOT_VISION_CLAW_NEAR_24_PX;
  const bool closed_at_target = frame->claw_closed && frame->area_open;
  const bool too_close = frame->area_open || forward_error <= near_limit;
  const bool strict_center = frame->claw_closed && (frame->area_near || frame->area_open);
  const int32_t x_limit = approach->aligned && !strict_center ? ROBOT_VISION_CLAW_REALIGN_X_PX :
      ROBOT_VISION_X_TOLERANCE_PX;
  approach->aligned = x <= x_limit;
  const VisionMotion_ClawDrive drive = approach->aligned
      ? (closed_at_target ? VISION_CLAW_HOLD : VISION_CLAW_STRAIGHT)
      : (!approach->backoff_used && too_close ? VISION_CLAW_BACKOFF : VISION_CLAW_ALIGN);

  /* Do not mix translation and yaw while the chassis is still braking. */
  if ((drive != approach->last_drive && !wheels_stopped) ||
      (drive == VISION_CLAW_BACKOFF && frame->age_ms > ROBOT_VISION_COORD_DECEL_START_MS))
  {
    approach->last_drive = VISION_CLAW_HOLD;
    return VISION_CLAW_HOLD;
  }
  approach->last_drive = (uint8_t)drive;
  if (drive == VISION_CLAW_BACKOFF)
  {
    approach->backoff_used = true;
    approach->phase_tick = now;
    approach->retreat_phase = CLAW_RETREAT_DRIVE;
  }
  else if (drive == VISION_CLAW_STRAIGHT && !frame->area_near &&
           forward_error > near_limit + ROBOT_VISION_CLAW_REARM_MARGIN_PX)
  {
    approach->backoff_used = false;
  }
  return drive;
}

void VisionMotion_ClawResetHeading(VisionMotion_ClawApproach *approach)
{
  approach->heading_latched = false;
  approach->heading_pid = (HeadingControl_State){0};
}

int16_t VisionMotion_ClawHoldHeading(VisionMotion_ClawApproach *approach, uint32_t now,
    int32_t yaw_cdeg, uint32_t initial_period_ms)
{
  if (!approach->heading_latched)
  {
    approach->heading_cdeg = yaw_cdeg;
    approach->heading_latched = true;
  }
  const int32_t error = HeadingControl_YawError(approach->heading_cdeg, yaw_cdeg);
  if (error >= -ROBOT_VISION_CLAW_YAW_DEADBAND_CDEG && error <= ROBOT_VISION_CLAW_YAW_DEADBAND_CDEG)
  {
    approach->heading_pid = (HeadingControl_State){0};
    return 0;
  }
  const HeadingControl_Config config = {
    .kp = ROBOT_VISION_CLAW_YAW_KP, .ki = ROBOT_VISION_CLAW_YAW_KI,
    .kd = ROBOT_VISION_CLAW_YAW_KD, .integral_limit = ROBOT_VISION_CLAW_YAW_I_LIMIT,
    .output_scale = 1.0f, .min_turn_rpm = 0.0f, .max_turn_rpm = ROBOT_VISION_CLAW_YAW_MAX_RPM,
    .initial_period_ms = initial_period_ms, .yaw_left_sign = ROBOT_YAW_LEFT_SIGN,
  };
  return HeadingControl_Update(&approach->heading_pid, &config, now, error);
}

int16_t VisionMotion_CalculateTurn(VisionMotion_Axis *axis, VisionMotion_Profile profile,
    uint32_t now, uint32_t sequence, int16_t error_px, uint32_t initial_period_ms)
{
  const bool final_track = profile == VISION_MOTION_FINAL;
  const float kp = final_track ? ROBOT_S4_FINAL_X_KP : ROBOT_VISION_X_KP;
  const float floor_rpm = final_track ? ROBOT_S4_FINAL_MIN_TURN_RPM : ROBOT_VISION_X_MIN_TURN_RPM;
  if (sequence != axis->last_sequence)
  {
    update_axis(axis, now, sequence, (float)error_px, initial_period_ms, kp,
        ROBOT_VISION_X_KI, ROBOT_VISION_X_KD, ROBOT_VISION_X_INTEGRAL_LIMIT,
        ROBOT_VISION_X_MAX_TURN_RPM);
  }
  return round_rpm(enforce_nonzero_floor(axis->output * (float)ROBOT_VISION_X_TURN_SIGN, floor_rpm));
}

static float object_forward_floor(VisionMotion_Profile profile, int32_t absolute_error,
    bool faster_forward, float min_forward_rpm)
{
  const bool final_track = profile == VISION_MOTION_FINAL;
  const bool search_track = profile == VISION_MOTION_SEARCH;
  float near_min_rpm = faster_forward
      ? (final_track ? ROBOT_VISION_24_NEAR_MIN_RPM : ROBOT_VISION_03_04_NEAR_MIN_RPM)
      : ROBOT_VISION_OBJECT_NEAR_MIN_RPM;
  float near_max_rpm = faster_forward
      ? (final_track ? ROBOT_VISION_24_NEAR_MAX_RPM : ROBOT_VISION_03_04_NEAR_MAX_RPM)
      : ROBOT_VISION_OBJECT_NEAR_MAX_RPM;
  if (faster_forward && search_track)
  {
    near_min_rpm = ROBOT_VISION_03_NEAR_MIN_RPM;
    near_max_rpm = ROBOT_VISION_03_NEAR_MAX_RPM;
  }
  if (absolute_error <= ROBOT_VISION_OBJECT_NEAR_END_PX)
  {
    float scale = clamp_float(
        (float)(absolute_error - ROBOT_VISION_Y_TOLERANCE_PX) /
        (float)(ROBOT_VISION_OBJECT_NEAR_END_PX - ROBOT_VISION_Y_TOLERANCE_PX), 0.0f, 1.0f);
    return near_min_rpm + scale * (near_max_rpm - near_min_rpm);
  }

  /* The search profile has its own rising floor for both middle and far bands. */
  float middle_min_rpm = faster_forward && !final_track
      ? (search_track ? ROBOT_VISION_03_MID_MIN_RPM : ROBOT_VISION_03_04_MID_MIN_RPM)
      : near_max_rpm;
  if (faster_forward && min_forward_rpm < middle_min_rpm) min_forward_rpm = middle_min_rpm;
  float scale = clamp_float(
      (float)(absolute_error - ROBOT_VISION_OBJECT_NEAR_END_PX) /
      (float)(ROBOT_VISION_OBJECT_FAR_BEGIN_PX - ROBOT_VISION_OBJECT_NEAR_END_PX), 0.0f, 1.0f);
  min_forward_rpm = middle_min_rpm + scale * (min_forward_rpm - middle_min_rpm);
  if (faster_forward && search_track && absolute_error >= ROBOT_VISION_OBJECT_FAR_BEGIN_PX)
  {
    float far_scale = clamp_float(
        (float)(absolute_error - ROBOT_VISION_OBJECT_FAR_BEGIN_PX) /
        (float)(ROBOT_VISION_03_FAR_FULL_SPEED_PX - ROBOT_VISION_OBJECT_FAR_BEGIN_PX), 0.0f, 1.0f);
    min_forward_rpm = ROBOT_VISION_03_MIN_FORWARD_RPM + far_scale *
        (ROBOT_VISION_03_MAX_FORWARD_RPM - ROBOT_VISION_03_MIN_FORWARD_RPM);
  }
  return min_forward_rpm;
}

int16_t VisionMotion_CalculateForward(VisionMotion_Axis *axis, VisionMotion_Profile profile,
    uint32_t now, uint32_t sequence, int16_t error_px, uint32_t initial_period_ms)
{
  const int32_t error = error_px;
  const int32_t absolute_error = error < 0 ? -error : error;
  const bool object_track = VisionMotion_IsObjectProfile(profile);
  const bool safe_zone = profile == VISION_MOTION_SAFE_ZONE;
  const float kp = profile == VISION_MOTION_FINAL ? ROBOT_S4_FINAL_Y_KP : ROBOT_VISION_Y_KP;
  float max_forward_rpm = ROBOT_VISION_Y_MAX_FORWARD_RPM;
  float min_forward_rpm = ROBOT_VISION_Y_MIN_FORWARD_RPM;
  switch (profile)
  {
    case VISION_MOTION_SEARCH:
      max_forward_rpm = ROBOT_VISION_03_MAX_FORWARD_RPM;
      min_forward_rpm = ROBOT_VISION_03_MIN_FORWARD_RPM;
      break;
    case VISION_MOTION_CENTER:
      max_forward_rpm = ROBOT_VISION_04_MAX_FORWARD_RPM;
      min_forward_rpm = ROBOT_VISION_04_MIN_FORWARD_RPM;
      break;
    case VISION_MOTION_FINAL:
      max_forward_rpm = ROBOT_S4_FINAL_FORWARD_MAX_RPM;
      min_forward_rpm = ROBOT_S4_FINAL_MIN_FORWARD_RPM;
      break;
    case VISION_MOTION_SAFE_ZONE:
      max_forward_rpm = ROBOT_S6_APPROACH_RPM;
      min_forward_rpm = ROBOT_S6_APPROACH_MIN_FORWARD_RPM;
      break;
    default: break;
  }
  if (object_track && absolute_error <= ROBOT_VISION_Y_TOLERANCE_PX)
  {
    VisionMotion_ResetAxis(axis);
    return 0;
  }
  if (!axis->initialized || sequence != axis->last_sequence)
  {
    update_axis(axis, now, sequence, (float)error, initial_period_ms, kp,
        ROBOT_VISION_Y_KI, ROBOT_VISION_Y_KD, ROBOT_VISION_Y_INTEGRAL_LIMIT, max_forward_rpm);
  }

  float rpm = axis->output * (float)ROBOT_VISION_Y_FORWARD_SIGN;
  if (safe_zone && rpm < 0.0f)
  {
    min_forward_rpm = ROBOT_S6_APPROACH_MIN_REVERSE_RPM;
    if (rpm < -(float)ROBOT_S6_APPROACH_MAX_REVERSE_RPM) rpm = -(float)ROBOT_S6_APPROACH_MAX_REVERSE_RPM;
  }
  if (object_track) min_forward_rpm = object_forward_floor(profile, absolute_error, rpm > 0.0f, min_forward_rpm);
  rpm = enforce_nonzero_floor(rpm, min_forward_rpm);
  if (object_track && rpm < -ROBOT_VISION_OBJECT_REVERSE_MAX_RPM) rpm = -ROBOT_VISION_OBJECT_REVERSE_MAX_RPM;
  return round_rpm(rpm);
}

static void reset_pending(VisionMotion_ReverseGate *gate)
{
  gate->pending_direction = 0;
  gate->pending_frames = 0U;
}

VisionMotion_GuardResult VisionMotion_GuardForward(VisionMotion_ReverseGate *gate,
    const VisionMotion_Frame *frame, uint32_t now, int16_t error_px, int16_t requested_rpm)
{
  VisionMotion_GuardResult result = {0};
  const int8_t direction = requested_rpm > 0 ? 1 : (requested_rpm < 0 ? -1 : 0);
  if (gate->context != frame->context)
  {
    gate->context = frame->context;
    gate->direction = 0;
    gate->frame_known = false;
    reset_pending(gate);
  }
  if (!frame->valid || frame->age_ms >= frame->timeout_ms)
  {
    reset_pending(gate);
    return result;
  }

  const uint32_t frame_tick = now - frame->age_ms;
  const bool new_frame = !gate->frame_known || frame->sequence != gate->last_sequence;
  if (new_frame)
  {
    const bool same_target = gate->last_id == frame->target_id;
    const int32_t jump = (int32_t)error_px - gate->last_y_error;
    result.y_jump = gate->frame_known && same_target &&
        (jump < 0 ? -jump : jump) >= ROBOT_VISION_Y_JUMP_LOG_PX;
    result.previous_y = gate->last_y_error;
    if (gate->frame_known && (!same_target || (uint32_t)(frame_tick - gate->last_frame_tick) > frame->timeout_ms))
      reset_pending(gate);
    gate->frame_known = true;
    gate->last_sequence = frame->sequence;
    gate->last_frame_tick = frame_tick;
    gate->last_y_error = error_px;
    gate->last_id = frame->target_id;
  }
  if (direction == 0 || direction == gate->direction)
  {
    reset_pending(gate);
    result.rpm = requested_rpm;
    return result;
  }
  /* First forward motion is immediate; backward or a reversal needs fresh frames. */
  if (gate->direction == 0 && direction > 0)
  {
    gate->direction = direction;
    result.rpm = requested_rpm;
    return result;
  }
  if (new_frame)
  {
    if (gate->pending_direction != direction)
    {
      gate->pending_direction = direction;
      gate->pending_frames = 0U;
    }
    if (gate->pending_frames < ROBOT_VISION_OBJECT_REVERSE_FRAMES) ++gate->pending_frames;
    result.confirmation_changed = true;
    result.confirmation_frames = gate->pending_frames;
  }
  if (gate->pending_frames >= ROBOT_VISION_OBJECT_REVERSE_FRAMES)
  {
    gate->direction = direction;
    reset_pending(gate);
    result.accepted = true;
    result.rpm = requested_rpm;
  }
  return result;
}

float VisionMotion_AgeScale(uint32_t age_ms, uint32_t fade_start_ms, uint32_t timeout_ms)
{
  if (age_ms >= timeout_ms) return 0.0f;
  if (age_ms <= fade_start_ms) return 1.0f;
  return (float)(timeout_ms - age_ms) / (float)(timeout_ms - fade_start_ms);
}

int16_t VisionMotion_LimitSafeZoneForward(int16_t rpm, int32_t absolute_y_error, bool aligned)
{
  int16_t forward_limit = aligned ? ROBOT_S6_APPROACH_RPM : ROBOT_S6_APPROACH_UNALIGNED_RPM;
  int16_t reverse_limit = aligned ? ROBOT_S6_APPROACH_MAX_REVERSE_RPM : ROBOT_S6_APPROACH_UNALIGNED_REVERSE_RPM;
  if (absolute_y_error <= ROBOT_S6_APPROACH_NEAR_Y_PX)
  {
    if (forward_limit > ROBOT_S6_APPROACH_NEAR_RPM) forward_limit = ROBOT_S6_APPROACH_NEAR_RPM;
    if (reverse_limit > ROBOT_S6_APPROACH_NEAR_RPM) reverse_limit = ROBOT_S6_APPROACH_NEAR_RPM;
  }
  if (rpm > forward_limit) return forward_limit;
  if (rpm < -reverse_limit) return (int16_t)-reverse_limit;
  return rpm;
}
