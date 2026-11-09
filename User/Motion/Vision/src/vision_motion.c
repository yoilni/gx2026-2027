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
