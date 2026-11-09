#include "arrange_flow.h"
#include "chassis_actions.h"
#include "heading_control.h"
#include "robot_config.h"
#include "recovery_flow.h"
#include <stddef.h>

ArrangeFlow_StepResult ArrangeFlow_Step(ArrangeFlow_State *flow, const ArrangeFlow_Io *io,
    ArrangeFlow_Phase phase, ArrangeFlow_Profile profile, uint32_t now, uint32_t started_at,
    int32_t yaw_now)
{
  ArrangeFlow_StepResult result = { .next = phase };
  const uint32_t elapsed = now - started_at;
  const int32_t side = (profile.turn_left ? 1L : -1L) * (int32_t)ROBOT_YAW_LEFT_SIGN;
  switch (phase)
  {
    case ARRANGE_OFFSET:
    {
      const FlowLeg leg = { .kind = FLOW_DRIVE_OPEN, .duration_ms = ROBOT_S4_FINE32_FORWARD_MS,
        .rpm = ROBOT_S4_FINE32_FORWARD_RPM };
      if (FlowMotion_RunLeg(&io->motion, leg, now, started_at)) result.next = ARRANGE_DONE;
      else { result.fixed_output = true; result.forward_rpm = leg.rpm; }
      break;
    }
    case ARRANGE_TRACK:
      if (elapsed < (profile.right_pass ? ROBOT_S4_PUSH_TRIAL_TRACK_MS : ROBOT_S4_FINE42_TRACK_MS))
        (void)io->track(now, NULL);
      else if (profile.first_green)
      {
        flow->forward_yaw = HeadingControl_WrapYaw(yaw_now);
        result.next = ARRANGE_PUSH;
        result.reset_controller = true;
      }
      else
      {
        flow->turn_yaw = HeadingControl_WrapYaw(yaw_now + side *
            (profile.casualty ? ROBOT_S4_FINE42_CASUALTY_TURN_CDEG : ROBOT_S4_FINE42_TURN_CDEG));
        result.next = ARRANGE_TURN;
        result.reset_controller = result.reset_stable = true;
      }
      break;
    case ARRANGE_TURN:
      if (io->motion.turn(now, flow->turn_yaw))
      {
        flow->forward_yaw = HeadingControl_WrapYaw(yaw_now);
        result.next = ARRANGE_PUSH;
        result.reset_controller = true;
      }
      break;
    case ARRANGE_PUSH:
    {
      const FlowLeg leg = { .kind = FLOW_DRIVE_OPEN, .rpm = ROBOT_S4_FINE42_FORWARD_RPM,
        .duration_ms = profile.first_green ? ROBOT_S4_INITIAL_GREEN02_FORWARD_MS
            : (profile.side_push ? ROBOT_S4_FINE42_FORWARD_MS : ROBOT_S4_PUSH_TRIAL_FORWARD_MS) };
      if (!FlowMotion_RunLeg(&io->motion, leg, now, started_at))
      {
        result.fixed_output = true;
        result.forward_rpm = leg.rpm;
        break;
      }
      if (profile.first_green)
      {
        io->motion.stop();
        flow->spin_previous_yaw = HeadingControl_WrapYaw(yaw_now);
        flow->spin_progress = 0;
        flow->turn_yaw = HeadingControl_WrapYaw(yaw_now + (int32_t)ROBOT_YAW_LEFT_SIGN * 17000L);
      }
      else
      {
        flow->turn_yaw = HeadingControl_WrapYaw(yaw_now - side * ROBOT_S4_FINE42_COUNTER_TURN_CDEG);
        result.reset_stable = true;
      }
      result.next = ARRANGE_COUNTER;
      result.reset_controller = true;
      break;
    }
    case ARRANGE_COUNTER:
      if (profile.first_green)
      {
        /* Directed spin: never use the shortest-path shortcut or soft 2 s turn. */
        RecoveryFlow_ObserveSpin(&flow->spin_previous_yaw, &flow->spin_progress,
            yaw_now, ROBOT_YAW_LEFT_SIGN);
        if (flow->spin_progress >= ROBOT_S4_INITIAL_GREEN02_SPIN_CDEG)
        {
          result.next = ARRANGE_DONE;
          result.reset_controller = true;
          break;
        }
        if (elapsed >= ROBOT_S4_INITIAL_GREEN02_SPIN_TIMEOUT_MS)
        {
          io->motion.stop();
          result.next = ARRANGE_TIMEOUT;
          break;
        }
        int32_t remaining = ROBOT_S4_INITIAL_GREEN02_SPIN_CDEG - flow->spin_progress;
        if (remaining > 17000L) remaining = 17000L;
        flow->turn_yaw = HeadingControl_WrapYaw(yaw_now + (int32_t)ROBOT_YAW_LEFT_SIGN * remaining);
        result.turn_rpm = io->yaw_pid(now, flow->turn_yaw,
            ROBOT_S4_INITIAL_GREEN02_SPIN_MIN_RPM, ROBOT_S4_INITIAL_GREEN02_SPIN_MAX_RPM);
        result.fixed_output = true;
        io->motion.wheels(result.turn_rpm, (int16_t)-result.turn_rpm);
      }
      else if (io->motion.turn(now, flow->turn_yaw))
      {
        result.reset_controller = true;
        result.next = profile.casualty ? ARRANGE_DONE : ARRANGE_FINISH_LEFT;
        if (!profile.casualty)
        {
          flow->turn_yaw = HeadingControl_WrapYaw(yaw_now +
              (int32_t)ROBOT_YAW_LEFT_SIGN * ROBOT_S4_FINE42_FINISH_LEFT_CDEG);
          result.reset_stable = true;
        }
      }
      break;
    case ARRANGE_FINISH_LEFT:
      if (io->motion.turn(now, flow->turn_yaw))
      {
        result.next = ARRANGE_DONE;
        result.reset_controller = true;
      }
      break;
    case ARRANGE_LEGACY_END:
      result.next = ARRANGE_DONE;
      break;
    default: break;
  }
  return result;
}

void ArrangeFlow_CornerRpm(uint32_t elapsed,
                                                uint32_t duration,
                                                bool push_right,
                                                int16_t *left_rpm,
                                                int16_t *right_rpm)
{
  float scale = ChassisMotion_RampScale(elapsed, duration, ROBOT_S4_PUSH_RAMP_MS);
  float base_rpm = (float)ROBOT_S4_CORNER_PUSH_RPM;
  if (!push_right)
  {
    float blend = elapsed <= ROBOT_S4_LEFT_CORNER_INITIAL_MS ? 0.0f :
        ChassisMotion_SmoothStep((float)(elapsed - ROBOT_S4_LEFT_CORNER_INITIAL_MS) /
                                (float)ROBOT_S4_LEFT_SPEED_BLEND_MS);
    base_rpm = (float)ROBOT_S4_LEFT_CORNER_INITIAL_RPM +
        ((float)ROBOT_S4_CORNER_PUSH_RPM - (float)ROBOT_S4_LEFT_CORNER_INITIAL_RPM) * blend;
  }
  int16_t forward_rpm = (int16_t)(base_rpm * scale + 0.5f);
  float progress = duration == 0U ? 0.0f : (float)elapsed / (float)duration;
  float middle_blend = ChassisMotion_SmoothStep(
      (progress - ROBOT_S4_CORNER_DIFF_MIDDLE_BEGIN) /
      (ROBOT_S4_CORNER_DIFF_MIDDLE_END -
       ROBOT_S4_CORNER_DIFF_MIDDLE_BEGIN));
  float late_blend = ChassisMotion_SmoothStep(
      (progress - ROBOT_S4_CORNER_DIFF_LATE_BEGIN) /
      (ROBOT_S4_CORNER_DIFF_LATE_END -
       ROBOT_S4_CORNER_DIFF_LATE_BEGIN));
  float early_diff_rpm = push_right ? (float)ROBOT_S4_CORNER_DIFF_EARLY_RPM
                                   : (float)ROBOT_S4_LEFT_DIFF_EARLY_RPM;
  float middle_diff_rpm = push_right ? (float)ROBOT_S4_CORNER_DIFF_MIDDLE_RPM
                                    : (float)ROBOT_S4_LEFT_DIFF_MIDDLE_RPM;
  float late_diff_rpm = push_right ? (float)ROBOT_S4_CORNER_DIFF_LATE_RPM
                                  : (float)ROBOT_S4_LEFT_DIFF_LATE_RPM;
  float differential = early_diff_rpm +
      (middle_diff_rpm - early_diff_rpm) * middle_blend +
      (late_diff_rpm - middle_diff_rpm) * late_blend;
  int16_t wheel_diff_rpm = (int16_t)(differential * scale + 0.5f);
  int16_t faster_rpm =
      (int16_t)(forward_rpm +
                ((wheel_diff_rpm + 1) / 2));
  int16_t slower_rpm =
      (int16_t)(forward_rpm -
                (wheel_diff_rpm / 2));
  *left_rpm = push_right ? faster_rpm : slower_rpm;
  *right_rpm = push_right ? slower_rpm : faster_rpm;
}

static uint32_t scale_corner_duration(uint32_t duration, bool push_right)
{
  float reference_rpm = (float)ROBOT_S4_CORNER_REFERENCE_RPM;
  float cruise_rpm = (float)ROBOT_S4_CORNER_PUSH_RPM;
  float slow_window_ms = (float)ROBOT_S4_LEFT_CORNER_INITIAL_MS +
      (float)ROBOT_S4_LEFT_SPEED_BLEND_MS * 0.5f -
      (float)ROBOT_S4_PUSH_RAMP_MS * 0.5f;
  float area = reference_rpm *
      ((float)duration - (float)ROBOT_S4_PUSH_RAMP_MS);

  if (cruise_rpm <= 0.0f) return 0U;
  if (!push_right)
  {
    area -= (reference_rpm -
             (float)ROBOT_S4_LEFT_CORNER_REFERENCE_INITIAL_RPM) * slow_window_ms;
  }
  float distance_scale = (float)ROBOT_S4_PUSH_DISTANCE_PERCENT / 100.0f;
  if (push_right)
  {
    distance_scale *= (float)ROBOT_S4_RIGHT_PUSH_DISTANCE_PERCENT / 100.0f;
  }
  area *= distance_scale * (float)ROBOT_S4_CORNER_TRAVEL_PERCENT / 100.0f;
  if (!push_right)
  {
    area += (cruise_rpm - (float)ROBOT_S4_LEFT_CORNER_INITIAL_RPM) * slow_window_ms;
  }
  return (uint32_t)(area / cruise_rpm + (float)ROBOT_S4_PUSH_RAMP_MS + 0.5f);
}

uint32_t ArrangeFlow_CornerPushMs(bool push_right)
{
  return scale_corner_duration(
      push_right ? ROBOT_S4_RIGHT_CORNER_FOLLOW_MS
                 : ROBOT_S4_LEFT_CORNER_FOLLOW_MS,
      push_right);
}

uint32_t ArrangeFlow_CornerReverseMs(bool push_right)
{
  float reverse_rpm = (float)(push_right ? ROBOT_S4_RIGHT_REVERSE_RPM
                                       : ROBOT_S4_LEFT_REVERSE_RPM);
  if (reverse_rpm <= 0.0f)
  {
    return 0U;
  }
  /* Match the forward travel at the configured reverse cruise speed. */
  float distance_scale = push_right ? 1.0f :
      (float)ROBOT_S4_LEFT_REVERSE_DISTANCE_PERCENT / 100.0f;
  return (uint32_t)((float)ArrangeFlow_CornerPushMs(push_right) *
                   (float)ROBOT_S4_CORNER_PUSH_RPM / reverse_rpm * distance_scale + 0.5f);
}

void ArrangeFlow_CornerReverseRpm(uint32_t elapsed, bool push_right, int16_t *left, int16_t *right)
{
  const uint32_t duration = ArrangeFlow_CornerReverseMs(push_right);
  const uint32_t push_duration = ArrangeFlow_CornerPushMs(push_right);
  *left = *right = 0;
  if (duration == 0U || elapsed >= duration) return;
  const uint32_t replay_elapsed = (uint32_t)(
      ((uint64_t)(duration - elapsed) * push_duration + duration / 2U) / duration);
  float speed_scale = (float)push_duration / (float)duration;
  if (!push_right) speed_scale *= (float)ROBOT_S4_LEFT_REVERSE_DISTANCE_PERCENT / 100.0f;
  ArrangeFlow_CornerRpm(replay_elapsed, push_duration, push_right, left, right);
  *left = (int16_t)-(int16_t)((float)*left * speed_scale + 0.5f);
  *right = (int16_t)-(int16_t)((float)*right * speed_scale + 0.5f);
}

ScatterFlow_Phase ArrangeFlow_StepScatter(const FlowMotion *io, ScatterFlow_Phase phase,
    uint32_t now, uint32_t started_at, int32_t yaw_now, int32_t *target_yaw)
{
  if (phase == SCATTER_DONE) return phase;
  FlowLeg leg = { .kind = FLOW_TURN, .yaw_cdeg = *target_yaw };
  if (phase == SCATTER_FORWARD)
  {
    leg.kind = FLOW_DRIVE_OPEN;
    leg.duration_ms = ROBOT_S5_E5_FORWARD_MS;
    leg.rpm = ROBOT_S4_FINE42_FORWARD_RPM;
  }
  if (!FlowMotion_RunLeg(io, leg, now, started_at)) return phase;
  if (phase == SCATTER_FORWARD)
    *target_yaw = HeadingControl_WrapYaw(yaw_now - (int32_t)ROBOT_YAW_LEFT_SIGN * ROBOT_S5_E5_COUNTER_TURN_CDEG);
  else if (phase == SCATTER_COUNTER)
    *target_yaw = HeadingControl_WrapYaw(yaw_now + (int32_t)ROBOT_YAW_LEFT_SIGN * ROBOT_S4_FINE42_FINISH_LEFT_CDEG);
  return (ScatterFlow_Phase)(phase + 1);
}
