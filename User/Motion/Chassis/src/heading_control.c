#include "heading_control.h"

static float clamp_float(float value, float minimum, float maximum)
{
  if (value < minimum) return minimum;
  if (value > maximum) return maximum;
  return value;
}

int16_t HeadingControl_Update(HeadingControl_State *state,
                              const HeadingControl_Config *config,
                              uint32_t now, int32_t error_cdeg)
{
  const float error = (float)error_cdeg;
  float dt_seconds;
  float derivative;
  if (!state->initialized)
  {
    dt_seconds = (float)config->initial_period_ms / 1000.0f;
    derivative = 0.0f;
    state->initialized = true;
  }
  else
  {
    dt_seconds = (float)(now - state->update_tick) / 1000.0f;
    dt_seconds = clamp_float(dt_seconds, 0.01f, 0.20f);
    derivative = (error - state->last_error) / dt_seconds;
  }

  state->integral += error * dt_seconds;
  state->integral = clamp_float(state->integral,
      -config->integral_limit, config->integral_limit);
  float output = config->kp * error + config->ki * state->integral + config->kd * derivative;
  output *= config->output_scale;
  output = clamp_float(output, -config->max_turn_rpm, config->max_turn_rpm);

  state->last_error = error;
  state->update_tick = now;

  output *= -(float)config->yaw_left_sign;
  const float magnitude = output < 0.0f ? -output : output;
  if (magnitude > 0.0f && magnitude < config->min_turn_rpm)
  {
    output = output < 0.0f ? -config->min_turn_rpm : config->min_turn_rpm;
  }
  return (int16_t)(output >= 0.0f ? output + 0.5f : output - 0.5f);
}

HeadingTurnStatus HeadingControl_CheckTurn(uint32_t *stable_since, uint32_t now,
    int32_t absolute_error_cdeg, int32_t enter_tolerance_cdeg,
    int32_t release_tolerance_cdeg, uint32_t settle_ms)
{
  const int32_t tolerance = *stable_since == 0U ? enter_tolerance_cdeg : release_tolerance_cdeg;
  if (absolute_error_cdeg > tolerance)
  {
    *stable_since = 0U;
    return HEADING_TURN_DRIVE;
  }
  if (settle_ms == 0U) return HEADING_TURN_REACHED;
  if (*stable_since == 0U)
  {
    *stable_since = now;
  }
  else if ((uint32_t)(now - *stable_since) >= settle_ms)
  {
    return HEADING_TURN_REACHED;
  }
  return HEADING_TURN_SETTLE;
}

void HeadingControl_LimitStraight(HeadingControl_State *state,
    int32_t absolute_error_cdeg, int32_t correction_deadband_cdeg,
    int32_t forward_stop_error_cdeg, bool stop_on_boundary,
    int16_t *forward_rpm, int16_t *turn_rpm)
{
  if (absolute_error_cdeg <= correction_deadband_cdeg)
  {
    *turn_rpm = 0;
    state->integral = 0.0f;
  }
  if (absolute_error_cdeg > forward_stop_error_cdeg ||
      (stop_on_boundary && absolute_error_cdeg == forward_stop_error_cdeg))
  {
    *forward_rpm = 0;
  }
}
