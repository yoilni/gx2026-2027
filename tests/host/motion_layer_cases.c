#include "chassis_motion.h"
#include "chassis_actions.h"
#include "heading_control.h"
#include "M2006_Speed.h"
#include "robot_config.h"
#include "vision_motion.h"

#include <assert.h>
#include <stdio.h>

static struct
{
  uint8_t id;
  float rpm;
  int16_t current_limit;
} commands[2];
static unsigned command_count, stop_count, emergency_count;

void SpeedLoop_SetMotorTarget(uint8_t motor_id, float rpm, int16_t current_limit)
{
  assert(command_count < 2U);
  commands[command_count].id = motor_id;
  commands[command_count].rpm = rpm;
  commands[command_count++].current_limit = current_limit;
}

void brake(void) { ++stop_count; }
void SpeedLoop_EmergencyStop(void) { ++emergency_count; }

static void check_wheels(void)
{
  ChassisWheelRpm pair = ChassisMotion_MixRpm(100, 30, 100);
  assert(pair.left_rpm == 100 && pair.right_rpm == 53);
  pair = ChassisMotion_MixRpm(-100, 30, 100);
  assert(pair.left_rpm == -53 && pair.right_rpm == -100);
  pair = ChassisMotion_MixRpm(0, -80, 60);
  assert(pair.left_rpm == -60 && pair.right_rpm == 60);
  pair = ChassisMotion_MixRpm(100, 30, 0);
  assert(pair.left_rpm == 0 && pair.right_rpm == 0);
  pair = ChassisMotion_MixRpm(0, 0, -1);
  assert(pair.left_rpm == 0 && pair.right_rpm == 0);
  assert(command_count == 0U); /* Mixing does not command the wheels. */

  ChassisMotion_SetWheelRpm(90, -70, 1800);
  assert(command_count == 2U);
  assert(commands[0].id == ROBOT_LEFT_WHEEL_MOTOR_ID);
  assert(commands[1].id == ROBOT_RIGHT_WHEEL_MOTOR_ID);
  assert(commands[0].rpm == 90.0f * ROBOT_LEFT_WHEEL_FORWARD_SIGN);
  assert(commands[1].rpm == -70.0f * ROBOT_RIGHT_WHEEL_FORWARD_SIGN);
  assert(commands[0].current_limit == 1800 && commands[1].current_limit == 1800);

  ChassisMotion_Stop();
  assert(stop_count == 1U && emergency_count == 0U);
  ChassisMotion_EmergencyStop();
  assert(stop_count == 1U && emergency_count == 1U);
}

static void check_heading(void)
{
  HeadingControl_State state = {0};
  HeadingControl_Config config = {
    .kp = 0.02f, .integral_limit = 25.0f, .output_scale = 1.0f,
    .min_turn_rpm = 10.0f, .max_turn_rpm = 30.0f,
    .initial_period_ms = 20U, .yaw_left_sign = 1,
  };
  const uint32_t start = UINT32_MAX - 9U;
  assert(HeadingControl_Update(&state, &config, start, 2000) == -30);
  assert(state.initialized && state.integral == 25.0f && state.update_tick == start);
  assert(HeadingControl_Update(&state, &config, 10U, -250) == 10);
  assert(state.integral == 20.0f && state.last_error == -250.0f && state.update_tick == 10U);
  assert(HeadingControl_Update(&state, &config, 20U, 0) == 0);
  config.output_scale = 0.5f;
  config.yaw_left_sign = -1;
  assert(HeadingControl_Update(&state, &config, 40U, 1400) == 14);

  state = (HeadingControl_State){0};
  config.kp = 0.0f;
  config.kd = 0.01f;
  config.output_scale = 1.0f;
  config.min_turn_rpm = 0.0f;
  config.max_turn_rpm = 200.0f;
  config.yaw_left_sign = 1;
  assert(HeadingControl_Update(&state, &config, 100U, 0) == 0);
  assert(HeadingControl_Update(&state, &config, 101U, 100) == -100); /* Clamp dt to 10 ms. */
  assert(HeadingControl_Update(&state, &config, 1101U, 200) == -5);  /* Clamp dt to 200 ms. */
}

static void check_motion_gates(void)
{
  uint32_t stable_since = 0U;
  const uint32_t start = UINT32_MAX - 99U;
  assert(HeadingControl_CheckTurn(&stable_since, start, 401, 400, 600, 200U) == HEADING_TURN_DRIVE);
  assert(HeadingControl_CheckTurn(&stable_since, start, 400, 400, 600, 200U) == HEADING_TURN_SETTLE);
  assert(HeadingControl_CheckTurn(&stable_since, 99U, 600, 400, 600, 200U) == HEADING_TURN_SETTLE);
  assert(HeadingControl_CheckTurn(&stable_since, 100U, 600, 400, 600, 200U) == HEADING_TURN_REACHED);
  assert(HeadingControl_CheckTurn(&stable_since, 101U, 601, 400, 600, 200U) == HEADING_TURN_DRIVE);
  assert(stable_since == 0U);
  assert(HeadingControl_CheckTurn(&stable_since, 102U, 100, 400, 400, 0U) == HEADING_TURN_REACHED);

  HeadingControl_State state = { .integral = 10.0f };
  int16_t forward = 70, turn = -20;
  HeadingControl_LimitStraight(&state, 9000, 300, 9000, true, &forward, &turn);
  assert(forward == 0 && turn == -20 && state.integral == 10.0f);
  forward = -100;
  HeadingControl_LimitStraight(&state, 1000, 300, 1000, false, &forward, &turn);
  assert(forward == -100 && turn == -20);
  HeadingControl_LimitStraight(&state, 300, 300, 1000, false, &forward, &turn);
  assert(forward == -100 && turn == 0 && state.integral == 0.0f);
}

static void check_action_steps(void)
{
  const uint32_t start = UINT32_MAX - 99U;
  ChassisActionStep step = ChassisMotion_TimedDriveStep(start, 99U, 200U, -70);
  assert(step.status == CHASSIS_ACTION_RUNNING && !step.brake);
  assert(step.wheels.left_rpm == -70 && step.wheels.right_rpm == -70);
  step = ChassisMotion_TimedDriveStep(start, 100U, 200U, -70);
  assert(step.status == CHASSIS_ACTION_DONE && step.brake && step.wheels.left_rpm == 0);

  HeadingControl_State controller = {0};
  uint32_t stable_since = 0U;
  const ChassisTurnRequest request = {
    .controller = { .kp = 0.025f, .integral_limit = 100.0f, .output_scale = 1.0f,
      .min_turn_rpm = 10.0f, .max_turn_rpm = 60.0f, .initial_period_ms = 20U, .yaw_left_sign = 1 },
    .target_yaw_cdeg = 100, .enter_tolerance_cdeg = 400, .release_tolerance_cdeg = 600,
    .settle_ms = 200U, .started_at = start, .timeout_ms = 400U, .wheel_max_rpm = 30,
  };
  step = ChassisMotion_TurnToYawStep(&controller, &stable_since, &request, start, 34600);
  assert(step.status == CHASSIS_ACTION_RUNNING && !step.brake);
  assert(step.wheels.left_rpm == -30 && step.wheels.right_rpm == 30); /* Across 0°. */
  step = ChassisMotion_TurnToYawStep(&controller, &stable_since, &request, 10U, 35700);
  assert(step.status == CHASSIS_ACTION_RUNNING && step.brake);
  step = ChassisMotion_TurnToYawStep(&controller, &stable_since, &request, 210U, 35500);
  assert(step.status == CHASSIS_ACTION_DONE && step.brake);
  step = ChassisMotion_TurnToYawStep(&controller, &stable_since, &request, 300U, 35500);
  assert(step.status == CHASSIS_ACTION_TIMEOUT && step.brake && !controller.initialized);
  assert(ChassisMotion_RampScale(0U, 100U, 150U) == 0.0f);
  assert(ChassisMotion_RampScale(50U, 100U, 150U) == 1.0f);
  assert(ChassisMotion_RampScale(75U, 100U, 150U) == 0.5f);
}

static void check_vision(void)
{
  VisionMotion_State vision;
  VisionMotion_Reset(&vision);
  assert(VisionMotion_CalculateForward(&vision.y, VISION_MOTION_SEARCH, 100U, 1U,
      448 * ROBOT_VISION_Y_FORWARD_SIGN, 20U) == 200);
  assert(VisionMotion_CalculateForward(&vision.y, VISION_MOTION_SEARCH, 120U, 2U, 12, 20U) == 0);
  assert(!vision.y.initialized);
  const int16_t turn = VisionMotion_CalculateTurn(&vision.x, VISION_MOTION_FINAL, 100U, 1U, 80, 20U);
  assert(turn != 0);
  assert(VisionMotion_CalculateTurn(&vision.x, VISION_MOTION_FINAL, 120U, 1U, -80, 20U) == turn);
  assert(vision.x.update_tick == 100U); /* Same frame does not run PID again. */
  assert(VisionMotion_AgeScale(250U, 250U, 500U) == 1.0f);
  assert(VisionMotion_AgeScale(375U, 250U, 500U) == 0.5f);
  assert(VisionMotion_AgeScale(500U, 250U, 500U) == 0.0f);
  assert(VisionMotion_LimitSafeZoneForward(180, 250, true) == 60);
  assert(VisionMotion_LimitSafeZoneForward(180, 251, false) == 120);

  VisionMotion_Frame frame = { .context = 1U, .sequence = 1U, .timeout_ms = 500U,
    .target_id = 5U, .valid = true };
  assert(VisionMotion_GuardForward(&vision.reverse, &frame, 100U, 100, 50).rpm == 50);
  ++frame.sequence;
  VisionMotion_GuardResult guard = VisionMotion_GuardForward(&vision.reverse, &frame, 120U, -100, -30);
  assert(guard.rpm == 0 && guard.y_jump && guard.confirmation_frames == 1U);
  guard = VisionMotion_GuardForward(&vision.reverse, &frame, 140U, -100, -30);
  assert(guard.rpm == 0 && !guard.confirmation_changed);
  ++frame.sequence;
  guard = VisionMotion_GuardForward(&vision.reverse, &frame, 160U, -100, -30);
  assert(guard.rpm == -30 && guard.accepted);
  frame.age_ms = frame.timeout_ms;
  assert(VisionMotion_GuardForward(&vision.reverse, &frame, 660U, -100, -30).rpm == 0);
}

int main(void)
{
  check_wheels();
  check_heading();
  check_motion_gates();
  check_action_steps();
  check_vision();
  puts("PASS: motion wheel/heading gates, timed/reverse/turn steps, vision profiles/freshness/reversal");
  return 0;
}
