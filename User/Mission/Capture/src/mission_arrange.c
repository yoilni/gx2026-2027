#include "mission_arrange.h"
#include "mission_internal.h"
#include "mission_state.h"
#include "mission_safety.h"
#include "mission_vision.h"
#include "mission_capture.h"
#include "mission_load.h"
#include "mission_supplement.h"
#include "mission_safe_zone.h"

#include "M2006.h"
#include "actuator_task.h"
#include "arrange_flow.h"
#include "capture_flow.h"
#include "recovery_flow.h"
#include "debug_uart_task.h"
#include "maixcam_task.h"
#include "robot_config.h"
#include "stm32f4xx_hal.h"
#include "flow_motion.h"

static uint32_t s4_fine_last_coordinate_tick;
static int32_t s4_initial_green02_previous_yaw_cdeg;
static int32_t s4_initial_green02_progress_cdeg;
static int32_t s4_e4_spin_previous_yaw_cdeg;
static int32_t s4_e4_spin_progress_cdeg;
static MissionState s4_arrange_recovery_resume_state;
static int32_t s4_arrange_recovery_base_yaw_cdeg;
static uint8_t s4_arrange_recovery_count;

static uint32_t MissionTask_S4ReverseDuration(bool push_right);

bool MissionTask_ReturnToS4TrackCenter(uint32_t now)
{
  if (mission_ctx.mission_snapshot.state == MISSION_STATE_S4_TRACK_CENTER)
  {
    return true;
  }

  /* Synchronize once per recovery return. Keep reacquired coordinates and
     enter 04 before TX so a quick camera reply is preserved. */
  MissionTask_EnterState(MISSION_STATE_S4_TRACK_CENTER, now);
  if (MaixCam_SendCommand(MAIXCAM_COMMAND_TRACK_CENTER_ACK) != HAL_OK)
  {
    mission_ctx.mission_snapshot.fault_flags |= MISSION_FAULT_VISION_TX;
    MissionTask_EnterState(MISSION_STATE_FAULT, now);
    (void)DebugUart_Log("[S4] RECOVERY RETURN TX04 FAILED\r\n");
    return false;
  }
  (void)DebugUart_Log(
      "[04] RECOVERY RETURN04, TX04, SYNC CAMERA\r\n");
  return true;
}

void MissionTask_StartS4TrackCenterFrom04(uint32_t now)
{
  if (MissionTask_HandleSupplementFinish(HAL_GetTick()) || MissionTask_LoadCheckPausesMotion()) return;
  MissionTask_ResetVisionPid();
  MaixCam_ClearObject();
  mission_ctx.mission_snapshot.vision_target_valid = false;
  if (Actuator_SetCameraObjectTrackView() != HAL_OK)
  {
    mission_ctx.mission_snapshot.fault_flags |= MISSION_FAULT_ACTUATOR;
    MissionTask_EnterState(MISSION_STATE_FAULT, now);
    return;
  }

  mission_ctx.s4_next_debug_tick = now;
  mission_ctx.s4_center_target_lost_since = now;
  mission_ctx.s4_center_search_sweep_completed = false;
  MissionTask_EnterState(MISSION_STATE_S4_TRACK_CENTER, now);
  if (MissionTask_HandleSupplementFinish(HAL_GetTick()) || MissionTask_LoadCheckPausesMotion()) return;
  if (MaixCam_SendCommand(MAIXCAM_COMMAND_TRACK_CENTER_ACK) != HAL_OK)
  {
    mission_ctx.mission_snapshot.fault_flags |= MISSION_FAULT_VISION_TX;
    MissionTask_EnterState(MISSION_STATE_FAULT, now);
    return;
  }
  if (MissionTask_HandleSupplementFinish(HAL_GetTick()) || MissionTask_LoadCheckPausesMotion()) return;
  (void)DebugUart_Log("[04] TX04, ACK RX04\r\n");
  (void)DebugUart_Logf(
      "[04] RX04 DOWN CAM=%u TRACK CENTER\r\n",
      (unsigned int)ROBOT_CAMERA_OBJECT_TRACK_ANGLE_DEG);
}

bool MissionTask_CommandS4VisionTracking(uint32_t now,
                                                 bool *xy_in_tolerance)
{
  const bool claw_approach =
      mission_ctx.mission_snapshot.state == MISSION_STATE_S4_TRACK_CENTER ||
      mission_ctx.mission_snapshot.state == MISSION_STATE_S4_TRACK_FINAL_BLOCK ||
      mission_ctx.mission_snapshot.state == MISSION_STATE_SUPPLEMENT_TRACK;
  bool final_track =
      mission_ctx.mission_snapshot.state == MISSION_STATE_S4_TRACK_FINAL_BLOCK ||
      mission_ctx.mission_snapshot.state == MISSION_STATE_S4_FINE32_TRACK ||
      mission_ctx.mission_snapshot.state == MISSION_STATE_S4_FINE42_TRACK ||
      (mission_ctx.mission_snapshot.state == MISSION_STATE_SUPPLEMENT_TRACK &&
       mission_ctx.supplement_reference_command == MAIXCAM_COMMAND_FINAL_CAPTURE);
  bool supplement_search =
      mission_ctx.mission_snapshot.state == MISSION_STATE_SUPPLEMENT_TRACK &&
      mission_ctx.supplement_reference_command == MAIXCAM_COMMAND_SEARCH_TARGET;
  int16_t turn_max_rpm = final_track ? ROBOT_S4_FINAL_TURN_MAX_RPM :
                                       ROBOT_S4_TRACK_TURN_MAX_RPM;
  int16_t forward_max_rpm = final_track ? ROBOT_S4_FINAL_FORWARD_MAX_RPM :
      (supplement_search ? (int16_t)ROBOT_VISION_03_MAX_FORWARD_RPM
                         : ROBOT_S4_TRACK_FORWARD_MAX_RPM);
  int16_t wheel_max_rpm = final_track ? ROBOT_S4_FINAL_WHEEL_MAX_RPM :
      (supplement_search ? ROBOT_VISION_03_WHEEL_MAX_RPM
                         : ROBOT_S4_TRACK_WHEEL_MAX_RPM);
  int32_t absolute_x_error;
  int32_t absolute_y_error;
  int16_t turn_rpm;
  int16_t forward_rpm;
  bool x_in_tolerance;
  bool y_in_tolerance;

  if (xy_in_tolerance != NULL)
  {
    *xy_in_tolerance = false;
  }

  if (!mission_ctx.mission_snapshot.vision_target_valid)
  {
    if (claw_approach)
      MissionTask_CommandClawApproach(now, turn_max_rpm, forward_max_rpm, wheel_max_rpm);
    else
      MissionTask_StopWheels();
    /* Keep the pixel PID history until an actual recovery/task change. */
    if ((int32_t)(now - mission_ctx.s4_next_debug_tick) >= 0)
    {
      mission_ctx.s4_next_debug_tick = now + ROBOT_S4_DEBUG_PERIOD_MS;
      (void)DebugUart_Logf("[S4-V] s=%u LOST w=0/0\r\n",
                           (unsigned int)mission_ctx.mission_snapshot.state);
    }
    return false;
  }

  absolute_x_error = MissionTask_Abs32(mission_ctx.mission_snapshot.vision_x_error_px);
  absolute_y_error = MissionTask_Abs32(mission_ctx.mission_snapshot.vision_y_error_px);
  x_in_tolerance = absolute_x_error <= ROBOT_VISION_X_TOLERANCE_PX;
  y_in_tolerance = absolute_y_error <= ROBOT_VISION_Y_TOLERANCE_PX;

  if (claw_approach)
  {
    MissionTask_CommandClawApproach(now, turn_max_rpm, forward_max_rpm, wheel_max_rpm);
    turn_rpm = mission_ctx.mission_snapshot.vision_turn_rpm;
    forward_rpm = mission_ctx.mission_snapshot.vision_forward_rpm;
  }
  else
  {
    turn_rpm = x_in_tolerance
        ? 0 : MissionTask_CalculateVisionTurn(now, mission_ctx.mission_snapshot.vision_x_error_px);
    forward_rpm = MissionTask_CalculateVisionForward(now, mission_ctx.mission_snapshot.vision_y_error_px);

    if (turn_rpm > turn_max_rpm) turn_rpm = turn_max_rpm;
    else if (turn_rpm < -turn_max_rpm) turn_rpm = -turn_max_rpm;
    if (forward_rpm > forward_max_rpm) forward_rpm = forward_max_rpm;
    else if (forward_rpm < -forward_max_rpm) forward_rpm = -forward_max_rpm;

    mission_ctx.mission_snapshot.vision_turn_rpm = turn_rpm;
    mission_ctx.mission_snapshot.vision_forward_rpm = forward_rpm;
    if (x_in_tolerance && y_in_tolerance) MissionTask_StopWheels();
    else MissionTask_SetVisionTrackingTargets(forward_rpm, turn_rpm, wheel_max_rpm);
  }

  if (xy_in_tolerance != NULL)
  {
    *xy_in_tolerance = x_in_tolerance && y_in_tolerance;
  }

  if ((int32_t)(now - mission_ctx.s4_next_debug_tick) >= 0)
  {
    mission_ctx.s4_next_debug_tick = now + ROBOT_S4_DEBUG_PERIOD_MS;
    (void)DebugUart_Logf(
        "[04/24] s=%u x=%d y=%d u=%d v=%d w=%d/%d\r\n",
        (unsigned int)mission_ctx.mission_snapshot.state,
        (int)mission_ctx.mission_snapshot.vision_x_error_px,
        (int)mission_ctx.mission_snapshot.vision_y_error_px,
        (int)turn_rpm,
        (int)forward_rpm,
        (int)mission_ctx.mission_snapshot.left_target_rpm,
        (int)mission_ctx.mission_snapshot.right_target_rpm);
  }
  return true;
}

void MissionTask_AckS4Center(uint32_t now)
{
  if (MissionTask_HandleSupplementFinish(HAL_GetTick()) || MissionTask_LoadCheckPausesMotion()) return;
  MissionTask_StopWheels();
  MissionTask_ResetVisionControllerState();
  MaixCam_ClearObject();
  mission_ctx.mission_snapshot.vision_target_valid = false;
  MissionTask_EnterState(MISSION_STATE_S4_WAIT_ARRANGE_READY, now);
  mission_ctx.s4_direct_capture_wait24 = ROBOT_S4_DIRECT_CAPTURE_AFTER_14 != 0U;
  if (MissionTask_HandleSupplementFinish(HAL_GetTick()) || MissionTask_LoadCheckPausesMotion()) return;
  /* RX14 has already been consumed. Do not clear events here: a quick RX02
     or RX24 arriving while the ACK is sent must remain available. */
  if (MaixCam_SendCommand(MAIXCAM_COMMAND_CENTER_ACK) != HAL_OK)
  {
    mission_ctx.mission_snapshot.fault_flags |= MISSION_FAULT_VISION_TX;
    MissionTask_EnterState(MISSION_STATE_FAULT, now);
    return;
  }
  if (MissionTask_HandleSupplementFinish(HAL_GetTick()) || MissionTask_LoadCheckPausesMotion()) return;
  (void)DebugUart_Log(mission_ctx.s4_direct_capture_wait24
      ? "[04] RX14 STOP, TX14, WAIT24/E4\r\n"
      : "[04] RX14 STOP, TX14, WAIT02/24/E4\r\n");
}

static void MissionTask_CommandS4CornerReverse(uint32_t now, bool push_right)
{
  uint32_t elapsed = (uint32_t)(now - mission_ctx.mission_snapshot.state_entry_tick);
  uint32_t duration = MissionTask_S4ReverseDuration(push_right);
  int16_t left_rpm, right_rpm;
  ArrangeFlow_CornerReverseRpm(elapsed, push_right, &left_rpm, &right_rpm);
  mission_ctx.mission_snapshot.vision_forward_rpm = (int16_t)((left_rpm + right_rpm) / 2);
  mission_ctx.mission_snapshot.vision_turn_rpm = (int16_t)((left_rpm - right_rpm) / 2);
  MissionTask_SetWheelTargets(left_rpm, right_rpm);
  if ((int32_t)(now - mission_ctx.s4_next_debug_tick) >= 0)
  {
    mission_ctx.s4_next_debug_tick = now + ROBOT_S4_DEBUG_PERIOD_MS;
    (void)DebugUart_Logf(
        "[PUSH-B] p=%s t=%lu/%lu w=%d/%d\r\n",
        push_right ? "02" : "12", (unsigned long)elapsed,
        (unsigned long)duration, (int)left_rpm, (int)right_rpm);
  }
}

static void MissionTask_CommandS4CornerPush(uint32_t now, bool push_right)
{
  uint32_t elapsed = (uint32_t)(now - mission_ctx.mission_snapshot.state_entry_tick);
  uint32_t duration = ArrangeFlow_CornerPushMs(push_right);
  int16_t left_rpm;
  int16_t right_rpm;
  ArrangeFlow_CornerRpm(elapsed, duration, push_right,
                                       &left_rpm, &right_rpm);
  mission_ctx.mission_snapshot.vision_turn_rpm =
      (int16_t)((left_rpm - right_rpm) / 2);
  mission_ctx.mission_snapshot.vision_forward_rpm = (int16_t)((left_rpm + right_rpm) / 2);
  MissionTask_SetWheelTargets(left_rpm, right_rpm);

  if ((int32_t)(now - mission_ctx.s4_next_debug_tick) >= 0)
  {
    int16_t actual_left_rpm = (int16_t)(
        ((float)moto_chassis[MISSION_LEFT_MOTOR_INDEX].speed_rpm / 36.0f) *
        ROBOT_LEFT_WHEEL_FORWARD_SIGN);
    int16_t actual_right_rpm = (int16_t)(
        ((float)moto_chassis[MISSION_RIGHT_MOTOR_INDEX].speed_rpm / 36.0f) *
        ROBOT_RIGHT_WHEEL_FORWARD_SIGN);
    mission_ctx.s4_next_debug_tick = now + ROBOT_S4_DEBUG_PERIOD_MS;
    (void)DebugUart_Logf(
        "[PUSH] p=%s t=%lu/%lu d=%d w=%d/%d act=%d/%d\r\n",
        push_right ? "02" : "12",
        (unsigned long)elapsed,
        (unsigned long)duration,
        (int)MissionTask_Abs32((int32_t)left_rpm - right_rpm),
        (int)mission_ctx.mission_snapshot.left_target_rpm,
        (int)mission_ctx.mission_snapshot.right_target_rpm,
        (int)actual_left_rpm,
        (int)actual_right_rpm);
  }
}

/* FOLLOW_MS is a distance calibration at the reference speeds. Solve the
   new duration from that integrated area so speed changes preserve nominal
   travel, including both ramps and12's300ms low-speed/150ms blend segments. */
static uint32_t MissionTask_S4ReverseDuration(bool push_right)
{
  return ArrangeFlow_CornerReverseMs(push_right);
}

static void MissionTask_StartS4TrackState(MissionState state, uint32_t now)
{
  if (state == MISSION_STATE_S4_TRACK_RIGHT_BLOCK)
  {
    if (mission_ctx.s4_arrange_cycle_count < UINT16_MAX)
    {
      ++mission_ctx.s4_arrange_cycle_count;
    }
    (void)DebugUart_Logf(
        "[PUSH] RX02 n=%u F=%d/%lu\r\n",
        (unsigned int)mission_ctx.s4_arrange_cycle_count,
        ROBOT_S4_FINE42_FORWARD_RPM,
        (unsigned long)ROBOT_S4_PUSH_TRIAL_FORWARD_MS);
  }
  /*02's RX boundary already isolated the previous reference. Keep ROI
     coordinates received immediately after02, including a partial frame. */
  if (state != MISSION_STATE_S4_TRACK_RIGHT_BLOCK) MaixCam_ClearObject();
  MissionTask_ResetVisionPid();
  if ((state == MISSION_STATE_S4_TRACK_RIGHT_BLOCK) ||
      (state == MISSION_STATE_S4_TRACK_LEFT_BLOCK))
  {
    s4_arrange_recovery_count = 0U;
  }
  mission_ctx.s4_next_debug_tick = now;
  MissionTask_EnterState(state, now);
}

static void MissionTask_StartS4ArrangeRecovery(uint32_t now,
                                                MissionState resume_state)
{
  MissionTask_StopWheels();
  MaixCam_ClearObject();
  mission_ctx.mission_snapshot.vision_target_valid = false;
  MissionTask_ResetVisionPid();
  MissionTask_ResetS6ControllerState();
  mission_ctx.s6_phase_stable_since = 0U;
  s4_arrange_recovery_resume_state = resume_state;
  s4_arrange_recovery_base_yaw_cdeg = MissionTask_WrapYaw(
      mission_ctx.mission_snapshot.yaw_cdeg);
  mission_ctx.s4_arrange_recovery_left_yaw_cdeg = MissionTask_WrapYaw(
      s4_arrange_recovery_base_yaw_cdeg +
      (int32_t)ROBOT_YAW_LEFT_SIGN *
          ROBOT_S4_ARRANGE_RECOVERY_LEFT_CDEG);
  mission_ctx.s4_arrange_recovery_right_yaw_cdeg = MissionTask_WrapYaw(
      mission_ctx.s4_arrange_recovery_left_yaw_cdeg -
      (int32_t)ROBOT_YAW_LEFT_SIGN *
          ROBOT_S4_ARRANGE_RECOVERY_RIGHT_CDEG);
  ++s4_arrange_recovery_count;
  mission_ctx.s4_next_debug_tick = now;
  MissionTask_EnterState(
      MISSION_STATE_S4_ARRANGE_RECOVERY_REVERSE, now);
  (void)DebugUart_Logf(
      "[E2] START n=%u ref=%s r=%ld L=%ld R=%ld back=%d/%lu\r\n",
      (unsigned int)s4_arrange_recovery_count,
      resume_state == MISSION_STATE_S4_TRACK_RIGHT_BLOCK ? "02" : "12",
      (long)s4_arrange_recovery_base_yaw_cdeg,
      (long)mission_ctx.s4_arrange_recovery_left_yaw_cdeg,
      (long)mission_ctx.s4_arrange_recovery_right_yaw_cdeg,
      ROBOT_S4_ARRANGE_RECOVERY_REVERSE_RPM,
      (unsigned long)ROBOT_S4_ARRANGE_RECOVERY_REVERSE_MS);
}

static bool MissionTask_TryS4ArrangeEvent(uint32_t now,
                                           MissionState resume_state)
{
  uint8_t event_code;
  (void)resume_state;

  if (!MaixCam_TakeEvent(&event_code))
  {
    return false;
  }
  if (event_code == MAIXCAM_EVENT_SKIP_TO_FINAL_TRACK)
  {
    MissionTask_SkipToS4FinalTrack(now);
    return true;
  }
  (void)DebugUart_Logf(
      "[E2] RX%02X WAIT24/XY E2_AFTER22\r\n",
                       (unsigned int)event_code);
  return false;
}

static bool MissionTask_TryResumeS4ArrangeRecovery(uint32_t now)
{
  uint8_t event_code;
  int16_t recovered_x_error;
  int16_t recovered_y_error;

  if (MaixCam_TakeEvent(&event_code))
  {
    if (event_code == MAIXCAM_EVENT_SKIP_TO_FINAL_TRACK)
    {
      MissionTask_SkipToS4FinalTrack(now);
      return true;
    }
    if (event_code != MAIXCAM_EVENT_ARRANGE_TARGET_LOST)
    {
      (void)DebugUart_Logf(
          "[E2] RX%02X WAIT24/XY\r\n",
          (unsigned int)event_code);
    }
  }

  if (!mission_ctx.mission_snapshot.vision_target_valid)
  {
    return false;
  }

  recovered_x_error = mission_ctx.mission_snapshot.vision_x_error_px;
  recovered_y_error = mission_ctx.mission_snapshot.vision_y_error_px;
  MissionTask_StopWheels();
  MissionTask_ResetVisionPid();
  MissionTask_ResetS6ControllerState();
  mission_ctx.s4_next_debug_tick = now;
  MissionTask_EnterState(s4_arrange_recovery_resume_state, now);
  (void)DebugUart_Logf(
      "[E2] FOUND x=%d y=%d >%s\r\n",
      (int)recovered_x_error,
      (int)recovered_y_error,
      s4_arrange_recovery_resume_state ==
              MISSION_STATE_S4_TRACK_RIGHT_BLOCK
          ? "02"
          : "12");
  return true;
}

static void MissionTask_ResumeS4AfterE4(uint32_t now)
{
  uint8_t event_code;
  bool synced;
  MissionTask_StopWheels();
  MissionTask_ResetVisionControllerState();
  MissionTask_ResetS6ControllerState();
  (void)MaixCam_DropEventIf(MAIXCAM_EVENT_CENTER_TARGET_LOST);
  mission_ctx.s4_center_search_sweep_completed = false;
  mission_ctx.s4_center_target_lost_since = mission_ctx.mission_snapshot.vision_target_valid ? 0U : now;
  mission_ctx.s4_next_debug_tick = now;
  /* Enter04 closes the E4 loss filter. Keep phase buffering open through TX
     so a quick14 followed by02/24 cannot overwrite the one-shot arrival. */
  MaixCam_SetE4StageBuffering(true);
  synced = MissionTask_ReturnToS4TrackCenter(now);
  MaixCam_SetE4StageBuffering(false);
  if (!synced) return;
  /* Sync04 first, then honor an already received one-shot14.02/24 remain
     queued for the ordinary arrange wait; neither is erased on return. */
  if (MaixCam_PeekEvent(&event_code))
  {
    if (event_code == MAIXCAM_EVENT_CENTER_REACHED)
    {
      (void)MaixCam_TakeEvent(&event_code);
      MissionTask_AckS4Center(now);
    }
    else if (event_code == MAIXCAM_EVENT_ARRANGE_READY ||
             event_code == MAIXCAM_EVENT_SKIP_TO_FINAL_TRACK ||
             event_code == MAIXCAM_EVENT_OFFSET_PUSH ||
             event_code == MAIXCAM_EVENT_SIDE_PUSH)
    {
      MissionTask_EnterState(MISSION_STATE_S4_WAIT_ARRANGE_READY, now);
    }
  }
}

static void MissionTask_StartS4E4Spin(uint32_t now, MissionState spin_state)
{
  MissionTask_StopWheels();
  MissionTask_ResetVisionControllerState();
  (void)MaixCam_DropEventIf(MAIXCAM_EVENT_CENTER_TARGET_LOST);
  MaixCam_ClearObject();
  mission_ctx.mission_snapshot.vision_target_valid = false;
  mission_ctx.s4_e4_spin_origin_yaw_cdeg =
      MissionTask_WrapYaw(mission_ctx.mission_snapshot.yaw_cdeg);
  s4_e4_spin_previous_yaw_cdeg = mission_ctx.s4_e4_spin_origin_yaw_cdeg;
  s4_e4_spin_progress_cdeg = 0L;
  mission_ctx.s4_next_debug_tick = now;
  if (spin_state == MISSION_STATE_S4_E4_MATERIAL_SPIN_CW_360)
  {
    MissionTask_EnterState(spin_state, now);
    (void)DebugUart_Logf(
        "[E4-M360] CW n=%u f=%ld v=%d XY BREAK\r\n",
        (unsigned int)mission_ctx.s4_material_e4_recovery_count,
        (long)MissionTask_WrapYaw(mission_ctx.s4_e4_spin_origin_yaw_cdeg - mission_ctx.mission_snapshot.yaw_start_cdeg),
        ROBOT_S4_E4_MATERIAL_SPIN_FAST_RPM);
    return;
  }
  if (spin_state == MISSION_STATE_S4_E4_RED_SPIN_360)
  {
    mission_ctx.s4_red_e4_align_away_active = true;
    mission_ctx.s4_e4_away_yaw_cdeg = MissionTask_WrapYaw(
        mission_ctx.mission_snapshot.safe_zone_yaw_target_cdeg + 18000L);
    mission_ctx.s3_search_yaw_target_cdeg = mission_ctx.s4_e4_away_yaw_cdeg;
    mission_ctx.s7_red_e3_phase = S7_RED_E3_FORWARD;
    mission_ctx.vision_search_forward_rpm = ROBOT_S7_E3_SEARCH_FORWARD_RPM;
    mission_ctx.vision_search_forward_ms = ROBOT_S7_E3_SEARCH_FORWARD_MS;
    mission_ctx.vision_search_forward_active = false;
    MissionTask_ResetS6ControllerState();
    MissionTask_EnterState(MISSION_STATE_S4_E4_RED_SPIN_360, now);
    (void)DebugUart_Logf(
        "[E4-R] AWAY r=%ld f=%ld >FWD=%d/%lu L60/500 R120/500 L60\r\n",
        (long)mission_ctx.s4_e4_away_yaw_cdeg,
        (long)MissionTask_WrapYaw(mission_ctx.s4_e4_away_yaw_cdeg - mission_ctx.mission_snapshot.yaw_start_cdeg),
        (int)mission_ctx.vision_search_forward_rpm, (unsigned long)mission_ctx.vision_search_forward_ms);
    return;
  }
  MissionTask_EnterState(MISSION_STATE_S4_E4_GREEN_SPIN_360, now);
  (void)DebugUart_Logf(
      "[E4-G360] n=%u/%u r=%ld WAIT XY\r\n",
      (unsigned int)mission_ctx.s4_green_e4_push_cycles,
      (unsigned int)ROBOT_S4_E4_GREEN_PUSH_MAX_CYCLES,
      (long)mission_ctx.s4_e4_spin_origin_yaw_cdeg);
}

static void MissionTask_RunS4E4Spin(uint32_t now)
{
  uint8_t event_code;
  int16_t turn_rpm;
  bool red_search = mission_ctx.mission_snapshot.state == MISSION_STATE_S4_E4_RED_SPIN_360;
  bool material_search = mission_ctx.mission_snapshot.state == MISSION_STATE_S4_E4_MATERIAL_SPIN_CW_360;
  int32_t yaw_direction = material_search ? -(int32_t)ROBOT_YAW_LEFT_SIGN
                                          : (int32_t)ROBOT_YAW_LEFT_SIGN;
  const char *log_tag = material_search ? "S4-E4-MAT360"
      : (red_search ? "E4-R" : "S4-E4-360");
  int32_t sweep_cdeg = material_search ? ROBOT_S4_E4_MATERIAL_SPIN_CDEG
      : ROBOT_S4_E4_GREEN_SPIN_CDEG;
  uint32_t timeout_ms = material_search ? ROBOT_S4_E4_MATERIAL_SPIN_TIMEOUT_MS
      : ROBOT_S4_E4_GREEN_SPIN_TIMEOUT_MS;
  int32_t current_yaw_cdeg = MissionTask_WrapYaw(mission_ctx.mission_snapshot.yaw_cdeg);

  if (MaixCam_PeekEvent(&event_code) &&
      (event_code == MAIXCAM_EVENT_CENTER_REACHED ||
       event_code == MAIXCAM_EVENT_ARRANGE_READY ||
       event_code == MAIXCAM_EVENT_SKIP_TO_FINAL_TRACK ||
       event_code == MAIXCAM_EVENT_OFFSET_PUSH ||
       event_code == MAIXCAM_EVENT_SIDE_PUSH))
  {
    if (!material_search) mission_ctx.s4_green_e4_spin_exhausted = false;
    (void)DebugUart_Logf("[%s] RX%02X STOP TX04 >PHASE\r\n",
        log_tag, (unsigned int)event_code);
    if (red_search)
    {
      mission_ctx.s4_red_e4_align_away_active = false;
      mission_ctx.vision_search_forward_active = false;
    }
    MissionTask_ResumeS4AfterE4(now);
    return;
  }
  if (mission_ctx.mission_snapshot.vision_target_valid &&
      (!red_search || mission_ctx.mission_snapshot.target_id == 4U) &&
      (!material_search || mission_ctx.mission_snapshot.target_id == 6U ||
       (mission_ctx.s3_target_select_command == MAIXCAM_COMMAND_SELECT_BLACK_GREEN &&
        mission_ctx.mission_snapshot.target_id == 5U)))
  {
    if (!material_search) mission_ctx.s4_green_e4_spin_exhausted = false;
    (void)DebugUart_Logf(
        "[%s] FOUND x=%d y=%d, RESUME04\r\n",
        log_tag,
        (int)mission_ctx.mission_snapshot.vision_x_error_px,
        (int)mission_ctx.mission_snapshot.vision_y_error_px);
    if (red_search)
    {
      mission_ctx.s4_red_e4_align_away_active = false;
      mission_ctx.vision_search_forward_active = false;
    }
    MissionTask_ResumeS4AfterE4(now);
    return;
  }

  if (red_search)
  {
    if (mission_ctx.s4_red_e4_align_away_active)
    {
      turn_rpm = MissionTask_CalculateS6YawTurn(now, mission_ctx.s4_e4_away_yaw_cdeg);
      mission_ctx.mission_snapshot.yaw_error_cdeg = MissionTask_YawError(
          mission_ctx.s4_e4_away_yaw_cdeg, mission_ctx.mission_snapshot.yaw_cdeg);
      if (MissionTask_HeadingTurnExpired(now, mission_ctx.s4_e4_away_yaw_cdeg) ||
          MissionTask_Abs32(mission_ctx.mission_snapshot.yaw_error_cdeg) <=
              ROBOT_S3_SEARCH_TURN_TOLERANCE_CDEG)
      {
        MissionTask_StopWheels();
        MissionTask_ResetS6ControllerState();
        mission_ctx.s4_red_e4_align_away_active = false;
        mission_ctx.vision_search_forward_active = true;
        /* Alignment has its own deadline; do not consume the800ms advance. */
        mission_ctx.mission_snapshot.state_entry_tick = now;
        (void)DebugUart_Logf("[E4-R] AWAY OK >FWD=%d/%lu\r\n",
            (int)mission_ctx.vision_search_forward_rpm, (unsigned long)mission_ctx.vision_search_forward_ms);
        return;
      }
      mission_ctx.mission_snapshot.vision_forward_rpm = 0;
      mission_ctx.mission_snapshot.vision_turn_rpm = turn_rpm;
      MissionTask_SetWheelTargets(turn_rpm, (int16_t)-turn_rpm);
      return;
    }
    if (MissionTask_RunRedSearchSweep(now))
    {
      mission_ctx.s3_target_select_command = MAIXCAM_COMMAND_SELECT_BLACK_GREEN;
      (void)DebugUart_Log("[E4-R] SCAN MISS >51/03\r\n");
      (void)MissionTask_BeginS3Vision(now);
    }
    return;
  }

  RecoveryFlow_ObserveSpin(&s4_e4_spin_previous_yaw_cdeg, &s4_e4_spin_progress_cdeg,
      current_yaw_cdeg, yaw_direction);
  if (s4_e4_spin_progress_cdeg >= sweep_cdeg)
  {
    MissionTask_StopWheels();
    if (material_search)
    {
      (void)DebugUart_Logf(
          "[E4-M360] OK deg100=%ld >04 KEEP B/G\r\n",
          (long)s4_e4_spin_progress_cdeg);
      MissionTask_ResumeS4AfterE4(now);
      return;
    }
    mission_ctx.s4_green_e4_spin_exhausted = true;
    (void)DebugUart_Logf(
        "[E4-G360] OK deg100=%ld LOST WAIT XY\r\n",
        (long)s4_e4_spin_progress_cdeg);
    MissionTask_ResumeS4AfterE4(now);
    return;
  }
  if ((uint32_t)(now - mission_ctx.mission_snapshot.state_entry_tick) >=
      timeout_ms)
  {
    mission_ctx.mission_snapshot.fault_flags |= MISSION_FAULT_S4_TIMEOUT;
    MissionTask_EnterState(MISSION_STATE_FAULT, now);
    (void)DebugUart_Logf("[%s] TIMEOUT progress=%ld\r\n", log_tag,
                         (long)s4_e4_spin_progress_cdeg);
    return;
  }

  turn_rpm = sweep_cdeg - s4_e4_spin_progress_cdeg >
                 ROBOT_S3_SEARCH_TURN_SLOW_THRESHOLD_CDEG
                 ? (material_search ? ROBOT_S4_E4_MATERIAL_SPIN_FAST_RPM
                     : ROBOT_S4_E4_GREEN_SPIN_FAST_RPM)
                 : (material_search ? ROBOT_S4_E4_MATERIAL_SPIN_SLOW_RPM
                     : ROBOT_S4_E4_GREEN_SPIN_SLOW_RPM);
  mission_ctx.mission_snapshot.vision_forward_rpm = 0;
  mission_ctx.mission_snapshot.vision_turn_rpm =
      (int16_t)(-yaw_direction * turn_rpm);
  MissionTask_SetWheelTargets((int16_t)(-yaw_direction * turn_rpm),
                              (int16_t)(yaw_direction * turn_rpm));
  if ((int32_t)(now - mission_ctx.s4_next_debug_tick) >= 0)
  {
    mission_ctx.s4_next_debug_tick = now + ROBOT_ANGLE_DEBUG_PERIOD_MS;
    (void)DebugUart_Logf(
        "[%s] progress=%ld/%ld yaw=%ld v=%d\r\n",
        log_tag, (long)s4_e4_spin_progress_cdeg, (long)sweep_cdeg,
        (long)current_yaw_cdeg, (int)turn_rpm);
  }
}

static void MissionTask_StartS4GreenHalfTurn(uint32_t now)
{
  MissionTask_StopWheels();
  if (Actuator_SetFrameLowered() != HAL_OK)
  {
    mission_ctx.mission_snapshot.fault_flags |= MISSION_FAULT_ACTUATOR;
    MissionTask_EnterState(MISSION_STATE_FAULT, now);
    return;
  }
  MissionTask_ResetVisionControllerState();
  MissionTask_ResetS6ControllerState();
  (void)MaixCam_FenceObjectReference();
  mission_ctx.mission_snapshot.vision_target_valid = false;
  s4_e4_spin_previous_yaw_cdeg = MissionTask_WrapYaw(mission_ctx.mission_snapshot.yaw_cdeg);
  mission_ctx.s4_e4_spin_origin_yaw_cdeg = s4_e4_spin_previous_yaw_cdeg;
  s4_e4_spin_progress_cdeg = 0L;
  mission_ctx.s4_green_e4_half_turn_active = true;
  mission_ctx.s4_next_debug_tick = now;
  /* Reuse the first fixed-recovery ID, not its old L10/FWD/R45/L20 path. */
  MissionTask_EnterState(MISSION_STATE_S4_E4_PUSH_RIGHT, now);
  (void)DebugUart_Log("[E4-180] CCW180 LOCK XY >44\r\n");
}

void MissionTask_StartS4GreenScatter(uint32_t now)
{
  mission_ctx.s4_green_e4_half_turn_active = false; /* E5 retains its existing fixed path. */
  MissionTask_StopWheels();
  if (Actuator_SetFrameLowered() != HAL_OK)
  {
    mission_ctx.mission_snapshot.fault_flags |= MISSION_FAULT_ACTUATOR;
    MissionTask_EnterState(MISSION_STATE_FAULT, now);
    return;
  }
  MissionTask_ResetVisionControllerState();
  MissionTask_ResetS6ControllerState();
  MaixCam_ClearObject();
  mission_ctx.mission_snapshot.vision_target_valid = false;
  mission_ctx.s4_green_scatter_yaw_cdeg = MissionTask_WrapYaw(mission_ctx.mission_snapshot.yaw_cdeg +
      (int32_t)ROBOT_YAW_LEFT_SIGN * ROBOT_S5_E5_TURN_CDEG);
  mission_ctx.s6_phase_stable_since = 0U;
  MissionTask_EnterState(MISSION_STATE_S4_E4_PUSH_RIGHT, now);
  (void)DebugUart_Logf("[E4/5] L%ld F%d/%lu R%ld L%ld >44\r\n",
      (long)(ROBOT_S5_E5_TURN_CDEG / 100L), ROBOT_S4_FINE42_FORWARD_RPM,
      (unsigned long)ROBOT_S5_E5_FORWARD_MS,
      (long)(ROBOT_S5_E5_COUNTER_TURN_CDEG / 100L),
      (long)(ROBOT_S4_FINE42_FINISH_LEFT_CDEG / 100L));
}

static void MissionTask_StartS4E4Recovery(uint32_t now)
{
  if (MissionTask_ActiveRecoveryLossEvent() != 0U) return;
  mission_ctx.s4_e4_protocol_active = true;
  MissionTask_StopWheels();
  MissionTask_ResetVisionControllerState();
  MaixCam_ClearObject();
  mission_ctx.mission_snapshot.vision_target_valid = false;
  if (!mission_ctx.s7_target_switch_enabled ||
      mission_ctx.s3_target_select_command == MAIXCAM_COMMAND_SELECT_GREEN)
  {
    if (mission_ctx.s4_green_e4_push_cycles >= ROBOT_S4_E4_GREEN_PUSH_MAX_CYCLES)
    {
      if (!mission_ctx.s4_green_e4_spin_exhausted)
      {
        MissionTask_StartS4E4Spin(now, MISSION_STATE_S4_E4_GREEN_SPIN_360);
      }
      else
      {
        /* An E4 reply to 14/44 must also leave the handshake wait after the
           recovery allowance is exhausted. Keep the existing no-more-pushes
           policy and wait for coordinates in 04 instead. */
        MissionTask_ResumeS4AfterE4(now);
        (void)DebugUart_Log(
            "[E4] GREEN LIMIT WAIT04 XY\r\n");
      }
      return;
    }
    ++mission_ctx.s4_green_e4_push_cycles;
    /* Unused first-green allowance: forced CCW180, no translation/XY. */
    MissionTask_StartS4GreenHalfTurn(now);
    (void)DebugUart_Logf(
        "[E4] GREEN n=%u/%u >TX44\r\n",
        (unsigned int)mission_ctx.s4_green_e4_push_cycles,
        (unsigned int)ROBOT_S4_E4_GREEN_PUSH_MAX_CYCLES);
    return;
  }
  if (mission_ctx.s7_target_switch_enabled &&
      ((mission_ctx.s3_target_select_command == MAIXCAM_COMMAND_SELECT_BLACK) ||
       (mission_ctx.s3_target_select_command == MAIXCAM_COMMAND_SELECT_BLACK_GREEN)))
  {
    if (mission_ctx.s4_material_e4_recovery_count < 2U) ++mission_ctx.s4_material_e4_recovery_count;
    if (mission_ctx.s4_material_e4_recovery_count >= 2U)
    {
      MissionTask_StartS4E4Spin(now, MISSION_STATE_S4_E4_MATERIAL_SPIN_CW_360);
      return;
    }
    (void)DebugUart_Logf("[E4-M] FIRST SWEEP max=%d XY BREAK\r\n",
                         (int)ROBOT_S4_E4_MATERIAL_TURN_MAX_RPM);
  }
  if (mission_ctx.s7_target_switch_enabled && mission_ctx.s4_e4_recovery_used &&
      (mission_ctx.s3_target_select_command == MAIXCAM_COMMAND_SELECT_RED))
  {
    mission_ctx.s3_target_select_command = MAIXCAM_COMMAND_SELECT_BLACK_GREEN;
    (void)DebugUart_Log("[E4] LOST2 >51/03\r\n");
    (void)MissionTask_BeginS3Vision(now);
    return;
  }
  mission_ctx.s4_e4_recovery_used = true;
  if (mission_ctx.s7_target_switch_enabled &&
      mission_ctx.s3_target_select_command == MAIXCAM_COMMAND_SELECT_RED)
  {
    MissionTask_StartS4E4Spin(now, MISSION_STATE_S4_E4_RED_SPIN_360);
    return;
  }
  mission_ctx.s4_e4_away_yaw_cdeg = MissionTask_WrapYaw(
      mission_ctx.mission_snapshot.safe_zone_yaw_target_cdeg + 18000L);
  mission_ctx.s4_e4_left_yaw_cdeg = MissionTask_WrapYaw(
      mission_ctx.s4_e4_away_yaw_cdeg + (int32_t)ROBOT_YAW_LEFT_SIGN *
          ROBOT_S4_E4_LEFT_CDEG);
  mission_ctx.s4_e4_right_yaw_cdeg = MissionTask_WrapYaw(
      mission_ctx.s4_e4_left_yaw_cdeg - (int32_t)ROBOT_YAW_LEFT_SIGN *
          ROBOT_S4_E4_RIGHT_CDEG);
  MissionTask_ResetS6ControllerState();
  mission_ctx.s6_phase_stable_since = 0U;
  MissionTask_EnterState(MISSION_STATE_S4_E4_TURN_AWAY, now);
  (void)DebugUart_Logf(
      "[E4] AWAY r=%ld field=%ld >F80/600 L45 R90\r\n",
      (long)mission_ctx.s4_e4_away_yaw_cdeg,
      (long)MissionTask_WrapYaw(mission_ctx.s4_e4_away_yaw_cdeg - mission_ctx.mission_snapshot.yaw_start_cdeg));
}

static void MissionTask_FinishS4GreenRecovery(uint32_t now, bool empty_recovery)
{
  MissionTask_StopWheels();
  if (MissionTask_HandleSupplementFinish(now) || MissionTask_LoadCheckPausesMotion()) return;
  if (Actuator_SetCameraObjectTrackView() != HAL_OK)
  {
    mission_ctx.mission_snapshot.fault_flags |= MISSION_FAULT_ACTUATOR;
    MissionTask_EnterState(MISSION_STATE_FAULT, now);
    return;
  }
  MissionTask_ResetVisionPid();
  MissionTask_ResetS6ControllerState();
  (void)MaixCam_FenceSearchReference();
  mission_ctx.mission_snapshot.vision_target_valid = false;
  mission_ctx.mission_snapshot.target_id = 0U;
  mission_ctx.mission_snapshot.target_present = false;
  mission_ctx.mission_snapshot.target_age_ms = UINT32_MAX;
  mission_ctx.s4_e4_protocol_active = false;
  mission_ctx.s4_post22_search_active = false;
  mission_ctx.s4_final_mode_already_active = false;
  mission_ctx.s4_final_event34_pending = false;
  mission_ctx.s3_track_stable_since = 0U;
  mission_ctx.s3_search_turn_stable_since = 0U;
  mission_ctx.s3_search_origin_yaw_cdeg = MissionTask_WrapYaw(mission_ctx.mission_snapshot.yaw_cdeg);
  mission_ctx.s3_search_yaw_target_cdeg = mission_ctx.s3_search_origin_yaw_cdeg;
  mission_ctx.s3_search_sweep_completed = false;
  mission_ctx.s3_search_ccw_midpoint_done = false;
  mission_ctx.vision_search_forward_active = false;
  mission_ctx.vision_search_e3_mode = false;
  mission_ctx.vision_search_green_e3_mode = false;
  mission_ctx.vision_search_resume_state = MISSION_STATE_S3_TRACK_GREEN;
  mission_ctx.s3_target_lost_since = now;
  mission_ctx.s3_wait_event04_logged = false;
  mission_ctx.s3_next_debug_tick = now;
  /*44 itself tells the camera to resume03. Do not initialize a new target,
     send21/03 again, or reset the shared E4 budget/recovery counts.
     Open03 BEFORE TX so immediate new image-center XY/E3/04 survive. */
  MaixCam_SetE4StageBuffering(false);
  MissionTask_EnterState(MISSION_STATE_S3_TRACK_GREEN, now);
  if (MaixCam_SendCommand(MAIXCAM_COMMAND_CENTER_RECOVERY_DONE) != HAL_OK)
  {
    mission_ctx.mission_snapshot.fault_flags |= MISSION_FAULT_VISION_TX;
    MissionTask_EnterState(MISSION_STATE_FAULT, now);
    return;
  }
  (void)DebugUart_Log(empty_recovery ? "[E5] TX44 >03 IMG XY/E3/04\r\n"
                                   : "[E4-180] TX44 >03 IMG XY/E3/04\r\n");
}

void MissionTask_RunS4E4Recovery(uint32_t now)
{
  uint32_t elapsed = (uint32_t)(now - mission_ctx.mission_snapshot.state_entry_tick);
  MissionState state = mission_ctx.mission_snapshot.state;
  bool empty_recovery = mission_ctx.s5_empty_recovery_active;
  uint8_t event_code;

  if ((state == MISSION_STATE_S4_E4_GREEN_SPIN_360) ||
      (state == MISSION_STATE_S4_E4_RED_SPIN_360) ||
      (state == MISSION_STATE_S4_E4_MATERIAL_SPIN_CW_360))
  {
    MissionTask_RunS4E4Spin(now);
    return;
  }
  if ((state < MISSION_STATE_S4_E4_PUSH_RIGHT) ||
      (state > MISSION_STATE_S4_E4_TURN_RIGHT))
  {
    return;
  }
  /* Only a repeated E4 is disposable.14/02/24 are retained by the receiver.
     Initial-green timed pushes below never consume/intercept these messages. */
  (void)MaixCam_DropEventIf(MAIXCAM_EVENT_CENTER_TARGET_LOST);
  if (mission_ctx.s4_green_e4_half_turn_active)
  {
    RecoveryFlow_ObserveSpin(&s4_e4_spin_previous_yaw_cdeg, &s4_e4_spin_progress_cdeg,
        mission_ctx.mission_snapshot.yaw_cdeg, ROBOT_YAW_LEFT_SIGN);
    mission_ctx.mission_snapshot.vision_target_valid = false;
    if (s4_e4_spin_progress_cdeg >= ROBOT_S4_E4_GREEN_FIRST_TURN_CDEG)
    {
      MissionTask_FinishS4GreenRecovery(now, false);
      return;
    }
    /* Never claim180 on the2s fixed-heading timeout. Keep the existing
       continuous-search20s safety limit, with no successful44 on failure. */
    if (elapsed >= ROBOT_S4_E4_GREEN_SPIN_TIMEOUT_MS)
    {
      mission_ctx.mission_snapshot.fault_flags |= MISSION_FAULT_S4_TIMEOUT;
      MissionTask_EnterState(MISSION_STATE_FAULT, now);
      (void)DebugUart_Log("[E4-180] TIMEOUT STOP\r\n");
      return;
    }
    int16_t rpm = RecoveryFlow_TaperSpin(ROBOT_S4_E4_GREEN_FIRST_TURN_CDEG, s4_e4_spin_progress_cdeg,
        ROBOT_S3_SEARCH_TURN_SLOW_THRESHOLD_CDEG, ROBOT_S4_E4_GREEN_SPIN_FAST_RPM, ROBOT_S4_E4_GREEN_SPIN_SLOW_RPM);
    mission_ctx.mission_snapshot.vision_forward_rpm = 0;
    mission_ctx.mission_snapshot.vision_turn_rpm = (int16_t)(-ROBOT_YAW_LEFT_SIGN * rpm);
    MissionTask_SetWheelTargets((int16_t)(-ROBOT_YAW_LEFT_SIGN * rpm),
                                (int16_t)(ROBOT_YAW_LEFT_SIGN * rpm));
    if ((int32_t)(now - mission_ctx.s4_next_debug_tick) >= 0)
    {
      mission_ctx.s4_next_debug_tick = now + ROBOT_VISION_DEBUG_PERIOD_MS;
      (void)DebugUart_Logf("[E4-180] p=%ld v=%d\r\n", (long)s4_e4_spin_progress_cdeg, rpm);
    }
    return;
  }
  if ((state >= MISSION_STATE_S4_E4_TURN_AWAY) &&
      MaixCam_PeekEvent(&event_code) &&
      (event_code == MAIXCAM_EVENT_CENTER_REACHED ||
       event_code == MAIXCAM_EVENT_ARRANGE_READY ||
       event_code == MAIXCAM_EVENT_SKIP_TO_FINAL_TRACK ||
       event_code == MAIXCAM_EVENT_OFFSET_PUSH ||
       event_code == MAIXCAM_EVENT_SIDE_PUSH))
  {
    MissionTask_ResumeS4AfterE4(now);
    return;
  }
  if ((state >= MISSION_STATE_S4_E4_TURN_AWAY) &&
      mission_ctx.mission_snapshot.vision_target_valid)
  {
    (void)DebugUart_Log("[E4] FOUND, RESUME04\r\n");
    MissionTask_ResumeS4AfterE4(now);
    return;
  }
  switch (state)
  {
    case MISSION_STATE_S4_E4_PUSH_RIGHT:
    case MISSION_STATE_S4_E4_REVERSE_RIGHT:
    case MISSION_STATE_S4_E4_PUSH_LEFT:
    case MISSION_STATE_S4_E4_REVERSE_LEFT:
    {
      /* Keep the old telemetry IDs; the two REVERSE names are not reverse legs. */
      static const MissionState stage[] = {
        [SCATTER_TURN_IN] = MISSION_STATE_S4_E4_PUSH_RIGHT,
        [SCATTER_FORWARD] = MISSION_STATE_S4_E4_REVERSE_RIGHT,
        [SCATTER_COUNTER] = MISSION_STATE_S4_E4_PUSH_LEFT,
        [SCATTER_FINISH] = MISSION_STATE_S4_E4_REVERSE_LEFT,
      };
      ScatterFlow_Phase phase = SCATTER_TURN_IN;
      while (phase < SCATTER_DONE && stage[phase] != state) ++phase;
      if (phase == SCATTER_DONE) return;
      const FlowMotion io = { .wheels = MissionTask_SetWheelTargets,
        .turn = MissionTask_RunS6TurnToHeading };
      const ScatterFlow_Phase next = ArrangeFlow_StepScatter(&io, phase, now,
          mission_ctx.mission_snapshot.state_entry_tick, mission_ctx.mission_snapshot.yaw_cdeg, &mission_ctx.s4_green_scatter_yaw_cdeg);
      if (next == phase)
      {
        if (phase == SCATTER_FORWARD)
        {
          mission_ctx.mission_snapshot.vision_forward_rpm = ROBOT_S4_FINE42_FORWARD_RPM;
          mission_ctx.mission_snapshot.vision_turn_rpm = 0;
          mission_ctx.mission_snapshot.safe_zone_fixed_heading_active = false;
        }
        return;
      }
      if (next == SCATTER_DONE)
      {
        MissionTask_FinishS4GreenRecovery(now, empty_recovery);
        break;
      }
      MissionTask_ResetS6ControllerState();
      if (phase != SCATTER_TURN_IN) mission_ctx.s6_phase_stable_since = 0U;
      MissionTask_EnterState(stage[next], now);
      break;
    }
    case MISSION_STATE_S4_E4_TURN_AWAY:
      if (!MissionTask_RunS6TurnToHeading(now, mission_ctx.s4_e4_away_yaw_cdeg))
      {
        return;
      }
      MissionTask_ResetS6ControllerState();
      mission_ctx.s6_phase_stable_since = 0U;
      MissionTask_EnterState(MISSION_STATE_S4_E4_FORWARD, now);
      (void)DebugUart_Log("[E4] OPPOSITE SAFE OK, FWD\r\n");
      break;
    case MISSION_STATE_S4_E4_FORWARD:
      if (elapsed < ROBOT_S4_E4_FORWARD_MS)
      {
        MissionTask_CommandS6Straight(now, mission_ctx.s4_e4_away_yaw_cdeg,
                                     ROBOT_S4_E4_FORWARD_RPM,
                                     ROBOT_S4_E4_FORWARD_WHEEL_MAX_RPM);
        return;
      }
      MissionTask_ResetS6ControllerState();
      mission_ctx.s6_phase_stable_since = 0U;
      MissionTask_EnterState(MISSION_STATE_S4_E4_TURN_LEFT, now);
      break;
    case MISSION_STATE_S4_E4_TURN_LEFT:
      if (!MissionTask_RunS6TurnToHeading(now, mission_ctx.s4_e4_left_yaw_cdeg))
      {
        return;
      }
      MissionTask_ResetS6ControllerState();
      mission_ctx.s6_phase_stable_since = 0U;
      MissionTask_EnterState(MISSION_STATE_S4_E4_TURN_RIGHT, now);
      break;
    case MISSION_STATE_S4_E4_TURN_RIGHT:
      if (MissionTask_RunS6TurnToHeading(now, mission_ctx.s4_e4_right_yaw_cdeg))
      {
        MissionTask_ResumeS4AfterE4(now);
        (void)DebugUart_Log("[E4] SCAN OK, WAIT04 XY OR NEXT E4\r\n");
      }
      break;
    default:
      break;
  }
}

/* A fine-arrange loss changes the coordinate reference back to the image
   center. Clear only coordinates BEFORE04; retain one-shot phase messages. */
static void MissionTask_RecoverS4FineArrange(uint32_t now)
{
  (void)MaixCam_CloseFineArrange();
  MissionTask_StopWheels();
  (void)MaixCam_FenceObjectReference();
  mission_ctx.mission_snapshot.vision_target_valid = false;
  MissionTask_ResetVisionControllerState();
  MissionTask_EnterState(MISSION_STATE_S4_TRACK_CENTER, now);
  MissionTask_StartS4E4Recovery(now);
  if (MissionTask_ActiveRecoveryLossEvent() == MAIXCAM_EVENT_CENTER_TARGET_LOST)
  {
    if (MaixCam_SendCommand(MAIXCAM_COMMAND_TRACK_CENTER_ACK) != HAL_OK)
    {
      mission_ctx.mission_snapshot.fault_flags |= MISSION_FAULT_VISION_TX;
      MissionTask_EnterState(MISSION_STATE_FAULT, now);
    }
    (void)DebugUart_Log("[FINE] E4 >04/REC\r\n");
  }
}

/* First-green02/32/42 consume the same allowance as E4's forced half-turn.
   Count a successfully started action once, not each PID tick or reply.
   Normal arrange commands still run; only the next E4's route changes. */
void MissionTask_RecordInitialGreenPush(uint8_t command)
{
  if (mission_ctx.s7_target_switch_enabled || mission_ctx.s3_target_select_command != MAIXCAM_COMMAND_SELECT_GREEN)
    return;
  if (mission_ctx.s4_green_e4_push_cycles < UINT8_MAX) ++mission_ctx.s4_green_e4_push_cycles;
  (void)DebugUart_Logf("[E4-B] cmd=%02X n=%u/%u\r\n", (unsigned int)command,
      (unsigned int)mission_ctx.s4_green_e4_push_cycles,
      (unsigned int)ROBOT_S4_E4_GREEN_PUSH_MAX_CYCLES);
}

void MissionTask_HandleS4FineArrangeRequest(uint32_t now)
{
  uint8_t command;
  MissionState state = mission_ctx.mission_snapshot.state;
  /* Neither first-green E4's forced180 nor E5's fixed path can be interrupted
     by fine requests.44 starts03; retire old-reference requests before TX. */
  if (state != MISSION_STATE_S4_TRACK_CENTER &&
      state != MISSION_STATE_S4_WAIT_ARRANGE_READY &&
      state != MISSION_STATE_S4_WAIT_LEFT_TARGET &&
      state != MISSION_STATE_S4_WAIT_FINAL_READY) return;
  if (!MaixCam_TakeFineArrangeRequest(&command)) return;
  if (state == MISSION_STATE_S4_WAIT_ARRANGE_READY && mission_ctx.s4_direct_capture_wait24)
  {
    (void)DebugUart_Logf("[14] IGNORE%02X WAIT24\r\n", (unsigned int)command);
    return;
  }
  MissionTask_StopWheels();
  if (Actuator_SetFrameLowered() != HAL_OK)
  {
    mission_ctx.mission_snapshot.fault_flags |= MISSION_FAULT_ACTUATOR;
    MissionTask_EnterState(MISSION_STATE_FAULT, now);
    return;
  }
  MissionTask_ResetVisionControllerState();
  MissionTask_ResetS6ControllerState();
  mission_ctx.s4_fine_turn_left = mission_ctx.s4_fine_next_left;
  mission_ctx.s4_fixed_motion_ack = MAIXCAM_COMMAND_SIDE_PUSH_DONE_ACK;
  s4_fine_last_coordinate_tick = now;
  mission_ctx.s4_next_debug_tick = now;
  MissionTask_EnterState(command == MAIXCAM_EVENT_OFFSET_PUSH
      ? MISSION_STATE_S4_FINE32_TRACK : MISSION_STATE_S4_FINE42_TRACK, now);
  MissionTask_RecordInitialGreenPush(command);
  /* RX32/42 invalidates the previous reference before publishing following
     ROI frames. Do not clear again here: a full UART burst may already exist. */
  MissionTask_UpdateVisionInput(now);
  (void)DebugUart_Logf("[FINE] RX%02X ms=%u side=%c\r\n", (unsigned int)command,
      (unsigned int)(command == MAIXCAM_EVENT_OFFSET_PUSH
          ? ROBOT_S4_FINE32_FORWARD_MS : ROBOT_S4_FINE42_TRACK_MS),
      mission_ctx.s4_fine_turn_left ? 'L' : 'R');
}

static void MissionTask_CompleteS4FineArrange(uint32_t now, uint8_t command)
{
  /* Last loss check is atomic with closing the action window. A received E4
     wins over the old42 reply, even if coordinates/14 followed it in the ISR. */
  if (MaixCam_CloseFineArrange())
  {
    MissionTask_RecoverS4FineArrange(now);
    return;
  }
  MissionTask_StopWheels();
  MissionTask_ResetVisionControllerState();
  (void)MaixCam_FenceObjectReference();
  mission_ctx.mission_snapshot.vision_target_valid = false;
  if (command == MAIXCAM_COMMAND_SIDE_PUSH_DONE_ACK)
    mission_ctx.s4_fine_next_left = !mission_ctx.s4_fine_turn_left;
  /* Buffer quick replies during synchronous TX. No TX14: vision now makes a
     fresh14 decision and sends02/12/24/32/42 itself. */
  MaixCam_SetE4StageBuffering(true);
  if (command == MAIXCAM_COMMAND_RIGHT_DONE_ACK)
  {
    MissionTask_EnterState(MISSION_STATE_S4_WAIT_LEFT_TARGET, now);
  }
  else if (command == MAIXCAM_COMMAND_LEFT_DONE_ACK)
  {
    mission_ctx.s4_post22_search_active = false;
    mission_ctx.s4_post22_failed_sweeps = 0U;
    MissionTask_EnterState(MISSION_STATE_S4_WAIT_FINAL_READY, now);
  }
  else
  {
    MissionTask_EnterState(MISSION_STATE_S4_WAIT_ARRANGE_READY, now);
    mission_ctx.s4_fine_waiting_decision = true;
  }
  HAL_StatusTypeDef result = MaixCam_SendCommand(command);
  MaixCam_SetE4StageBuffering(false);
  if (result != HAL_OK)
  {
    mission_ctx.mission_snapshot.fault_flags |= MISSION_FAULT_VISION_TX;
    MissionTask_EnterState(MISSION_STATE_FAULT, now);
    return;
  }
  (void)DebugUart_Logf("[FINE] TX%02X >WAIT\r\n", (unsigned int)command);
  (void)MissionTask_HandleLoadCheckRequest(now);
}

void MissionTask_RunS4FineArrange(uint32_t now)
{
  MissionState state = mission_ctx.mission_snapshot.state;
  if (state < MISSION_STATE_S4_FINE32_TRACK || state > MISSION_STATE_S4_FINE42_FINISH_TURN_LEFT)
    return;
  if (mission_ctx.s4_fixed_motion_ack != MAIXCAM_COMMAND_SIDE_PUSH_DONE_ACK)
  {
    uint8_t event_code;
    /* Preserve02/12's existing ability to skip straight to24. Other phase
       messages stay pending through the fixed path and its ACK boundary. */
    if (MaixCam_PeekEvent(&event_code) && event_code == MAIXCAM_EVENT_SKIP_TO_FINAL_TRACK)
    {
      (void)MaixCam_TakeEvent(&event_code);
      MissionTask_SkipToS4FinalTrack(now);
      return;
    }
    (void)MaixCam_DropEventIf(mission_ctx.s4_fixed_motion_ack == MAIXCAM_COMMAND_RIGHT_DONE_ACK
        ? MAIXCAM_EVENT_ARRANGE_READY : MAIXCAM_EVENT_LEFT_TARGET_READY);
  }
  if ((state == MISSION_STATE_S4_FINE32_TRACK || state == MISSION_STATE_S4_FINE42_TRACK) &&
      MaixCam_FineArrangeLossPending())
  {
    MissionTask_RecoverS4FineArrange(now);
    return;
  }
  if (state == MISSION_STATE_S4_FINE42_TRACK)
  {
    if (mission_ctx.mission_snapshot.vision_target_valid)
      s4_fine_last_coordinate_tick = now - mission_ctx.mission_snapshot.target_age_ms;
    else if ((uint32_t)(now - s4_fine_last_coordinate_tick) >= MAIXCAM_DATA_TIMEOUT_MS)
    {
      MissionTask_RecoverS4FineArrange(now);
      return;
    }
    if (!mission_ctx.mission_snapshot.vision_target_valid)
    {
      /* Never start a blind fixed push merely because the window expired. */
      MissionTask_StopWheels();
      return;
    }
  }

  static const MissionState stage[] = {
    [ARRANGE_OFFSET] = MISSION_STATE_S4_FINE32_TRACK,
    [ARRANGE_TRACK] = MISSION_STATE_S4_FINE42_TRACK,
    [ARRANGE_TURN] = MISSION_STATE_S4_FINE42_TURN,
    [ARRANGE_PUSH] = MISSION_STATE_S4_FINE42_FORWARD,
    [ARRANGE_COUNTER] = MISSION_STATE_S4_FINE42_COUNTER_TURN,
    [ARRANGE_FINISH_LEFT] = MISSION_STATE_S4_FINE42_FINISH_TURN_LEFT,
    [ARRANGE_LEGACY_END] = MISSION_STATE_S4_FINE42_REVERSE,
  };
  ArrangeFlow_Phase phase = ARRANGE_OFFSET;
  while (phase < ARRANGE_DONE && stage[phase] != state) ++phase;
  if (phase == ARRANGE_DONE) return;
  ArrangeFlow_State flow = { .turn_yaw = mission_ctx.s4_fine_turn_yaw_cdeg, .forward_yaw = mission_ctx.s4_fine_forward_yaw_cdeg,
    .spin_previous_yaw = s4_initial_green02_previous_yaw_cdeg,
    .spin_progress = s4_initial_green02_progress_cdeg };
  const ArrangeFlow_Profile profile = {
    .first_green = mission_ctx.s4_initial_green02_active,
    .side_push = mission_ctx.s4_fixed_motion_ack == MAIXCAM_COMMAND_SIDE_PUSH_DONE_ACK,
    .right_pass = mission_ctx.s4_fixed_motion_ack == MAIXCAM_COMMAND_RIGHT_DONE_ACK,
    .casualty = mission_ctx.s3_target_select_command == MAIXCAM_COMMAND_SELECT_RED,
    .turn_left = mission_ctx.s4_fine_turn_left,
  };
  const ArrangeFlow_Io io = {
    .motion = { .stop = MissionTask_StopWheels, .wheels = MissionTask_SetWheelTargets,
                .turn = MissionTask_RunS6TurnToHeading },
    .track = MissionTask_CommandS4VisionTracking,
    .yaw_pid = MissionTask_CalculateS6YawPid,
  };
  const ArrangeFlow_StepResult step = ArrangeFlow_Step(&flow, &io, phase, profile, now,
      mission_ctx.mission_snapshot.state_entry_tick, mission_ctx.mission_snapshot.yaw_cdeg);
  mission_ctx.s4_fine_turn_yaw_cdeg = flow.turn_yaw;
  mission_ctx.s4_fine_forward_yaw_cdeg = flow.forward_yaw;
  s4_initial_green02_previous_yaw_cdeg = flow.spin_previous_yaw;
  s4_initial_green02_progress_cdeg = flow.spin_progress;
  if (step.fixed_output)
  {
    mission_ctx.mission_snapshot.vision_forward_rpm = step.forward_rpm;
    mission_ctx.mission_snapshot.vision_turn_rpm = step.turn_rpm;
    mission_ctx.mission_snapshot.safe_zone_fixed_heading_active = false;
    if (phase == ARRANGE_PUSH)
    {
      mission_ctx.mission_snapshot.safe_zone_yaw_command_cdeg = flow.forward_yaw;
      mission_ctx.mission_snapshot.safe_zone_yaw_error_cdeg = MissionTask_YawError(
          flow.forward_yaw, mission_ctx.mission_snapshot.yaw_cdeg);
      MissionTask_DebugS6(now);
    }
    if (phase == ARRANGE_COUNTER && (int32_t)(now - mission_ctx.s4_next_debug_tick) >= 0)
    {
      mission_ctx.s4_next_debug_tick = now + ROBOT_VISION_DEBUG_PERIOD_MS;
      (void)DebugUart_Logf("[G02] p=%ld v=%d\r\n", (long)flow.spin_progress, step.turn_rpm);
    }
  }
  if (step.next == phase) return;
  if (step.reset_controller) MissionTask_ResetS6ControllerState();
  if (step.reset_stable) mission_ctx.s6_phase_stable_since = 0U;
  if (step.next == ARRANGE_TIMEOUT)
  {
    mission_ctx.mission_snapshot.fault_flags |= MISSION_FAULT_S4_TIMEOUT;
    MissionTask_EnterState(MISSION_STATE_FAULT, now);
    (void)DebugUart_Log("[G02] SPIN TIMEOUT\r\n");
  }
  else if (step.next == ARRANGE_DONE)
  {
    if (phase == ARRANGE_COUNTER && profile.casualty && !profile.first_green)
      (void)DebugUart_Log("[FINE] RED SKIP L20\r\n");
    MissionTask_CompleteS4FineArrange(now, phase == ARRANGE_OFFSET
        ? MAIXCAM_COMMAND_OFFSET_PUSH_DONE_ACK : mission_ctx.s4_fixed_motion_ack);
  }
  else
  {
    MissionTask_EnterState(stage[step.next], now);
    if (step.next == ARRANGE_COUNTER && profile.first_green)
      (void)DebugUart_Log("[G02] CCW280 v100\r\n");
  }
}

/*02: ROI200 then normal left-first path, except initial-green first02's
   F500/CCW280.12: ACK22 only. */
static void MissionTask_StartS4PushTrial(uint32_t now, bool command02)
{
  /*12 is now handshake-only: keep the normal22 completion and subsequent
     wait, but do not move wheels/frame or start any of42's fixed legs. */
  if (!command02)
  {
    MissionTask_ResetS6ControllerState();
    MissionTask_CompleteS4FineArrange(now, MAIXCAM_COMMAND_LEFT_DONE_ACK);
    return;
  }
  MissionTask_StopWheels();
  if (Actuator_SetFrameLowered() != HAL_OK)
  {
    mission_ctx.mission_snapshot.fault_flags |= MISSION_FAULT_ACTUATOR;
    MissionTask_EnterState(MISSION_STATE_FAULT, now);
    return;
  }
  MissionTask_ResetVisionControllerState();
  MissionTask_ResetS6ControllerState();
  /* The RX02 reference boundary has already fenced old image coordinates.
     Never discard the ROI frame (or its half-frame) following the command. */
  mission_ctx.s4_fixed_motion_ack = MAIXCAM_COMMAND_RIGHT_DONE_ACK;
  mission_ctx.s4_fine_turn_left = true;
  mission_ctx.s4_initial_green02_active = !mission_ctx.s7_target_switch_enabled &&
      mission_ctx.s3_target_select_command == MAIXCAM_COMMAND_SELECT_GREEN &&
      !mission_ctx.s4_initial_green02_used;
  if (mission_ctx.s4_initial_green02_active) mission_ctx.s4_initial_green02_used = true;
  s4_fine_last_coordinate_tick = now;
  mission_ctx.s4_next_debug_tick = now;
  mission_ctx.s6_phase_stable_since = 0U;
  MissionTask_EnterState(MISSION_STATE_S4_FINE42_TRACK, now);
  MissionTask_RecordInitialGreenPush(MAIXCAM_EVENT_ARRANGE_READY);
  MissionTask_UpdateVisionInput(now);
  if (mission_ctx.s4_initial_green02_active)
  {
    (void)DebugUart_Logf("[G02] T%lu F%d/%lu CCW280/100 >12\r\n",
        (unsigned long)ROBOT_S4_PUSH_TRIAL_TRACK_MS, ROBOT_S4_FINE42_FORWARD_RPM,
        (unsigned long)ROBOT_S4_INITIAL_GREEN02_FORWARD_MS);
    return;
  }
  (void)DebugUart_Logf("[PUSH] 02 T%lu L%ld F%d/%lu R%ld L%ld >12\r\n",
      (unsigned long)ROBOT_S4_PUSH_TRIAL_TRACK_MS,
      (long)((mission_ctx.s3_target_select_command == MAIXCAM_COMMAND_SELECT_RED
          ? ROBOT_S4_FINE42_CASUALTY_TURN_CDEG : ROBOT_S4_FINE42_TURN_CDEG) / 100L),
      ROBOT_S4_FINE42_FORWARD_RPM,
      (unsigned long)ROBOT_S4_PUSH_TRIAL_FORWARD_MS,
      (long)(ROBOT_S4_FINE42_COUNTER_TURN_CDEG / 100L),
      mission_ctx.s3_target_select_command == MAIXCAM_COMMAND_SELECT_RED
          ? 0L : (long)(ROBOT_S4_FINE42_FINISH_LEFT_CDEG / 100L));
}

static void MissionTask_StartS4Post22Recovery(uint32_t now)
{
  MissionTask_StopWheels();
  MissionTask_ResetVisionControllerState();
  MissionTask_ResetS6ControllerState();
  MaixCam_ClearObject();
  mission_ctx.mission_snapshot.vision_target_valid = false;
  mission_ctx.s4_post22_left_yaw_cdeg = MissionTask_WrapYaw(
      mission_ctx.mission_snapshot.yaw_cdeg + (int32_t)ROBOT_YAW_LEFT_SIGN *
          ROBOT_S4_POST22_LEFT_CDEG);
  mission_ctx.s4_post22_right_yaw_cdeg = MissionTask_WrapYaw(
      mission_ctx.s4_post22_left_yaw_cdeg - (int32_t)ROBOT_YAW_LEFT_SIGN *
          ROBOT_S4_POST22_RIGHT_CDEG);
  mission_ctx.s6_phase_stable_since = 0U;
  MissionTask_EnterState(MISSION_STATE_S4_POST22_REVERSE, now);
  (void)DebugUart_Logf(
      "[22E2] back=%d/%lu L=%ld R=%ld break=%s\r\n",
      ROBOT_S4_POST22_REVERSE_RPM,
      (unsigned long)ROBOT_S4_POST22_REVERSE_MS,
      (long)mission_ctx.s4_post22_left_yaw_cdeg, (long)mission_ctx.s4_post22_right_yaw_cdeg,
      mission_ctx.s4_post22_search_active ? "RX04/24 OR 03 XY" : "RX04/24");
}

static void MissionTask_ResumeS3FromPost22Search(uint32_t now)
{
  MissionTask_StopWheels();
  MissionTask_ResetVisionControllerState();
  MissionTask_ResetS6ControllerState();
  /* Keep the fresh error and a possibly queued 04 handshake for S3. */
  mission_ctx.s4_post22_search_active = false;
  mission_ctx.s4_e4_protocol_active = false;
  mission_ctx.s3_track_stable_since = 0U;
  mission_ctx.s3_search_sweep_completed = false;
  mission_ctx.s3_search_origin_yaw_cdeg = MissionTask_WrapYaw(mission_ctx.mission_snapshot.yaw_cdeg);
  mission_ctx.vision_search_resume_state = MISSION_STATE_S3_TRACK_GREEN;
  mission_ctx.s3_target_lost_since = 0U;
  mission_ctx.s3_wait_event04_logged = false;
  mission_ctx.s3_next_debug_tick = now;
  MissionTask_EnterState(MISSION_STATE_S3_TRACK_GREEN, now);
  (void)DebugUart_Logf(
      "[22E2] FOUND id=%u x=%d y=%d >03\r\n",
      (unsigned int)mission_ctx.mission_snapshot.target_id,
      (int)mission_ctx.mission_snapshot.vision_x_error_px,
      (int)mission_ctx.mission_snapshot.vision_y_error_px);
}

static bool MissionTask_StartPost22BlackGreenSearch(uint32_t now)
{
  MissionTask_StopWheels();
  if (Actuator_SetFrameLowered() != HAL_OK ||
      Actuator_SetCameraObjectTrackView() != HAL_OK)
  {
    mission_ctx.mission_snapshot.fault_flags |= MISSION_FAULT_ACTUATOR;
    MissionTask_EnterState(MISSION_STATE_FAULT, now);
    return false;
  }
  MaixCam_ClearEvent();
  MaixCam_ClearObject();
  mission_ctx.mission_snapshot.vision_target_valid = false;
  if ((MaixCam_SendCommand(MAIXCAM_COMMAND_SELECT_BLACK_GREEN) != HAL_OK) ||
      (MaixCam_SendCommand(MAIXCAM_COMMAND_SEARCH_TARGET) != HAL_OK))
  {
    mission_ctx.mission_snapshot.fault_flags |= MISSION_FAULT_VISION_TX;
    MissionTask_EnterState(MISSION_STATE_FAULT, now);
    return false;
  }
  if (mission_ctx.s3_target_select_command != MAIXCAM_COMMAND_SELECT_BLACK_GREEN)
  {
    mission_ctx.s51_e3_recovery_count = 0U;
    mission_ctx.s51_second_e3_pending = false;
  }
  mission_ctx.s3_target_select_command = MAIXCAM_COMMAND_SELECT_BLACK_GREEN;
  mission_ctx.s4_post22_search_active = true;
  (void)DebugUart_Log("[22E2] TX51/03 SEARCH B/G\r\n");
  return true;
}

static bool MissionTask_HandlePost22E3(uint32_t now, uint8_t event_code)
{
  if (event_code != MAIXCAM_EVENT_SEARCH_TARGET_LOST ||
      !mission_ctx.s4_post22_search_active ||
      mission_ctx.s3_target_select_command != MAIXCAM_COMMAND_SELECT_BLACK_GREEN)
    return false;

  (void)DebugUart_Log("[22E2] RXE3 >REC03\r\n");
  if (mission_ctx.mission_snapshot.vision_target_valid)
  {
    MissionTask_ResumeS3FromPost22Search(now);
    return true;
  }
  mission_ctx.s4_post22_search_active = false;
  mission_ctx.s3_search_origin_yaw_cdeg = MissionTask_WrapYaw(mission_ctx.mission_snapshot.yaw_cdeg);
  /* No retransmission, no new target initialization or retry-count reset. */
  (void)MissionTask_HandleS3TargetLost(now);
  return true;
}

static void MissionTask_EscalatePost22Search(uint32_t now)
{
  mission_ctx.s4_post22_search_active = false;
  (void)DebugUart_Logf("[22E2] MISS n=%u/%u >CCW360\r\n",
      (unsigned int)mission_ctx.s4_post22_failed_sweeps,
      (unsigned int)ROBOT_S4_POST22_MAX_SWEEPS);
  /* A silent escalation is not an additional RX E3 and must not consume
     the ordinary51 E3 count or send51/03 again. */
  mission_ctx.s51_second_e3_pending = false;
  MissionTask_StartVisionSearchRecovery(
      now, MISSION_STATE_S3_TRACK_GREEN, mission_ctx.mission_snapshot.yaw_cdeg, false);
}

static bool MissionTask_RecoveryScanLeg(RecoveryScan_Profile profile, RecoveryScan_Phase phase,
    uint32_t now, int32_t heading, bool (*turn)(uint32_t, int32_t))
{
  const FlowMotion io = { .stop = MissionTask_StopWheels, .wheels = MissionTask_SetWheelTargets,
    .turn = turn != NULL ? turn : MissionTask_RunS6TurnToHeading };
  return RecoveryFlow_StepScan(&io, profile, phase, now, mission_ctx.mission_snapshot.state_entry_tick, heading) != phase;
}

void MissionTask_RunS4Post22Recovery(uint32_t now)
{
  uint8_t event_code;
  if ((mission_ctx.mission_snapshot.state < MISSION_STATE_S4_POST22_REVERSE) ||
      (mission_ctx.mission_snapshot.state > MISSION_STATE_S4_POST22_TURN_RIGHT))
  {
    return;
  }
  if (MaixCam_TakeEvent(&event_code))
  {
    if (event_code == MAIXCAM_EVENT_SKIP_TO_FINAL_TRACK)
    {
      MissionTask_StopWheels();
      MissionTask_ResetS6ControllerState();
      mission_ctx.s4_post22_search_active = false;
      (void)DebugUart_Log("[22E2] RX24 STOP >UP\r\n");
      MissionTask_SkipToS4FinalTrack(now);
      return;
    }
    if (event_code == MAIXCAM_EVENT_OBJECT_IN_FRAME)
    {
      MissionTask_StopWheels();
      MissionTask_ResetS6ControllerState();
      mission_ctx.s4_post22_search_active = false;
      (void)DebugUart_Log("[22E2] RX04 STOP >TRACK\r\n");
      MissionTask_StartS4TrackCenterFrom04(now);
      return;
    }
    if (MissionTask_HandlePost22E3(now, event_code)) return;
    /* Repeated E2 must not restart a running recovery. */
    if (event_code != MAIXCAM_EVENT_ARRANGE_TARGET_LOST)
    {
      (void)DebugUart_Logf("[22E2] RX%02X, WAIT04/24\r\n",
                           (unsigned int)event_code);
    }
  }
  if (mission_ctx.s4_post22_search_active && mission_ctx.mission_snapshot.vision_target_valid &&
      ((mission_ctx.mission_snapshot.target_id == 5U) ||
       (mission_ctx.mission_snapshot.target_id == 6U)))
  {
    MissionTask_ResumeS3FromPost22Search(now);
    return;
  }
  switch (mission_ctx.mission_snapshot.state)
  {
    case MISSION_STATE_S4_POST22_REVERSE:
      if (!MissionTask_RecoveryScanLeg(RECOVERY_POST22, RECOVERY_SCAN_BACK, now, 0, NULL)) return;
      MissionTask_ResetS6ControllerState();
      mission_ctx.s6_phase_stable_since = 0U;
      MissionTask_EnterState(MISSION_STATE_S4_POST22_TURN_LEFT, now);
      break;
    case MISSION_STATE_S4_POST22_TURN_LEFT:
      if (!MissionTask_RecoveryScanLeg(RECOVERY_POST22, RECOVERY_SCAN_FIRST, now, mission_ctx.s4_post22_left_yaw_cdeg, NULL))
      {
        return;
      }
      MissionTask_ResetS6ControllerState();
      mission_ctx.s6_phase_stable_since = 0U;
      MissionTask_EnterState(MISSION_STATE_S4_POST22_TURN_RIGHT, now);
      break;
    case MISSION_STATE_S4_POST22_TURN_RIGHT:
      if (MissionTask_RecoveryScanLeg(RECOVERY_POST22, RECOVERY_SCAN_SECOND, now, mission_ctx.s4_post22_right_yaw_cdeg, NULL))
      {
        MissionTask_ResetS6ControllerState();
        if (mission_ctx.s4_post22_search_active)
        {
          if (mission_ctx.s4_post22_failed_sweeps < ROBOT_S4_POST22_MAX_SWEEPS)
            ++mission_ctx.s4_post22_failed_sweeps;
          if (mission_ctx.s4_post22_failed_sweeps >= ROBOT_S4_POST22_MAX_SWEEPS)
          {
            MissionTask_EscalatePost22Search(now);
            return;
          }
          (void)DebugUart_Logf("[22E2] MISS n=%u/%u\r\n",
              (unsigned int)mission_ctx.s4_post22_failed_sweeps,
              (unsigned int)ROBOT_S4_POST22_MAX_SWEEPS);
        }
        MissionTask_EnterState(MISSION_STATE_S4_WAIT_FINAL_READY, now);
        (void)DebugUart_Log(mission_ctx.s4_post22_search_active
            ? "[22E2] REC OK DOWN WAIT03\r\n"
            : "[22E2] REC OK DOWN WAIT04/24/E2\r\n");
      }
      break;
    default:
      break;
  }
}

void MissionTask_RunS4Arrange(uint32_t now)
{
  uint32_t elapsed = (uint32_t)(now - mission_ctx.mission_snapshot.state_entry_tick);
  uint8_t event_code;
  bool xy_in_tolerance;
  bool camera_already_in_final_mode;

  switch (mission_ctx.mission_snapshot.state)
  {
    case MISSION_STATE_S4_TRACK_CENTER:
      if (MaixCam_TakeEvent(&event_code))
      {
        if (event_code == MAIXCAM_EVENT_CENTER_TARGET_LOST)
        {
          MissionTask_StartS4E4Recovery(now);
          return;
        }
        if (event_code == MAIXCAM_EVENT_CENTER_REACHED)
        {
          MissionTask_AckS4Center(now);
          return;
        }
        (void)DebugUart_Logf(
            "[04] RX%02X, WAIT14\r\n",
            (unsigned int)event_code);
      }
      if (!mission_ctx.mission_snapshot.vision_target_valid)
      {
        MissionTask_StopWheels();
        if (mission_ctx.s4_center_target_lost_since == 0U)
        {
          mission_ctx.s4_center_target_lost_since = now;
        }
        if (!mission_ctx.s4_e4_protocol_active && !mission_ctx.s4_center_search_sweep_completed &&
            ((uint32_t)(now - mission_ctx.s4_center_target_lost_since) >=
             ROBOT_S3_GREEN_WAIT_MS))
        {
          (void)DebugUart_Logf(
              "[04] LOST %lu, SAME RECOVERY AS 03\r\n",
              (unsigned long)ROBOT_S3_GREEN_WAIT_MS);
          MissionTask_StartVisionSearchRecovery(
              now, MISSION_STATE_S4_TRACK_CENTER, mission_ctx.mission_snapshot.yaw_cdeg,
              false);
          return;
        }
        (void)MissionTask_CommandS4VisionTracking(now, &xy_in_tolerance);
        break;
      }
      else
      {
        mission_ctx.s4_center_target_lost_since = 0U;
        mission_ctx.s4_center_search_sweep_completed = false;
        mission_ctx.s4_green_e4_spin_exhausted = false;
      }
      if (elapsed >= ROBOT_S4_TRACK_TIMEOUT_MS)
      {
        mission_ctx.mission_snapshot.fault_flags |= MISSION_FAULT_S4_TIMEOUT;
        MissionTask_EnterState(MISSION_STATE_FAULT, now);
        return;
      }
      (void)MissionTask_CommandS4VisionTracking(now, &xy_in_tolerance);
      break;

    case MISSION_STATE_S4_CENTER_FOLLOW_THROUGH:
      /* Legacy state: never drive; acknowledge on the next task tick. */
      MissionTask_AckS4Center(now);
      break;

    case MISSION_STATE_S4_WAIT_ARRANGE_READY:
      MissionTask_StopWheels();
      if (MaixCam_TakeEvent(&event_code))
      {
        if (event_code == MAIXCAM_EVENT_SKIP_TO_FINAL_TRACK)
        {
          MissionTask_SkipToS4FinalTrack(now);
          return;
        }
        else if (!mission_ctx.s4_direct_capture_wait24 && event_code == MAIXCAM_EVENT_ARRANGE_READY)
        {
          MissionTask_StartS4TrackState(MISSION_STATE_S4_TRACK_RIGHT_BLOCK, now);
          (void)DebugUart_Log("[04] RX02 >PUSH-R\r\n");
          return;
        }
        else if (!mission_ctx.s4_direct_capture_wait24 && event_code == MAIXCAM_EVENT_LEFT_TARGET_READY &&
                 mission_ctx.s4_fine_waiting_decision)
        {
          MissionTask_StartS4TrackState(MISSION_STATE_S4_TRACK_LEFT_BLOCK, now);
          (void)DebugUart_Log("[FINE] RX12 >ACK22\r\n");
          return;
        }
        else if (event_code == MAIXCAM_EVENT_CENTER_TARGET_LOST)
        {
          /* MaixCAM owns the reliable-target observation after14. Even
             fresh coordinates here do not override its E4 decision. */
          (void)DebugUart_Log(
              "[04] RXE4 IN WAIT02/24 >REC\r\n");
          MissionTask_StartS4E4Recovery(now);
          return;
        }
        else if (event_code == MAIXCAM_EVENT_CENTER_REACHED &&
                 (mission_ctx.s4_e4_protocol_active || mission_ctx.s4_fine_waiting_decision))
        {
          /* An actual14 following04/fine-arrange is still acknowledged here.
             A44 return uses03 instead and fences pre44 arrival decisions. */
          MissionTask_AckS4Center(now);
          return;
        }
        (void)DebugUart_Logf("[04] RX%02X WAIT%s\r\n", (unsigned int)event_code,
            mission_ctx.s4_direct_capture_wait24 ? "24/E4" : "02/24/E4");
      }
      if (elapsed >= ROBOT_S4_HANDSHAKE_TIMEOUT_MS)
      {
        mission_ctx.mission_snapshot.fault_flags |= MISSION_FAULT_S4_TIMEOUT;
        MissionTask_EnterState(MISSION_STATE_FAULT, now);
        (void)DebugUart_Log("[S4] WAIT AFTER TX14 TIMEOUT\r\n");
      }
      break;

    case MISSION_STATE_S4_TRACK_RIGHT_BLOCK:
      if (MissionTask_TryS4ArrangeEvent(
              now, MISSION_STATE_S4_TRACK_RIGHT_BLOCK))
      {
        return;
      }
      MissionTask_StartS4PushTrial(now, true);
      break;

    case MISSION_STATE_S4_REVERSE_RIGHT_BLOCK:
      if (MissionTask_TryS4ArrangeEvent(
              now, MISSION_STATE_S4_TRACK_RIGHT_BLOCK))
      {
        return;
      }
      if (elapsed < MissionTask_S4ReverseDuration(true))
      {
        MissionTask_CommandS4CornerReverse(now, true);
        return;
      }
      MissionTask_StopWheels();
      MaixCam_ClearEvent();
      MaixCam_ClearObject();
      MissionTask_EnterState(MISSION_STATE_S4_WAIT_LEFT_TARGET, now);
      if (MaixCam_SendCommand(MAIXCAM_COMMAND_RIGHT_DONE_ACK) != HAL_OK)
      {
        mission_ctx.mission_snapshot.fault_flags |= MISSION_FAULT_VISION_TX;
        MissionTask_EnterState(MISSION_STATE_FAULT, now);
        return;
      }
      (void)DebugUart_Log(
          "[04] TX12 WAIT12/24\r\n");
      break;

    case MISSION_STATE_S4_WAIT_LEFT_TARGET:
      if (MaixCam_TakeEvent(&event_code))
      {
        if (event_code == MAIXCAM_EVENT_SKIP_TO_FINAL_TRACK)
        {
          MissionTask_SkipToS4FinalTrack(now);
          return;
        }
        else if (event_code == MAIXCAM_EVENT_LEFT_TARGET_READY)
        {
          MissionTask_StartS4TrackState(
              MISSION_STATE_S4_TRACK_LEFT_BLOCK, now);
          (void)DebugUart_Log(
              "[04] RX12 >ACK22\r\n");
          return;
        }
        (void)DebugUart_Logf(
            "[04] RX%02X, WAIT12 OR 24\r\n",
            (unsigned int)event_code);
      }
      if (elapsed >= ROBOT_S4_HANDSHAKE_TIMEOUT_MS)
      {
        mission_ctx.mission_snapshot.fault_flags |= MISSION_FAULT_S4_TIMEOUT;
        MissionTask_EnterState(MISSION_STATE_FAULT, now);
      }
      break;

    case MISSION_STATE_S4_TRACK_LEFT_BLOCK:
      MissionTask_StopWheels();
      /* No timed motion remains. Keep pending next-stage events for the
         normal post22 wait instead of consuming them while skipping12. */
      (void)MaixCam_DropEventIf(MAIXCAM_EVENT_LEFT_TARGET_READY);
      if (MaixCam_PeekEvent(&event_code) &&
          event_code == MAIXCAM_EVENT_SKIP_TO_FINAL_TRACK)
      {
        (void)MaixCam_TakeEvent(&event_code);
        MissionTask_SkipToS4FinalTrack(now);
        return;
      }
      MissionTask_StartS4PushTrial(now, false);
      break;

    case MISSION_STATE_S4_REVERSE_LEFT_BLOCK:
      if (MissionTask_TryS4ArrangeEvent(
              now, MISSION_STATE_S4_TRACK_LEFT_BLOCK))
      {
        return;
      }
      if (elapsed < MissionTask_S4ReverseDuration(false))
      {
        MissionTask_CommandS4CornerReverse(now, false);
        return;
      }
      MissionTask_StopWheels();
      MaixCam_ClearEvent();
      MaixCam_ClearObject();
      mission_ctx.mission_snapshot.vision_target_valid = false;
      MissionTask_EnterState(MISSION_STATE_S4_WAIT_FINAL_READY, now);
      mission_ctx.s4_post22_search_active = false;
      mission_ctx.s4_post22_failed_sweeps = 0U;
      if (MaixCam_SendCommand(MAIXCAM_COMMAND_LEFT_DONE_ACK) != HAL_OK)
      {
        mission_ctx.mission_snapshot.fault_flags |= MISSION_FAULT_VISION_TX;
        MissionTask_EnterState(MISSION_STATE_FAULT, now);
        return;
      }
      (void)DebugUart_Log(
          "[12] OK TX22 DOWN WAIT04/14/24/E4/E2\r\n");
      break;

    case MISSION_STATE_S4_WAIT_FINAL_READY:
      MissionTask_StopWheels();
      if (MaixCam_TakeEvent(&event_code))
      {
        if (event_code == MAIXCAM_EVENT_OBJECT_IN_FRAME)
        {
          mission_ctx.s4_post22_search_active = false;
          MissionTask_StartS4TrackCenterFrom04(now);
          return;
        }
        if (event_code == MAIXCAM_EVENT_SKIP_TO_FINAL_TRACK)
        {
          mission_ctx.s4_post22_search_active = false;
          MissionTask_SkipToS4FinalTrack(now);
          return;
        }
        if (MissionTask_HandlePost22E3(now, event_code)) return;
        if (!mission_ctx.s4_post22_search_active)
        {
          if (event_code == MAIXCAM_EVENT_CENTER_REACHED)
          {
            MissionTask_AckS4Center(now);
            return;
          }
          if (event_code == MAIXCAM_EVENT_CENTER_TARGET_LOST)
          {
            MissionTask_StartS4E4Recovery(now);
            return;
          }
          if (event_code == MAIXCAM_EVENT_ARRANGE_TARGET_LOST)
          {
            if (mission_ctx.s7_target_switch_enabled &&
                !MissionTask_StartPost22BlackGreenSearch(now))
            {
              return;
            }
            MissionTask_StartS4Post22Recovery(now);
            return;
          }
        }
        (void)DebugUart_Logf(
            "[04] RX%02X WAIT04/14/24/E4/E2\r\n",
            (unsigned int)event_code);
      }
      if (mission_ctx.s4_post22_search_active && mission_ctx.mission_snapshot.vision_target_valid &&
          ((mission_ctx.mission_snapshot.target_id == 5U) ||
           (mission_ctx.mission_snapshot.target_id == 6U)))
      {
        MissionTask_ResumeS3FromPost22Search(now);
        return;
      }
      if (elapsed >= ROBOT_S4_HANDSHAKE_TIMEOUT_MS)
      {
        if (mission_ctx.s4_post22_search_active || !mission_ctx.s7_target_switch_enabled)
        {
          MissionTask_StartS4Post22Recovery(now);
          return;
        }
        mission_ctx.mission_snapshot.fault_flags |= MISSION_FAULT_S4_TIMEOUT;
        MissionTask_EnterState(MISSION_STATE_FAULT, now);
        (void)DebugUart_Log("[04] WAIT04/14/24/E4/E2 TMO\r\n");
      }
      break;

    case MISSION_STATE_S4_ARRANGE_RECOVERY_REVERSE:
      if (MissionTask_TryResumeS4ArrangeRecovery(now))
      {
        return;
      }
      if (!MissionTask_RecoveryScanLeg(RECOVERY_ARRANGE_LOSS, RECOVERY_SCAN_BACK, now, 0, NULL)) return;
      MissionTask_ResetS6ControllerState();
      mission_ctx.s6_phase_stable_since = 0U;
      mission_ctx.s4_next_debug_tick = now;
      MissionTask_EnterState(
          MISSION_STATE_S4_ARRANGE_RECOVERY_TURN_LEFT, now);
      (void)DebugUart_Logf(
          "[E2] BACK OK, LEFT45 goal=%ld\r\n",
          (long)mission_ctx.s4_arrange_recovery_left_yaw_cdeg);
      break;

    case MISSION_STATE_S4_ARRANGE_RECOVERY_TURN_LEFT:
      if (MissionTask_TryResumeS4ArrangeRecovery(now))
      {
        return;
      }
      if (MissionTask_RecoveryScanLeg(RECOVERY_ARRANGE_LOSS, RECOVERY_SCAN_FIRST, now, mission_ctx.s4_arrange_recovery_left_yaw_cdeg, NULL))
      {
        MissionTask_ResetS6ControllerState();
        mission_ctx.s6_phase_stable_since = 0U;
        mission_ctx.s4_next_debug_tick = now;
        MissionTask_EnterState(
            MISSION_STATE_S4_ARRANGE_RECOVERY_TURN_BACK, now);
        (void)DebugUart_Logf(
            "[E2] L45 OK, RIGHT90 goal=%ld\r\n",
            (long)mission_ctx.s4_arrange_recovery_right_yaw_cdeg);
      }
      break;

    case MISSION_STATE_S4_ARRANGE_RECOVERY_TURN_BACK:
      if (MissionTask_TryResumeS4ArrangeRecovery(now))
      {
        return;
      }
      if (!MissionTask_RecoveryScanLeg(RECOVERY_ARRANGE_LOSS, RECOVERY_SCAN_SECOND, now, mission_ctx.s4_arrange_recovery_right_yaw_cdeg, NULL))
      {
        return;
      }
      (void)DebugUart_Log(
          "[E2] SCAN OK LOST >REC\r\n");
      MissionTask_StartS4ArrangeRecovery(
          now, s4_arrange_recovery_resume_state);
      break;

    case MISSION_STATE_S4_RAISE_FRAME:
      /* Keep the claw closed here. Preserve a one-shot34 that arrives before
         the next tracking dispatch; the capture flow opens it if needed. */
      if (mission_ctx.s4_final_mode_already_active &&
          MaixCam_TakeEvent(&event_code))
      {
        if (event_code == MAIXCAM_EVENT_FINAL_OBJECT_IN_FRAME)
        {
          if (!mission_ctx.s4_final_event34_pending)
          {
            (void)DebugUart_Log(
                "[24] RX34 LATCH\r\n");
          }
          mission_ctx.s4_final_event34_pending = true;
        }
        else
        {
          (void)DebugUart_Logf(
              "[24] RX%02X WAIT34\r\n",
              (unsigned int)event_code);
        }
      }
      camera_already_in_final_mode = mission_ctx.s4_final_mode_already_active;
      mission_ctx.s4_final_mode_already_active = false;
      if (camera_already_in_final_mode && mission_ctx.s4_final_event34_pending)
      {
        mission_ctx.s4_final_event34_pending = false;
        (void)DebugUart_Log(
            "[24] APPLY34\r\n");
        MissionTask_CaptureS4From34(now);
        return;
      }
      mission_ctx.s4_final_event34_pending = false;
      if (camera_already_in_final_mode)
      {
        /* Keep new24 coordinates received during the capture handoff. */
        MissionTask_ResetVisionControllerState();
        mission_ctx.s4_next_debug_tick = now;
        MissionTask_EnterState(MISSION_STATE_S4_TRACK_FINAL_BLOCK, now);
        mission_ctx.s4_final_missing_since =
            mission_ctx.mission_snapshot.vision_target_valid ? 0U : now;
      }
      else
      {
        MaixCam_ClearEvent();
        MissionTask_StartS4TrackState(
            MISSION_STATE_S4_TRACK_FINAL_BLOCK, now);
        mission_ctx.s4_final_missing_since = now;
      }
      if (!camera_already_in_final_mode &&
          (MaixCam_SendCommand(MAIXCAM_COMMAND_FINAL_CAPTURE) != HAL_OK))
      {
        mission_ctx.mission_snapshot.fault_flags |= MISSION_FAULT_VISION_TX;
        MissionTask_EnterState(MISSION_STATE_FAULT, now);
        return;
      }
      if (camera_already_in_final_mode)
      {
        (void)DebugUart_Log(
            "[24] TRACK WAIT NEAR TO OPEN\r\n");
      }
      else
      {
        (void)DebugUart_Log(
            "[24] TX24 TRACK\r\n");
      }
      break;

    case MISSION_STATE_S4_TRACK_FINAL_BLOCK:
    {
      if (MaixCam_TakeEvent(&event_code))
      {
        if (event_code == MAIXCAM_EVENT_FINAL_OBJECT_IN_FRAME)
        {
          MissionTask_CaptureS4From34(now);
          return;
        }
        (void)DebugUart_Logf(
            "[24] RX%02X, WAIT34\r\n",
            (unsigned int)event_code);
      }
      if (MissionTask_HoldS4ClawOpening(now)) return;
      const CaptureTrackStatus status = CaptureFlow_Track(now, mission_ctx.mission_snapshot.state_entry_tick,
          mission_ctx.mission_snapshot.vision_target_valid, &mission_ctx.s4_final_missing_since,
          MissionTask_CommandS4VisionTracking);
      if (status == CAPTURE_TRACK_TIMEOUT)
      {
        MissionTask_HandleS4FinalTrackTimeout(now);
        return;
      }
      if (status == CAPTURE_TRACK_LOST)
      {
        MissionTask_StartS4FinalRecovery(now);
        return;
      }
      break;
    }

    case MISSION_STATE_S4_FINAL_TIMEOUT_BACKOFF:
      if (CaptureFlow_BackoffDone(now, mission_ctx.mission_snapshot.state_entry_tick))
        MissionTask_RetryS3AfterS4(now, false);
      return;

    case MISSION_STATE_S4_FINAL_RECOVERY_REVERSE:
      if (MissionTask_TryResumeS4FinalRecovery(now))
      {
        return;
      }
      if (!MissionTask_RecoveryScanLeg(RECOVERY_CAPTURE_LOSS, RECOVERY_SCAN_BACK, now, 0, NULL)) return;
      mission_ctx.s4_final_turn_stable_since = 0U;
      mission_ctx.s4_next_debug_tick = now;
      MissionTask_EnterState(
          MISSION_STATE_S4_FINAL_RECOVERY_TURN_270, now);
      (void)DebugUart_Logf(
          "[EE] BACK OK >L SIDE1 r=%ld\r\n",
          (long)mission_ctx.s4_final_first_yaw_cdeg);
      break;

    case MISSION_STATE_S4_FINAL_RECOVERY_TURN_270:
      if (MissionTask_TryResumeS4FinalRecovery(now))
      {
        return;
      }
      if (MissionTask_RecoveryScanLeg(RECOVERY_CAPTURE_LOSS, RECOVERY_SCAN_FIRST, now, mission_ctx.s4_final_first_yaw_cdeg, MissionTask_RunS4ForcedLeftTurn))
      {
        MissionTask_ResetS6ControllerState();
        mission_ctx.s6_phase_stable_since = 0U;
        mission_ctx.s6_next_debug_tick = now;
        MissionTask_EnterState(
            MISSION_STATE_S4_FINAL_RECOVERY_TURN_90, now);
        (void)DebugUart_Logf(
            "[EE] SIDE1 OK >SIDE2 r=%ld\r\n",
            (long)mission_ctx.s4_final_second_yaw_cdeg);
      }
      break;

    case MISSION_STATE_S4_FINAL_RECOVERY_TURN_90:
      if (MissionTask_TryResumeS4FinalRecovery(now))
      {
        return;
      }
      if (MissionTask_RecoveryScanLeg(RECOVERY_CAPTURE_LOSS, RECOVERY_SCAN_SECOND, now, mission_ctx.s4_final_second_yaw_cdeg, NULL))
      {
        MissionTask_ResetS6ControllerState();
        MaixCam_ClearObject();
        mission_ctx.mission_snapshot.vision_target_valid = false;
        MissionTask_ResetVisionPid();
        if (MaixCam_SendCommand(MAIXCAM_COMMAND_FINAL_RECOVERY_DONE) != HAL_OK)
        {
          mission_ctx.mission_snapshot.fault_flags |= MISSION_FAULT_VISION_TX;
          MissionTask_EnterState(MISSION_STATE_FAULT, now);
          return;
        }
        (void)DebugUart_Log(
            "[EE] REC OK TXF1\r\n");
        mission_ctx.s4_final_missing_since = now;
        mission_ctx.s4_next_debug_tick = now;
        MissionTask_EnterState(
            MISSION_STATE_S4_TRACK_FINAL_BLOCK, now);
        (void)DebugUart_Log(
            "[EE] SIDE2 OK >24 WAIT XY\r\n");
      }
      break;

    case MISSION_STATE_S4_FINAL_CENTER_OBJECT:
      MissionTask_FinishS4Capture(now);
      break;

    case MISSION_STATE_S4_FINAL_LOWER_FRAME:
      /* Reserved old lower/reverse state. */
      MissionTask_StopWheels();
      break;

    case MISSION_STATE_S4_FINAL_TURN_SAFE:
      mission_ctx.mission_snapshot.safe_zone_fixed_heading_active = true;
      if (MissionTask_RunS6TurnToHeading(now, mission_ctx.mission_snapshot.safe_zone_yaw_target_cdeg))
      {
        MissionTask_StopWheels();
        MissionTask_ResetS6ControllerState();
        const bool regrip_requested = MaixCam_TakeCasualtyRegripRequest() &&
            MissionTask_GetSelectedObjectId() == 4U;
        /* The shared turn has a soft timeout. Never push in an unaligned
           direction merely because that deadline elapsed. */
        if (regrip_requested &&
            MissionTask_Abs32(MissionTask_YawError(mission_ctx.mission_snapshot.safe_zone_yaw_target_cdeg,
                mission_ctx.mission_snapshot.yaw_cdeg)) <= ROBOT_S5_ARRANGE_TURN_RELEASE_CDEG)
        {
          if (Actuator_SetClawAngle(ROBOT_CLAW_MATERIAL_OPEN_DEG) != HAL_OK)
          {
            mission_ctx.mission_snapshot.fault_flags |= MISSION_FAULT_ACTUATOR;
            MissionTask_EnterState(MISSION_STATE_FAULT, now);
            return;
          }
          MissionTask_EnterState(MISSION_STATE_S4_CASUALTY_REGRIP, now);
          mission_ctx.mission_snapshot.vision_forward_rpm = ROBOT_S4_CASUALTY_REGRIP_RPM;
          MissionTask_SetWheelTargets(ROBOT_S4_CASUALTY_REGRIP_RPM,
                                      ROBOT_S4_CASUALTY_REGRIP_RPM);
          (void)DebugUart_Logf("[54] CLAW=%u FWD=%d/%lu\r\n",
              (unsigned int)ROBOT_CLAW_MATERIAL_OPEN_DEG, ROBOT_S4_CASUALTY_REGRIP_RPM,
              (unsigned long)ROBOT_S4_CASUALTY_REGRIP_MS);
        }
        else
        {
          if (regrip_requested) (void)DebugUart_Log("[54] SKIP NOT ALIGNED\r\n");
          MissionTask_EnterS5(now);
        }
      }
      break;

    case MISSION_STATE_S4_CASUALTY_REGRIP:
    {
      const CaptureFlow_Result result = CaptureFlow_EndRegrip(now,
          mission_ctx.mission_snapshot.state_entry_tick, MissionTask_StopWheels, Actuator_SetClawClosed);
      if (result == CAPTURE_RUNNING) break;
      mission_ctx.mission_snapshot.vision_forward_rpm = 0;
      if (result == CAPTURE_ACTUATOR_FAULT)
      {
        mission_ctx.mission_snapshot.fault_flags |= MISSION_FAULT_ACTUATOR;
        MissionTask_EnterState(MISSION_STATE_FAULT, now);
        return;
      }
      MissionTask_EnterS5(now);
      break;
    }

    default:
      break;
  }
}
