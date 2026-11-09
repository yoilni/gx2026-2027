#include "mission_capture.h"
#include "mission_internal.h"
#include "mission_state.h"
#include "mission_safety.h"
#include "mission_vision.h"
#include "mission_load.h"
#include "mission_supplement.h"
#include "mission_safe_zone.h"

#include "actuator_task.h"
#include "capture_flow.h"
#include "debug_uart_task.h"
#include "maixcam_task.h"
#include "robot_config.h"
#include "stm32f4xx_hal.h"

uint8_t MissionTask_GetSelectedObjectId(void)
{
  switch (mission_ctx.s3_target_select_command)
  {
    case MAIXCAM_COMMAND_SELECT_BLUE: return 3U;
    case MAIXCAM_COMMAND_SELECT_RED: return 4U;
    case MAIXCAM_COMMAND_SELECT_GREEN: return 5U;
    case MAIXCAM_COMMAND_SELECT_BLACK: return 6U;
    case MAIXCAM_COMMAND_SELECT_BLACK_GREEN:
      return (mission_ctx.s51_active_object_id == 5U || mission_ctx.s51_active_object_id == 6U)
          ? mission_ctx.s51_active_object_id : 0U;
    default: return 0U;
  }
}

uint8_t MissionTask_GetCarriedObjectId(void)
{
  return mission_ctx.s6_carried_object_id != 0U ? mission_ctx.s6_carried_object_id
                                   : MissionTask_GetSelectedObjectId();
}

void MissionTask_SkipToS4FinalTrack(uint32_t now)
{
  const bool casualty = MissionTask_GetSelectedObjectId() == 4U;
  const uint16_t open_deg = casualty
      ? ROBOT_CLAW_CASUALTY_CAPTURE_OPEN_DEG : ROBOT_CLAW_MATERIAL_CAPTURE_OPEN_DEG;
  mission_ctx.s4_arrange_cycle_count = 0U;
  MissionTask_StopWheels();
  MaixCam_ClearObject();
  MissionTask_ResetVisionPid();
  mission_ctx.mission_snapshot.vision_target_valid = false;
  if (CaptureFlow_Open(casualty ? Actuator_SetCameraFinalTrackView : Actuator_SetCameraWideView,
      Actuator_SetClawAngle, open_deg) != HAL_OK)
  {
    mission_ctx.mission_snapshot.fault_flags |= MISSION_FAULT_ACTUATOR;
    MissionTask_EnterState(MISSION_STATE_FAULT, now);
    return;
  }

  mission_ctx.s4_final_mode_already_active = true;
  mission_ctx.s4_final_event34_pending = false;
  MissionTask_EnterState(MISSION_STATE_S4_RAISE_FRAME, now);
  (void)DebugUart_Logf("[24] RX24 CAM=%u CLAW=%u >TRACK\r\n",
      (unsigned int)(casualty ? ROBOT_CAMERA_FINAL_TRACK_ANGLE_DEG
                             : ROBOT_CAMERA_WIDE_ANGLE_DEG),
      (unsigned int)open_deg);
}

void MissionTask_FinishS4Capture(uint32_t now)
{
  mission_ctx.mission_snapshot.vision_forward_rpm = 0;
  const CaptureFlow_Result result = CaptureFlow_Close(MissionTask_StopWheels,
      Actuator_ClawCloseDeferred, Actuator_SetClawClosed);
  if (result == CAPTURE_RUNNING)
  {
    MissionTask_EnterState(MISSION_STATE_S4_FINAL_CENTER_OBJECT, now);
    return;
  }
  if (result == CAPTURE_ACTUATOR_FAULT)
  {
    mission_ctx.mission_snapshot.fault_flags |= MISSION_FAULT_ACTUATOR;
    MissionTask_EnterState(MISSION_STATE_FAULT, now);
    return;
  }
  const uint8_t captured_id = MissionTask_GetSelectedObjectId();
  /* A34 has no ID. If51 captured directly in the ROI without publishing
     coordinates, the active visual selection still guarantees green/black. */
  const bool supply_capture = captured_id == 5U || captured_id == 6U ||
      (captured_id == 0U && mission_ctx.s3_target_select_command == MAIXCAM_COMMAND_SELECT_BLACK_GREEN);
  if (mission_ctx.s7_target_switch_enabled && supply_capture &&
      !mission_ctx.supplement_first_supply_ack_sent && !mission_ctx.supplement_budget_started &&
      !mission_ctx.supplement_budget_exhausted && !mission_ctx.supplement_capture_active &&
      mission_ctx.supplement_capture_count == 0U && !mission_ctx.s5_recheck_resume_safe_tracking)
  {
    const uint32_t capture_tick = HAL_GetTick();
    mission_ctx.s6_carried_object_id = captured_id;
    mission_ctx.s5_load_class_confirmed = false;
    (void)MaixCam_FenceSearchReference();
    if (MaixCam_SendCommand(MAIXCAM_COMMAND_FIRST_SUPPLY_CAPTURE_DONE) != HAL_OK)
    {
      mission_ctx.mission_snapshot.fault_flags |= MISSION_FAULT_VISION_TX;
      MissionTask_EnterState(MISSION_STATE_FAULT, HAL_GetTick());
      return;
    }
    mission_ctx.supplement_first_supply_ack_sent = true;
    mission_ctx.s3_target_select_command = MAIXCAM_COMMAND_SELECT_BLACK_GREEN;
    (void)DebugUart_Logf("[34] FIRST SUPPLY id=%u TX34 >SUPP03\r\n", (unsigned int)captured_id);
    MissionTask_StartSupplement(HAL_GetTick(), capture_tick, false);
    return;
  }
  MissionTask_ResetS6ControllerState();
  mission_ctx.s6_phase_stable_since = 0U;
  MissionTask_EnterState(MISSION_STATE_S4_FINAL_TURN_SAFE, now);
  (void)DebugUart_Logf("[34] CLAW=%u TURN SAFE goal=%ld\r\n",
      (unsigned int)ROBOT_CLAW_CLOSED_DEG, (long)mission_ctx.mission_snapshot.safe_zone_yaw_target_cdeg);
}

void MissionTask_CaptureS4From34(uint32_t now)
{
  /* Both cargo classes stop and close at the visual capture point. */
  MissionTask_ResetVisionPid();
  MaixCam_ClearObject();
  mission_ctx.mission_snapshot.vision_target_valid = false;
  mission_ctx.s4_final_missing_since = 0U;
  mission_ctx.s4_final_event34_pending = false;
  MissionTask_FinishS4Capture(now);
}

void MissionTask_StartS4FinalRecovery(uint32_t now)
{
  if (MissionTask_ActiveRecoveryLossEvent() != 0U) return;
  if (mission_ctx.s7_target_switch_enabled && mission_ctx.s4_final_recovery_count >= 1U)
  {
    MissionTask_StopWheels();
    MaixCam_ClearEvent();
    MaixCam_ClearObject();
    mission_ctx.mission_snapshot.vision_target_valid = false;
    (void)DebugUart_Log(
        "[EE] LOST2 >51/03\r\n");
    (void)MissionTask_StartS3Align(now, MAIXCAM_COMMAND_SELECT_BLACK_GREEN);
    return;
  }
  if (!mission_ctx.s7_target_switch_enabled && mission_ctx.s4_final_recovery_count >= 1U)
  {
    (void)DebugUart_Log(
        "[EE] GREEN LOST2 KEEP21 >REC24\r\n");
  }
  MissionTask_StopWheels();
  MissionTask_ResetVisionPid();
  mission_ctx.s4_final_turn_stable_since = 0U;
  mission_ctx.s4_final_first_yaw_cdeg = MissionTask_WrapYaw(
      mission_ctx.mission_snapshot.safe_zone_yaw_target_cdeg +
      ROBOT_S4_FINAL_RECOVERY_FIRST_SAFE_OFFSET_CDEG);
  mission_ctx.s4_final_second_yaw_cdeg = MissionTask_WrapYaw(
      mission_ctx.mission_snapshot.safe_zone_yaw_target_cdeg +
      ROBOT_S4_FINAL_RECOVERY_SECOND_SAFE_OFFSET_CDEG);
  if (mission_ctx.s4_final_recovery_count < UINT8_MAX)
  {
    ++mission_ctx.s4_final_recovery_count;
  }
  MissionTask_EnterState(
      MISSION_STATE_S4_FINAL_RECOVERY_REVERSE, now);
  (void)DebugUart_Logf(
      "[EE] n=%u lost=%lu back=%d/%lu\r\n",
      (unsigned int)mission_ctx.s4_final_recovery_count,
      (unsigned long)ROBOT_S4_FINAL_MISSING_TIMEOUT_MS,
      ROBOT_S4_FINAL_RECOVERY_REVERSE_RPM,
      (unsigned long)ROBOT_S4_FINAL_RECOVERY_REVERSE_MS);
  (void)DebugUart_Logf(
      "[EE] zone=%u team=%u safe=%ld L=%ld R=%ld\r\n",
      (unsigned int)mission_ctx.mission_snapshot.start_zone,
      (unsigned int)mission_ctx.mission_snapshot.team,
      (long)MissionTask_WrapYaw(mission_ctx.mission_snapshot.safe_zone_yaw_target_cdeg -
                               mission_ctx.mission_snapshot.yaw_start_cdeg),
      (long)MissionTask_WrapYaw(mission_ctx.s4_final_first_yaw_cdeg - mission_ctx.mission_snapshot.yaw_start_cdeg),
      (long)MissionTask_WrapYaw(mission_ctx.s4_final_second_yaw_cdeg - mission_ctx.mission_snapshot.yaw_start_cdeg));
}

bool MissionTask_TryResumeS4FinalRecovery(uint32_t now)
{
  uint8_t event_code;

  if (MaixCam_TakeEvent(&event_code))
  {
    if (event_code == MAIXCAM_EVENT_FINAL_OBJECT_IN_FRAME)
    {
      MissionTask_CaptureS4From34(now);
      return true;
    }
    (void)DebugUart_Logf(
        "[EE] RX%02X, WAIT34 OR COORDS\r\n",
        (unsigned int)event_code);
  }

  if (!mission_ctx.mission_snapshot.vision_target_valid)
  {
    return false;
  }

  MissionTask_StopWheels();
  MissionTask_ResetVisionPid();
  mission_ctx.s4_final_missing_since = 0U;
  mission_ctx.s4_next_debug_tick = now;
  MissionTask_EnterState(MISSION_STATE_S4_TRACK_FINAL_BLOCK, now);
  (void)DebugUart_Logf(
      "[EE] FOUND x=%d y=%d >24\r\n",
      (int)mission_ctx.mission_snapshot.vision_x_error_px,
      (int)mission_ctx.mission_snapshot.vision_y_error_px);
  return true;
}

bool MissionTask_RunS4ForcedLeftTurn(uint32_t now,
                                             int32_t target_yaw_cdeg)
{
  int32_t yaw_error = MissionTask_YawError(
      target_yaw_cdeg, mission_ctx.mission_snapshot.yaw_cdeg);
  int32_t absolute_error = MissionTask_Abs32(yaw_error);
  int16_t turn_rpm;

  mission_ctx.mission_snapshot.safe_zone_yaw_command_cdeg = target_yaw_cdeg;
  mission_ctx.mission_snapshot.safe_zone_yaw_error_cdeg = yaw_error;
  mission_ctx.mission_snapshot.vision_forward_rpm = 0;

  if (MissionTask_HeadingTurnExpired(now, target_yaw_cdeg)) return true;

  if (absolute_error <= ROBOT_S6_TURN_TOLERANCE_CDEG)
  {
    MissionTask_StopWheels();
    mission_ctx.mission_snapshot.vision_turn_rpm = 0;
    if (mission_ctx.s4_final_turn_stable_since == 0U)
    {
      mission_ctx.s4_final_turn_stable_since = now;
    }
    else if ((uint32_t)(now - mission_ctx.s4_final_turn_stable_since) >=
             ROBOT_S6_TURN_STABLE_MS)
    {
      return true;
    }
  }
  else
  {
    mission_ctx.s4_final_turn_stable_since = 0U;
    turn_rpm = absolute_error >
                       ROBOT_S4_FINAL_RECOVERY_TURN_SLOW_CDEG
                   ? ROBOT_S4_FINAL_RECOVERY_TURN_FAST_RPM
                   : ROBOT_S4_FINAL_RECOVERY_TURN_SLOW_RPM;
    mission_ctx.mission_snapshot.vision_turn_rpm = -turn_rpm;
    MissionTask_SetWheelTargets(-turn_rpm, turn_rpm);
  }

  if ((int32_t)(now - mission_ctx.s4_next_debug_tick) >= 0)
  {
    mission_ctx.s4_next_debug_tick = now + ROBOT_S4_DEBUG_PERIOD_MS;
    (void)DebugUart_Logf(
        "[EE] LEFT r=%ld goal=%ld e=%ld w=%d/%d\r\n",
        (long)MissionTask_WrapYaw(mission_ctx.mission_snapshot.yaw_cdeg),
        (long)target_yaw_cdeg,
        (long)yaw_error,
        (int)mission_ctx.mission_snapshot.left_target_rpm,
        (int)mission_ctx.mission_snapshot.right_target_rpm);
  }
  return false;
}

void MissionTask_RetryS3AfterS4(uint32_t now, bool from_arrange02)
{
  MissionTask_StopWheels();
  if (MissionTask_HandleSupplementFinish(now) || MissionTask_LoadCheckPausesMotion()) return;
  if (from_arrange02 && Actuator_ClawCloseDeferred() != 0U) return;
  if ((from_arrange02 ? Actuator_SetClawClosed() : Actuator_SetFrameLowered()) != HAL_OK ||
      Actuator_SetCameraObjectTrackView() != HAL_OK)
  {
    mission_ctx.mission_snapshot.fault_flags |= MISSION_FAULT_ACTUATOR;
    MissionTask_EnterState(MISSION_STATE_FAULT, now);
    return;
  }
  if (from_arrange02 && mission_ctx.supplement_budget_exhausted)
    MissionTask_EndSupplementRun();

  MissionTask_ResetVisionPid();
  MissionTask_ResetS6ControllerState();
  mission_ctx.s4_final_mode_already_active = false;
  mission_ctx.s4_final_event34_pending = false;
  mission_ctx.s4_final_missing_since = 0U;
  mission_ctx.s4_e4_protocol_active = false;
  mission_ctx.s4_post22_search_active = false;
  mission_ctx.s3_track_stable_since = 0U;
  mission_ctx.s3_search_turn_stable_since = 0U;
  mission_ctx.s3_search_origin_yaw_cdeg = MissionTask_WrapYaw(mission_ctx.mission_snapshot.yaw_cdeg);
  mission_ctx.s3_search_yaw_target_cdeg = mission_ctx.s3_search_origin_yaw_cdeg;
  mission_ctx.s3_search_sweep_completed = false;
  mission_ctx.s3_search_ccw_midpoint_done = false;
  mission_ctx.s3_target_lost_since = now;
  mission_ctx.s3_wait_event04_logged = false;
  mission_ctx.s3_next_debug_tick = now;
  mission_ctx.vision_search_forward_active = false;
  mission_ctx.vision_search_e3_mode = false;
  mission_ctx.vision_search_green_e3_mode = false;
  mission_ctx.vision_search_resume_state = MISSION_STATE_S3_TRACK_GREEN;
  if (mission_ctx.supplement_capture_active)
  {
    mission_ctx.supplement_reference_command = MAIXCAM_COMMAND_SEARCH_TARGET;
    mission_ctx.supplement_recovery_event = 0U;
    mission_ctx.supplement_missing_since = now;
  }
  MissionTask_EnterState(mission_ctx.supplement_capture_active ? MISSION_STATE_SUPPLEMENT_TRACK
                                                  : MISSION_STATE_S3_TRACK_GREEN, now);
  /* Switch references before TX03. Keep independent load/safety events
     and retain fresh image-center responses received during the send. */
  uint32_t sequence_floor = MaixCam_FenceSearchReference();
  if (mission_ctx.supplement_capture_active) mission_ctx.supplement_coordinate_floor = sequence_floor;
  mission_ctx.mission_snapshot.vision_target_valid = false;
  mission_ctx.mission_snapshot.target_present = false;
  mission_ctx.mission_snapshot.target_age_ms = UINT32_MAX;
  if (MaixCam_SendCommand(MAIXCAM_COMMAND_SEARCH_TARGET) != HAL_OK)
  {
    mission_ctx.mission_snapshot.fault_flags |= MISSION_FAULT_VISION_TX;
    MissionTask_EnterState(MISSION_STATE_FAULT, HAL_GetTick());
    return;
  }
  (void)DebugUart_Log(from_arrange02 ? "[02] BACK OK CLOSE TX03\r\n" : "[24] TMO TX03 RETRY\r\n");
  (void)MissionTask_HandleSupplementFinish(HAL_GetTick());
}

void MissionTask_HandleS4FinalTrackTimeout(uint32_t now)
{
  uint8_t selected_id = MissionTask_GetSelectedObjectId();
  if (selected_id != 3U && selected_id != 5U && selected_id != 6U &&
      mission_ctx.s3_target_select_command != MAIXCAM_COMMAND_SELECT_BLACK_GREEN)
  {
    MissionTask_RetryS3AfterS4(now, false);
    return;
  }

  MissionTask_StopWheels();
  if (MissionTask_HandleSupplementFinish(now) || MissionTask_LoadCheckPausesMotion()) return;
  if (Actuator_ClawCloseDeferred() != 0U) return;
  if (Actuator_SetFrameLowered() != HAL_OK)
  {
    mission_ctx.mission_snapshot.fault_flags |= MISSION_FAULT_ACTUATOR;
    MissionTask_EnterState(MISSION_STATE_FAULT, now);
    return;
  }

  (void)MaixCam_FenceSearchReference();
  mission_ctx.mission_snapshot.vision_target_valid = false;
  MissionTask_ResetVisionPid();
  MissionTask_ResetS6ControllerState();
  MissionTask_EnterState(MISSION_STATE_S4_FINAL_TIMEOUT_BACKOFF, now);
  mission_ctx.mission_snapshot.vision_forward_rpm = -ROBOT_S4_FINAL_TIMEOUT_BACKOFF_RPM;
  MissionTask_SetWheelTargets(-ROBOT_S4_FINAL_TIMEOUT_BACKOFF_RPM,
                             -ROBOT_S4_FINAL_TIMEOUT_BACKOFF_RPM);
  (void)DebugUart_Logf("[24] TMO BACK=%d/%lu >03\r\n",
      ROBOT_S4_FINAL_TIMEOUT_BACKOFF_RPM, (unsigned long)ROBOT_S4_FINAL_TIMEOUT_BACKOFF_MS);
}
