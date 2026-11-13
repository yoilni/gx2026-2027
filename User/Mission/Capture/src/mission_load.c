#include "mission_load.h"
#include "mission_internal.h"
#include "mission_state.h"
#include "mission_safety.h"
#include "mission_vision.h"
#include "mission_arrange.h"
#include "mission_capture.h"
#include "mission_supplement.h"
#include "mission_safe_zone.h"

#include "actuator_task.h"
#include "debug_uart_task.h"
#include "maixcam_task.h"
#include "robot_config.h"
#include "stm32f4xx_hal.h"

static uint32_t s5_load_check_sequence_floor;
static uint32_t s5_next_debug_tick;
static uint32_t s5_casualty_adjust_start_tick;

void MissionTask_DropUnexpectedS5E5(void)
{
  uint16_t ignored = MaixCam_TakeIgnoredLoadEmptyRecoveryCount();
  bool allowed = (mission_ctx.mission_snapshot.state == MISSION_STATE_S5_WAIT_SINGLE_GREEN ||
      mission_ctx.mission_snapshot.state == MISSION_STATE_S6_RECHECK_WAIT_RESULT) &&
      !mission_ctx.s5_recheck_resume_safe_tracking &&
      !mission_ctx.s7_target_switch_enabled &&
      mission_ctx.s3_target_select_command == MAIXCAM_COMMAND_SELECT_GREEN;

  /* Catch an E5 accepted just before a state boundary as well. The receive
     gate rejects out-of-window E5 without overwriting another pending event. */
  if (!allowed && MaixCam_DropEventIf(MAIXCAM_EVENT_LOAD_EMPTY_RECOVERY) &&
      ignored < UINT16_MAX)
  {
    ++ignored;
  }
  if (ignored != 0U)
  {
    (void)DebugUart_Logf(
        "[E5] IGNORE n=%u s=%u cmd=%02X after07=%u; "
        "FIRST GREEN05 ONLY\r\n",
        (unsigned int)ignored, (unsigned int)mission_ctx.mission_snapshot.state,
        (unsigned int)mission_ctx.s3_target_select_command,
        mission_ctx.s7_target_switch_enabled ? 1U : 0U);
  }
}

bool MissionTask_CanPauseForLoadCheck(void)
{
  MissionState state = mission_ctx.mission_snapshot.state;
  /* The post34 turn must finish before the normal15/25 ->05 handshake. */
  return (state >= MISSION_STATE_S3_ALIGN_GREEN && state <= MISSION_STATE_S6_TRACK_SAFE_ZONE) ||
      (state >= MISSION_STATE_S7_SEARCH_RED && state <= MISSION_STATE_S7_SEARCH_BLACK) ||
      (state >= MISSION_STATE_S6_OBSTACLE_TURN_OUT && state <= MISSION_STATE_S4_E4_RED_SPIN_360) ||
      state == MISSION_STATE_S4_E4_MATERIAL_SPIN_CW_360 ||
      state == MISSION_STATE_S4_FINAL_TIMEOUT_BACKOFF ||
      state == MISSION_STATE_SUPPLEMENT_E3_SPIN_CCW ||
      state == MISSION_STATE_VISION_AVOID_START_TURN ||
      state == MISSION_STATE_VISION_AVOID_START_WAIT_ACK ||
      (state >= MISSION_STATE_S6_BAR_REVERSE && state <= MISSION_STATE_S6_BAR_WAIT_ACK) ||
      (state >= MISSION_STATE_S4_FINE32_TRACK && state <= MISSION_STATE_S4_FINE42_FINISH_TURN_LEFT) ||
      (state >= MISSION_STATE_SUPPLEMENT_WIDE_SETTLE && state <= MISSION_STATE_SUPPLEMENT_WAIT_RECHECK);
}

bool MissionTask_CanLatchCasualtyRegrip(MissionState state)
{
  return !mission_ctx.s5_casualty_timeout_check &&
      mission_ctx.s3_target_select_command == MAIXCAM_COMMAND_SELECT_RED &&
      (state == MISSION_STATE_S4_RAISE_FRAME ||
       state == MISSION_STATE_S4_TRACK_FINAL_BLOCK ||
       state == MISSION_STATE_S4_FINAL_RECOVERY_REVERSE ||
       state == MISSION_STATE_S4_FINAL_RECOVERY_TURN_270 ||
       state == MISSION_STATE_S4_FINAL_RECOVERY_TURN_90 ||
       state == MISSION_STATE_S4_FINAL_CENTER_OBJECT ||
       state == MISSION_STATE_S4_FINAL_TURN_SAFE);
}

static void MissionTask_UpdateS5CarriedObject(uint32_t now)
{
  MaixCam_Object object;
  /*51 counting reports have their own RX mailbox and are confirmed by06.
    The last wide-view tracking object is not a new cargo classification. */
  if (mission_ctx.s7_target_switch_enabled &&
      mission_ctx.s3_target_select_command == MAIXCAM_COMMAND_SELECT_BLACK_GREEN)
  {
    return;
  }
  if (!mission_ctx.s5_close_view_active || !MaixCam_GetObjectSnapshot(&object) ||
      (object.geometry & MAIXCAM_OBJECT_GEOMETRY_PRESENT) != 0U ||
      (int32_t)(object.sequence - s5_load_check_sequence_floor) <= 0 ||
      (uint32_t)(now - object.update_tick) > MISSION_VISION_TARGET_TIMEOUT_MS ||
      object.object_id < 3U || object.object_id > 6U)
  {
    return;
  }

  /* In 05 the vision-side ID must describe the confirmed object inside the
     frame, not an unrelated object elsewhere in the image. */
  s5_load_check_sequence_floor = object.sequence;
  if (!mission_ctx.s5_load_class_confirmed || mission_ctx.s6_carried_object_id != object.object_id)
  {
    mission_ctx.s6_carried_object_id = object.object_id;
    mission_ctx.s5_load_class_confirmed = true;
    (void)DebugUart_Logf(
        "[CARGO] id=%u kind=%s cmd=%02X q=%lu age=%lu\r\n",
        (unsigned int)mission_ctx.s6_carried_object_id,
        MissionTask_S6IsSupplyTarget() ? "SUPPLY" : "CASUALTY",
        (unsigned int)mission_ctx.s3_target_select_command,
        (unsigned long)object.sequence,
        (unsigned long)(now - object.update_tick));
  }
}

void MissionTask_EnterS5(uint32_t now)
{
  if (MissionTask_HandleSupplementFinish(now) || MissionTask_LoadCheckPausesMotion()) return;
  /* Request a smooth stop before entering05 or sending the load-check request. */
  MissionTask_StopWheels();
  uint8_t analyze_command = mission_ctx.mission_snapshot.team == ROBOT_TEAM_RED
                                ? MAIXCAM_COMMAND_ANALYZE_LOAD_RED
                                : MAIXCAM_COMMAND_ANALYZE_LOAD_BLUE;
  /* Keep a completed corner through review-requested02 arrangement. A24
     empty/recapture or the next07 task explicitly starts a different load. */
  mission_ctx.s5_recheck_resume_supplement = false;
  if (!mission_ctx.s5_recheck_resume_safe_tracking) mission_ctx.s6_corner_completed = false;
  MaixCam_SetLoadRecheckEnabled(false);

  /* Start a new load record. Never carry the previous red task's identity
     into a 51 capture; prefer the actual black/green tracking ID. */
  if (!mission_ctx.supplement_budget_started)
  {
    mission_ctx.s6_carried_object_id = mission_ctx.s5_casualty_timeout_check
        ? 0U : MissionTask_GetSelectedObjectId();
    mission_ctx.s5_load_class_confirmed = false;
  }
  mission_ctx.supplement_capture_active = false;
  MaixCam_SetSupplementWindowEnabled(mission_ctx.s7_target_switch_enabled &&
      mission_ctx.s3_target_select_command == MAIXCAM_COMMAND_SELECT_BLACK_GREEN &&
      !mission_ctx.s5_recheck_resume_safe_tracking);
  s5_load_check_sequence_floor = 0U;
  if (mission_ctx.supplement_budget_started) MaixCam_ClearPendingInput();
  else MaixCam_ClearEvent();
  (void)MaixCam_TakeSafeZoneAlignRequest();
  mission_ctx.s6_align26_pending = false;
  MaixCam_ClearObject();
  s5_next_debug_tick = now;
  mission_ctx.s5_close_view_active = false;
  MissionTask_EnterState(MISSION_STATE_S5_WAIT_SINGLE_GREEN, now);
  if (MissionTask_HandleSupplementFinish(HAL_GetTick()) || MissionTask_LoadCheckPausesMotion()) return;
  if (MaixCam_SendCommand(analyze_command) != HAL_OK)
  {
    mission_ctx.mission_snapshot.fault_flags |= MISSION_FAULT_VISION_TX;
    MissionTask_EnterState(MISSION_STATE_FAULT, now);
    return;
  }
  (void)DebugUart_Logf(
      "[05] TX%02X, WAIT RX05 CHECK\r\n",
      (unsigned int)analyze_command);
  (void)DebugUart_Logf("[CARGO] INIT id=%u cmd=%02X kind=%s src=TASK\r\n",
      (unsigned int)mission_ctx.s6_carried_object_id, (unsigned int)mission_ctx.s3_target_select_command,
      MissionTask_S6IsSupplyTarget() ? "SUPPLY" :
          (MissionTask_S6IsCasualtyTarget() ? "CASUALTY" : "UNKNOWN"));
}

static void MissionTask_StartS5Arrange02(uint32_t now)
{
  MissionTask_StopWheels();
  if (mission_ctx.mission_snapshot.state == MISSION_STATE_S6_RECHECK_WAIT_RESULT)
    MaixCam_SetLoadRecheckEnabled(false);
  MaixCam_SetSupplementWindowEnabled(false);
  mission_ctx.s5_close_view_active = false;
  (void)MaixCam_FenceSearchReference();
  mission_ctx.mission_snapshot.vision_target_valid = false;
  MissionTask_ResetVisionPid();
  MissionTask_ResetS6ControllerState();
  /* Set the search view now so it has settled before the next TX03. */
  if (Actuator_SetCameraObjectTrackView() != HAL_OK ||
      Actuator_SetClawAngle(ROBOT_CLAW_ARRANGE02_OPEN_DEG) != HAL_OK)
  {
    mission_ctx.mission_snapshot.fault_flags |= MISSION_FAULT_ACTUATOR;
    MissionTask_EnterState(MISSION_STATE_FAULT, now);
    return;
  }
  mission_ctx.s5_arrange_backing = false;
  MissionTask_EnterState(MISSION_STATE_S5_REVERSE_FOR_ARRANGE, now);
  MissionTask_RecordInitialGreenPush(MAIXCAM_EVENT_ARRANGE_READY);
  mission_ctx.mission_snapshot.vision_forward_rpm = 0;
  mission_ctx.mission_snapshot.vision_turn_rpm = 0;
  (void)DebugUart_Logf("[05/02] OPEN=%u WAIT=%lu >BACK\r\n",
      (unsigned int)ROBOT_CLAW_ARRANGE02_OPEN_DEG,
      (unsigned long)ROBOT_S5_ARRANGE_CLAW_OPEN_SETTLE_MS);
}

void MissionTask_RunS5WaitSingleGreen(uint32_t now)
{
  uint8_t event_code;
  MaixCam_Object load_check_object;

  if (mission_ctx.mission_snapshot.state == MISSION_STATE_S5_CASUALTY_EMPTY_WIDE_SETTLE)
  {
    MissionTask_StopWheels();
    if ((uint32_t)(now - mission_ctx.mission_snapshot.state_entry_tick) >=
        ROBOT_S5_CASUALTY_EMPTY_WIDE_SETTLE_MS)
      MissionTask_RetryS3AfterS4(now, false);
    return;
  }

  if (mission_ctx.mission_snapshot.state != MISSION_STATE_S5_WAIT_SINGLE_GREEN &&
      mission_ctx.mission_snapshot.state != MISSION_STATE_SUPPLEMENT_LOAD_CHECK &&
      mission_ctx.mission_snapshot.state != MISSION_STATE_S6_RECHECK_WAIT_RESULT)
  {
    return;
  }

  bool rechecking = mission_ctx.mission_snapshot.state == MISSION_STATE_S6_RECHECK_WAIT_RESULT;
  if (mission_ctx.s5_casualty_adjust_active ||
      (rechecking && MaixCam_LoadRecheckFinishPending())) return;
  /* Hold zero wheel targets in every05 count state, including ordinary05. */
  MissionTask_StopWheels();
  if (rechecking && (uint32_t)(now - mission_ctx.mission_snapshot.state_entry_tick) <
      ROBOT_S6_RECHECK_CAMERA_SETTLE_MS) return;

  /* Latch a fresh 05 class before RX06 clears the coordinate mailbox. */
  if (!rechecking) MissionTask_UpdateS5CarriedObject(now);
  if (MissionTask_HandleSupplementFinish(now)) return;
  if (!MaixCam_TakeEvent(&event_code))
  {
    if ((int32_t)(now - s5_next_debug_tick) >= 0)
    {
      s5_next_debug_tick = now + ROBOT_S5_DEBUG_PERIOD_MS;
      (void)DebugUart_Logf(
          "[05] WAIT %s ms=%lu team=%s cam=%s\r\n",
          mission_ctx.s5_close_view_active ? "03/02/06/24/E5" : "05/24/E5",
          (unsigned long)(now - mission_ctx.mission_snapshot.state_entry_tick),
          mission_ctx.mission_snapshot.team == ROBOT_TEAM_RED ? "RED" : "BLUE",
          mission_ctx.s5_close_view_active ? "NEAR" : "WIDE");
    }
    return;
  }

  if (event_code == MAIXCAM_EVENT_LOAD_CHECK_REQUEST)
  {
    if (Actuator_SetCameraNearView() != HAL_OK)
    {
      mission_ctx.mission_snapshot.fault_flags |= MISSION_FAULT_ACTUATOR;
      MissionTask_EnterState(MISSION_STATE_FAULT, now);
      return;
    }
    if (!mission_ctx.s5_close_view_active)
    {
      /* Arm a new confirmation window once; duplicate05 must not discard
         the confirmed class or fence out a class frame already received. */
      mission_ctx.s5_load_class_confirmed = false;
      (void)MaixCam_GetObjectSnapshot(&load_check_object);
      s5_load_check_sequence_floor = load_check_object.sequence;
    }
    mission_ctx.s5_close_view_active = true;
    s5_next_debug_tick = now;
    (void)DebugUart_Logf(
        "[05] RX05 near=%u WAIT02/06/24/E5\r\n",
        (unsigned int)ROBOT_CAMERA_NEAR_ANGLE_DEG);
    return;
  }

  if (event_code == MAIXCAM_EVENT_SUPPLEMENT_CAPTURE_REQUEST)
  {
    /* Out-of-window requests reach the ordinary mailbox only to be logged. */
    (void)DebugUart_Log("[SUPP] IGNORE03 NEED07/51/05\r\n");
    return;
  }

  if (event_code == MAIXCAM_EVENT_SKIP_TO_FINAL_TRACK)
  {
    if (mission_ctx.s5_casualty_timeout_check && !mission_ctx.s5_close_view_active)
    {
      (void)DebugUart_Log("[05] IGNORE24 WAIT RX05\r\n");
      return;
    }
    if (rechecking)
    {
      MaixCam_SetLoadRecheckEnabled(false);
      MaixCam_ClearPendingInput();
      MissionTask_ResetVisionPid();
    }
    mission_ctx.s5_close_view_active = false;
    MissionTask_EndSupplementRun();
    mission_ctx.s6_carried_object_id = 0U;
    mission_ctx.s5_load_class_confirmed = false;
    mission_ctx.s5_recheck_resume_safe_tracking = false;
    mission_ctx.s6_corner_completed = false;
    if (mission_ctx.s5_casualty_timeout_check)
    {
      if (Actuator_SetCameraObjectTrackView() != HAL_OK)
      {
        mission_ctx.mission_snapshot.fault_flags |= MISSION_FAULT_ACTUATOR;
        MissionTask_EnterState(MISSION_STATE_FAULT, now);
        return;
      }
      MissionTask_EnterState(MISSION_STATE_S5_CASUALTY_EMPTY_WIDE_SETTLE, now);
      (void)DebugUart_Log("[05] RED EMPTY RX24 >WIDE >03\r\n");
      return;
    }
    (void)DebugUart_Logf(
        "[05] RX24 EMPTY >24\r\n");
    MissionTask_SkipToS4FinalTrack(now);
    return;
  }

  if (event_code == MAIXCAM_EVENT_LOAD_EMPTY_RECOVERY)
  {
    if (mission_ctx.s7_target_switch_enabled ||
        mission_ctx.s3_target_select_command != MAIXCAM_COMMAND_SELECT_GREEN)
    {
      (void)DebugUart_Log(
          "[E5] IGNORE NEED FIRST GREEN\r\n");
      return;
    }
    MissionTask_StopWheels();
    if (Actuator_SetCameraWideView() != HAL_OK)
    {
      mission_ctx.mission_snapshot.fault_flags |= MISSION_FAULT_ACTUATOR;
      MissionTask_EnterState(MISSION_STATE_FAULT, now);
      return;
    }
    mission_ctx.s5_close_view_active = false;
    mission_ctx.s6_carried_object_id = 0U;
    mission_ctx.s5_load_class_confirmed = false;
    MaixCam_SetLoadRecheckEnabled(false);
    MissionTask_ResetVisionPid();
    MaixCam_ClearObject();
    mission_ctx.mission_snapshot.vision_target_valid = false;
    /* Retain E5's independent uninterruptible L10/F450/R45/L20 path.
       E5 is requested by the camera's empty05 loop counter, not E4's loss
       budget: neither reset nor consume the existing E4 push allowance. */
    mission_ctx.s4_e4_protocol_active = true;
    mission_ctx.s5_empty_recovery_active = true;
    MissionTask_StartS4GreenScatter(now);
    (void)DebugUart_Logf(
        "[E5] GREEN EMPTY wide=%u DOWN "
        "FIXED>44 E4n=%u KEEP\r\n",
        (unsigned int)ROBOT_CAMERA_WIDE_ANGLE_DEG,
        (unsigned int)mission_ctx.s4_green_e4_push_cycles);
    return;
  }

  if (event_code == MAIXCAM_EVENT_ARRANGE_READY)
  {
    MissionTask_StartS5Arrange02(now);
    return;
  }

  if (event_code != MAIXCAM_EVENT_NO_ARRANGE_REQUIRED)
  {
    (void)DebugUart_Logf(
        "[05] RX%02X WAIT05/02/06/24/E5\r\n",
        (unsigned int)event_code);
    return;
  }

  /* With the final-review protocol06 is accepted only by the independent
    05 result handler, with its frozen fresh class report. */
  MissionTask_StopWheels();
  (void)DebugUart_Log("[LOAD05] IGNORE06 NEED05/ID+06\r\n");
}

void MissionTask_StartS6LoadRecheck(uint32_t now)
{
  MissionTask_StopWheels();
  MaixCam_SetSupplementWindowEnabled(false);
  mission_ctx.supplement_capture_active = false;
  mission_ctx.supplement_budget_started = false;
  MaixCam_SetLoadRecheckEnabled(false);
  MaixCam_ClearPendingInput();
  mission_ctx.mission_snapshot.vision_target_valid = false;
  MissionTask_ResetVisionPid();
  MissionTask_ResetS6ControllerState();
  mission_ctx.s6_align26_pending = false;
  mission_ctx.s6_obstacle_side_decision_pending = false;
  mission_ctx.s6_obstacle_side_decision_done = false;
  mission_ctx.s5_load_class_confirmed = false;
  mission_ctx.s6_corner_completed = true;
  /* Keep the enclosed cargo and its last ID, but require a new05 report.
     Do not call the ordinary search/supplement initializer or lift the frame. */
  if (Actuator_SetFrameLowered() != HAL_OK)
  {
    mission_ctx.mission_snapshot.fault_flags |= MISSION_FAULT_ACTUATOR;
    MissionTask_EnterState(MISSION_STATE_FAULT, now);
    return;
  }
  /* Send while still wide; the acknowledged05 starts near-view counting. */
  mission_ctx.s5_close_view_active = false;
  s5_next_debug_tick = now;
  MaixCam_SetLoadRecheckEnabled(true);
  MissionTask_EnterState(MISSION_STATE_S6_RECHECK_WAIT_ACK, now);
  if (MaixCam_SendCommand(MAIXCAM_COMMAND_RECHECK_LOAD) != HAL_OK)
  {
    mission_ctx.mission_snapshot.fault_flags |= MISSION_FAULT_VISION_TX;
    MissionTask_EnterState(MISSION_STATE_FAULT, now);
    return;
  }
  (void)DebugUart_Log("[CHECK05] TX05 DOWN WAIT05\r\n");
}

bool MissionTask_LoadCheckPausesMotion(void)
{
  return mission_ctx.mission_snapshot.state == MISSION_STATE_S6_RECHECK_WAIT_ACK ||
      mission_ctx.mission_snapshot.state == MISSION_STATE_S6_RECHECK_WAIT_RESULT ||
      mission_ctx.mission_snapshot.state == MISSION_STATE_SUPPLEMENT_WAIT_RECHECK;
}

bool MissionTask_HandleLoadCheckRequest(uint32_t now)
{
  uint16_t skipped05 = MaixCam_TakeIgnoredSupplement05Count();
  if (skipped05 != 0U)
    (void)DebugUart_Logf("[SUPP] IGNORE05 n=%u s=%u DIRECT03\r\n",
        (unsigned int)skipped05, (unsigned int)mission_ctx.mission_snapshot.state);
  if (!MaixCam_TakeLoadRecheckAck()) return false;
  if (mission_ctx.s5_casualty_adjust_active)
  {
    /* A duplicate05 must not zero the wheel targets during the45 backoff. */
    return false;
  }
  if (!MissionTask_CanPauseForLoadCheck())
  {
    MaixCam_SetLoadRecheckEnabled(false);
    (void)DebugUart_Logf("[LOAD05] RX05 IGNORE INACTIVE s=%u\r\n",
        (unsigned int)mission_ctx.mission_snapshot.state);
    return false;
  }
  MissionState previous = mission_ctx.mission_snapshot.state;
  MissionTask_StopWheels();
  /* Secure the load if a valid05 interrupts a maneuver with the claw open. */
  if ((previous == MISSION_STATE_S4_FINAL_CENTER_OBJECT ||
       previous == MISSION_STATE_S5_REVERSE_FOR_ARRANGE) &&
      Actuator_SetClawClosed() != HAL_OK)
  {
    mission_ctx.mission_snapshot.fault_flags |= MISSION_FAULT_ACTUATOR;
    MissionTask_EnterState(MISSION_STATE_FAULT, now);
    return true;
  }
  /* A05 is meaningful even if already near. Reapply the requested camera
     pose, but duplicate packets in one count window do not erase results. */
  if (Actuator_SetCameraNearView() != HAL_OK)
  {
    mission_ctx.mission_snapshot.fault_flags |= MISSION_FAULT_ACTUATOR;
    MissionTask_EnterState(MISSION_STATE_FAULT, now);
    return true;
  }
  if (previous == MISSION_STATE_S6_RECHECK_WAIT_RESULT)
  {
    (void)DebugUart_Log("[LOAD05] RX05 DUP KEEP WINDOW\r\n");
    return true;
  }
  mission_ctx.s5_recheck_resume_supplement = mission_ctx.supplement_budget_started &&
      mission_ctx.supplement_capture_active && !mission_ctx.supplement_budget_exhausted;
  mission_ctx.s5_recheck_resume_safe_tracking = mission_ctx.s6_corner_completed &&
      !mission_ctx.supplement_budget_started;
  mission_ctx.s5_close_view_active = true;
  mission_ctx.s5_load_class_confirmed = false;
  MaixCam_ClearObject();
  mission_ctx.mission_snapshot.vision_target_valid = false;
  MissionTask_ResetVisionPid();
  MissionTask_ResetS6ControllerState();
  mission_ctx.vision_search_forward_active = false;
  mission_ctx.vision_search_e3_mode = false;
  mission_ctx.vision_search_green_e3_mode = false;
  mission_ctx.s4_final_event34_pending = false;
  mission_ctx.s4_e4_protocol_active = false;
  mission_ctx.s4_post22_search_active = false;
  mission_ctx.supplement_recovery_event = 0U;
  mission_ctx.s6_align26_pending = false;
  mission_ctx.s6_obstacle_side_decision_pending = false;
  mission_ctx.s6_obstacle_side_decision_done = false;
  mission_ctx.s6_phase_stable_since = 0U;
  s5_next_debug_tick = now;
  /* RX already fenced OLD events; do not clear02/24/06 received AFTER05. */
  MissionTask_EnterState(MISSION_STATE_S6_RECHECK_WAIT_RESULT, now);
  (void)DebugUart_Logf("[LOAD05] RX05 s=%u id=%u near=%u n=%u WAIT%lu>06/02/24\r\n",
      (unsigned int)previous, (unsigned int)mission_ctx.s6_carried_object_id,
      (unsigned int)ROBOT_CAMERA_NEAR_ANGLE_DEG, (unsigned int)mission_ctx.supplement_capture_count,
      (unsigned long)ROBOT_S6_RECHECK_CAMERA_SETTLE_MS);
  return true;
}

void MissionTask_WaitFinalLoadCheck(uint32_t now, const char *reason)
{
  mission_ctx.supplement_budget_exhausted = true;
  mission_ctx.supplement_capture_active = false;
  mission_ctx.supplement_recovery_event = 0U;
  mission_ctx.s4_final_event34_pending = false;
  mission_ctx.s4_e4_protocol_active = false;
  mission_ctx.s4_post22_search_active = false;
  mission_ctx.vision_search_forward_active = false;
  mission_ctx.vision_search_e3_mode = false;
  mission_ctx.vision_search_green_e3_mode = false;
  MissionTask_StopWheels();
  MissionTask_ResetVisionPid();
  MissionTask_ResetS6ControllerState();
  (void)DebugUart_Logf("[SUPP] STOP why=%s n=%u ms=%lu WAIT05\r\n",
      reason, (unsigned int)mission_ctx.supplement_capture_count,
      (unsigned long)(now - mission_ctx.supplement_first_request_tick));
  if (mission_ctx.mission_snapshot.state == MISSION_STATE_S6_RECHECK_WAIT_RESULT ||
      mission_ctx.mission_snapshot.state == MISSION_STATE_S6_RECHECK_WAIT_ACK ||
      mission_ctx.mission_snapshot.state == MISSION_STATE_SUPPLEMENT_WAIT_RECHECK) return;
  /* Open the result gate before requesting05. A visual05 may arrive during
     this transition or the polling TX, followed immediately by ID/06/02/24.
     Fence only OLD coordinates: clearing the whole mailbox here could erase
     a one-shot result that already followed the new05. Old motion events
     cannot run in WAIT_RECHECK and the RX05 parser fences them itself. */
  MaixCam_SetLoadRecheckEnabled(true);
  (void)MaixCam_FenceObjectReference();
  mission_ctx.mission_snapshot.vision_target_valid = false;
  /*24 may have partially lifted the frame when the budget expired. Close it
     to retain the cargo, without restarting a capture or its time budget. */
  if (Actuator_SetFrameLowered() != HAL_OK ||
      Actuator_SetCameraNearView() != HAL_OK)
  {
    mission_ctx.mission_snapshot.fault_flags |= MISSION_FAULT_ACTUATOR;
    MissionTask_EnterState(MISSION_STATE_FAULT, now);
    return;
  }
  mission_ctx.s5_close_view_active = true;
  s5_next_debug_tick = now;
  MissionTask_EnterState(MISSION_STATE_SUPPLEMENT_WAIT_RECHECK, now);
  /* If an unsolicited final05 is already received, use its existing result
     window instead of requesting another count or clearing its report. */
  if (MissionTask_HandleLoadCheckRequest(HAL_GetTick())) return;
  if (MaixCam_SendCommand(MAIXCAM_COMMAND_RECHECK_LOAD) != HAL_OK)
  {
    mission_ctx.mission_snapshot.fault_flags |= MISSION_FAULT_VISION_TX;
    MissionTask_EnterState(MISSION_STATE_FAULT, HAL_GetTick());
    return;
  }
  /* No input clearing after TX: keep a fast05 + class/result reply intact. */
  (void)DebugUart_Log("[SUPP] TX05 DOWN/NEAR WAIT05>06/02/24\r\n");
}

static bool MissionTask_RunS5CasualtyAdjust(uint32_t now)
{
  if (mission_ctx.s5_casualty_adjust_active)
  {
    if ((uint32_t)(now - s5_casualty_adjust_start_tick) <
        ROBOT_S5_CASUALTY_ADJUST_REVERSE_MS) return true;

    MissionTask_StopWheels();
    mission_ctx.mission_snapshot.vision_forward_rpm = 0;
    if (Actuator_SetClawClosed() != HAL_OK)
    {
      mission_ctx.mission_snapshot.fault_flags |= MISSION_FAULT_ACTUATOR;
      MissionTask_EnterState(MISSION_STATE_FAULT, now);
      return true;
    }
    /* Open the 06 result gate before TX45 so an immediate reply survives. */
    MaixCam_SetCasualtyLoadAdjustActive(false);
    mission_ctx.s5_casualty_adjust_done = true;
  }
  else
  {
    MaixCam_Object report;
    if (!MaixCam_TakeCasualtyLoadAdjust(&report)) return false;
    if (!mission_ctx.s5_casualty_adjust_done)
    {
      const bool fresh_class = report.object_id >= 3U && report.object_id <= 6U &&
          (uint32_t)(now - report.update_tick) <= MAIXCAM_DATA_TIMEOUT_MS;
      const uint8_t cargo_id = fresh_class ? report.object_id : MissionTask_GetCarriedObjectId();
      if (!mission_ctx.s7_target_switch_enabled || mission_ctx.supplement_capture_active || cargo_id != 4U)
      {
        (void)DebugUart_Log("[45] IGNORE NOT CASUALTY\r\n");
        return false;
      }
      MissionTask_StopWheels();
      if (Actuator_SetClawAngle(ROBOT_CLAW_CASUALTY_ADJUST_OPEN_DEG) != HAL_OK)
      {
        mission_ctx.mission_snapshot.fault_flags |= MISSION_FAULT_ACTUATOR;
        MissionTask_EnterState(MISSION_STATE_FAULT, now);
        return true;
      }
      MaixCam_SetCasualtyLoadAdjustActive(true);
      MissionTask_ResetVisionPid();
      mission_ctx.mission_snapshot.vision_target_valid = false;
      mission_ctx.s5_casualty_adjust_active = true;
      s5_casualty_adjust_start_tick = now;
      mission_ctx.mission_snapshot.vision_forward_rpm = -ROBOT_S5_CASUALTY_ADJUST_REVERSE_RPM;
      MissionTask_SetWheelTargets(-ROBOT_S5_CASUALTY_ADJUST_REVERSE_RPM,
                                  -ROBOT_S5_CASUALTY_ADJUST_REVERSE_RPM);
      (void)DebugUart_Logf("[45] CLAW=%u BACK=%d/%lu\r\n",
          (unsigned int)ROBOT_CLAW_CASUALTY_ADJUST_OPEN_DEG,
          ROBOT_S5_CASUALTY_ADJUST_REVERSE_RPM,
          (unsigned long)ROBOT_S5_CASUALTY_ADJUST_REVERSE_MS);
      return true;
    }
  }

  /* Also answer a repeat after completion without moving again. */
  if (MaixCam_SendCommand(MAIXCAM_COMMAND_CASUALTY_ADJUST_DONE) != HAL_OK)
  {
    MaixCam_SetLoadRecheckEnabled(false);
    mission_ctx.mission_snapshot.fault_flags |= MISSION_FAULT_VISION_TX;
    MissionTask_EnterState(MISSION_STATE_FAULT, HAL_GetTick());
    return true;
  }
  mission_ctx.s5_casualty_adjust_active = false;
  (void)DebugUart_Log("[45] TX45 WAIT ID+06\r\n");
  return true;
}

void MissionTask_RunS6LoadRecheck(uint32_t now)
{
  MaixCam_Object report;
  MissionState state = mission_ctx.mission_snapshot.state;
  if (MissionTask_HandleLoadCheckRequest(now)) return;
  if (state != MISSION_STATE_S6_RECHECK_WAIT_ACK &&
      state != MISSION_STATE_S6_RECHECK_WAIT_RESULT &&
      state != MISSION_STATE_S6_RECHECK_WIDE_SETTLE &&
      state != MISSION_STATE_SUPPLEMENT_WAIT_RECHECK) return;
  if (state == MISSION_STATE_S6_RECHECK_WAIT_RESULT &&
      (uint32_t)(now - mission_ctx.mission_snapshot.state_entry_tick) >= ROBOT_S6_RECHECK_CAMERA_SETTLE_MS &&
      MissionTask_RunS5CasualtyAdjust(now)) return;
  MissionTask_StopWheels();
  if (state == MISSION_STATE_SUPPLEMENT_WAIT_RECHECK)
  {
    if ((int32_t)(now - s5_next_debug_tick) >= 0)
    {
      s5_next_debug_tick = now + ROBOT_S5_DEBUG_PERIOD_MS;
      (void)DebugUart_Log("[LOAD05] WAIT05>06/02/24 BUDGET END\r\n");
    }
    return;
  }
  if (state == MISSION_STATE_S6_RECHECK_WIDE_SETTLE)
  {
    /* Duplicate result/ack packets cannot restart this hold. Preserve new
       safe-zone coordinates,36 and latched26 arriving at the resume edge. */
    (void)MaixCam_DropEventIf(MAIXCAM_EVENT_LOAD_CHECK_REQUEST);
    (void)MaixCam_DropEventIf(MAIXCAM_EVENT_NO_ARRANGE_REQUIRED);
    if ((uint32_t)(now - mission_ctx.mission_snapshot.state_entry_tick) <
        ROBOT_S6_RECHECK_WIDE_SETTLE_MS) return;
    (void)MaixCam_GetObjectSnapshot(&report);
    if (report.object_id != 1U && report.object_id != 2U)
    {
      MaixCam_ClearObject();
      mission_ctx.mission_snapshot.vision_target_valid = false;
    }
    mission_ctx.s6_track_accounting_tick = now;
    MissionTask_StartS6VisionApproach(now);
    (void)DebugUart_Log("[CHECK05] WIDE OK >16 KEEP CORNER/TIMER\r\n");
    return;
  }
  if (state == MISSION_STATE_S6_RECHECK_WAIT_RESULT &&
      (uint32_t)(now - mission_ctx.mission_snapshot.state_entry_tick) >= ROBOT_S6_RECHECK_CAMERA_SETTLE_MS &&
      MaixCam_TakeLoadRecheckFinish(&report))
  {
    if (report.object_id < 3U || report.object_id > 6U ||
        (mission_ctx.s5_casualty_timeout_check && report.object_id != 4U) ||
        (!mission_ctx.s7_target_switch_enabled && report.object_id != 5U))
    {
      MaixCam_ClearEvent();
      (void)DebugUart_Logf("[CHECK05] BLOCK06 id=%u NEED%s+06\r\n",
          (unsigned int)report.object_id, mission_ctx.s7_target_switch_enabled ? "ID3..6" : "ID5");
      return;
    }
    mission_ctx.s6_carried_object_id = report.object_id;
    mission_ctx.s5_load_class_confirmed = true;
    MaixCam_SetLoadRecheckEnabled(false);
    /* RX already fenced pre06 events. Keep a new16 following this06, but
       discard near-view cargo coordinates/partial frames at the gate exit. */
    MaixCam_ClearObject();
    mission_ctx.mission_snapshot.vision_target_valid = false;
    mission_ctx.s5_recheck_resume_supplement = false;
    if (!mission_ctx.s5_recheck_resume_safe_tracking)
    {
      MissionTask_StartS6FromLoad(now);
      (void)DebugUart_Logf("[LOAD05] RX06 id=%u >SAFE\r\n",
          (unsigned int)mission_ctx.s6_carried_object_id);
      return;
    }
    mission_ctx.supplement_capture_active = false;
    mission_ctx.supplement_budget_started = false;
    MaixCam_SetSupplementWindowEnabled(false);
    if (Actuator_StopBafflePwm() != HAL_OK || Actuator_SetFrameLowered() != HAL_OK ||
        Actuator_SetCameraSafeZoneView() != HAL_OK)
    {
      mission_ctx.mission_snapshot.fault_flags |= MISSION_FAULT_ACTUATOR;
      MissionTask_EnterState(MISSION_STATE_FAULT, now);
      return;
    }
    mission_ctx.s5_close_view_active = false;
    MissionTask_ResetVisionPid();
    MissionTask_ResetS6ControllerState();
    MissionTask_EnterState(MISSION_STATE_S6_RECHECK_WIDE_SETTLE, now);
    (void)DebugUart_Logf("[CHECK05] RX06 id=%u CAM=%u wait=%lu >XY\r\n",
        (unsigned int)mission_ctx.s6_carried_object_id, (unsigned int)ROBOT_CAMERA_SAFE_ZONE_ANGLE_DEG,
        (unsigned long)ROBOT_S6_RECHECK_WIDE_SETTLE_MS);
    return;
  }
  if ((int32_t)(now - s5_next_debug_tick) >= 0)
  {
    s5_next_debug_tick = now + ROBOT_S5_DEBUG_PERIOD_MS;
    (void)DebugUart_Logf("[CHECK05] WAIT %s ms=%lu\r\n",
        state == MISSION_STATE_S6_RECHECK_WAIT_ACK ? "RX05" : "ID+06/02/24",
        (unsigned long)(now - mission_ctx.mission_snapshot.state_entry_tick));
  }
}

void MissionTask_RunS5PrepareArrange(uint32_t now)
{
  if (mission_ctx.mission_snapshot.state != MISSION_STATE_S5_REVERSE_FOR_ARRANGE) return;

  const uint32_t elapsed = now - mission_ctx.mission_snapshot.state_entry_tick;
  if (!mission_ctx.s5_arrange_backing)
  {
    if (elapsed < ROBOT_S5_ARRANGE_CLAW_OPEN_SETTLE_MS) return;
    mission_ctx.s5_arrange_backing = true;
    mission_ctx.mission_snapshot.state_entry_tick = now;
    mission_ctx.mission_snapshot.vision_forward_rpm = -ROBOT_S5_ARRANGE_REVERSE_RPM;
    mission_ctx.mission_snapshot.vision_turn_rpm = 0;
    MissionTask_SetWheelTargets(-ROBOT_S5_ARRANGE_REVERSE_RPM, -ROBOT_S5_ARRANGE_REVERSE_RPM);
    (void)DebugUart_Logf("[05/02] BACK=%d/%lu\r\n", ROBOT_S5_ARRANGE_REVERSE_RPM,
        (unsigned long)ROBOT_S5_ARRANGE_REVERSE_MS);
    return;
  }
  if (elapsed < ROBOT_S5_ARRANGE_REVERSE_MS) return;

  MissionTask_RetryS3AfterS4(now, true);
}
