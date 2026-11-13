#include "mission_supplement.h"
#include "mission_internal.h"
#include "mission_state.h"
#include "mission_safety.h"
#include "mission_vision.h"
#include "mission_arrange.h"
#include "mission_capture.h"
#include "mission_load.h"
#include "mission_safe_zone.h"

#include "actuator_task.h"
#include "recovery_flow.h"
#include "debug_uart_task.h"
#include "maixcam_task.h"
#include "robot_config.h"
#include "stm32f4xx_hal.h"

static uint32_t supplement_next_debug_tick;
/* A retransmitted E3 is not round2 until round1 actually completed TX13. */
static bool supplement_e3_first_ack_sent;
static uint32_t supplement_e3_first_hold_tick;
static int32_t supplement_e3_spin_progress_cdeg;
static int16_t supplement_e3_spin_rpm;

void MissionTask_EndSupplementRun(void)
{
  mission_ctx.supplement_budget_started = false;
  mission_ctx.supplement_capture_active = false;
  mission_ctx.s5_recheck_resume_supplement = false;
  MaixCam_SetSupplementWindowEnabled(false);
  /* Keep the per-trip time origin, attempt count and exhausted latch. */
}

void MissionTask_ResetSupplement(void)
{
  mission_ctx.supplement_budget_started = false;
  mission_ctx.supplement_budget_exhausted = false;
  mission_ctx.supplement_capture_active = false;
  mission_ctx.supplement_first_supply_ack_sent = false;
  mission_ctx.supplement_capture_count = 0U;
  mission_ctx.supplement_first_request_tick = 0U;
  mission_ctx.supplement_coordinate_floor = 0U;
  mission_ctx.supplement_reference_command = MAIXCAM_COMMAND_SEARCH_TARGET;
  mission_ctx.supplement_recovery_event = 0U;
  mission_ctx.supplement_e3_recovery_count = 0U;
  supplement_e3_first_ack_sent = false;
  mission_ctx.supplement_e3_first_hold_active = false;
  supplement_e3_first_hold_tick = 0U;
  mission_ctx.supplement_e3_spin_previous_yaw_cdeg = 0L;
  supplement_e3_spin_progress_cdeg = 0L;
  mission_ctx.supplement_e3_spin_goal_cdeg = 0L;
  supplement_e3_spin_rpm = 0;
  MaixCam_SetSupplementWindowEnabled(false);
}

bool MissionTask_HandleSupplementFinish(uint32_t now)
{
  MaixCam_Object report;
  if (MissionTask_HandleLoadCheckRequest(now)) return true;
  bool received06 = MaixCam_TakeSupplementFinishRequest(&report);
  bool expired = mission_ctx.supplement_budget_started && !mission_ctx.supplement_budget_exhausted &&
      (uint32_t)(now - mission_ctx.supplement_first_request_tick) >= ROBOT_SUPPLEMENT_TOTAL_MS;
  if (!received06 && !expired) return false;

  if (mission_ctx.mission_snapshot.state == MISSION_STATE_FAULT ||
      mission_ctx.mission_snapshot.state == MISSION_STATE_STOPPED ||
      mission_ctx.mission_snapshot.state == MISSION_STATE_WAIT_START)
  {
    return false;
  }
  if (received06)
  {
    (void)DebugUart_Log("[SUPP] IGNORE06 NEED05+ID\r\n");
  }
  if (!expired) return false;
  MissionTask_WaitFinalLoadCheck(now, "TIME_LIMIT");
  return true;
}

static void MissionTask_FenceSupplementCoordinates(void)
{
  MaixCam_Object object;
  MaixCam_ClearPendingInput();
  (void)MaixCam_GetObjectSnapshot(&object);
  mission_ctx.supplement_coordinate_floor = object.sequence;
  mission_ctx.mission_snapshot.vision_target_valid = false;
  MissionTask_ResetVisionPid();
}

static bool MissionTask_SendSupplementCommand(uint8_t command, uint32_t now)
{
  /* An interrupt may receive06 during the preceding blocking UART TX.
     Check between commands, not just once per20ms mission iteration. */
  if (MissionTask_HandleSupplementFinish(HAL_GetTick())) return false;
  if (mission_ctx.mission_snapshot.state == MISSION_STATE_S6_RECHECK_WAIT_RESULT ||
      mission_ctx.mission_snapshot.state == MISSION_STATE_SUPPLEMENT_WAIT_RECHECK) return false;
  /* Supplement03 has the same frame rule, but must retain cargo/budget and
     must not initialize an ordinary search or reopen the collection frame. */
  if (command == MAIXCAM_COMMAND_SEARCH_TARGET &&
      Actuator_SetFrameLowered() != HAL_OK)
  {
    MissionTask_StopWheels();
    mission_ctx.mission_snapshot.fault_flags |= MISSION_FAULT_ACTUATOR;
    MissionTask_EnterState(MISSION_STATE_FAULT, now);
    return false;
  }
  if (MaixCam_SendCommand(command) != HAL_OK)
  {
    mission_ctx.mission_snapshot.fault_flags |= MISSION_FAULT_VISION_TX;
    MissionTask_EnterState(MISSION_STATE_FAULT, now);
    return false;
  }
  (void)DebugUart_Logf("[SUPP] TX%02X n=%u ms=%lu\r\n",
      (unsigned int)command, (unsigned int)mission_ctx.supplement_capture_count,
      (unsigned long)(HAL_GetTick() - mission_ctx.supplement_first_request_tick));
  return !MissionTask_HandleSupplementFinish(HAL_GetTick());
}

void MissionTask_StartSupplement(uint32_t now, uint32_t request_tick,
                                        bool confirmed_by_load_check)
{
  if (mission_ctx.supplement_budget_exhausted)
  {
    MissionTask_WaitFinalLoadCheck(now, "BUDGET_ALREADY_USED");
    return;
  }
  if (!mission_ctx.supplement_budget_started)
  {
    mission_ctx.supplement_budget_started = true;
    /* Reopening after an empty05 may use only the remaining budget. */
    if (mission_ctx.supplement_capture_count == 0U)
    {
      mission_ctx.supplement_first_request_tick = request_tick;
      mission_ctx.supplement_e3_recovery_count = 0U;
      supplement_e3_first_ack_sent = false;
    }
  }
  if (MissionTask_HandleSupplementFinish(now)) return;
  if (mission_ctx.supplement_capture_count >= ROBOT_SUPPLEMENT_MAX_CAPTURES)
  {
    MissionTask_WaitFinalLoadCheck(now, "MAX2_LIMIT");
    return;
  }
  ++mission_ctx.supplement_capture_count;
  mission_ctx.s4_fine_next_left = true;
  MaixCam_SetLoadRecheckEnabled(false);
  MaixCam_SetSupplementDirectSearch(true);
  mission_ctx.supplement_capture_active = true;
  mission_ctx.supplement_reference_command = MAIXCAM_COMMAND_SEARCH_TARGET;
  mission_ctx.supplement_recovery_event = 0U;
  mission_ctx.supplement_missing_since = now;
  supplement_next_debug_tick = now;
  /* Keep the first supply while tracking the next. The automatic34 branch
     has not yet done a05 inventory, so its class remains provisional. */
  if (mission_ctx.s6_carried_object_id != 5U && mission_ctx.s6_carried_object_id != 6U)
      mission_ctx.s6_carried_object_id = 5U;
  mission_ctx.s5_load_class_confirmed = confirmed_by_load_check;
  mission_ctx.s5_close_view_active = false;
  MissionTask_StopWheels();
  MissionTask_FenceSupplementCoordinates();
  if ((Actuator_SetFrameLowered() != HAL_OK) ||
      (Actuator_SetCameraObjectTrackView() != HAL_OK))
  {
    mission_ctx.mission_snapshot.fault_flags |= MISSION_FAULT_ACTUATOR;
    MissionTask_EnterState(MISSION_STATE_FAULT, now);
    return;
  }
  /* Deliberately no ordinary StartS3Align/SkipToS4FinalTrack here. They clear
     cargo and/or raise the whole frame. Keep the current closed frame. */
  MissionTask_EnterState(MISSION_STATE_SUPPLEMENT_WIDE_SETTLE, now);
  (void)DebugUart_Logf("[SUPP] START n=%u id=%u wide=%u wait=%lu budget=%lu\r\n",
      (unsigned int)mission_ctx.supplement_capture_count, (unsigned int)mission_ctx.s6_carried_object_id,
      (unsigned int)ROBOT_CAMERA_OBJECT_TRACK_ANGLE_DEG,
      (unsigned long)ROBOT_SUPPLEMENT_WIDE_SETTLE_MS,
      (unsigned long)ROBOT_SUPPLEMENT_TOTAL_MS);
}

void MissionTask_HandleSupplementControl(uint32_t now)
{
  uint32_t request_tick;
  if (MissionTask_HandleSupplementFinish(now)) return;
  /*05 is handled for every active phase by the independent global review
     mailbox, not by a51-only branch that ignores already-near requests. */
  /*05 and03 can arrive in the same interrupt burst. Let the ordinary05
     handler establish the near-view count window before consuming03. */
  if (mission_ctx.mission_snapshot.state == MISSION_STATE_S5_WAIT_SINGLE_GREEN &&
      !mission_ctx.s5_close_view_active) return;
  if (mission_ctx.mission_snapshot.state == MISSION_STATE_S6_RECHECK_WAIT_RESULT &&
      ((uint32_t)(now - mission_ctx.mission_snapshot.state_entry_tick) < ROBOT_S6_RECHECK_CAMERA_SETTLE_MS ||
       MaixCam_LoadRecheckResultPending())) return;
  if (!MaixCam_TakeSupplementRequest(&request_tick)) return;
  if (mission_ctx.supplement_budget_exhausted)
  {
    (void)DebugUart_Log("[SUPP] IGNORE03 BUDGET END WAIT05\r\n");
    return;
  }
  if ((mission_ctx.mission_snapshot.state == MISSION_STATE_SUPPLEMENT_LOAD_CHECK ||
       mission_ctx.mission_snapshot.state == MISSION_STATE_S6_RECHECK_WAIT_RESULT) &&
      mission_ctx.supplement_budget_started && mission_ctx.supplement_capture_active &&
      !mission_ctx.supplement_budget_exhausted && mission_ctx.s5_recheck_resume_supplement &&
      mission_ctx.s7_target_switch_enabled &&
      mission_ctx.s3_target_select_command == MAIXCAM_COMMAND_SELECT_BLACK_GREEN)
  {
    /* Continue the current attempt: neither increment the count nor move
       first_request_tick. Clear counting-era data before the new handshake. */
    MissionTask_StopWheels();
    MaixCam_SetLoadRecheckEnabled(false);
    MaixCam_SetSupplementDirectSearch(true);
    MissionTask_FenceSupplementCoordinates();
    MissionTask_ResetS6ControllerState();
    mission_ctx.s6_phase_stable_since = 0U;
    mission_ctx.supplement_recovery_event = 0U;
    mission_ctx.supplement_reference_command = MAIXCAM_COMMAND_SEARCH_TARGET;
    mission_ctx.supplement_missing_since = now;
    mission_ctx.s5_close_view_active = false;
    if ((Actuator_SetFrameLowered() != HAL_OK) ||
        (Actuator_SetCameraObjectTrackView() != HAL_OK))
    {
      mission_ctx.mission_snapshot.fault_flags |= MISSION_FAULT_ACTUATOR;
      MissionTask_EnterState(MISSION_STATE_FAULT, now);
      return;
    }
    MissionTask_EnterState(MISSION_STATE_SUPPLEMENT_WIDE_SETTLE, now);
    (void)DebugUart_Logf("[SUPP] CONT03 n=%u ms=%lu id=%u WIDE500>51/03\r\n",
        (unsigned int)mission_ctx.supplement_capture_count,
        (unsigned long)(now - mission_ctx.supplement_first_request_tick),
        (unsigned int)mission_ctx.s6_carried_object_id);
    return;
  }
  if ((mission_ctx.mission_snapshot.state != MISSION_STATE_S5_WAIT_SINGLE_GREEN &&
       mission_ctx.mission_snapshot.state != MISSION_STATE_S6_RECHECK_WAIT_RESULT) ||
      !mission_ctx.s7_target_switch_enabled ||
      mission_ctx.s3_target_select_command != MAIXCAM_COMMAND_SELECT_BLACK_GREEN ||
      mission_ctx.supplement_capture_active)
  {
    (void)DebugUart_Logf("[SUPP] IGNORE03 s=%u NEED07/51/05\r\n",
        (unsigned int)mission_ctx.mission_snapshot.state);
    return;
  }
  MissionTask_StartSupplement(now, request_tick, true);
}

static void MissionTask_StartSupplementRecovery(uint32_t now, uint8_t event_code,
                                                bool received_event)
{
  bool search_e3 = event_code == MAIXCAM_EVENT_SEARCH_TARGET_LOST &&
      mission_ctx.supplement_reference_command == MAIXCAM_COMMAND_SEARCH_TARGET;
  MissionTask_StopWheels();
  MissionTask_ResetVisionControllerState();
  MissionTask_ResetS6ControllerState();
  mission_ctx.s6_phase_stable_since = 0U;
  mission_ctx.supplement_recovery_event = event_code;
  mission_ctx.supplement_e3_first_hold_active = false;
  if (search_e3 && received_event)
  {
    if (mission_ctx.supplement_e3_recovery_count == 0U) mission_ctx.supplement_e3_recovery_count = 1U;
    else if (supplement_e3_first_ack_sent && mission_ctx.supplement_e3_recovery_count < UINT8_MAX)
      ++mission_ctx.supplement_e3_recovery_count;
  }
  if (search_e3 && mission_ctx.supplement_e3_recovery_count != 0U &&
      (!supplement_e3_first_ack_sent || mission_ctx.supplement_e3_recovery_count >= 2U))
  {
    mission_ctx.supplement_e3_spin_previous_yaw_cdeg = MissionTask_WrapYaw(mission_ctx.mission_snapshot.yaw_cdeg);
    supplement_e3_spin_progress_cdeg = 0L;
    mission_ctx.supplement_e3_spin_goal_cdeg = !supplement_e3_first_ack_sent
        ? ROBOT_SUPPLEMENT_E3_FIRST_SPIN_CDEG : ROBOT_SUPPLEMENT_E3_SPIN_CDEG;
    supplement_e3_spin_rpm = !supplement_e3_first_ack_sent
        ? ROBOT_SUPPLEMENT_E3_FIRST_SPIN_RPM : ROBOT_SUPPLEMENT_E3_SPIN_RPM;
    MissionTask_EnterState(MISSION_STATE_SUPPLEMENT_E3_SPIN_CCW, now);
    (void)DebugUart_Logf("[SUPP] E3 n=%u CCW%ld v=%d\r\n",
        (unsigned int)mission_ctx.supplement_e3_recovery_count,
        (long)(mission_ctx.supplement_e3_spin_goal_cdeg / 100L), (int)supplement_e3_spin_rpm);
    return;
  }
  mission_ctx.supplement_left_yaw_cdeg = MissionTask_WrapYaw(mission_ctx.mission_snapshot.yaw_cdeg +
      (int32_t)ROBOT_YAW_LEFT_SIGN * 4500L);
  mission_ctx.supplement_right_yaw_cdeg = MissionTask_WrapYaw(mission_ctx.supplement_left_yaw_cdeg -
      (int32_t)ROBOT_YAW_LEFT_SIGN * 9000L);
  MissionTask_EnterState(MISSION_STATE_SUPPLEMENT_TURN_LEFT, now);
  (void)DebugUart_Logf("[SUPP] REC%02X L=%ld R=%ld e3=%u ms=%lu\r\n",
      (unsigned int)event_code, (long)mission_ctx.supplement_left_yaw_cdeg,
      (long)mission_ctx.supplement_right_yaw_cdeg, (unsigned int)mission_ctx.supplement_e3_recovery_count,
      (unsigned long)(now - mission_ctx.supplement_first_request_tick));
}

static void MissionTask_ResumeSupplement(uint32_t now, bool recovery_complete)
{
  uint8_t loss = mission_ctx.supplement_recovery_event;
  bool ack_first_e3 = recovery_complete && loss == MAIXCAM_EVENT_SEARCH_TARGET_LOST &&
      mission_ctx.supplement_reference_command == MAIXCAM_COMMAND_SEARCH_TARGET &&
      mission_ctx.supplement_e3_recovery_count == 1U && !supplement_e3_first_ack_sent;
  mission_ctx.supplement_recovery_event = 0U;
  MissionTask_StopWheels();
  MissionTask_ResetVisionControllerState();
  MissionTask_ResetS6ControllerState();
  mission_ctx.s6_phase_stable_since = 0U;
  mission_ctx.supplement_missing_since = now;
  MissionTask_EnterState(MISSION_STATE_SUPPLEMENT_TRACK, now);
  if (ack_first_e3)
  {
    /* Discard only an old repeated E3 BEFORE TX. A new E3 received during
       TX13 belongs to the next round and must remain in the mailbox. */
    (void)MaixCam_DropEventIf(MAIXCAM_EVENT_SEARCH_TARGET_LOST);
    if (!MissionTask_SendSupplementCommand(MAIXCAM_COMMAND_SUPPLEMENT_SEARCH_DONE, now)) return;
    supplement_e3_first_ack_sent = true;
    (void)DebugUart_Log("[SUPP] TX13 E3 ROUND2 READY\r\n");
  }
  if (loss == MAIXCAM_EVENT_CENTER_TARGET_LOST)
  {
    MissionTask_FenceSupplementCoordinates();
    if (!MissionTask_SendSupplementCommand(MAIXCAM_COMMAND_CENTER_RECOVERY_DONE, now)) return;
    MissionTask_StartS4TrackCenterFrom04(now);
  }
  else if (loss == MAIXCAM_EVENT_NO_TARGET)
  {
    (void)MissionTask_SendSupplementCommand(MAIXCAM_COMMAND_FINAL_RECOVERY_DONE, now);
  }
  /* E3 resumes the same51 search, without reinitializing the vision lock. */
}

void MissionTask_RunSupplement(uint32_t now)
{
  uint8_t event_code;
  bool recovering;
  bool xy_in_tolerance;
  uint32_t elapsed = (uint32_t)(now - mission_ctx.mission_snapshot.state_entry_tick);
  if (!mission_ctx.supplement_capture_active) return;
  /*04/24/34 hand off to the ordinary S4 state machine. Do not consume its
     events or command wheels after the same task tick has changed state. */
  if (mission_ctx.mission_snapshot.state != MISSION_STATE_SUPPLEMENT_WIDE_SETTLE &&
      mission_ctx.mission_snapshot.state != MISSION_STATE_SUPPLEMENT_TRACK &&
      mission_ctx.mission_snapshot.state != MISSION_STATE_SUPPLEMENT_TURN_LEFT &&
      mission_ctx.mission_snapshot.state != MISSION_STATE_SUPPLEMENT_TURN_RIGHT &&
      mission_ctx.mission_snapshot.state != MISSION_STATE_SUPPLEMENT_E3_SPIN_CCW) return;
  if (MissionTask_HandleSupplementFinish(now)) return;
  if (mission_ctx.mission_snapshot.state == MISSION_STATE_SUPPLEMENT_WIDE_SETTLE)
  {
    MissionTask_StopWheels();
    if (elapsed < ROBOT_SUPPLEMENT_WIDE_SETTLE_MS) return;
    MissionTask_FenceSupplementCoordinates();
    MissionTask_EnterState(MISSION_STATE_SUPPLEMENT_TRACK, now);
    mission_ctx.supplement_missing_since = now;
    if (!MissionTask_SendSupplementCommand(MAIXCAM_COMMAND_SELECT_BLACK_GREEN, now)) return;
    if (!MissionTask_SendSupplementCommand(MAIXCAM_COMMAND_SEARCH_TARGET, now)) return;
    (void)DebugUart_Log("[SUPP] DIRECT03 WAIT ID5/6\r\n");
    return;
  }
  recovering = mission_ctx.mission_snapshot.state == MISSION_STATE_SUPPLEMENT_TURN_LEFT ||
      mission_ctx.mission_snapshot.state == MISSION_STATE_SUPPLEMENT_TURN_RIGHT ||
      mission_ctx.mission_snapshot.state == MISSION_STATE_SUPPLEMENT_E3_SPIN_CCW;
  if (mission_ctx.mission_snapshot.state == MISSION_STATE_SUPPLEMENT_E3_SPIN_CCW &&
      mission_ctx.supplement_e3_first_hold_active)
  {
    /* Only post-turn coordinates may interrupt this nonblocking hold.
       Keep phase events queued; do not clear a prompt new04/24 on resume. */
    MissionTask_StopWheels();
    if (mission_ctx.mission_snapshot.vision_target_valid)
      MissionTask_ResumeSupplement(now, false);
    else if ((uint32_t)(now - supplement_e3_first_hold_tick) >=
             ROBOT_SUPPLEMENT_E3_FIRST_HOLD_MS)
      MissionTask_ResumeSupplement(now, true);
    return;
  }
  if (MaixCam_TakeEvent(&event_code))
  {
    if (event_code == MAIXCAM_EVENT_FINAL_OBJECT_IN_FRAME)
    {
      mission_ctx.supplement_recovery_event = 0U;
      MissionTask_CaptureS4From34(now);
      return;
    }
    if (event_code == MAIXCAM_EVENT_SKIP_TO_FINAL_TRACK)
    {
      mission_ctx.supplement_recovery_event = 0U;
      MissionTask_SkipToS4FinalTrack(now);
      if (mission_ctx.mission_snapshot.state == MISSION_STATE_S4_RAISE_FRAME)
        (void)DebugUart_Log("[SUPP] RX24 >24 CLAW CLOSED\r\n");
      return;
    }
    if (event_code == MAIXCAM_EVENT_OBJECT_IN_FRAME)
    {
      mission_ctx.supplement_recovery_event = 0U;
      MissionTask_StartS4TrackCenterFrom04(now);
      if (mission_ctx.mission_snapshot.state == MISSION_STATE_S4_TRACK_CENTER)
        (void)DebugUart_Log("[SUPP] RX04 >04\r\n");
      return;
    }
    if (event_code == MAIXCAM_EVENT_CENTER_REACHED)
    {
      mission_ctx.supplement_recovery_event = 0U;
      MissionTask_AckS4Center(now);
      return;
    }
    if (event_code == MAIXCAM_EVENT_SEARCH_TARGET_LOST ||
        event_code == MAIXCAM_EVENT_CENTER_TARGET_LOST ||
        event_code == MAIXCAM_EVENT_NO_TARGET)
    {
      if (!recovering) MissionTask_StartSupplementRecovery(now, event_code, true);
      else if (event_code == MAIXCAM_EVENT_SEARCH_TARGET_LOST &&
          mission_ctx.supplement_reference_command == MAIXCAM_COMMAND_SEARCH_TARGET &&
          mission_ctx.supplement_e3_recovery_count == 0U)
      {
        /* A real first E3 supersedes a preceding silent L45/R90 sweep.
           Only this transition starts180; further duplicate E3 cannot reset it. */
        MissionTask_StartSupplementRecovery(now, event_code, true);
      }
      /* Repeated loss notifications must not restart the maneuver. */
    }
    else
    {
      (void)DebugUart_Logf("[SUPP] IGNORE%02X WAIT XY/04/14/24/34/06\r\n",
          (unsigned int)event_code);
    }
    return;
  }
  if (recovering)
  {
    bool first_spin_turning = mission_ctx.mission_snapshot.state == MISSION_STATE_SUPPLEMENT_E3_SPIN_CCW &&
        mission_ctx.supplement_e3_spin_goal_cdeg == ROBOT_SUPPLEMENT_E3_FIRST_SPIN_CDEG;
    if (!first_spin_turning && mission_ctx.mission_snapshot.vision_target_valid)
    {
      MissionTask_ResumeSupplement(now, false);
      return;
    }
    if (mission_ctx.mission_snapshot.state == MISSION_STATE_SUPPLEMENT_E3_SPIN_CCW)
    {
      RecoveryFlow_ObserveSpin(&mission_ctx.supplement_e3_spin_previous_yaw_cdeg, &supplement_e3_spin_progress_cdeg,
          mission_ctx.mission_snapshot.yaw_cdeg, ROBOT_YAW_LEFT_SIGN);
      if (supplement_e3_spin_progress_cdeg >= mission_ctx.supplement_e3_spin_goal_cdeg)
      {
        (void)DebugUart_Logf("[SUPP] CCW%ld OK\r\n",
            (long)(mission_ctx.supplement_e3_spin_goal_cdeg / 100L));
        if (mission_ctx.supplement_e3_spin_goal_cdeg == ROBOT_SUPPLEMENT_E3_FIRST_SPIN_CDEG)
        {
          MissionTask_StopWheels();
          MissionTask_ResetVisionControllerState();
          /* Fence while the RX motion gate is still closed. Then admit only
             new parking-view coordinates; partial turn-era frames are stale. */
          mission_ctx.supplement_coordinate_floor = MaixCam_FenceObjectReference();
          mission_ctx.mission_snapshot.vision_target_valid = false;
          mission_ctx.mission_snapshot.target_present = false;
          mission_ctx.supplement_e3_first_hold_active = true;
          supplement_e3_first_hold_tick = now;
          MaixCam_SetSupplementE3TurnActive(false);
          (void)DebugUart_Log("[SUPP] STOP500 WAIT NEW XY\r\n");
          return;
        }
        MissionTask_ResumeSupplement(now, true);
        return;
      }
      /* Continuous rotation uses accumulated yaw, not the2s fixed-turn timer.
         The session-wide review deadline still preempts every tick. */
      int16_t turn_rpm = (int16_t)(-ROBOT_YAW_LEFT_SIGN * supplement_e3_spin_rpm);
      mission_ctx.mission_snapshot.vision_forward_rpm = 0;
      mission_ctx.mission_snapshot.vision_turn_rpm = turn_rpm;
      MissionTask_SetWheelTargets(turn_rpm, (int16_t)-turn_rpm);
      return;
    }
    if (!MissionTask_RunS6TurnToHeading(now,
        mission_ctx.mission_snapshot.state == MISSION_STATE_SUPPLEMENT_TURN_LEFT
            ? mission_ctx.supplement_left_yaw_cdeg : mission_ctx.supplement_right_yaw_cdeg)) return;
    MissionTask_ResetS6ControllerState();
    mission_ctx.s6_phase_stable_since = 0U;
    if (mission_ctx.mission_snapshot.state == MISSION_STATE_SUPPLEMENT_TURN_LEFT)
      MissionTask_EnterState(MISSION_STATE_SUPPLEMENT_TURN_RIGHT, now);
    else MissionTask_ResumeSupplement(now, true);
    return;
  }
  if (mission_ctx.mission_snapshot.state != MISSION_STATE_SUPPLEMENT_TRACK) return;
  if (MissionTask_CommandS4VisionTracking(now, &xy_in_tolerance))
  {
    mission_ctx.supplement_missing_since = now;
  }
  else if ((uint32_t)(now - mission_ctx.supplement_missing_since) >= ROBOT_SUPPLEMENT_LOST_MS)
  {
    MissionTask_StartSupplementRecovery(now,
        mission_ctx.supplement_reference_command == MAIXCAM_COMMAND_FINAL_CAPTURE
            ? MAIXCAM_EVENT_NO_TARGET
            : (mission_ctx.supplement_reference_command == MAIXCAM_COMMAND_TRACK_CENTER_ACK
                  ? MAIXCAM_EVENT_CENTER_TARGET_LOST : MAIXCAM_EVENT_SEARCH_TARGET_LOST), false);
  }
  if ((int32_t)(now - supplement_next_debug_tick) >= 0)
  {
    supplement_next_debug_tick = now + ROBOT_S5_DEBUG_PERIOD_MS;
    (void)DebugUart_Logf("[SUPP] ref=%02X n=%u ms=%lu id=%u q=%lu age=%lu ok=%u\r\n",
        (unsigned int)mission_ctx.supplement_reference_command, (unsigned int)mission_ctx.supplement_capture_count,
        (unsigned long)(now - mission_ctx.supplement_first_request_tick), (unsigned int)mission_ctx.mission_snapshot.target_id,
        (unsigned long)mission_ctx.mission_snapshot.target_sequence, (unsigned long)mission_ctx.mission_snapshot.target_age_ms,
        mission_ctx.mission_snapshot.vision_target_valid ? 1U : 0U);
  }
}
