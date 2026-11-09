#include "mission_strategy.h"
#include "mission_internal.h"
#include "mission_state.h"
#include "mission_safety.h"
#include "mission_vision.h"
#include "mission_supplement.h"
#include "mission_safe_zone.h"

#include "actuator_task.h"
#include "capture_flow.h"
#include "debug_uart_task.h"
#include "maixcam_task.h"
#include "robot_config.h"
#include "stm32f4xx_hal.h"

static MissionState s7_silent_resume_state;

bool MissionTask_S7MaterialOnly(void)
{
  return mission_ctx.s7_material_delivery_count >= ROBOT_S7_MATERIAL_ONLY_AFTER_TRIPS ||
      mission_ctx.s7_casualty_delivery_count >= ROBOT_S7_MATERIAL_ONLY_AFTER_CASUALTIES;
}

void MissionTask_RunS7RedE3Recovery(uint32_t now)
{
  if (MissionTask_RunRedSearchSweep(now)) MissionTask_CompleteS7RedRecovery(now);
}

void MissionTask_CompleteS7RedRecovery(uint32_t now)
{
  MissionTask_StopWheels();
  MissionTask_ResetVisionControllerState();
  MaixCam_ClearEvent();
  MaixCam_ClearObject();
  mission_ctx.mission_snapshot.vision_target_valid = false;

  if (MaixCam_SendCommand(MAIXCAM_COMMAND_RED_SEARCH_DONE) != HAL_OK)
  {
    mission_ctx.mission_snapshot.fault_flags |= MISSION_FAULT_VISION_TX;
    MissionTask_EnterState(MISSION_STATE_FAULT, now);
    return;
  }

  MissionTask_EnterState(MISSION_STATE_S7_WAIT_SECOND_E3, now);
  (void)DebugUart_Log(
      "[07] RED REC OK TX13 WAIT E3\r\n");
}

void MissionTask_StartS7Decision(uint32_t now)
{
  /* Start lowering PA2 now. Only PA3 closing waits;07 proceeds immediately. */
  MissionTask_StopWheels();
  if (Actuator_SetBaffleAngle(ROBOT_BAFFLE_DOWN_DEG) != HAL_OK)
  {
    mission_ctx.mission_snapshot.fault_flags |= MISSION_FAULT_ACTUATOR;
    MissionTask_EnterState(MISSION_STATE_FAULT, now);
    return;
  }
  Actuator_DeferClawClose(HAL_GetTick());
  mission_ctx.s6_corner_completed = false;
  mission_ctx.s5_recheck_resume_safe_tracking = false;
  mission_ctx.s5_recheck_resume_supplement = false;
  MissionTask_ResetSupplement();
  mission_ctx.s7_delivery_counted = false;
  mission_ctx.s7_material_only_reselect_pending = false;
  mission_ctx.s7_target_switch_enabled = true;
  mission_ctx.s7_red_recovery_used = false;
  mission_ctx.s51_active_object_id = 0U;
  mission_ctx.s6_carried_object_id = 0U;
  mission_ctx.s5_load_class_confirmed = false;
  mission_ctx.s7_red_switch_sync_pending = false;
  mission_ctx.s7_red_coordinate_fence_active = false;
  mission_ctx.s3_target_select_command =
      MissionTask_S7MaterialOnly()
          ? MAIXCAM_COMMAND_SELECT_BLACK_GREEN : MAIXCAM_COMMAND_SELECT_RED;
  mission_ctx.s4_arrange_cycle_count = 0U;
  mission_ctx.s4_e4_recovery_used = false;
  mission_ctx.s4_red_e4_align_away_active = false;
  mission_ctx.s4_material_e4_recovery_count = 0U;
  MissionTask_StopWheels();
  MissionTask_ResetVisionControllerState();
  MaixCam_ClearPendingInput();
  mission_ctx.mission_snapshot.vision_target_valid = false;

  if (mission_ctx.s3_target_select_command == MAIXCAM_COMMAND_SELECT_BLACK_GREEN)
  {
    /* Ordinary51/03 accepts ID5/6 and retains normal E3/04/24 handling.
       The legacy07 red-only wait cannot handle this combined task. */
    (void)DebugUart_Logf("[07] 51trips=%u red=%u >51/03\r\n",
                         (unsigned int)mission_ctx.s7_material_delivery_count,
                         (unsigned int)mission_ctx.s7_casualty_delivery_count);
    mission_ctx.s3_entry_pose_ready = true;
    (void)MissionTask_BeginS3Vision(now);
    return;
  }

  if (Actuator_SetCameraObjectTrackView() != HAL_OK)
  {
    mission_ctx.mission_snapshot.fault_flags |= MISSION_FAULT_ACTUATOR;
    MissionTask_EnterState(MISSION_STATE_FAULT, now);
    return;
  }
  if ((MaixCam_SendCommand(MAIXCAM_COMMAND_SELECT_RED) != HAL_OK) ||
      (MaixCam_SendCommand(MAIXCAM_COMMAND_SEARCH_TARGET) != HAL_OK))
  {
    mission_ctx.mission_snapshot.fault_flags |= MISSION_FAULT_VISION_TX;
    MissionTask_EnterState(MISSION_STATE_FAULT, now);
    return;
  }

  MissionTask_EnterState(MISSION_STATE_S7_SEARCH_RED, now);
  /* UART ISR can publish a new coordinate/event during the synchronous TX.
     Refresh now so the immediately following07 handler does not consume04
     against the stale pre-turn snapshot. Nothing is cleared after TX. */
  MissionTask_UpdateVisionInput(HAL_GetTick());
  (void)DebugUart_Log(
      "[07] DOWN TX11/03 WAIT XY\r\n");
}

static void MissionTask_StartBlackFinalTrack(uint32_t now)
{
  mission_ctx.s4_fine_next_left = true;
  mission_ctx.s4_arrange_cycle_count = 0U;
  MissionTask_StopWheels();
  MissionTask_ResetVisionControllerState();
  MaixCam_ClearEvent();
  MaixCam_ClearObject();
  mission_ctx.mission_snapshot.vision_target_valid = false;
  mission_ctx.s3_target_select_command = MAIXCAM_COMMAND_SELECT_BLACK;
  mission_ctx.s4_final_mode_already_active = false;
  mission_ctx.s4_final_event34_pending = false;
  mission_ctx.s4_final_missing_since = 0U;
  mission_ctx.s4_final_turn_stable_since = 0U;
  mission_ctx.s4_final_recovery_count = 0U;

  if (CaptureFlow_Open(Actuator_SetCameraWideView, Actuator_SetClawAngle,
      ROBOT_CLAW_MATERIAL_CAPTURE_OPEN_DEG) != HAL_OK)
  {
    mission_ctx.mission_snapshot.fault_flags |= MISSION_FAULT_ACTUATOR;
    MissionTask_EnterState(MISSION_STATE_FAULT, now);
    return;
  }
  if (MaixCam_SendCommand(MAIXCAM_COMMAND_SELECT_BLACK) != HAL_OK)
  {
    mission_ctx.mission_snapshot.fault_flags |= MISSION_FAULT_VISION_TX;
    MissionTask_EnterState(MISSION_STATE_FAULT, now);
    return;
  }

  /* The next dispatch sends24 without waiting for the claw to settle. */
  MissionTask_EnterState(MISSION_STATE_S4_RAISE_FRAME, now);
  (void)DebugUart_Logf(
      "[07-B] TX31 cam=%u claw=%u >TX24 SKIP03/04\r\n",
      (unsigned int)ROBOT_CAMERA_WIDE_ANGLE_DEG,
      (unsigned int)ROBOT_CLAW_MATERIAL_CAPTURE_OPEN_DEG);
}

void MissionTask_SwitchS7ToBlackGreen(uint32_t now)
{
  MissionTask_StopWheels();
  (void)DebugUart_Log("[07] E3#2 >51/03\r\n");
  (void)MissionTask_StartS3Align(now, MAIXCAM_COMMAND_SELECT_BLACK_GREEN);
}

void MissionTask_HandleRedPriorityRequest(uint32_t now)
{
  bool can_switch = false;
  /* Keep a valid11 pending, rather than consume it in the fixed AE turn. */
  if (mission_ctx.mission_snapshot.state == MISSION_STATE_VISION_AVOID_START_TURN ||
      mission_ctx.mission_snapshot.state == MISSION_STATE_VISION_AVOID_START_WAIT_ACK) return;
  if (!MaixCam_TakeRedPriorityRequest())
  {
    return;
  }
  if (mission_ctx.supplement_capture_active)
  {
    (void)DebugUart_Log("[SUPP] IGNORE11 KEEP B/G\r\n");
    return;
  }
  if (mission_ctx.s3_target_select_command == MAIXCAM_COMMAND_SELECT_RED)
  {
    (void)DebugUart_Log("[07-R] REPEAT11 KEEP PID/LOCK/N\r\n");
    return;
  }
  MissionState state = mission_ctx.mission_snapshot.state;
  bool material_command = mission_ctx.s3_target_select_command == MAIXCAM_COMMAND_SELECT_BLACK ||
      mission_ctx.s3_target_select_command == MAIXCAM_COMMAND_SELECT_BLACK_GREEN;
  bool e3_recovery = mission_ctx.vision_search_e3_mode && mission_ctx.vision_search_resume_state == MISSION_STATE_S3_TRACK_GREEN &&
      (state == MISSION_STATE_S3_E3_TURN_AWAY || state == MISSION_STATE_S3_SEARCH_TURN_CW ||
       state == MISSION_STATE_S3_SEARCH_TURN_CCW ||
       (state == MISSION_STATE_S3_TRACK_GREEN && mission_ctx.s3_search_sweep_completed && !mission_ctx.mission_snapshot.vision_target_valid));
  /* E3 and11 can both be received before the next task tick. Honor11
     without having to start an obsolete material recovery motion first. */
  uint8_t event_code;
  bool queued_e3 = material_command &&
      (state == MISSION_STATE_S3_TRACK_GREEN ||
       (mission_ctx.s4_post22_search_active && (state == MISSION_STATE_S4_WAIT_FINAL_READY ||
        (state >= MISSION_STATE_S4_POST22_REVERSE && state <= MISSION_STATE_S4_POST22_TURN_RIGHT)))) &&
      MaixCam_PeekEvent(&event_code) && event_code == MAIXCAM_EVENT_SEARCH_TARGET_LOST;
  bool material_e3_search = material_command && (e3_recovery || queued_e3);
  if (material_e3_search || (state == MISSION_STATE_S7_SEARCH_BLACK) ||
      (((mission_ctx.mission_snapshot.state == MISSION_STATE_S7_SILENT_TURN_LEFT) ||
        (mission_ctx.mission_snapshot.state == MISSION_STATE_S7_SILENT_TURN_BACK)) &&
       (s7_silent_resume_state == MISSION_STATE_S7_SEARCH_BLACK)))
  {
    can_switch = true;
  }
  else if ((mission_ctx.s3_target_select_command == MAIXCAM_COMMAND_SELECT_BLACK) ||
           ((mission_ctx.s3_target_select_command == MAIXCAM_COMMAND_SELECT_BLACK_GREEN) &&
            (mission_ctx.s51_active_object_id == 6U)))
  {
    switch (mission_ctx.mission_snapshot.state)
    {
      case MISSION_STATE_S3_ALIGN_GREEN:
      case MISSION_STATE_S3_TRACK_GREEN:
      case MISSION_STATE_S3_LOWER_FRAME:
      case MISSION_STATE_S3_SEARCH_TURN_CW:
      case MISSION_STATE_S3_SEARCH_TURN_CCW:
      case MISSION_STATE_S3_E3_TURN_AWAY:
      case MISSION_STATE_S4_TRACK_CENTER:
      case MISSION_STATE_S4_RAISE_FRAME:
      case MISSION_STATE_S4_TRACK_FINAL_BLOCK:
      case MISSION_STATE_S4_FINAL_RECOVERY_REVERSE:
      case MISSION_STATE_S4_FINAL_RECOVERY_TURN_270:
      case MISSION_STATE_S4_FINAL_RECOVERY_TURN_90:
      case MISSION_STATE_S4_E4_TURN_AWAY:
      case MISSION_STATE_S4_E4_FORWARD:
      case MISSION_STATE_S4_E4_TURN_LEFT:
      case MISSION_STATE_S4_E4_TURN_RIGHT:
      case MISSION_STATE_S4_E4_MATERIAL_SPIN_CW_360:
        can_switch = true;
        break;
      default:
        break;
    }
  }
  if (!mission_ctx.s7_target_switch_enabled || !can_switch)
  {
    (void)DebugUart_Logf("[07-R] IGNORE11 s=%u cmd=%02X id51=%u\r\n",
        (unsigned int)mission_ctx.mission_snapshot.state, (unsigned int)mission_ctx.s3_target_select_command,
        (unsigned int)mission_ctx.s51_active_object_id);
    return;
  }
  /* Material-only still selects51 on entering07. An actual ordinary E3
     allows the confirmed red target to take priority over another sweep. */
  bool keep_material = !material_e3_search && MissionTask_S7MaterialOnly();
  if (keep_material && mission_ctx.s7_material_only_reselect_pending)
  {
    (void)DebugUart_Log("[07-M] DUP11 WAIT MATERIAL XY\r\n");
    return;
  }
  MissionTask_StopWheels();
  MissionTask_ResetVisionControllerState();
  MissionTask_ResetS6ControllerState();
  MaixCam_ClearPendingInput();
  mission_ctx.mission_snapshot.vision_target_valid = false;
  mission_ctx.vision_search_forward_active = false;
  mission_ctx.s4_post22_search_active = false;
  mission_ctx.s7_material_only_reselect_pending = keep_material;
  mission_ctx.s7_red_switch_sync_pending = !keep_material;
  if (keep_material)
  {
    uint8_t material_select = mission_ctx.s3_target_select_command == MAIXCAM_COMMAND_SELECT_BLACK
        ? MAIXCAM_COMMAND_SELECT_BLACK : MAIXCAM_COMMAND_SELECT_BLACK_GREEN;
    (void)DebugUart_Logf("[07-M] RX11 KEEP >%02X/03\r\n", (unsigned int)material_select);
    (void)MissionTask_StartS3Align(now, material_select);
    return;
  }
  (void)DebugUart_Logf(
      "[07-R] RX11 cmd=%02X id51=%u >11/03\r\n",
      (unsigned int)mission_ctx.s3_target_select_command, (unsigned int)mission_ctx.s51_active_object_id);
  (void)MissionTask_StartS3Align(now, MAIXCAM_COMMAND_SELECT_RED);
}

void MissionTask_ResumeRedTracking(uint32_t now)
{
  if (!MissionTask_RedCoordinateIsFresh())
  {
    MissionTask_StopWheels();
    return;
  }
  /* Camera is already searching red. Preserve its lock, coordinates and any
     pending 04; do not send 11/03 or clear the consumed recovery allowance. */
  MissionTask_StopWheels();
  MissionTask_ResetVisionControllerState();
  MissionTask_ResetS6ControllerState();
  if ((Actuator_SetFrameLowered() != HAL_OK) ||
      (Actuator_SetCameraObjectTrackView() != HAL_OK))
  {
    mission_ctx.mission_snapshot.fault_flags |= MISSION_FAULT_ACTUATOR;
    MissionTask_EnterState(MISSION_STATE_FAULT, now);
    return;
  }
  mission_ctx.s3_target_select_command = MAIXCAM_COMMAND_SELECT_RED;
  mission_ctx.vision_search_forward_active = false;
  mission_ctx.vision_search_e3_mode = false;
  mission_ctx.vision_search_green_e3_mode = false;
  mission_ctx.vision_search_resume_state = MISSION_STATE_S3_TRACK_GREEN;
  mission_ctx.s3_search_sweep_completed = false;
  mission_ctx.s3_target_lost_since = 0U;
  mission_ctx.s3_track_stable_since = 0U;
  mission_ctx.s3_wait_event04_logged = false;
  mission_ctx.s3_next_debug_tick = now;
  MissionTask_EnterState(MISSION_STATE_S3_TRACK_GREEN, now);
  (void)DebugUart_Logf("[07] RED RESUME n=%u KEEP LOCK\r\n",
                       (unsigned int)mission_ctx.s7_red_recovery_used);
}

void MissionTask_RunS7Decision(uint32_t now)
{
  uint8_t event_code;

  if ((mission_ctx.mission_snapshot.state != MISSION_STATE_S7_SEARCH_RED) &&
      (mission_ctx.mission_snapshot.state != MISSION_STATE_S7_WAIT_SECOND_E3) &&
      (mission_ctx.mission_snapshot.state != MISSION_STATE_S7_SEARCH_BLACK))
  {
    return;
  }

  MissionTask_StopWheels();
  if (mission_ctx.mission_snapshot.vision_target_valid &&
      (mission_ctx.mission_snapshot.target_sequence != mission_ctx.s7_last_coordinate_sequence))
  {
    mission_ctx.s7_last_coordinate_sequence = mission_ctx.mission_snapshot.target_sequence;
    mission_ctx.s7_last_message_tick = now;
  }

  if (((mission_ctx.mission_snapshot.state == MISSION_STATE_S7_SEARCH_RED) ||
       (mission_ctx.mission_snapshot.state == MISSION_STATE_S7_WAIT_SECOND_E3) ||
       (mission_ctx.mission_snapshot.state == MISSION_STATE_S7_SEARCH_BLACK)) &&
      mission_ctx.mission_snapshot.vision_target_valid &&
      (mission_ctx.mission_snapshot.target_id ==
       (mission_ctx.mission_snapshot.state == MISSION_STATE_S7_SEARCH_BLACK ? 6U : 4U)))
  {
    uint8_t select_command =
        mission_ctx.mission_snapshot.state == MISSION_STATE_S7_SEARCH_BLACK
            ? MAIXCAM_COMMAND_SELECT_BLACK
            : MAIXCAM_COMMAND_SELECT_RED;
    const char *target_name =
        select_command == MAIXCAM_COMMAND_SELECT_RED ? "RED" : "BLACK";

    if (select_command == MAIXCAM_COMMAND_SELECT_BLACK)
    {
      MissionTask_StartBlackFinalTrack(now);
      return;
    }
    (void)DebugUart_Logf(
        "[07] %s FOUND id=%u x=%d y=%d "
        ">CAPTURE\r\n",
        target_name,
        (unsigned int)mission_ctx.mission_snapshot.target_id,
        (int)mission_ctx.mission_snapshot.vision_x_error_px,
        (int)mission_ctx.mission_snapshot.vision_y_error_px);
    MissionTask_ResumeRedTracking(now);
    return;
  }

  if (!MaixCam_TakeEvent(&event_code))
  {
    if ((uint32_t)(now - mission_ctx.s7_last_message_tick) >= ROBOT_S7_SILENT_TIMEOUT_MS)
    {
      if ((mission_ctx.mission_snapshot.state == MISSION_STATE_S7_SEARCH_RED) ||
          (mission_ctx.mission_snapshot.state == MISSION_STATE_S7_WAIT_SECOND_E3))
      {
        (void)DebugUart_Log("[07-S] RED SILENT, TX51/03\r\n");
        MissionTask_SwitchS7ToBlackGreen(now);
        return;
      }
      s7_silent_resume_state = mission_ctx.mission_snapshot.state;
      mission_ctx.s7_silent_left_yaw_cdeg = MissionTask_WrapYaw(
          mission_ctx.mission_snapshot.yaw_cdeg + (int32_t)ROBOT_YAW_LEFT_SIGN *
              ROBOT_S7_SILENT_LEFT_CDEG);
      mission_ctx.s7_silent_right_yaw_cdeg = MissionTask_WrapYaw(
          mission_ctx.s7_silent_left_yaw_cdeg - (int32_t)ROBOT_YAW_LEFT_SIGN *
              ROBOT_S7_SILENT_RIGHT_CDEG);
      MissionTask_ResetVisionControllerState();
      MissionTask_ResetS6ControllerState();
      mission_ctx.s6_phase_stable_since = 0U;
      mission_ctx.s6_next_debug_tick = now;
      MissionTask_EnterState(MISSION_STATE_S7_SILENT_TURN_LEFT, now);
      (void)DebugUart_Logf(
          "[07-S] SILENT=%lu L=%ld R=%ld resume=%u\r\n",
          (unsigned long)ROBOT_S7_SILENT_TIMEOUT_MS,
          (long)mission_ctx.s7_silent_left_yaw_cdeg, (long)mission_ctx.s7_silent_right_yaw_cdeg,
          (unsigned int)s7_silent_resume_state);
    }
    return;
  }
  mission_ctx.s7_last_message_tick = now;

  if (event_code != MAIXCAM_EVENT_SEARCH_TARGET_LOST)
  {
    (void)DebugUart_Logf("[07] RX%02X WAIT E3\r\n",
                         (unsigned int)event_code);
    return;
  }

  if (mission_ctx.mission_snapshot.state == MISSION_STATE_S7_SEARCH_RED)
  {
    if (mission_ctx.mission_snapshot.vision_target_valid)
    {
      (void)DebugUart_Log(
          "[07] RXE3 FRESH RED XY KEEP\r\n");
      return;
    }

    MaixCam_ClearObject();
    mission_ctx.mission_snapshot.vision_target_valid = false;
    (void)DebugUart_Log(
        "[07] E3#1 >REC\r\n");
    MissionTask_StartVisionSearchRecovery(
        now, MISSION_STATE_S7_SEARCH_RED, mission_ctx.mission_snapshot.yaw_cdeg, false);
    return;
  }

  if (mission_ctx.mission_snapshot.state == MISSION_STATE_S7_WAIT_SECOND_E3)
  {
    MissionTask_SwitchS7ToBlackGreen(now);
    return;
  }

  (void)DebugUart_Log(
      "[07] BLACK RXE3 WAIT POLICY\r\n");
}

void MissionTask_RunS7SilentSearch(uint32_t now)
{
  uint8_t expected_id;
  bool turning_left;
  if ((mission_ctx.mission_snapshot.state != MISSION_STATE_S7_SILENT_TURN_LEFT) &&
      (mission_ctx.mission_snapshot.state != MISSION_STATE_S7_SILENT_TURN_BACK))
  {
    return;
  }
  expected_id = s7_silent_resume_state == MISSION_STATE_S7_SEARCH_BLACK ? 6U : 4U;
  if (mission_ctx.mission_snapshot.vision_target_valid &&
      (mission_ctx.mission_snapshot.target_id == expected_id))
  {
    MissionTask_StopWheels();
    MissionTask_ResetS6ControllerState();
    (void)DebugUart_Logf("[07-S] FOUND id=%u STOP >CAPTURE\r\n",
                         (unsigned int)expected_id);
    MissionTask_EnterState(s7_silent_resume_state, now);
    return;
  }
  turning_left = mission_ctx.mission_snapshot.state == MISSION_STATE_S7_SILENT_TURN_LEFT;
  if (!MissionTask_RunS6TurnToHeading(
          now, turning_left ? mission_ctx.s7_silent_left_yaw_cdeg : mission_ctx.s7_silent_right_yaw_cdeg))
  {
    return;
  }
  MissionTask_StopWheels();
  MissionTask_ResetS6ControllerState();
  mission_ctx.s6_phase_stable_since = 0U;
  if (turning_left)
  {
    MissionTask_EnterState(MISSION_STATE_S7_SILENT_TURN_BACK, now);
    (void)DebugUart_Log("[07-S] L45 OK, RIGHT90\r\n");
  }
  else
  {
    MissionTask_EnterState(s7_silent_resume_state, now);
    (void)DebugUart_Log("[07-S] R90 OK, RESUME 07 WAIT\r\n");
  }
}
