#include "mission_vision.h"
#include "mission_internal.h"
#include "mission_state.h"
#include "mission_safety.h"
#include "mission_arrange.h"
#include "mission_load.h"
#include "mission_supplement.h"
#include "mission_safe_zone.h"
#include "mission_strategy.h"

#include "M2006.h"
#include "actuator_task.h"
#include "chassis_motion.h"
#include "recovery_flow.h"
#include "debug_uart_task.h"
#include "maixcam_task.h"
#include "robot_config.h"
#include "stm32f4xx_hal.h"
#include "vision_motion.h"
#include "flow_motion.h"

static uint32_t vision_age_next_debug_tick;
static uint32_t recovery_duplicate_next_debug_tick;
static uint32_t s7_red_coordinate_sequence_floor;
static int32_t vision_search_turn_angle_cdeg;
static int32_t s7_red_e3_origin_yaw_cdeg;
typedef enum
{
  S3_GREEN_E3_FORWARD = 0,
  S3_GREEN_E3_SPIN_CCW
} S3GreenE3Phase;
static S3GreenE3Phase s3_green_e3_phase;
static int32_t s3_green_e3_origin_yaw_cdeg;
static int32_t e3_spin_previous_yaw_cdeg;
static int32_t e3_spin_progress_cdeg;
static int32_t ae_previous_yaw_cdeg;
static int32_t ae_turn_progress_cdeg;
static uint32_t ae_next_debug_tick;
static uint32_t ae_reply_sequence_floor;
static uint32_t ae_fresh_coordinate_sequence;
static uint32_t ae_next_reply_tick;
static bool ae_resume_prepared;
static bool ae_fresh_coordinate_seen;

void MissionTask_ResetVisionControllerState(void)
{
  VisionMotion_Reset(&mission_ctx.vision_motion);
  mission_ctx.mission_snapshot.vision_turn_rpm = 0;
  mission_ctx.mission_snapshot.vision_forward_rpm = 0;
}

void MissionTask_ResetVisionPid(void)
{
  MissionTask_ResetVisionControllerState();
  mission_ctx.mission_snapshot.vision_x_error_px = 0;
  mission_ctx.mission_snapshot.vision_y_error_px = 0;
}

uint8_t MissionTask_ActiveRecoveryLossEvent(void)
{
  MissionState state = mission_ctx.mission_snapshot.state;
  if (state == MISSION_STATE_SUPPLEMENT_TURN_LEFT ||
      state == MISSION_STATE_SUPPLEMENT_TURN_RIGHT ||
      state == MISSION_STATE_SUPPLEMENT_E3_SPIN_CCW)
  {
    /* A silent sweep must still accept the FIRST real E3 so it can switch
       to the180deg action. Only later duplicate E3 packets are filtered. */
    if (mission_ctx.supplement_recovery_event == MAIXCAM_EVENT_SEARCH_TARGET_LOST &&
        mission_ctx.supplement_e3_recovery_count == 0U) return 0U;
    return mission_ctx.supplement_recovery_event;
  }
  if ((state == MISSION_STATE_S3_SEARCH_TURN_CW) ||
      (state == MISSION_STATE_S3_SEARCH_TURN_CCW) ||
      (state == MISSION_STATE_S3_E3_TURN_AWAY))
  {
    return mission_ctx.vision_search_resume_state == MISSION_STATE_S4_TRACK_CENTER
        ? MAIXCAM_EVENT_CENTER_TARGET_LOST : MAIXCAM_EVENT_SEARCH_TARGET_LOST;
  }
  if (((state >= MISSION_STATE_S4_E4_PUSH_RIGHT) &&
       (state <= MISSION_STATE_S4_E4_TURN_RIGHT)) ||
      (state == MISSION_STATE_S4_E4_GREEN_SPIN_360) ||
      (state == MISSION_STATE_S4_E4_RED_SPIN_360) ||
      (state == MISSION_STATE_S4_E4_MATERIAL_SPIN_CW_360))
    return MAIXCAM_EVENT_CENTER_TARGET_LOST;
  if ((state >= MISSION_STATE_S4_FINAL_RECOVERY_REVERSE) &&
      (state <= MISSION_STATE_S4_FINAL_RECOVERY_TURN_90))
    return MAIXCAM_EVENT_NO_TARGET;
  return 0U;
}

void MissionTask_DropDuplicateRecoveryEvent(uint32_t now)
{
  uint8_t event_code = MissionTask_ActiveRecoveryLossEvent();
  if ((event_code != 0U) && MaixCam_DropEventIf(event_code) &&
      ((int32_t)(now - recovery_duplicate_next_debug_tick) >= 0))
  {
    recovery_duplicate_next_debug_tick = now + ROBOT_VISION_COORD_DEBUG_PERIOD_MS;
    (void)DebugUart_Logf("[DUP] s=%u RX%02X IGNORE\r\n",
                         (unsigned int)mission_ctx.mission_snapshot.state,
                         (unsigned int)event_code);
  }
}

static bool MissionTask_IsObjectTrackingPhase(void)
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
    case MISSION_STATE_S4_FINE32_TRACK:
    case MISSION_STATE_S4_FINE42_TRACK:
    case MISSION_STATE_S4_RAISE_FRAME:
    case MISSION_STATE_S4_TRACK_FINAL_BLOCK:
    case MISSION_STATE_S4_FINAL_RECOVERY_REVERSE:
    case MISSION_STATE_S4_FINAL_RECOVERY_TURN_270:
    case MISSION_STATE_S4_FINAL_RECOVERY_TURN_90:
    case MISSION_STATE_S4_E4_TURN_AWAY:
    case MISSION_STATE_S4_E4_FORWARD:
    case MISSION_STATE_S4_E4_TURN_LEFT:
    case MISSION_STATE_S4_E4_TURN_RIGHT:
    case MISSION_STATE_S4_E4_GREEN_SPIN_360:
    case MISSION_STATE_S4_E4_RED_SPIN_360:
    case MISSION_STATE_S4_E4_MATERIAL_SPIN_CW_360:
    case MISSION_STATE_S7_SEARCH_RED:
    case MISSION_STATE_S7_WAIT_SECOND_E3:
    case MISSION_STATE_S7_SEARCH_BLACK:
    case MISSION_STATE_S7_SILENT_TURN_LEFT:
    case MISSION_STATE_S7_SILENT_TURN_BACK:
    case MISSION_STATE_SUPPLEMENT_TRACK:
    case MISSION_STATE_SUPPLEMENT_TURN_LEFT:
    case MISSION_STATE_SUPPLEMENT_TURN_RIGHT:
    case MISSION_STATE_SUPPLEMENT_E3_SPIN_CCW:
      return true;
    default:
      return false;
  }
}

bool MissionTask_RedCoordinateIsFresh(void)
{
  return mission_ctx.mission_snapshot.vision_target_valid &&
      mission_ctx.mission_snapshot.target_id == 4U &&
      mission_ctx.mission_snapshot.target_age_ms <= MISSION_VISION_TARGET_TIMEOUT_MS &&
      !mission_ctx.s7_red_switch_sync_pending &&
      (!mission_ctx.s7_red_coordinate_fence_active ||
       (int32_t)(mission_ctx.mission_snapshot.target_sequence -
                 s7_red_coordinate_sequence_floor) > 0);
}

void MissionTask_UpdateVisionInput(uint32_t now)
{
  MaixCam_Object object;
  bool object_present = MaixCam_GetObjectSnapshot(&object);
  mission_ctx.mission_snapshot.target_present = object_present;
  mission_ctx.mission_snapshot.target_geometry = object_present ? object.geometry : 0U;
  mission_ctx.mission_snapshot.target_age_ms = object.sequence != 0U
      ? (uint32_t)(now - object.update_tick) : UINT32_MAX;
  mission_ctx.mission_snapshot.vision_target_valid = object_present &&
      mission_ctx.mission_snapshot.target_age_ms <= MISSION_VISION_TARGET_TIMEOUT_MS;
  if (object.sequence != 0U)
  {
    mission_ctx.mission_snapshot.target_id = object.object_id;
    mission_ctx.mission_snapshot.vision_x_error_px = object.x_error_px;
    mission_ctx.mission_snapshot.vision_y_error_px = object.y_error_px;
    mission_ctx.mission_snapshot.target_sequence = object.sequence;
  }

  if (MissionTask_IsObjectTrackingPhase())
  {
    if (!mission_ctx.s7_target_switch_enabled)
    {
      /* Before the first delivery, target tracking is locked to green.
         Do not apply this filter to 02/12 corner arrangement or zone tracking. */
      mission_ctx.mission_snapshot.vision_target_valid = mission_ctx.mission_snapshot.vision_target_valid &&
          object.object_id == 5U;
    }
    else if (mission_ctx.s3_target_select_command == MAIXCAM_COMMAND_SELECT_RED)
    {
      mission_ctx.mission_snapshot.vision_target_valid = MissionTask_RedCoordinateIsFresh();
    }
    else if (mission_ctx.s3_target_select_command == MAIXCAM_COMMAND_SELECT_BLACK_GREEN)
    {
      mission_ctx.mission_snapshot.vision_target_valid = mission_ctx.mission_snapshot.vision_target_valid &&
          ((object.object_id == 5U) || (object.object_id == 6U));
      if (mission_ctx.mission_snapshot.vision_target_valid)
      {
        mission_ctx.s51_active_object_id = object.object_id;
        if (mission_ctx.mission_snapshot.state == MISSION_STATE_S3_TRACK_GREEN)
          mission_ctx.s7_material_only_reselect_pending = false;
      }
    }
    else if (mission_ctx.s3_target_select_command == MAIXCAM_COMMAND_SELECT_BLACK)
    {
      mission_ctx.mission_snapshot.vision_target_valid = mission_ctx.mission_snapshot.vision_target_valid && object.object_id == 6U;
      if (mission_ctx.mission_snapshot.vision_target_valid && mission_ctx.mission_snapshot.state == MISSION_STATE_S3_TRACK_GREEN)
        mission_ctx.s7_material_only_reselect_pending = false;
    }
  }
  if (mission_ctx.supplement_capture_active)
  {
    mission_ctx.mission_snapshot.vision_target_valid = mission_ctx.mission_snapshot.vision_target_valid &&
        (object.object_id == 5U || object.object_id == 6U) &&
        (int32_t)(object.sequence - mission_ctx.supplement_coordinate_floor) > 0;
  }
  if (mission_ctx.mission_snapshot.state == MISSION_STATE_S6_RECHECK_WIDE_SETTLE ||
      (mission_ctx.mission_snapshot.state >= MISSION_STATE_S6_SEARCH_SAFE_ZONE &&
       mission_ctx.mission_snapshot.state <= MISSION_STATE_S6_TRACK_SAFE_ZONE) ||
      (mission_ctx.mission_snapshot.state >= MISSION_STATE_S6_OBSTACLE_TURN_OUT &&
       mission_ctx.mission_snapshot.state <= MISSION_STATE_S6_TRACK_TIMEOUT_REVERSE) ||
      mission_ctx.mission_snapshot.state == MISSION_STATE_S6_REPOSITION_TURN_SAFE ||
      (mission_ctx.mission_snapshot.state >= MISSION_STATE_S6_CORNER_TURN_AWAY &&
       mission_ctx.mission_snapshot.state <= MISSION_STATE_S6_CORNER_TURN_SAFE))
  {
    /* A late full cargo-report duplicate is still not a zone PID target.
       Retain only ID1/2 zone frames across the near-to-wide boundary. */
    mission_ctx.mission_snapshot.vision_target_valid = mission_ctx.mission_snapshot.vision_target_valid &&
        (object.object_id == 1U || object.object_id == 2U);
  }
  /* Keep UART reception live, but do not track during fixed maneuvers. */
  if (mission_ctx.mission_snapshot.state == MISSION_STATE_S6_EXIT_TURN_180 ||
      mission_ctx.mission_snapshot.state == MISSION_STATE_S4_FINAL_TIMEOUT_BACKOFF ||
      mission_ctx.mission_snapshot.state == MISSION_STATE_S5_REVERSE_FOR_ARRANGE ||
      mission_ctx.mission_snapshot.state == MISSION_STATE_S4_FINAL_CENTER_OBJECT ||
      mission_ctx.mission_snapshot.state == MISSION_STATE_S4_FINAL_TURN_SAFE ||
      mission_ctx.mission_snapshot.state == MISSION_STATE_S4_CASUALTY_REGRIP ||
      mission_ctx.s4_green_e4_half_turn_active ||
      mission_ctx.mission_snapshot.state == MISSION_STATE_VISION_AVOID_START_TURN ||
      mission_ctx.mission_snapshot.state == MISSION_STATE_VISION_AVOID_START_WAIT_ACK ||
      (mission_ctx.mission_snapshot.state >= MISSION_STATE_S6_BAR_REVERSE &&
       mission_ctx.mission_snapshot.state <= MISSION_STATE_S6_BAR_WAIT_ACK))
  {
    mission_ctx.mission_snapshot.vision_target_valid = false;
  }
  /* A wrong-class or maneuver-era frame must not authorize another180.
     Remember the first usable post-ACK frame even if it later expires. */
  if (mission_ctx.ae_ack_completed && !ae_fresh_coordinate_seen && mission_ctx.mission_snapshot.vision_target_valid &&
      MissionTask_IsObjectTrackingPhase() &&
      (int32_t)(mission_ctx.mission_snapshot.target_sequence - ae_reply_sequence_floor) > 0)
  {
    ae_fresh_coordinate_sequence = mission_ctx.mission_snapshot.target_sequence;
    ae_fresh_coordinate_seen = true;
  }
}

bool MissionTask_BeginS3Vision(uint32_t now)
{
  if (MissionTask_HandleSupplementFinish(now) || MissionTask_LoadCheckPausesMotion()) return false;
  /* Every new03 search starts with the frame DOWN, even without coordinates.
     Some recovery paths enter here directly, bypassing StartS3Align. */
  MissionTask_StopWheels();
  bool pose_ready = mission_ctx.s3_entry_pose_ready;
  mission_ctx.s3_entry_pose_ready = false;
  if ((!pose_ready && Actuator_SetFrameLowered() != HAL_OK) ||
      Actuator_SetCameraObjectTrackView() != HAL_OK)
  {
    mission_ctx.mission_snapshot.fault_flags |= MISSION_FAULT_ACTUATOR;
    MissionTask_EnterState(MISSION_STATE_FAULT, now);
    return false;
  }
  mission_ctx.s4_e4_protocol_active = false;
  if ((mission_ctx.s3_target_select_command != MAIXCAM_COMMAND_SELECT_BLACK) &&
      (mission_ctx.s3_target_select_command != MAIXCAM_COMMAND_SELECT_BLACK_GREEN))
  {
    mission_ctx.s4_material_e4_recovery_count = 0U;
  }
  /* This entry initializes a new03 task. Coordinate-only resume never calls it. */
  mission_ctx.s4_fine_next_left = true;
  mission_ctx.s51_e3_recovery_count = 0U;
  mission_ctx.s51_second_e3_pending = false;
  if (mission_ctx.s3_target_select_command == MAIXCAM_COMMAND_SELECT_BLACK_GREEN)
  {
    mission_ctx.s51_active_object_id = 0U;
  }
  const char *target_name =
      mission_ctx.s3_target_select_command == MAIXCAM_COMMAND_SELECT_RED
          ? "RED"
          : (mission_ctx.s3_target_select_command == MAIXCAM_COMMAND_SELECT_BLACK
                 ? "BLACK"
                 : (mission_ctx.s3_target_select_command == MAIXCAM_COMMAND_SELECT_BLACK_GREEN
                        ? "BLACK/GREEN" : "GREEN"));

  MissionTask_EnterState(MISSION_STATE_S3_TRACK_GREEN, now);

  /* A coordinate received before command 03 belongs to the previous camera
     mode and must never feed the new PID loop. */
  MaixCam_ClearObject();
  MaixCam_ClearEvent();
  mission_ctx.mission_snapshot.vision_target_valid = false;
  MissionTask_ResetVisionPid();
  mission_ctx.s3_track_stable_since = 0U;
  mission_ctx.s3_search_turn_stable_since = 0U;
  mission_ctx.s3_search_origin_yaw_cdeg =
      MissionTask_WrapYaw(mission_ctx.mission_snapshot.yaw_cdeg);
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

  bool priority_reply = mission_ctx.s7_red_switch_sync_pending || mission_ctx.s7_material_only_reselect_pending;
  if (priority_reply)
  {
    /* Retire the old material search before the select/03 handshake. */
    MaixCam_ClearPendingInput();
  }
  if (MaixCam_SendCommand(mission_ctx.s3_target_select_command) != HAL_OK)
  {
    mission_ctx.mission_snapshot.fault_flags |= MISSION_FAULT_VISION_TX;
    MissionTask_EnterState(MISSION_STATE_FAULT, now);
    return false;
  }
  (void)DebugUart_Logf(
      "[03] TX%02X, SELECT %s\r\n",
      (unsigned int)mission_ctx.s3_target_select_command,
      target_name);
  if (MissionTask_HandleSupplementFinish(HAL_GetTick()) || MissionTask_LoadCheckPausesMotion()) return false;

  if (priority_reply)
  {
    /* Frames before03 are not image-center coordinates. Arm the fence now,
       so a new coordinate sent immediately on RX03 survives the blocking TX. */
    uint32_t sequence_floor = MaixCam_FenceObjectReference();
    if (mission_ctx.s7_red_switch_sync_pending)
    {
      s7_red_coordinate_sequence_floor = sequence_floor;
      mission_ctx.s7_red_coordinate_fence_active = true;
    }
  }
  if (MaixCam_SendCommand(MAIXCAM_COMMAND_SEARCH_TARGET) != HAL_OK)
  {
    mission_ctx.mission_snapshot.fault_flags |= MISSION_FAULT_VISION_TX;
    MissionTask_EnterState(MISSION_STATE_FAULT, now);
    return false;
  }
  (void)DebugUart_Log("[03] TX03, SEARCH\r\n");
  if (mission_ctx.s7_red_switch_sync_pending)
  {
    mission_ctx.mission_snapshot.vision_target_valid = false;
    mission_ctx.s7_red_switch_sync_pending = false;
    (void)DebugUart_Logf(
        "[07-R] TX11/03 OK WAIT ID4 q>%lu REF=IMG\r\n",
        (unsigned long)s7_red_coordinate_sequence_floor);
  }
  return true;
}

bool MissionTask_StartS3Align(uint32_t now, uint8_t select_command)
{
  if (!mission_ctx.s7_target_switch_enabled &&
      select_command != MAIXCAM_COMMAND_SELECT_GREEN)
  {
    (void)DebugUart_Logf(
        "[03] SWITCH %02X BLOCK<07, KEEP GREEN21\r\n",
        (unsigned int)select_command);
    select_command = MAIXCAM_COMMAND_SELECT_GREEN;
  }
  if ((select_command == MAIXCAM_COMMAND_SELECT_RED) &&
      (mission_ctx.s3_target_select_command != MAIXCAM_COMMAND_SELECT_RED))
  {
    /* A newly selected red target gets its own first E4 search. */
    mission_ctx.s4_e4_recovery_used = false;
    mission_ctx.s4_red_e4_align_away_active = false;
    mission_ctx.s7_red_recovery_used = false;
    mission_ctx.s7_red_coordinate_fence_active = false;
  }
  if (mission_ctx.s3_target_select_command != select_command)
  {
    mission_ctx.s4_material_e4_recovery_count = 0U;
  }
  mission_ctx.s3_target_select_command = select_command;
  if (select_command == MAIXCAM_COMMAND_SELECT_GREEN)
  {
    mission_ctx.s4_green_e4_push_cycles = 0U;
    mission_ctx.s4_green_e4_spin_exhausted = false;
  }
  mission_ctx.s4_final_mode_already_active = false;
  mission_ctx.s4_final_event34_pending = false;
  mission_ctx.s4_final_missing_since = 0U;
  mission_ctx.s4_final_turn_stable_since = 0U;
  mission_ctx.s4_final_recovery_count = 0U;
  /* S2 already set PA2 down/PA3 closed after its dedicated waits. */
  mission_ctx.s3_entry_pose_ready = mission_ctx.mission_snapshot.state == MISSION_STATE_S2_CROSS_BUMP;
  if (!mission_ctx.s3_entry_pose_ready && Actuator_SetFrameLowered() != HAL_OK)
  {
    mission_ctx.mission_snapshot.fault_flags |= MISSION_FAULT_ACTUATOR;
    MissionTask_EnterState(MISSION_STATE_FAULT, now);
    return false;
  }

  MissionTask_EnterState(MISSION_STATE_S3_LOWER_FRAME, now);
  (void)DebugUart_Logf(
      "[03] DOWN PA2=%u CLAW=%u wait=%lu, > 03\r\n",
      (unsigned int)ROBOT_BAFFLE_DOWN_DEG,
      (unsigned int)ROBOT_CLAW_CLOSED_DEG,
      (unsigned long)ROBOT_S3_FRAME_LOWER_SETTLE_MS);
  return true;
}

static VisionMotion_Profile MissionTask_GetVisionMotionProfile(void)
{
  switch (mission_ctx.mission_snapshot.state)
  {
    case MISSION_STATE_S3_TRACK_GREEN: return VISION_MOTION_SEARCH;
    case MISSION_STATE_S4_TRACK_CENTER: return VISION_MOTION_CENTER;
    case MISSION_STATE_S4_TRACK_FINAL_BLOCK:
    case MISSION_STATE_S4_FINE32_TRACK:
    case MISSION_STATE_S4_FINE42_TRACK: return VISION_MOTION_FINAL;
    case MISSION_STATE_S6_TRACK_SAFE_ZONE: return VISION_MOTION_SAFE_ZONE;
    case MISSION_STATE_SUPPLEMENT_TRACK:
      if (mission_ctx.supplement_reference_command == MAIXCAM_COMMAND_FINAL_CAPTURE) return VISION_MOTION_FINAL;
      if (mission_ctx.supplement_reference_command == MAIXCAM_COMMAND_SEARCH_TARGET) return VISION_MOTION_SEARCH;
      if (mission_ctx.supplement_reference_command == MAIXCAM_COMMAND_TRACK_CENTER_ACK) return VISION_MOTION_CENTER;
      break;
    default: break;
  }
  return VISION_MOTION_GENERAL;
}

int16_t MissionTask_CalculateVisionTurn(uint32_t now, int16_t error_px)
{
  return VisionMotion_CalculateTurn(&mission_ctx.vision_motion.x, MissionTask_GetVisionMotionProfile(),
      now, mission_ctx.mission_snapshot.target_sequence, error_px, MISSION_TASK_PERIOD_TICKS);
}

static void MissionTask_DebugS3(uint32_t now)
{
  int16_t left_actual_rpm;
  int16_t right_actual_rpm;

  if ((int32_t)(now - mission_ctx.s3_next_debug_tick) < 0)
  {
    return;
  }
  mission_ctx.s3_next_debug_tick = now + ROBOT_VISION_DEBUG_PERIOD_MS;

  left_actual_rpm = (int16_t)(
      ((float)moto_chassis[MISSION_LEFT_MOTOR_INDEX].speed_rpm / 36.0f) *
      ROBOT_LEFT_WHEEL_FORWARD_SIGN);
  right_actual_rpm = (int16_t)(
      ((float)moto_chassis[MISSION_RIGHT_MOTOR_INDEX].speed_rpm / 36.0f) *
      ROBOT_RIGHT_WHEEL_FORWARD_SIGN);

  if (!mission_ctx.mission_snapshot.vision_target_valid)
  {
    (void)DebugUart_Logf("[03] s=%u LOST w=0/0\r\n",
                         (unsigned int)mission_ctx.mission_snapshot.state);
    return;
  }

  (void)DebugUart_Logf(
      "[03] id=%u x=%d y=%d u=%d v=%d w=%d/%d act=%d/%d\r\n",
      (unsigned int)mission_ctx.mission_snapshot.target_id,
      (int)mission_ctx.mission_snapshot.vision_x_error_px,
      (int)mission_ctx.mission_snapshot.vision_y_error_px,
      (int)mission_ctx.mission_snapshot.vision_turn_rpm,
      (int)mission_ctx.mission_snapshot.vision_forward_rpm,
      (int)mission_ctx.mission_snapshot.left_target_rpm,
      (int)mission_ctx.mission_snapshot.right_target_rpm,
      (int)left_actual_rpm,
      (int)right_actual_rpm);
}

/* Both ordinary03 and the post22 TX51/03 handoff use exactly this E3 policy. */
bool MissionTask_HandleS3TargetLost(uint32_t now)
{
  /* A coordinate that became fresh in the same control cycle is newer
     evidence than a queued E3 and must resume the vision controller. */
  if (mission_ctx.mission_snapshot.vision_target_valid)
  {
    (void)DebugUart_Log(
        "[03] RX E3 WITH NEW XY, KEEP\r\n");
    return false;
  }

  MaixCam_ClearObject();
  mission_ctx.mission_snapshot.vision_target_valid = false;
  mission_ctx.s3_target_lost_since = now;
  mission_ctx.s3_search_sweep_completed = false;

  if ((mission_ctx.mission_snapshot.state == MISSION_STATE_S3_SEARCH_TURN_CW) ||
      (mission_ctx.mission_snapshot.state == MISSION_STATE_S3_SEARCH_TURN_CCW) ||
      (mission_ctx.mission_snapshot.state == MISSION_STATE_S3_E3_TURN_AWAY))
  {
    (void)DebugUart_Log(
        "[03-S] RXE3, REC ACTIVE\r\n");
    return true;
  }

  (void)DebugUart_Log(
      "[03] RXE3, REC\r\n");
  if (mission_ctx.s3_target_select_command == MAIXCAM_COMMAND_SELECT_RED)
  {
    /* A transient red coordinate may have moved07 into03. Keep its recovery
       allowance across reacquisition rather than initializing the task. */
    (void)DebugUart_Log(
        "[07] RED LOST >REC/TX13; NEXT E3>51/03\r\n");
    MissionTask_StartVisionSearchRecovery(
        now, MISSION_STATE_S7_SEARCH_RED, mission_ctx.mission_snapshot.yaw_cdeg, false);
    return true;
  }
  if (mission_ctx.s7_target_switch_enabled &&
      mission_ctx.s3_target_select_command == MAIXCAM_COMMAND_SELECT_BLACK_GREEN &&
      !mission_ctx.supplement_budget_started && !mission_ctx.supplement_capture_active)
  {
    if (mission_ctx.s51_e3_recovery_count < UINT16_MAX) ++mission_ctx.s51_e3_recovery_count;
    mission_ctx.s51_second_e3_pending = mission_ctx.s51_e3_recovery_count == 2U;
    (void)DebugUart_Logf("[S51-E3] RX E3 n=%u second=%u\r\n",
        (unsigned int)mission_ctx.s51_e3_recovery_count,
        (unsigned int)mission_ctx.s51_second_e3_pending);
  }
  MissionTask_StartVisionSearchRecovery(
      now, MISSION_STATE_S3_TRACK_GREEN, mission_ctx.s3_search_origin_yaw_cdeg,
      mission_ctx.s3_target_select_command == MAIXCAM_COMMAND_SELECT_GREEN);
  return true;
}

static bool MissionTask_TryStartS3Capture(uint32_t now)
{
  uint8_t event_code;

  if (!MaixCam_TakeEvent(&event_code))
  {
    return false;
  }

  if (event_code == MAIXCAM_EVENT_SEARCH_TARGET_LOST)
  {
    return MissionTask_HandleS3TargetLost(now);
  }

  if (event_code != MAIXCAM_EVENT_OBJECT_IN_FRAME)
  {
    return false;
  }

  (void)DebugUart_Log("[03] RX04, OBJECT IN FRAME\r\n");
  MissionTask_StartS4TrackCenterFrom04(now);
  return true;
}

void MissionTask_StartVisionSearchRecovery(
    uint32_t now, MissionState resume_state, int32_t origin_yaw_cdeg,
    bool initial_green_e3)
{
  const char *source = resume_state == MISSION_STATE_S4_TRACK_CENTER
                           ? "04"
                           : (resume_state == MISSION_STATE_S7_SEARCH_RED
                                  ? "07"
                                  : "03");
  bool ordinary_s51_recovery = mission_ctx.s7_target_switch_enabled &&
      mission_ctx.s3_target_select_command == MAIXCAM_COMMAND_SELECT_BLACK_GREEN &&
      resume_state == MISSION_STATE_S3_TRACK_GREEN &&
      !mission_ctx.supplement_budget_started && !mission_ctx.supplement_capture_active;
  bool second_s51_e3 = mission_ctx.s51_second_e3_pending && ordinary_s51_recovery;
  /* Consume only the explicit E3 request: a later silent timeout is not E3#2. */
  mission_ctx.s51_second_e3_pending = false;

  if (MissionTask_ActiveRecoveryLossEvent() != 0U) return;

  if (mission_ctx.s7_target_switch_enabled &&
      resume_state == MISSION_STATE_S7_SEARCH_RED)
  {
    if (mission_ctx.s7_red_recovery_used)
    {
      (void)DebugUart_Log("[07] RED LOST2 >51/03\r\n");
      MissionTask_SwitchS7ToBlackGreen(now);
      return;
    }
    mission_ctx.s7_red_recovery_used = true;
    (void)DebugUart_Log("[07] RED REC n=1 KEEP\r\n");
  }

  MissionTask_StopWheels();
  MissionTask_ResetVisionControllerState();
  mission_ctx.vision_search_resume_state = resume_state;
  mission_ctx.vision_search_forward_active = true;
  mission_ctx.vision_search_green_e3_mode = initial_green_e3 &&
      resume_state == MISSION_STATE_S3_TRACK_GREEN &&
      mission_ctx.s3_target_select_command == MAIXCAM_COMMAND_SELECT_GREEN;
  mission_ctx.vision_search_e3_mode = resume_state != MISSION_STATE_S4_TRACK_CENTER &&
      !mission_ctx.vision_search_green_e3_mode;
  /* Keep legacy 04 coordinate-timeout recovery separate from E3 searches. */
  mission_ctx.vision_search_forward_rpm = resume_state == MISSION_STATE_S4_TRACK_CENTER
      ? ROBOT_VISION_SEARCH_FORWARD_RPM : ROBOT_E3_SEARCH_REVERSE_RPM;
  mission_ctx.vision_search_forward_ms = resume_state == MISSION_STATE_S4_TRACK_CENTER
      ? ROBOT_VISION_SEARCH_FORWARD_MS : ROBOT_E3_SEARCH_REVERSE_MS;
  vision_search_turn_angle_cdeg = ROBOT_S3_SEARCH_TURN_ANGLE_CDEG;
  mission_ctx.s3_search_origin_yaw_cdeg = MissionTask_WrapYaw(origin_yaw_cdeg);
  mission_ctx.s3_search_yaw_target_cdeg = MissionTask_WrapYaw(
      mission_ctx.s3_search_origin_yaw_cdeg -
      (int32_t)ROBOT_YAW_LEFT_SIGN * vision_search_turn_angle_cdeg);
  mission_ctx.s3_search_turn_stable_since = 0U;
  mission_ctx.s3_search_ccw_midpoint_done = false;
  mission_ctx.s3_next_debug_tick = now;
  if (second_s51_e3)
  {
    mission_ctx.s3_search_yaw_target_cdeg = MissionTask_WrapYaw(
        mission_ctx.mission_snapshot.safe_zone_yaw_target_cdeg + 18000L);
    MissionTask_ResetS6ControllerState();
  }
  MissionTask_EnterState(second_s51_e3 ? MISSION_STATE_S3_E3_TURN_AWAY
                                    : MISSION_STATE_S3_SEARCH_TURN_CW, now);
  if (mission_ctx.vision_search_green_e3_mode)
  {
    s3_green_e3_phase = S3_GREEN_E3_FORWARD;
    s3_green_e3_origin_yaw_cdeg = MissionTask_WrapYaw(mission_ctx.mission_snapshot.yaw_cdeg);
    mission_ctx.s3_search_yaw_target_cdeg = s3_green_e3_origin_yaw_cdeg;
    mission_ctx.vision_search_forward_rpm = ROBOT_S3_GREEN_E3_FORWARD_RPM;
    mission_ctx.vision_search_forward_ms = ROBOT_S3_GREEN_E3_FORWARD_MS;
    MissionTask_ResetS6ControllerState();
    (void)DebugUart_Logf(
        "[03-E3] FWD=%d/%lu >CCW360 ref=%ld\r\n",
        ROBOT_S3_GREEN_E3_FORWARD_RPM,
        (unsigned long)ROBOT_S3_GREEN_E3_FORWARD_MS,
        (long)s3_green_e3_origin_yaw_cdeg);
    return;
  }
  if (mission_ctx.vision_search_e3_mode)
  {
    /* Freeze straight parameters at entry; the second ordinary51 E3 has
       its own absolute-heading turn before its forward leg. */
    if (second_s51_e3)
    {
      mission_ctx.vision_search_forward_rpm = ROBOT_S51_SECOND_E3_FORWARD_RPM;
      mission_ctx.vision_search_forward_ms = ROBOT_S51_SECOND_E3_FORWARD_MS;
    }
    else if (resume_state == MISSION_STATE_S7_SEARCH_RED)
    {
      mission_ctx.vision_search_forward_rpm = ROBOT_S7_E3_SEARCH_FORWARD_RPM;
      mission_ctx.vision_search_forward_ms = ROBOT_S7_E3_SEARCH_FORWARD_MS;
      mission_ctx.s7_red_e3_phase = S7_RED_E3_FORWARD;
    }
    else if (ordinary_s51_recovery)
    {
      /* Changing ordinary51 search direction must not alter supplementary,
         standalone31 or other callers of the legacy recovery entry. */
      mission_ctx.vision_search_forward_rpm = ROBOT_S51_E3_FORWARD_RPM;
      mission_ctx.vision_search_forward_ms = ROBOT_S51_E3_FORWARD_MS;
    }
    else
    {
      mission_ctx.vision_search_forward_rpm = -ROBOT_E3_SEARCH_REVERSE_RPM;
      mission_ctx.vision_search_forward_ms = ROBOT_E3_SEARCH_REVERSE_MS;
    }
    e3_spin_progress_cdeg = 0L;
    if (second_s51_e3)
    {
      (void)DebugUart_Logf(
          "[S51-E3] AWAY r=%ld f=%ld safe=%ld >FWD=%d/%lu CCW360\r\n",
          (long)mission_ctx.s3_search_yaw_target_cdeg,
          (long)MissionTask_WrapYaw(mission_ctx.s3_search_yaw_target_cdeg - mission_ctx.mission_snapshot.yaw_start_cdeg),
          (long)mission_ctx.mission_snapshot.safe_zone_yaw_target_cdeg,
          (int)mission_ctx.vision_search_forward_rpm, (unsigned long)mission_ctx.vision_search_forward_ms);
      return;
    }
    mission_ctx.s3_search_yaw_target_cdeg = MissionTask_WrapYaw(mission_ctx.mission_snapshot.yaw_cdeg);
    if (resume_state == MISSION_STATE_S7_SEARCH_RED)
    {
      (void)DebugUart_Logf(
          "[07-E3] FWD=%d/%lu L60/500 R120/500 L60 v=%d\r\n",
          (int)mission_ctx.vision_search_forward_rpm, (unsigned long)mission_ctx.vision_search_forward_ms,
          ROBOT_S7_E3_SEARCH_TURN_FAST_RPM);
      return;
    }
    (void)DebugUart_Logf(
        "[E3-R] src=%s %s v=%d ms=%lu > CCW360\r\n",
        source, mission_ctx.vision_search_forward_rpm > 0 ? "FWD" : "BACK",
        (int)MissionTask_Abs32(mission_ctx.vision_search_forward_rpm),
        (unsigned long)mission_ctx.vision_search_forward_ms);
    return;
  }
  (void)DebugUart_Logf(
      "[SEARCH] src=%s FWD=%d/%lu CW=%ld "
      "ref=%ld goal=%ld\r\n",
      source,
      mission_ctx.vision_search_forward_rpm,
      (unsigned long)mission_ctx.vision_search_forward_ms,
      (long)(vision_search_turn_angle_cdeg / 100L),
      (long)mission_ctx.s3_search_origin_yaw_cdeg,
      (long)mission_ctx.s3_search_yaw_target_cdeg);
}

static void MissionTask_RunS3GreenE3Recovery(uint32_t now)
{
  uint32_t elapsed = (uint32_t)(now - mission_ctx.mission_snapshot.state_entry_tick);
  int16_t turn_rpm;

  if (s3_green_e3_phase == S3_GREEN_E3_FORWARD)
  {
    if (elapsed < ROBOT_S3_GREEN_E3_FORWARD_MS)
    {
      mission_ctx.mission_snapshot.vision_forward_rpm = ROBOT_S3_GREEN_E3_FORWARD_RPM;
      mission_ctx.mission_snapshot.vision_turn_rpm = 0;
      MissionTask_SetWheelTargets(ROBOT_S3_GREEN_E3_FORWARD_RPM,
                                  ROBOT_S3_GREEN_E3_FORWARD_RPM);
      return;
    }
    MissionTask_StopWheels();
    MissionTask_ResetS6ControllerState();
    mission_ctx.vision_search_forward_active = false;
    s3_green_e3_phase = S3_GREEN_E3_SPIN_CCW;
    e3_spin_previous_yaw_cdeg = MissionTask_WrapYaw(mission_ctx.mission_snapshot.yaw_cdeg);
    e3_spin_progress_cdeg = 0L;
    mission_ctx.s3_search_yaw_target_cdeg = e3_spin_previous_yaw_cdeg;
    MissionTask_EnterState(MISSION_STATE_S3_SEARCH_TURN_CCW, now);
    (void)DebugUart_Log("[03-E3] FWD OK >CCW360\r\n");
    return;
  }

  /* A full circle returns to the original heading, so use accumulated yaw,
     never a shortest-path heading PID or the2s fixed-turn completion rule. */
  RecoveryFlow_ObserveSpin(&e3_spin_previous_yaw_cdeg, &e3_spin_progress_cdeg,
      mission_ctx.mission_snapshot.yaw_cdeg, ROBOT_YAW_LEFT_SIGN);
  if (e3_spin_progress_cdeg >= ROBOT_S3_GREEN_E3_SPIN_CDEG)
  {
    MissionTask_StopWheels();
    MissionTask_ResetVisionControllerState();
    MissionTask_ResetS6ControllerState();
    mission_ctx.vision_search_green_e3_mode = false;
    mission_ctx.s3_search_sweep_completed = true;
    mission_ctx.s3_target_lost_since = now;
    MissionTask_EnterState(MISSION_STATE_S3_TRACK_GREEN, now);
    (void)DebugUart_Log("[03-E3] CCW360 OK WAIT GREEN XY\r\n");
    return;
  }
  if (elapsed >= ROBOT_E3_SEARCH_SPIN_TIMEOUT_MS)
  {
    mission_ctx.mission_snapshot.fault_flags |= MISSION_FAULT_S3_TIMEOUT;
    MissionTask_EnterState(MISSION_STATE_FAULT, now);
    (void)DebugUart_Logf("[03-E3] CCW360 TIMEOUT p=%ld\r\n", (long)e3_spin_progress_cdeg);
    return;
  }
  turn_rpm = RecoveryFlow_TaperSpin(ROBOT_S3_GREEN_E3_SPIN_CDEG, e3_spin_progress_cdeg,
      ROBOT_S3_SEARCH_TURN_SLOW_THRESHOLD_CDEG, ROBOT_S3_SEARCH_TURN_FAST_RPM, ROBOT_S3_SEARCH_TURN_SLOW_RPM);
  mission_ctx.mission_snapshot.vision_forward_rpm = 0;
  mission_ctx.mission_snapshot.vision_turn_rpm = (int16_t)(-ROBOT_YAW_LEFT_SIGN * turn_rpm);
  MissionTask_SetWheelTargets((int16_t)(-ROBOT_YAW_LEFT_SIGN * turn_rpm),
                              (int16_t)(ROBOT_YAW_LEFT_SIGN * turn_rpm));
  if ((int32_t)(now - mission_ctx.s3_next_debug_tick) >= 0)
  {
    mission_ctx.s3_next_debug_tick = now + ROBOT_VISION_DEBUG_PERIOD_MS;
    (void)DebugUart_Logf("[03-E3] CCW360 p=%ld v=%d\r\n", (long)e3_spin_progress_cdeg, turn_rpm);
  }
}

/* Shared motion only: E3 finishes with13, while E4 failure selects51/03.
   The legacy E3-named phase storage is reset whenever either sweep starts. */
bool MissionTask_RunRedSearchSweep(uint32_t now)
{
  const bool center_recovery = mission_ctx.mission_snapshot.state == MISSION_STATE_S4_E4_RED_SPIN_360;
  const char *log_tag = center_recovery ? "E4-R" : "07-E3";
  const RedSweep_Phase phase = (RedSweep_Phase)mission_ctx.s7_red_e3_phase;
  const FlowMotion io = { .stop = MissionTask_StopWheels, .wheels = MissionTask_SetWheelTargets };
  if (phase != RECOVERY_RED_FORWARD)
  {
    mission_ctx.mission_snapshot.vision_forward_rpm = 0;
    mission_ctx.mission_snapshot.vision_turn_rpm = 0;
  }
  const RedSweep_Phase next = RecoveryFlow_StepRed(&io, phase, MissionTask_HeadingTurnExpired,
      now, mission_ctx.mission_snapshot.state_entry_tick, mission_ctx.vision_search_forward_ms, mission_ctx.vision_search_forward_rpm,
      mission_ctx.mission_snapshot.yaw_cdeg, &s7_red_e3_origin_yaw_cdeg, &mission_ctx.s3_search_yaw_target_cdeg,
      &mission_ctx.mission_snapshot.vision_turn_rpm, &mission_ctx.mission_snapshot.yaw_error_cdeg);
  if (next == phase)
  {
    if (phase == RECOVERY_RED_FORWARD)
    {
      mission_ctx.mission_snapshot.vision_forward_rpm = mission_ctx.vision_search_forward_rpm;
      mission_ctx.mission_snapshot.vision_turn_rpm = 0;
    }
    return false;
  }
  if (next == RECOVERY_RED_DONE) return true;
  mission_ctx.s7_red_e3_phase = (S7RedE3Phase)next;
  if (phase == RECOVERY_RED_FORWARD)
  {
    mission_ctx.vision_search_forward_active = false;
    if (center_recovery) mission_ctx.mission_snapshot.state_entry_tick = now;
    else MissionTask_EnterState(MISSION_STATE_S3_SEARCH_TURN_CCW, now);
    (void)DebugUart_Logf("[%s] L60 goal=%ld\r\n", log_tag, (long)mission_ctx.s3_search_yaw_target_cdeg);
    return false;
  }
  if (next == RECOVERY_RED_RIGHT)
    (void)DebugUart_Logf("[%s] R120 goal=%ld\r\n", log_tag, (long)mission_ctx.s3_search_yaw_target_cdeg);
  else if (next == RECOVERY_RED_HOME)
    (void)DebugUart_Logf("[%s] L60 HOME=%ld\r\n", log_tag, (long)mission_ctx.s3_search_yaw_target_cdeg);
  else
    (void)DebugUart_Logf("[%s] HOLD %lu\r\n", log_tag, (unsigned long)ROBOT_S7_E3_SEARCH_HOLD_MS);
  mission_ctx.mission_snapshot.state_entry_tick = now;
  return false;
}

void MissionTask_RunS3SearchTurn(uint32_t now)
{
  int32_t yaw_error;
  int32_t absolute_error;
  int16_t turn_rpm;
  bool turn_left;
  bool clockwise_phase;
  bool turn_expired;

  clockwise_phase =
      mission_ctx.mission_snapshot.state == MISSION_STATE_S3_SEARCH_TURN_CW;
  if (!clockwise_phase &&
      (mission_ctx.mission_snapshot.state != MISSION_STATE_S3_SEARCH_TURN_CCW) &&
      (mission_ctx.mission_snapshot.state != MISSION_STATE_S3_E3_TURN_AWAY))
  {
    return;
  }

  if ((mission_ctx.vision_search_resume_state == MISSION_STATE_S3_TRACK_GREEN) &&
      MissionTask_TryStartS3Capture(now))
  {
    return;
  }

  /* Do not finish a blind search leg after vision has found a target. */
  if (mission_ctx.mission_snapshot.vision_target_valid &&
      ((mission_ctx.vision_search_resume_state != MISSION_STATE_S7_SEARCH_RED) ||
       (mission_ctx.mission_snapshot.target_id == 4U)))
  {
    mission_ctx.vision_search_forward_active = false;
    MissionTask_ResetVisionControllerState();
    if (mission_ctx.vision_search_green_e3_mode ||
        mission_ctx.mission_snapshot.state == MISSION_STATE_S3_E3_TURN_AWAY)
    {
      mission_ctx.vision_search_green_e3_mode = false;
      MissionTask_ResetS6ControllerState();
    }
    if (mission_ctx.vision_search_resume_state == MISSION_STATE_S4_TRACK_CENTER)
    {
      mission_ctx.s4_center_search_sweep_completed = false;
      mission_ctx.s4_center_target_lost_since = 0U;
      mission_ctx.s4_next_debug_tick = now;
      if (!MissionTask_ReturnToS4TrackCenter(now))
      {
        return;
      }
    }
    else if (mission_ctx.vision_search_resume_state == MISSION_STATE_S7_SEARCH_RED)
    {
      (void)DebugUart_Log(
          "[07] RED FOUND >RESUME\r\n");
      MissionTask_ResumeRedTracking(now);
    }
    else
    {
      /* Arm one new sweep if this reacquired target is later lost again. */
      mission_ctx.s3_search_sweep_completed = false;
      mission_ctx.s3_target_lost_since = 0U;
      mission_ctx.s3_track_stable_since = 0U;
      MissionTask_EnterState(MISSION_STATE_S3_TRACK_GREEN, now);
    }
    (void)DebugUart_Logf("[SEARCH] src=%s FOUND x=%d y=%d "
                         "RESUME PID\r\n",
                         mission_ctx.vision_search_resume_state ==
                                 MISSION_STATE_S4_TRACK_CENTER
                             ? "04"
                             : (mission_ctx.vision_search_resume_state ==
                                        MISSION_STATE_S7_SEARCH_RED
                                    ? "07"
                                    : "03"),
                         (int)mission_ctx.mission_snapshot.vision_x_error_px,
                         (int)mission_ctx.mission_snapshot.vision_y_error_px);
    return;
  }

  if (mission_ctx.vision_search_green_e3_mode)
  {
    MissionTask_RunS3GreenE3Recovery(now);
    return;
  }

  if (mission_ctx.vision_search_e3_mode &&
      mission_ctx.vision_search_resume_state == MISSION_STATE_S7_SEARCH_RED)
  {
    MissionTask_RunS7RedE3Recovery(now);
    return;
  }

  if (mission_ctx.vision_search_e3_mode)
  {
    if (mission_ctx.mission_snapshot.state == MISSION_STATE_S3_E3_TURN_AWAY)
    {
      /* Start the1200ms forward timer on alignment or the2s turn deadline. */
      turn_rpm = MissionTask_CalculateS6YawTurn(now, mission_ctx.s3_search_yaw_target_cdeg);
      yaw_error = mission_ctx.mission_snapshot.safe_zone_yaw_error_cdeg;
      mission_ctx.mission_snapshot.yaw_error_cdeg = yaw_error;
      turn_expired = MissionTask_HeadingTurnExpired(now, mission_ctx.s3_search_yaw_target_cdeg);
      if (turn_expired || MissionTask_Abs32(yaw_error) <= ROBOT_S3_SEARCH_TURN_TOLERANCE_CDEG)
      {
        MissionTask_StopWheels();
        if (!turn_expired && mission_ctx.s3_search_turn_stable_since == 0U) mission_ctx.s3_search_turn_stable_since = now;
        else if (turn_expired || (uint32_t)(now - mission_ctx.s3_search_turn_stable_since) >=
                 ROBOT_S3_SEARCH_TURN_STABLE_MS)
        {
          mission_ctx.s3_search_turn_stable_since = 0U;
          MissionTask_ResetS6ControllerState();
          MissionTask_EnterState(MISSION_STATE_S3_SEARCH_TURN_CW, now);
          (void)DebugUart_Logf("[S51-E3] AWAY OK, FWD v=%d ms=%lu\r\n",
              (int)mission_ctx.vision_search_forward_rpm, (unsigned long)mission_ctx.vision_search_forward_ms);
        }
      }
      else
      {
        mission_ctx.s3_search_turn_stable_since = 0U;
        mission_ctx.mission_snapshot.vision_forward_rpm = 0;
        mission_ctx.mission_snapshot.vision_turn_rpm = turn_rpm;
        MissionTask_SetWheelTargets(turn_rpm, (int16_t)-turn_rpm);
      }
      return;
    }
    if (mission_ctx.vision_search_forward_active)
    {
      if ((uint32_t)(now - mission_ctx.mission_snapshot.state_entry_tick) <
          mission_ctx.vision_search_forward_ms)
      {
        mission_ctx.mission_snapshot.vision_forward_rpm = mission_ctx.vision_search_forward_rpm;
        mission_ctx.mission_snapshot.vision_turn_rpm = 0;
        MissionTask_SetWheelTargets(mission_ctx.vision_search_forward_rpm,
                                    mission_ctx.vision_search_forward_rpm);
        return;
      }
      MissionTask_StopWheels();
      mission_ctx.vision_search_forward_active = false;
      e3_spin_previous_yaw_cdeg = MissionTask_WrapYaw(mission_ctx.mission_snapshot.yaw_cdeg);
      e3_spin_progress_cdeg = 0L;
      mission_ctx.s3_search_yaw_target_cdeg = e3_spin_previous_yaw_cdeg;
      MissionTask_EnterState(MISSION_STATE_S3_SEARCH_TURN_CCW, now);
      (void)DebugUart_Log(mission_ctx.vision_search_forward_rpm > 0
          ? "[E3-R] FWD OK, CCW360\r\n"
          : "[E3-R] BACK OK, CCW360\r\n");
      return;
    }
    RecoveryFlow_ObserveSpin(&e3_spin_previous_yaw_cdeg, &e3_spin_progress_cdeg,
        mission_ctx.mission_snapshot.yaw_cdeg, ROBOT_YAW_LEFT_SIGN);
    if (e3_spin_progress_cdeg >= ROBOT_E3_SEARCH_CCW_CDEG)
    {
      MissionTask_StopWheels();
      MissionTask_ResetVisionControllerState();
      if (mission_ctx.vision_search_resume_state == MISSION_STATE_S7_SEARCH_RED)
      {
        MissionTask_CompleteS7RedRecovery(now);
      }
      else
      {
        mission_ctx.s3_search_sweep_completed = true;
        mission_ctx.s3_target_lost_since = now;
        MissionTask_EnterState(MISSION_STATE_S3_TRACK_GREEN, now);
      }
      (void)DebugUart_Logf("[E3-R] CCW360 DONE progress=%ld cdeg\r\n",
                           (long)e3_spin_progress_cdeg);
      return;
    }
    if ((uint32_t)(now - mission_ctx.mission_snapshot.state_entry_tick) >=
        ROBOT_E3_SEARCH_SPIN_TIMEOUT_MS)
    {
      mission_ctx.mission_snapshot.fault_flags |=
          mission_ctx.vision_search_resume_state == MISSION_STATE_S7_SEARCH_RED
              ? MISSION_FAULT_S7_TIMEOUT : MISSION_FAULT_S3_TIMEOUT;
      MissionTask_EnterState(MISSION_STATE_FAULT, now);
      (void)DebugUart_Logf("[E3-R] CCW360 TIMEOUT progress=%ld cdeg\r\n",
                           (long)e3_spin_progress_cdeg);
      return;
    }
    turn_rpm = RecoveryFlow_TaperSpin(ROBOT_E3_SEARCH_CCW_CDEG, e3_spin_progress_cdeg,
        ROBOT_S3_SEARCH_TURN_SLOW_THRESHOLD_CDEG, ROBOT_S3_SEARCH_TURN_FAST_RPM, ROBOT_S3_SEARCH_TURN_SLOW_RPM);
    mission_ctx.mission_snapshot.vision_forward_rpm = 0;
    mission_ctx.mission_snapshot.vision_turn_rpm = (int16_t)(-ROBOT_YAW_LEFT_SIGN * turn_rpm);
    MissionTask_SetWheelTargets((int16_t)(-ROBOT_YAW_LEFT_SIGN * turn_rpm),
                                (int16_t)(ROBOT_YAW_LEFT_SIGN * turn_rpm));
    if ((int32_t)(now - mission_ctx.s3_next_debug_tick) >= 0)
    {
      mission_ctx.s3_next_debug_tick = now + ROBOT_VISION_DEBUG_PERIOD_MS;
      (void)DebugUart_Logf("[E3-R] CCW360 progress=%ld/36000 v=%d\r\n",
                           (long)e3_spin_progress_cdeg, (int)turn_rpm);
    }
    return;
  }

  if (mission_ctx.vision_search_forward_active)
  {
    if ((uint32_t)(now - mission_ctx.mission_snapshot.state_entry_tick) <
        mission_ctx.vision_search_forward_ms)
    {
      mission_ctx.mission_snapshot.vision_forward_rpm =
          mission_ctx.vision_search_forward_rpm;
      mission_ctx.mission_snapshot.vision_turn_rpm = 0;
      MissionTask_SetWheelTargets(mission_ctx.vision_search_forward_rpm,
                                  mission_ctx.vision_search_forward_rpm);
      return;
    }

    MissionTask_StopWheels();
    mission_ctx.mission_snapshot.vision_forward_rpm = 0;
    mission_ctx.vision_search_forward_active = false;
    mission_ctx.mission_snapshot.state_entry_tick = now;
    mission_ctx.s3_search_turn_stable_since = 0U;
    (void)DebugUart_Logf(
        "[V-SEARCH] src=%s FWD OK, CW LEG\r\n",
        mission_ctx.vision_search_resume_state == MISSION_STATE_S4_TRACK_CENTER
            ? "04"
            : (mission_ctx.vision_search_resume_state == MISSION_STATE_S7_SEARCH_RED
                   ? "07"
                   : "03"));
  }

  yaw_error = MissionTask_YawError(mission_ctx.s3_search_yaw_target_cdeg,
                                   mission_ctx.mission_snapshot.yaw_cdeg);
  mission_ctx.mission_snapshot.yaw_error_cdeg = yaw_error;
  absolute_error = MissionTask_Abs32(yaw_error);

  turn_expired = MissionTask_HeadingTurnExpired(now, mission_ctx.s3_search_yaw_target_cdeg);
  if (turn_expired || absolute_error <= ROBOT_S3_SEARCH_TURN_TOLERANCE_CDEG)
  {
    MissionTask_StopWheels();
    if (!turn_expired && mission_ctx.s3_search_turn_stable_since == 0U)
    {
      mission_ctx.s3_search_turn_stable_since = now;
    }
    else if (turn_expired || (uint32_t)(now - mission_ctx.s3_search_turn_stable_since) >=
             ROBOT_S3_SEARCH_TURN_STABLE_MS)
    {
      mission_ctx.s3_search_turn_stable_since = 0U;
      if (clockwise_phase)
      {
        /* Split the CCW sweep into two legs via the search origin. */
        mission_ctx.s3_search_ccw_midpoint_done = false;
        mission_ctx.s3_search_yaw_target_cdeg = mission_ctx.s3_search_origin_yaw_cdeg;
        MissionTask_EnterState(MISSION_STATE_S3_SEARCH_TURN_CCW, now);
        (void)DebugUart_Logf(
            "[S3-S] CW DONE, CCW SWEEP midpoint=%ld\r\n",
            (long)mission_ctx.s3_search_yaw_target_cdeg);
      }
      else if (!mission_ctx.s3_search_ccw_midpoint_done)
      {
        mission_ctx.s3_search_ccw_midpoint_done = true;
        /* The second target in the same CCW state owns a fresh2s budget. */
        mission_ctx.mission_snapshot.state_entry_tick = now;
        mission_ctx.s3_search_yaw_target_cdeg = MissionTask_WrapYaw(
            mission_ctx.s3_search_origin_yaw_cdeg +
            (int32_t)ROBOT_YAW_LEFT_SIGN *
                vision_search_turn_angle_cdeg);
        (void)DebugUart_Logf(
            "[03-S] CCW MID OK >goal=%ld\r\n",
            (long)mission_ctx.s3_search_yaw_target_cdeg);
      }
      else
      {
        if (mission_ctx.vision_search_resume_state == MISSION_STATE_S4_TRACK_CENTER)
        {
          mission_ctx.s4_center_search_sweep_completed = true;
          mission_ctx.s4_center_target_lost_since = now;
          mission_ctx.s4_next_debug_tick = now;
          if (!MissionTask_ReturnToS4TrackCenter(now))
          {
            return;
          }
        }
        else if (mission_ctx.vision_search_resume_state == MISSION_STATE_S7_SEARCH_RED)
        {
          MissionTask_CompleteS7RedRecovery(now);
        }
        else
        {
          mission_ctx.s3_search_sweep_completed = true;
          mission_ctx.s3_target_lost_since = now;
          MissionTask_EnterState(MISSION_STATE_S3_TRACK_GREEN, now);
        }
        if (mission_ctx.vision_search_resume_state == MISSION_STATE_S7_SEARCH_RED)
        {
          (void)DebugUart_Log(
              "[SEARCH] 07 CCW OK RED LOST\r\n");
        }
        else
        {
          (void)DebugUart_Logf(
              "[SEARCH] src=%s CCW OK WAIT XY\r\n",
              mission_ctx.vision_search_resume_state == MISSION_STATE_S4_TRACK_CENTER
                  ? "04"
                  : "03");
        }
      }
      return;
    }
  }
  else
  {
    mission_ctx.s3_search_turn_stable_since = 0U;
    turn_rpm = absolute_error >
                       ROBOT_S3_SEARCH_TURN_SLOW_THRESHOLD_CDEG
                   ? ROBOT_S3_SEARCH_TURN_FAST_RPM
                   : ROBOT_S3_SEARCH_TURN_SLOW_RPM;
    turn_left = (yaw_error * (int32_t)ROBOT_YAW_LEFT_SIGN) > 0L;
    MissionTask_SetWheelTargets(turn_left ? -turn_rpm : turn_rpm,
                                turn_left ? turn_rpm : -turn_rpm);
  }

  if ((int32_t)(now - mission_ctx.s3_next_debug_tick) >= 0)
  {
    mission_ctx.s3_next_debug_tick = now + ROBOT_VISION_DEBUG_PERIOD_MS;
    (void)DebugUart_Logf(
        "[E3] leg=%s r=%ld goal=%ld e=%ld w=%d/%d\r\n",
        clockwise_phase ? "CW" :
            (mission_ctx.s3_search_ccw_midpoint_done ? "CCW2" : "CCW1"),
        (long)MissionTask_WrapYaw(mission_ctx.mission_snapshot.yaw_cdeg),
        (long)mission_ctx.s3_search_yaw_target_cdeg,
        (long)yaw_error,
        (int)mission_ctx.mission_snapshot.left_target_rpm,
        (int)mission_ctx.mission_snapshot.right_target_rpm);
  }
}

static int16_t MissionTask_GuardVisionForward(uint32_t now, int16_t error_px, int16_t requested_rpm)
{
  const uint8_t reference = mission_ctx.mission_snapshot.state == MISSION_STATE_SUPPLEMENT_TRACK
      ? mission_ctx.supplement_reference_command : 0U;
  const VisionMotion_Frame frame = {
    .context = ((uint32_t)mission_ctx.mission_snapshot.state << 8U) | reference,
    .sequence = mission_ctx.mission_snapshot.target_sequence,
    .age_ms = mission_ctx.mission_snapshot.target_age_ms,
    .timeout_ms = MISSION_VISION_TARGET_TIMEOUT_MS,
    .target_id = mission_ctx.mission_snapshot.target_id,
    .valid = mission_ctx.mission_snapshot.vision_target_valid,
  };
  const VisionMotion_GuardResult result = VisionMotion_GuardForward(
      &mission_ctx.vision_motion.reverse, &frame, now, error_px, requested_rpm);
  if (result.y_jump)
    (void)DebugUart_Logf("[Y-JUMP] s=%u id=%u q=%lu y=%d>%d age=%lu\r\n",
        (unsigned int)mission_ctx.mission_snapshot.state, (unsigned int)frame.target_id,
        (unsigned long)frame.sequence, (int)result.previous_y, (int)error_px,
        (unsigned long)frame.age_ms);
  if (result.confirmation_changed)
    (void)DebugUart_Logf("[Y-REV] s=%u id=%u q=%lu y=%d req=%d n=%u/%u %s\r\n",
        (unsigned int)mission_ctx.mission_snapshot.state, (unsigned int)frame.target_id,
        (unsigned long)frame.sequence, (int)error_px, (int)requested_rpm,
        (unsigned int)result.confirmation_frames, (unsigned int)ROBOT_VISION_OBJECT_REVERSE_FRAMES,
        result.accepted ? "ACCEPT" : "WAIT_ZERO");
  return result.rpm;
}

int16_t MissionTask_CalculateVisionForward(uint32_t now, int16_t error_px)
{
  const VisionMotion_Profile profile = MissionTask_GetVisionMotionProfile();
  const int16_t rpm = VisionMotion_CalculateForward(&mission_ctx.vision_motion.y, profile,
      now, mission_ctx.mission_snapshot.target_sequence, error_px, MISSION_TASK_PERIOD_TICKS);
  return VisionMotion_IsObjectProfile(profile) ? MissionTask_GuardVisionForward(now, error_px, rpm) : rpm;
}

void MissionTask_SetTrackingTargets(int16_t forward_rpm,
                                           int16_t turn_rpm,
                                           int16_t wheel_max_rpm)
{
  const ChassisWheelRpm targets = ChassisMotion_MixRpm(forward_rpm, turn_rpm, wheel_max_rpm);
  MissionTask_SetWheelTargets(targets.left_rpm, targets.right_rpm);
}

static float MissionTask_VisionAgeScale(uint32_t age_ms)
{
  return VisionMotion_AgeScale(age_ms, ROBOT_VISION_COORD_DECEL_START_MS,
                               MISSION_VISION_TARGET_TIMEOUT_MS);
}

void MissionTask_SetVisionTrackingTargets(int16_t forward_rpm,
                                                int16_t turn_rpm,
                                                int16_t wheel_max_rpm)
{
  float scale = mission_ctx.mission_snapshot.vision_target_valid
      ? MissionTask_VisionAgeScale(mission_ctx.mission_snapshot.target_age_ms) : 0.0f;
  /* Scale both channels together. Never reapply the PID minimum after fading. */
  forward_rpm = (int16_t)((float)forward_rpm * scale);
  turn_rpm = (int16_t)((float)turn_rpm * scale);
  mission_ctx.mission_snapshot.vision_forward_rpm = forward_rpm;
  mission_ctx.mission_snapshot.vision_turn_rpm = turn_rpm;
  MissionTask_SetTrackingTargets(forward_rpm, turn_rpm, wheel_max_rpm);
}

void MissionTask_CommandClawApproach(uint32_t now, int16_t turn_limit,
                                    int16_t forward_limit, int16_t wheel_limit)
{
  const MissionState state = mission_ctx.mission_snapshot.state;
  const uint8_t reference = state == MISSION_STATE_SUPPLEMENT_TRACK
      ? mission_ctx.supplement_reference_command : 0U;
  const VisionMotion_Frame frame = {
    .context = ((uint32_t)state << 8U) | reference,
    .sequence = mission_ctx.mission_snapshot.target_sequence,
    .age_ms = mission_ctx.mission_snapshot.target_age_ms,
    .timeout_ms = MISSION_VISION_TARGET_TIMEOUT_MS,
    .target_id = mission_ctx.mission_snapshot.target_id,
    .area_near = (mission_ctx.mission_snapshot.target_geometry & MAIXCAM_OBJECT_GEOMETRY_NEAR) != 0U,
    .area_open = (mission_ctx.mission_snapshot.target_geometry & MAIXCAM_OBJECT_GEOMETRY_OPEN_AREA) != 0U,
    .claw_closed = MissionTask_GetVisionMotionProfile() != VISION_MOTION_FINAL ||
                   !mission_ctx.s4_capture_claw_open,
    .valid = mission_ctx.mission_snapshot.vision_target_valid,
  };
  const bool wheels_stopped =
      MissionTask_Abs32(moto_chassis[MISSION_LEFT_MOTOR_INDEX].speed_rpm) <= ROBOT_VISION_CLAW_STOP_MOTOR_RPM &&
      MissionTask_Abs32(moto_chassis[MISSION_RIGHT_MOTOR_INDEX].speed_rpm) <= ROBOT_VISION_CLAW_STOP_MOTOR_RPM;
  const int16_t x_error = mission_ctx.mission_snapshot.vision_x_error_px;
  const int16_t y_error = mission_ctx.mission_snapshot.vision_y_error_px;
  const uint8_t previous_drive = mission_ctx.vision_motion.claw.last_drive;
  const VisionMotion_ClawDrive drive = VisionMotion_ClawApproachStep(
      &mission_ctx.vision_motion.claw, MissionTask_GetVisionMotionProfile(),
      &frame, now, x_error, y_error, wheels_stopped);
  int16_t turn = 0, forward = 0;
  if (drive != VISION_CLAW_ALIGN && drive != VISION_CLAW_FAR_TRACK)
    VisionMotion_ResetAxis(&mission_ctx.vision_motion.x);
  if (drive != VISION_CLAW_STRAIGHT && drive != VISION_CLAW_FAR_TRACK)
    VisionMotion_ResetAxis(&mission_ctx.vision_motion.y);
  if (drive != VISION_CLAW_STRAIGHT || !mission_ctx.mission_snapshot.imu_valid)
    VisionMotion_ClawResetHeading(&mission_ctx.vision_motion.claw);
  switch (drive)
  {
    case VISION_CLAW_FAR_TRACK:
      /* Preserve the original simultaneous image X/Y control at distance. */
      forward = MissionTask_CalculateVisionForward(now, y_error);
      if (MissionTask_Abs32(x_error) > ROBOT_VISION_X_TOLERANCE_PX)
        turn = MissionTask_CalculateVisionTurn(now, x_error);
      else
        VisionMotion_ResetAxis(&mission_ctx.vision_motion.x);
      break;
    case VISION_CLAW_ALIGN:
      turn = MissionTask_CalculateVisionTurn(now, x_error);
      break;
    case VISION_CLAW_STRAIGHT:
    {
      if (!mission_ctx.mission_snapshot.imu_valid) break;
      forward = MissionTask_CalculateVisionForward(now, y_error);
      if (forward < 0 || (int32_t)y_error * ROBOT_VISION_Y_FORWARD_SIGN <= ROBOT_VISION_Y_TOLERANCE_PX)
        forward = 0;
      if ((frame.area_near || frame.area_open) && forward > ROBOT_VISION_CLAW_AREA_APPROACH_MAX_RPM)
        forward = ROBOT_VISION_CLAW_AREA_APPROACH_MAX_RPM;
      const int16_t correction = VisionMotion_ClawHoldHeading(&mission_ctx.vision_motion.claw,
          now, mission_ctx.mission_snapshot.yaw_cdeg, MISSION_TASK_PERIOD_TICKS);
      if (forward > 0) turn = correction;
      break;
    }
    case VISION_CLAW_BACKOFF:
      forward = -ROBOT_VISION_CLAW_BACKOFF_RPM;
      if (previous_drive != VISION_CLAW_BACKOFF)
        (void)DebugUart_Logf("[CLAW-TRACK] s=%u x=%d y=%d BACK=%d/%lu\r\n",
            (unsigned int)state, (int)x_error, (int)y_error,
            ROBOT_VISION_CLAW_BACKOFF_RPM, (unsigned long)ROBOT_VISION_CLAW_BACKOFF_MS);
      break;
    case VISION_CLAW_HOLD:
    default:
      break;
  }
  if (turn > turn_limit) turn = turn_limit;
  else if (turn < -turn_limit) turn = -turn_limit;
  if (forward > forward_limit) forward = forward_limit;
  else if (forward < -forward_limit) forward = -forward_limit;
  MissionTask_SetVisionTrackingTargets(forward, turn, wheel_limit);
}

void MissionTask_RunS3Track(uint32_t now)
{
  int32_t absolute_x_error;
  int32_t absolute_y_error;
  bool x_in_tolerance;
  bool y_in_tolerance;

  if (mission_ctx.mission_snapshot.state != MISSION_STATE_S3_TRACK_GREEN)
  {
    return;
  }

  if (MissionTask_TryStartS3Capture(now))
  {
    return;
  }

  if (!mission_ctx.mission_snapshot.vision_target_valid)
  {
    MissionTask_CommandClawApproach(now, (int16_t)ROBOT_VISION_X_MAX_TURN_RPM,
        (int16_t)ROBOT_VISION_03_MAX_FORWARD_RPM, ROBOT_VISION_03_WHEEL_MAX_RPM);
    mission_ctx.s3_track_stable_since = 0U;
    mission_ctx.s3_wait_event04_logged = false;
    if (mission_ctx.s3_target_lost_since == 0U)
    {
      mission_ctx.s3_target_lost_since = now;
    }
    if (!mission_ctx.s3_search_sweep_completed &&
        ((uint32_t)(now - mission_ctx.s3_target_lost_since) >=
         ROBOT_S3_GREEN_WAIT_MS))
    {
      (void)DebugUart_Logf(
          "[03-T] LOST %lu, COMMON RECOVERY\r\n",
          (unsigned long)ROBOT_S3_GREEN_WAIT_MS);
      MissionTask_StartVisionSearchRecovery(
          now,
          mission_ctx.s3_target_select_command == MAIXCAM_COMMAND_SELECT_RED
              ? MISSION_STATE_S7_SEARCH_RED : MISSION_STATE_S3_TRACK_GREEN,
          mission_ctx.s3_target_select_command == MAIXCAM_COMMAND_SELECT_RED
              ? mission_ctx.mission_snapshot.yaw_cdeg : mission_ctx.s3_search_origin_yaw_cdeg,
          false);
      return;
    }
    MissionTask_DebugS3(now);
    return;
  }

  mission_ctx.s3_target_lost_since = 0U;
  mission_ctx.s3_search_sweep_completed = false;

  if ((uint32_t)(now - mission_ctx.mission_snapshot.state_entry_tick) >=
      ROBOT_VISION_TRACK_TIMEOUT_MS)
  {
    mission_ctx.mission_snapshot.fault_flags |= MISSION_FAULT_S3_TIMEOUT;
    MissionTask_EnterState(MISSION_STATE_FAULT, now);
    return;
  }

  absolute_x_error = MissionTask_Abs32(mission_ctx.mission_snapshot.vision_x_error_px);
  absolute_y_error = MissionTask_Abs32(mission_ctx.mission_snapshot.vision_y_error_px);
  x_in_tolerance = absolute_x_error <= ROBOT_VISION_X_TOLERANCE_PX;
  y_in_tolerance = absolute_y_error <= ROBOT_VISION_Y_TOLERANCE_PX;

  MissionTask_CommandClawApproach(now, (int16_t)ROBOT_VISION_X_MAX_TURN_RPM,
      (int16_t)ROBOT_VISION_03_MAX_FORWARD_RPM, ROBOT_VISION_03_WHEEL_MAX_RPM);

  if (x_in_tolerance && y_in_tolerance)
  {
    if (mission_ctx.s3_track_stable_since == 0U)
    {
      mission_ctx.s3_track_stable_since = now;
      mission_ctx.s3_wait_event04_logged = false;
      (void)DebugUart_Log("[03-T] XY OK\r\n");
    }
    else if ((uint32_t)(now - mission_ctx.s3_track_stable_since) >=
                 ROBOT_VISION_Y_STABLE_MS &&
             !mission_ctx.s3_wait_event04_logged)
    {
      mission_ctx.s3_wait_event04_logged = true;
      (void)DebugUart_Log("[03-T] XY OK, WAIT 04\r\n");
    }

    MissionTask_DebugS3(now);
    return;
  }

  mission_ctx.s3_track_stable_since = 0U;
  mission_ctx.s3_wait_event04_logged = false;

  MissionTask_DebugS3(now);
}

void MissionTask_RunS3LowerFrame(uint32_t now)
{
  if (mission_ctx.mission_snapshot.state != MISSION_STATE_S3_LOWER_FRAME)
  {
    return;
  }

#if ROBOT_S3_FRAME_LOWER_SETTLE_MS > 0U
  if ((uint32_t)(now - mission_ctx.mission_snapshot.state_entry_tick) <
      ROBOT_S3_FRAME_LOWER_SETTLE_MS)
  {
    return;
  }
#endif

  (void)DebugUart_Log("[03] DOWN DONE, BEGIN VISION 03\r\n");
  (void)MissionTask_BeginS3Vision(now);
}

bool MissionTask_CanAvoidStartZone(void)
{
  switch (mission_ctx.mission_snapshot.state)
  {
    case MISSION_STATE_S3_TRACK_GREEN:
    case MISSION_STATE_S3_SEARCH_TURN_CW:
    case MISSION_STATE_S3_SEARCH_TURN_CCW:
    case MISSION_STATE_S3_E3_TURN_AWAY:
    case MISSION_STATE_SUPPLEMENT_TRACK:
    case MISSION_STATE_SUPPLEMENT_TURN_LEFT:
    case MISSION_STATE_SUPPLEMENT_TURN_RIGHT:
    case MISSION_STATE_SUPPLEMENT_E3_SPIN_CCW:
    case MISSION_STATE_S7_SEARCH_RED:
    case MISSION_STATE_S7_WAIT_SECOND_E3:
    case MISSION_STATE_S7_SEARCH_BLACK:
    case MISSION_STATE_S7_SILENT_TURN_LEFT:
    case MISSION_STATE_S7_SILENT_TURN_BACK:
    case MISSION_STATE_S4_TRACK_CENTER:
    case MISSION_STATE_S4_E4_TURN_AWAY:
    case MISSION_STATE_S4_E4_FORWARD:
    case MISSION_STATE_S4_E4_TURN_LEFT:
    case MISSION_STATE_S4_E4_TURN_RIGHT:
    case MISSION_STATE_S4_E4_GREEN_SPIN_360:
    case MISSION_STATE_S4_E4_RED_SPIN_360:
    case MISSION_STATE_S4_E4_MATERIAL_SPIN_CW_360:
    case MISSION_STATE_S4_RAISE_FRAME:
    case MISSION_STATE_S4_TRACK_FINAL_BLOCK:
    case MISSION_STATE_S4_FINAL_TIMEOUT_BACKOFF:
    case MISSION_STATE_S4_FINAL_RECOVERY_REVERSE:
    case MISSION_STATE_S4_FINAL_RECOVERY_TURN_270:
    case MISSION_STATE_S4_FINAL_RECOVERY_TURN_90:
      return true;
    case MISSION_STATE_S4_E4_PUSH_RIGHT:
    case MISSION_STATE_S4_E4_REVERSE_RIGHT:
    case MISSION_STATE_S4_E4_PUSH_LEFT:
    case MISSION_STATE_S4_E4_REVERSE_LEFT:
      /* Green04 E4 recovery is cancelable too. The same numeric states
         also serve05's E5 scatter, which is outside AE's object-search scope. */
      return !mission_ctx.s5_empty_recovery_active;
    case MISSION_STATE_S4_POST22_REVERSE:
    case MISSION_STATE_S4_POST22_TURN_LEFT:
    case MISSION_STATE_S4_POST22_TURN_RIGHT:
      return mission_ctx.s4_post22_search_active; /* Only its03 search, not fixed sorting. */
    default:
      return false;
  }
}

void MissionTask_HandleStartZoneAvoidanceRequest(uint32_t now)
{
  uint32_t request_sequence;
  if (!MaixCam_TakeStartZoneAvoidanceRequest(&request_sequence)) return;
  MissionState state = mission_ctx.mission_snapshot.state;
  if (state == MISSION_STATE_VISION_AVOID_START_TURN)
  {
    (void)DebugUart_Log("[AE] DUP KEEP\r\n");
    return;
  }
  if (state == MISSION_STATE_VISION_AVOID_START_WAIT_ACK)
  {
    /* Already scheduled: duplicates neither restart motion nor postpone TX. */
    return;
  }
  if (!MissionTask_CanAvoidStartZone())
  {
    (void)DebugUart_Logf("[AE] IGNORE s=%u\r\n", (unsigned int)state);
    return;
  }
  /* Keep the completed latch through short loss searches until a fresh,
     class-matching coordinate is accepted. A duplicate only retries ACK. */
  bool reply_only = mission_ctx.ae_ack_completed && (!ae_fresh_coordinate_seen ||
      (int32_t)(request_sequence - ae_fresh_coordinate_sequence) < 0);
  mission_ctx.ae_resume_state = mission_ctx.supplement_capture_active
      ? MISSION_STATE_SUPPLEMENT_TRACK : MISSION_STATE_S3_TRACK_GREEN;
  MissionTask_StopWheels();
  /* Cancel the action, not the mission. Never finish its old13/44/F1 after
     AE. Preserve class, all recovery counts and the absolute time budget. */
  mission_ctx.vision_search_forward_active = false;
  mission_ctx.vision_search_e3_mode = false;
  mission_ctx.vision_search_green_e3_mode = false;
  mission_ctx.vision_search_resume_state = mission_ctx.ae_resume_state;
  mission_ctx.s51_second_e3_pending = false;
  mission_ctx.s3_search_sweep_completed = false;
  mission_ctx.s3_search_ccw_midpoint_done = false;
  mission_ctx.s3_search_turn_stable_since = 0U;
  mission_ctx.s4_post22_search_active = false;
  mission_ctx.s4_red_e4_align_away_active = false;
  mission_ctx.s4_e4_protocol_active = false;
  mission_ctx.s4_final_mode_already_active = false;
  mission_ctx.s4_final_event34_pending = false;
  mission_ctx.supplement_recovery_event = 0U;
  if (mission_ctx.supplement_capture_active)
    mission_ctx.supplement_reference_command = MAIXCAM_COMMAND_SEARCH_TARGET;
  MaixCam_SetE4StageBuffering(false);
  MissionTask_ResetVisionPid();
  MissionTask_ResetS6ControllerState();
  mission_ctx.s6_phase_stable_since = 0U;
  ae_resume_prepared = false;
  mission_ctx.ae_reply_pending = reply_only;
  ae_next_reply_tick = now;
  MissionTask_EnterState(reply_only ? MISSION_STATE_VISION_AVOID_START_WAIT_ACK
                                  : MISSION_STATE_VISION_AVOID_START_TURN, now);
  ae_reply_sequence_floor = MaixCam_FenceObjectReference();
  mission_ctx.mission_snapshot.target_present = false;
  mission_ctx.mission_snapshot.target_age_ms = UINT32_MAX;
  mission_ctx.mission_snapshot.vision_target_valid = false;
  if (reply_only)
  {
    (void)DebugUart_Log("[AE] REACK\r\n");
    return;
  }
  ae_previous_yaw_cdeg = MissionTask_WrapYaw(mission_ctx.mission_snapshot.yaw_cdeg);
  ae_turn_progress_cdeg = 0L;
  mission_ctx.ae_turn_target_cdeg = MissionTask_WrapYaw(ae_previous_yaw_cdeg +
      (int32_t)ROBOT_YAW_LEFT_SIGN * ROBOT_AE_TURN_ANGLE_CDEG);
  mission_ctx.ae_ack_completed = false;
  ae_next_debug_tick = now;
  (void)DebugUart_Logf("[AE] RX s=%u CCW180 >03\r\n", (unsigned int)state);
}

void MissionTask_RunStartZoneAvoidance(uint32_t now)
{
  if (mission_ctx.mission_snapshot.state == MISSION_STATE_VISION_AVOID_START_TURN)
  {
    RecoveryFlow_ObserveSpin(&ae_previous_yaw_cdeg, &ae_turn_progress_cdeg,
        mission_ctx.mission_snapshot.yaw_cdeg, ROBOT_YAW_LEFT_SIGN);
    mission_ctx.mission_snapshot.safe_zone_yaw_command_cdeg = mission_ctx.ae_turn_target_cdeg;
    mission_ctx.mission_snapshot.safe_zone_yaw_error_cdeg = MissionTask_YawError(
        mission_ctx.ae_turn_target_cdeg, mission_ctx.mission_snapshot.yaw_cdeg);
    mission_ctx.mission_snapshot.vision_forward_rpm = 0;
    if (ae_turn_progress_cdeg >= ROBOT_AE_TURN_ANGLE_CDEG - ROBOT_S6_TURN_TOLERANCE_CDEG ||
        MissionTask_HeadingTurnExpired(now, mission_ctx.ae_turn_target_cdeg))
    {
      MissionTask_StopWheels();
      MissionTask_ResetVisionControllerState();
      mission_ctx.ae_reply_pending = true;
      ae_next_reply_tick = now;
      MissionTask_EnterState(MISSION_STATE_VISION_AVOID_START_WAIT_ACK, now);
      (void)DebugUart_Logf("[AE] TURN OK p=%ld\r\n", (long)ae_turn_progress_cdeg);
      return;
    }
    int16_t turn_rpm = RecoveryFlow_TaperSpin(ROBOT_AE_TURN_ANGLE_CDEG, ae_turn_progress_cdeg,
        ROBOT_AE_TURN_SLOW_THRESHOLD_CDEG, ROBOT_AE_TURN_FAST_RPM, ROBOT_AE_TURN_SLOW_RPM);
    /* Physical CCW is always left wheel backward, right wheel forward.
       LEFT_SIGN converts only gyro progress; no shortest-path180 ambiguity. */
    mission_ctx.mission_snapshot.vision_turn_rpm = -turn_rpm;
    MissionTask_SetWheelTargets(-turn_rpm, turn_rpm);
    if ((int32_t)(now - ae_next_debug_tick) >= 0)
    {
      ae_next_debug_tick = now + ROBOT_VISION_DEBUG_PERIOD_MS;
      (void)DebugUart_Logf("[AE] p=%ld v=%d\r\n", (long)ae_turn_progress_cdeg, turn_rpm);
    }
    return;
  }
  if (mission_ctx.mission_snapshot.state != MISSION_STATE_VISION_AVOID_START_WAIT_ACK) return;
  MissionTask_StopWheels();
  if (!mission_ctx.ae_reply_pending || (int32_t)(now - ae_next_reply_tick) < 0) return;
  if (MissionTask_HandleSupplementFinish(now) || MissionTask_LoadCheckPausesMotion()) return;
  if (!ae_resume_prepared)
  {
    /* Vision resumes03 (image center), including an interrupted24 capture.
       Apply03's closed-frame and camera pose without clearing retained cargo. */
    if (Actuator_SetFrameLowered() != HAL_OK || Actuator_SetCameraObjectTrackView() != HAL_OK)
    {
      mission_ctx.mission_snapshot.fault_flags |= MISSION_FAULT_ACTUATOR;
      MissionTask_EnterState(MISSION_STATE_FAULT, now);
      return;
    }
    ae_resume_prepared = true;
  }
  MissionTask_ResetVisionPid();
  MissionTask_ResetS6ControllerState();
  ae_reply_sequence_floor = MaixCam_FenceObjectReference();
  if (mission_ctx.supplement_capture_active) mission_ctx.supplement_coordinate_floor = ae_reply_sequence_floor;
  /* Open BEFORE TX so prompt new-frame responses survive; no post-TX clear.
     Old partial target frames remain fenced;05/06/11/AE stay parseable. */
  MaixCam_SetStartZoneAvoidanceActive(false);
  if (MaixCam_SendCommand(MAIXCAM_COMMAND_START_ZONE_AVOID_DONE) != HAL_OK)
  {
    MaixCam_SetStartZoneAvoidanceActive(true);
    /* Keep pending until TX succeeds; no camera retransmission is required. */
    ae_next_reply_tick = HAL_GetTick() + ROBOT_AE_ACK_RETRY_MS;
    (void)DebugUart_Log("[AE] TX FAIL RETRY\r\n");
    return;
  }
  mission_ctx.ae_reply_pending = false;
  mission_ctx.ae_ack_completed = true;
  ae_fresh_coordinate_seen = false;
  now = HAL_GetTick();
  if (MissionTask_HandleLoadCheckRequest(now) || MissionTask_HandleSupplementFinish(now) ||
      MissionTask_LoadCheckPausesMotion()) return;
  MissionTask_EnterState(mission_ctx.ae_resume_state, now);
  /* A new03 reference starts now, not the old04/24 elapsed phase. None of
     the mission/recovery counters or supplement budget are restarted. */
  mission_ctx.s3_search_origin_yaw_cdeg = MissionTask_WrapYaw(mission_ctx.mission_snapshot.yaw_cdeg);
  mission_ctx.s3_target_lost_since = now;
  mission_ctx.s3_track_stable_since = 0U;
  mission_ctx.s3_wait_event04_logged = false;
  mission_ctx.s4_center_target_lost_since = now;
  mission_ctx.s4_final_missing_since = now;
  mission_ctx.supplement_missing_since = now;
  mission_ctx.s7_last_message_tick = now;
  MissionTask_UpdateVisionInput(now);
  (void)DebugUart_Logf("[AE] TXAE >03 s=%u\r\n", (unsigned int)mission_ctx.ae_resume_state);
}

void MissionTask_DebugVisionAge(uint32_t now)
{
  const char *reason;
  MissionState state = mission_ctx.mission_snapshot.state;
  bool tracking = state == MISSION_STATE_S3_TRACK_GREEN ||
      state == MISSION_STATE_S4_TRACK_CENTER ||
      state == MISSION_STATE_S4_TRACK_FINAL_BLOCK ||
      state == MISSION_STATE_S6_TRACK_SAFE_ZONE ||
      state == MISSION_STATE_S6_REPOSITION_FACE_SAFE;
  if ((int32_t)(now - vision_age_next_debug_tick) < 0) return;
  vision_age_next_debug_tick = now + ROBOT_VISION_COORD_DEBUG_PERIOD_MS;
  if (tracking)
  {
    if (!mission_ctx.mission_snapshot.vision_target_valid)
      reason = mission_ctx.mission_snapshot.target_present ? "OLD" : "NONE";
    else if (mission_ctx.mission_snapshot.target_age_ms >= MISSION_VISION_TARGET_TIMEOUT_MS)
      reason = "OLD";
    else if (mission_ctx.mission_snapshot.target_age_ms > ROBOT_VISION_COORD_DECEL_START_MS)
      reason = "SLOW";
    else if (mission_ctx.commanded_left_rpm == 0 && mission_ctx.commanded_right_rpm == 0)
      reason = "XY";
    else reason = "PID";
  }
  else if (state == MISSION_STATE_S5_WAIT_SINGLE_GREEN ||
           state == MISSION_STATE_SUPPLEMENT_LOAD_CHECK ||
           state == MISSION_STATE_S6_RECHECK_WAIT_RESULT) reason = "CHECK05";
  else if (state == MISSION_STATE_S6_RECHECK_WAIT_ACK) reason = "RX05";
  else if (state == MISSION_STATE_SUPPLEMENT_WAIT_RECHECK) reason = "LIMIT05";
  else if (state == MISSION_STATE_S6_RECHECK_WIDE_SETTLE) reason = "WIDE";
  else if (state == MISSION_STATE_S4_WAIT_ARRANGE_READY ||
           state == MISSION_STATE_S4_WAIT_LEFT_TARGET ||
           state == MISSION_STATE_S4_WAIT_FINAL_READY) reason = "ACK";
  else if (state == MISSION_STATE_S3_LOWER_FRAME ||
           state == MISSION_STATE_S4_RAISE_FRAME ||
           state == MISSION_STATE_S5_LOWER_FOR_ARRANGE)
    reason = "FRAME";
  else if (state == MISSION_STATE_FAULT) reason = "FAULT";
  else if (state == MISSION_STATE_STOPPED || state == MISSION_STATE_WAIT_START)
    reason = "IDLE";
  else reason = mission_ctx.commanded_left_rpm == 0 && mission_ctx.commanded_right_rpm == 0
      ? "HOLD" : "MOVE";

  (void)DebugUart_Logf(
      "[V] s=%u id=%u q=%lu age=%lu ok=%u why=%s w=%d/%d\r\n",
      (unsigned int)state, (unsigned int)mission_ctx.mission_snapshot.target_id,
      (unsigned long)mission_ctx.mission_snapshot.target_sequence,
      (unsigned long)mission_ctx.mission_snapshot.target_age_ms,
      mission_ctx.mission_snapshot.vision_target_valid ? 1U : 0U, reason,
      (int)mission_ctx.commanded_left_rpm, (int)mission_ctx.commanded_right_rpm);
  if (mission_ctx.s6_pre_reposition_decision_pending || mission_ctx.s6_align26_pending)
    (void)DebugUart_Logf("[PRE] ms=%lu/%lu xN=%u/%u span=%lu finalN=%u dec=%u p26=%u\r\n",
        (unsigned long)mission_ctx.s6_pre_reposition_track_elapsed_ms,
        (unsigned long)ROBOT_S6_PRE_REPOSITION_TRACK_MS,
        (unsigned int)mission_ctx.s6_side_x_align_frames, (unsigned int)ROBOT_S6_X_ALIGN_MIN_FRAMES,
        (unsigned long)(mission_ctx.s6_side_x_align_frames ? mission_ctx.s6_x_align_last_tick - mission_ctx.s6_side_x_align_first_tick : 0U),
        (unsigned int)mission_ctx.s6_x_align_frames,
        mission_ctx.s6_pre_reposition_decision_pending ? 1U : 0U, mission_ctx.s6_align26_pending ? 1U : 0U);
}
