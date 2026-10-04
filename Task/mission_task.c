#include "mission_task.h"

#include "M2006.h"
#include "M2006_Speed.h"
#include "actuator_task.h"
#include "cmsis_os.h"
#include "debug_uart_task.h"
#include "jy901s.h"
#include "maixcam_task.h"
#include "robot_config.h"
#include "stm32f4xx_hal.h"

#define MISSION_TASK_PERIOD_TICKS       20U
#define MISSION_TASK_STACK_BYTES        (512U * 4U)
#define MISSION_MOTOR_TIMEOUT_MS        100U
#define MISSION_IMU_TIMEOUT_MS          150U
#define MISSION_VISION_TARGET_TIMEOUT_MS MAIXCAM_DATA_TIMEOUT_MS
#if ROBOT_VISION_COORD_DECEL_START_MS >= MAIXCAM_DATA_TIMEOUT_MS
#error "Coordinate fade must start before the coordinate timeout"
#endif
#define MISSION_PE13_DEBOUNCE_SAMPLES     5U
#define MISSION_PE14_DEBOUNCE_SAMPLES     5U
#define MISSION_PE15_DEBOUNCE_SAMPLES     5U

#define MISSION_LEFT_MOTOR_INDEX        (ROBOT_LEFT_WHEEL_MOTOR_ID - 1U)
#define MISSION_RIGHT_MOTOR_INDEX       (ROBOT_RIGHT_WHEEL_MOTOR_ID - 1U)

#define MISSION_EVENT_START             (1UL << 0)
#define MISSION_EVENT_STOP              (1UL << 1)
#define MISSION_EVENT_RESET             (1UL << 2)
#define MISSION_EVENT_EMERGENCY_STOP    (1UL << 3)
#define MISSION_EVENT_MASK              (MISSION_EVENT_START | \
                                         MISSION_EVENT_STOP | \
                                         MISSION_EVENT_RESET | \
                                         MISSION_EVENT_EMERGENCY_STOP)

static osThreadId_t mission_task_handle;
static osEventFlagsId_t mission_event_handle;
static osMutexId_t mission_snapshot_mutex;
static MissionSnapshot mission_snapshot;
static uint32_t angle_next_debug_tick;
static uint32_t vision_age_next_debug_tick;
static uint32_t recovery_duplicate_next_debug_tick;
/* Target changes are allowed only after this mission has entered 07. */
static bool s7_target_switch_enabled;
/* A recovery is consumed when it starts, even if coordinates interrupt it. */
static bool s7_red_recovery_used;
/* Retain 51's actual selected object through a confirmed loss. ID4 frames
   arriving with RX11 must not overwrite the last black/green identity. */
static uint8_t s51_active_object_id;
/* Freeze the carried class for 06; safe-zone coordinate IDs are not cargo. */
static uint8_t s6_carried_object_id;
static bool s5_load_class_confirmed;
static uint32_t s5_load_check_sequence_floor;
static bool s7_red_switch_sync_pending;
static bool s7_red_coordinate_fence_active;
static uint32_t s7_red_coordinate_sequence_floor;
static uint32_t s7_last_message_tick;
static uint32_t s7_last_coordinate_sequence;
static MissionState s7_silent_resume_state;
static int32_t s7_silent_left_yaw_cdeg;
static int32_t s7_silent_right_yaw_cdeg;
static uint32_t s1_turn_stable_since;
static uint32_t s1_next_debug_tick;
static bool s2_drive_finished;
static uint32_t s2_next_debug_tick;
static uint32_t s3_search_turn_stable_since;
static int32_t s3_search_origin_yaw_cdeg;
static int32_t s3_search_yaw_target_cdeg;
static uint8_t s3_target_select_command = MAIXCAM_COMMAND_SELECT_GREEN;
static bool s3_search_sweep_completed;
static bool s3_search_ccw_midpoint_done;
static bool vision_search_forward_active;
static int16_t vision_search_forward_rpm;
static uint32_t vision_search_forward_ms;
static int32_t vision_search_turn_angle_cdeg;
static bool vision_search_e3_mode;
typedef enum
{
  S3_GREEN_E3_FORWARD = 0,
  S3_GREEN_E3_TURN_LEFT,
  S3_GREEN_E3_TURN_RIGHT,
  S3_GREEN_E3_TURN_BACK
} S3GreenE3Phase;
static bool vision_search_green_e3_mode;
static S3GreenE3Phase s3_green_e3_phase;
static int32_t s3_green_e3_origin_yaw_cdeg;
static int32_t e3_spin_previous_yaw_cdeg;
static int32_t e3_spin_progress_cdeg;
static MissionState vision_search_resume_state;
static uint32_t s3_target_lost_since;
static uint32_t s3_track_stable_since;
static bool s3_wait_event04_logged;
static uint32_t s3_next_debug_tick;
static uint32_t s3_pid_update_tick;
static uint32_t s3_last_target_sequence;
static float s3_pid_integral;
static float s3_pid_last_error;
static float s3_pid_output;
static bool s3_pid_initialized;
static uint32_t s3_y_pid_update_tick;
static uint32_t s3_y_last_target_sequence;
static float s3_y_pid_integral;
static float s3_y_pid_last_error;
static float s3_y_pid_output;
static bool s3_y_pid_initialized;
static uint32_t s4_next_debug_tick;
static bool s4_final_mode_already_active;
static bool s4_final_event34_pending;
static bool s4_final_frame_lowered;
static bool s4_e4_recovery_used;
static bool s4_e4_protocol_active;
static uint8_t s4_green_e4_push_cycles;
static bool s4_green_e4_spin_exhausted;
static int32_t s4_e4_spin_origin_yaw_cdeg;
static int32_t s4_e4_spin_previous_yaw_cdeg;
static int32_t s4_e4_spin_progress_cdeg;
static int32_t s4_e4_left_yaw_cdeg;
static int32_t s4_e4_away_yaw_cdeg;
static int32_t s4_e4_right_yaw_cdeg;
static int32_t s4_post22_left_yaw_cdeg;
static int32_t s4_post22_right_yaw_cdeg;
static bool s4_post22_search_active;
static uint16_t s4_arrange_cycle_count;
static uint32_t s4_center_target_lost_since;
static bool s4_center_search_sweep_completed;
static uint32_t s4_final_missing_since;
static uint32_t s4_final_turn_stable_since;
static int32_t s4_final_first_yaw_cdeg;
static int32_t s4_final_second_yaw_cdeg;
static uint8_t s4_final_recovery_count;
static MissionState s4_arrange_recovery_resume_state;
static int32_t s4_arrange_recovery_base_yaw_cdeg;
static int32_t s4_arrange_recovery_left_yaw_cdeg;
static int32_t s4_arrange_recovery_right_yaw_cdeg;
static uint8_t s4_arrange_recovery_count;
static uint32_t s5_next_debug_tick;
static bool s5_close_view_active;
static bool s5_empty_recovery_active;
static uint32_t s6_phase_stable_since;
static uint32_t s6_next_debug_tick;
static uint32_t s6_yaw_pid_update_tick;
static float s6_yaw_pid_integral;
static float s6_yaw_pid_last_error;
static bool s6_yaw_pid_initialized;
static int32_t s6_recovery_base_yaw_cdeg;
static int32_t s6_recovery_left_yaw_cdeg;
static int32_t s6_recovery_right_yaw_cdeg;
static int32_t s6_push_yaw_target_cdeg;
static int32_t s6_exit_yaw_target_cdeg;
static int32_t s6_obstacle_left_yaw_cdeg;
static int32_t s6_obstacle_right_yaw_cdeg;
static uint32_t s6_missing_since;
static uint32_t s6_search_start_tick;
static uint32_t s6_approach_start_tick;
static uint32_t s6_track_elapsed_ms;
static uint32_t s6_track_accounting_tick;
static int32_t s6_reposition_side_yaw_cdeg;
static uint32_t s6_x_align_last_sequence;
static uint32_t s6_x_align_first_tick;
static uint32_t s6_x_align_last_tick;
static uint8_t s6_x_align_id;
static uint8_t s6_x_align_frames;
static uint32_t s6_side_x_align_first_tick;
static uint8_t s6_side_x_align_frames;
static bool s6_x_align_has_sequence;
static bool s6_pre_reposition_decision_pending;
static uint32_t s6_pre_reposition_track_elapsed_ms;
static uint32_t s6_pre_reposition_track_tick;
static bool s6_pre_reposition_track_active;
static bool s6_align26_pending;
static bool s6_supply_y_started;
static bool s6_obstacle_side_required;
static bool s6_obstacle_side_decision_pending;
static bool s6_obstacle_side_decision_done;
static int32_t s6_obstacle_decision_yaw_cdeg;
static uint8_t s6_reposition_count;
static uint8_t s6_recovery_count;
static bool s6_recovery_from_tracking;
static int16_t commanded_left_rpm;
static int16_t commanded_right_rpm;
static GPIO_PinState pe13_candidate;
static GPIO_PinState pe13_stable;
static uint8_t pe13_stable_samples;
static bool pe13_initialized;
static GPIO_PinState pe14_candidate;
static GPIO_PinState pe14_stable;
static uint8_t pe14_stable_samples;
static bool pe14_initialized;
static GPIO_PinState pe15_candidate;
static GPIO_PinState pe15_stable;
static uint8_t pe15_stable_samples;
static bool pe15_initialized;

static const osThreadAttr_t mission_task_attributes = {
  .name = "missionTask",
  .stack_size = MISSION_TASK_STACK_BYTES,
  .priority = (osPriority_t)osPriorityAboveNormal,
};

static int16_t MissionTask_CalculateS6YawPid(uint32_t now,
                                             int32_t yaw_command_cdeg,
                                             float turn_min_rpm,
                                             float turn_max_rpm);
static int16_t MissionTask_CalculateS6YawTurn(uint32_t now,
                                              int32_t yaw_command_cdeg);
static bool MissionTask_RunS6TurnToHeading(uint32_t now,
                                            int32_t target_yaw_cdeg);
static void MissionTask_CommandS6Straight(uint32_t now,
                                           int32_t target_yaw_cdeg,
                                           int16_t forward_rpm,
                                           int16_t wheel_max_rpm);
static void MissionTask_StartVisionSearchRecovery(
    uint32_t now, MissionState resume_state, int32_t origin_yaw_cdeg,
    bool initial_green_e3);
static void MissionTask_CompleteS7RedRecovery(uint32_t now);
static void MissionTask_SwitchS7ToBlackGreen(uint32_t now);
static void MissionTask_ResumeRedTracking(uint32_t now);
static void MissionTask_StartS7Decision(uint32_t now);
static void MissionTask_StartS6ObstacleAvoidance(uint32_t now);
static void MissionTask_DebugAngles(uint32_t now, bool force);
static uint32_t MissionTask_S4PushDuration(bool push_right);
static uint32_t MissionTask_S4ScalePushDuration(uint32_t duration, bool push_right);
static uint32_t MissionTask_S4ReverseDuration(bool push_right);
static uint8_t MissionTask_ActiveRecoveryLossEvent(void);
static void MissionTask_HandleS6AlignRequest(uint32_t now);
static void MissionTask_DebugVisionAge(uint32_t now);
static void MissionTask_ResetS6XConfirmation(void);
static bool MissionTask_S6IsSupplyTarget(void);
static bool MissionTask_S6IsCasualtyTarget(void);

static int32_t MissionTask_WrapYaw(int32_t angle_cdeg)
{
  while (angle_cdeg < 0L)
  {
    angle_cdeg += 36000L;
  }
  while (angle_cdeg >= 36000L)
  {
    angle_cdeg -= 36000L;
  }
  return angle_cdeg;
}

static int32_t MissionTask_YawError(int32_t target_cdeg,
                                    int32_t current_cdeg)
{
  int32_t error = MissionTask_WrapYaw(target_cdeg) -
                  MissionTask_WrapYaw(current_cdeg);

  if (error > 18000L)
  {
    error -= 36000L;
  }
  else if (error < -18000L)
  {
    error += 36000L;
  }
  return error;
}

static int32_t MissionTask_Abs32(int32_t value)
{
  return value < 0L ? -value : value;
}

static float MissionTask_ClampFloat(float value, float minimum, float maximum)
{
  if (value < minimum)
  {
    return minimum;
  }
  if (value > maximum)
  {
    return maximum;
  }
  return value;
}

static void MissionTask_ResetVisionControllerState(void)
{
  s3_pid_integral = 0.0f;
  s3_pid_last_error = 0.0f;
  s3_pid_output = 0.0f;
  s3_pid_update_tick = 0U;
  s3_last_target_sequence = UINT32_MAX;
  s3_pid_initialized = false;

  s3_y_pid_integral = 0.0f;
  s3_y_pid_last_error = 0.0f;
  s3_y_pid_output = 0.0f;
  s3_y_pid_update_tick = 0U;
  s3_y_last_target_sequence = UINT32_MAX;
  s3_y_pid_initialized = false;

  mission_snapshot.vision_turn_rpm = 0;
  mission_snapshot.vision_forward_rpm = 0;
}

static void MissionTask_ResetVisionPid(void)
{
  MissionTask_ResetVisionControllerState();
  mission_snapshot.vision_x_error_px = 0;
  mission_snapshot.vision_y_error_px = 0;
}

static void MissionTask_ResetS6ControllerState(void)
{
  s6_yaw_pid_integral = 0.0f;
  s6_yaw_pid_last_error = 0.0f;
  s6_yaw_pid_update_tick = 0U;
  s6_yaw_pid_initialized = false;

  mission_snapshot.vision_turn_rpm = 0;
  mission_snapshot.vision_forward_rpm = 0;
  mission_snapshot.safe_zone_yaw_command_cdeg =
      mission_snapshot.safe_zone_yaw_target_cdeg;
  mission_snapshot.safe_zone_yaw_error_cdeg = 0;
  mission_snapshot.safe_zone_fixed_heading_active = false;
}

static void MissionTask_SetWheelTargets(int16_t left_rpm,
                                        int16_t right_rpm)
{
  if ((left_rpm == commanded_left_rpm) &&
      (right_rpm == commanded_right_rpm))
  {
    return;
  }

  commanded_left_rpm = left_rpm;
  commanded_right_rpm = right_rpm;
  mission_snapshot.left_target_rpm = left_rpm;
  mission_snapshot.right_target_rpm = right_rpm;

  SpeedLoop_SetMotorTarget(ROBOT_LEFT_WHEEL_MOTOR_ID,
      (float)left_rpm * ROBOT_LEFT_WHEEL_FORWARD_SIGN,
      ROBOT_S1_CURRENT_LIMIT);
  SpeedLoop_SetMotorTarget(ROBOT_RIGHT_WHEEL_MOTOR_ID,
      (float)right_rpm * ROBOT_RIGHT_WHEEL_FORWARD_SIGN,
      ROBOT_S1_CURRENT_LIMIT);
}

static void MissionTask_StopWheels(void)
{
  brake();
  commanded_left_rpm = 0;
  commanded_right_rpm = 0;
  mission_snapshot.left_target_rpm = 0;
  mission_snapshot.right_target_rpm = 0;
}

static uint8_t MissionTask_ActiveRecoveryLossEvent(void)
{
  MissionState state = mission_snapshot.state;
  if ((state == MISSION_STATE_S3_SEARCH_TURN_CW) ||
      (state == MISSION_STATE_S3_SEARCH_TURN_CCW))
  {
    return vision_search_resume_state == MISSION_STATE_S4_TRACK_CENTER
        ? MAIXCAM_EVENT_CENTER_TARGET_LOST : MAIXCAM_EVENT_SEARCH_TARGET_LOST;
  }
  if (((state >= MISSION_STATE_S4_E4_PUSH_RIGHT) &&
       (state <= MISSION_STATE_S4_E4_TURN_RIGHT)) ||
      (state == MISSION_STATE_S4_E4_GREEN_SPIN_360) ||
      (state == MISSION_STATE_S4_E4_RED_SPIN_360))
    return MAIXCAM_EVENT_CENTER_TARGET_LOST;
  if ((state >= MISSION_STATE_S4_FINAL_RECOVERY_REVERSE) &&
      (state <= MISSION_STATE_S4_FINAL_RECOVERY_TURN_90))
    return MAIXCAM_EVENT_NO_TARGET;
  return 0U;
}

static void MissionTask_DropDuplicateRecoveryEvent(uint32_t now)
{
  uint8_t event_code = MissionTask_ActiveRecoveryLossEvent();
  if ((event_code != 0U) && MaixCam_DropEventIf(event_code) &&
      ((int32_t)(now - recovery_duplicate_next_debug_tick) >= 0))
  {
    recovery_duplicate_next_debug_tick = now + ROBOT_VISION_COORD_DEBUG_PERIOD_MS;
    (void)DebugUart_Logf("[REC-DUP] st=%u event=%02X IGNORE_ACTIVE, TIMER_UNCHANGED\r\n",
                         (unsigned int)mission_snapshot.state,
                         (unsigned int)event_code);
  }
}

static void MissionTask_DropUnexpectedS5E5(void)
{
  uint16_t ignored = MaixCam_TakeIgnoredLoadEmptyRecoveryCount();
  bool allowed = mission_snapshot.state == MISSION_STATE_S5_WAIT_SINGLE_GREEN &&
      !s7_target_switch_enabled &&
      s3_target_select_command == MAIXCAM_COMMAND_SELECT_GREEN;

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
        "[S5-E5] IGNORE count=%u current_st=%u cmd=%02X after07=%u; "
        "ONLY INITIAL GREEN05, TIMER_UNCHANGED\r\n",
        (unsigned int)ignored, (unsigned int)mission_snapshot.state,
        (unsigned int)s3_target_select_command,
        s7_target_switch_enabled ? 1U : 0U);
  }
}

static void MissionTask_SetS1Phase(MissionS1Phase phase, uint32_t now)
{
  mission_snapshot.s1_phase = phase;
  mission_snapshot.phase_entry_tick = now;
}

static void MissionTask_UpdatePe13(void)
{
  GPIO_PinState sample = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_13);

  if (!pe13_initialized)
  {
    pe13_candidate = sample;
    pe13_stable = sample;
    pe13_stable_samples = 1U;
    pe13_initialized = true;
    return;
  }

  if (sample == pe13_candidate)
  {
    if (pe13_stable_samples < MISSION_PE13_DEBOUNCE_SAMPLES)
    {
      ++pe13_stable_samples;
    }
  }
  else
  {
    pe13_candidate = sample;
    pe13_stable_samples = 1U;
  }

  if (pe13_stable_samples >= MISSION_PE13_DEBOUNCE_SAMPLES)
  {
    pe13_stable = pe13_candidate;
  }
}

static void MissionTask_UpdatePe14(void)
{
  GPIO_PinState sample = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_14);

  if (!pe14_initialized)
  {
    pe14_candidate = sample;
    pe14_stable = sample;
    pe14_stable_samples = 1U;
    pe14_initialized = true;
    return;
  }

  if (sample == pe14_candidate)
  {
    if (pe14_stable_samples < MISSION_PE14_DEBOUNCE_SAMPLES)
    {
      ++pe14_stable_samples;
    }
  }
  else
  {
    pe14_candidate = sample;
    pe14_stable_samples = 1U;
  }

  if (pe14_stable_samples >= MISSION_PE14_DEBOUNCE_SAMPLES)
  {
    pe14_stable = pe14_candidate;
  }
}

static void MissionTask_UpdatePe15(void)
{
  GPIO_PinState sample = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_15);

  if (!pe15_initialized)
  {
    pe15_candidate = sample;
    pe15_stable = sample;
    pe15_stable_samples = 1U;
    pe15_initialized = true;
    return;
  }

  if (sample == pe15_candidate)
  {
    if (pe15_stable_samples < MISSION_PE15_DEBOUNCE_SAMPLES)
    {
      ++pe15_stable_samples;
    }
  }
  else
  {
    pe15_candidate = sample;
    pe15_stable_samples = 1U;
  }

  if (pe15_stable_samples >= MISSION_PE15_DEBOUNCE_SAMPLES)
  {
    pe15_stable = pe15_candidate;
  }
}

static uint8_t MissionTask_DecodeStartZone(GPIO_PinState pe14,
                                           GPIO_PinState pe15)
{
  uint8_t selector = (uint8_t)((pe14 == GPIO_PIN_SET ? 2U : 0U) |
                               (pe15 == GPIO_PIN_SET ? 1U : 0U));

  switch (selector)
  {
    case 0U: return ROBOT_START_ZONE_BITS_00;
    case 1U: return ROBOT_START_ZONE_BITS_01;
    case 2U: return ROBOT_START_ZONE_BITS_10;
    case 3U: return ROBOT_START_ZONE_BITS_11;
    default: return ROBOT_START_ZONE_BITS_00;
  }
}

static int32_t MissionTask_GetSafeZoneYawOffset(uint8_t start_zone,
                                                uint8_t team)
{
  bool reverse_heading;

  if ((start_zone == 1U) || (start_zone == 2U))
  {
    reverse_heading = team == ROBOT_TEAM_RED;
  }
  else
  {
    reverse_heading = team == ROBOT_TEAM_BLUE;
  }

  return reverse_heading ? ROBOT_SAFE_ZONE_REVERSE_OFFSET_CDEG
                         : ROBOT_SAFE_ZONE_FORWARD_OFFSET_CDEG;
}

static bool MissionTask_MotorIsOnline(uint8_t motor_index, uint32_t now)
{
  uint32_t primask;
  uint32_t message_count;
  uint32_t last_rx_tick;

  if (motor_index >= 4U)
  {
    return false;
  }

  primask = __get_PRIMASK();
  __disable_irq();
  message_count = moto_chassis[motor_index].msg_cnt;
  last_rx_tick = moto_chassis[motor_index].last_rx_tick;
  if (primask == 0U)
  {
    __enable_irq();
  }

  return (message_count != 0U) &&
         ((uint32_t)(now - last_rx_tick) <=
          MISSION_MOTOR_TIMEOUT_MS);
}

static void MissionTask_EnterState(MissionState state, uint32_t now)
{
  if (mission_snapshot.state == state)
  {
    return;
  }

  mission_snapshot.state = state;
  mission_snapshot.state_entry_tick = now;
  MaixCam_SetLoadEmptyRecoveryEnabled(
      state == MISSION_STATE_S5_WAIT_SINGLE_GREEN &&
      !s7_target_switch_enabled &&
      s3_target_select_command == MAIXCAM_COMMAND_SELECT_GREEN);
  if ((state < MISSION_STATE_S4_E4_PUSH_RIGHT) ||
      (state > MISSION_STATE_S4_E4_REVERSE_LEFT))
  {
    s5_empty_recovery_active = false;
  }
  if (state != MISSION_STATE_S6_TRACK_SAFE_ZONE)
  {
    s6_pre_reposition_track_active = false;
    s6_pre_reposition_track_tick = now;
  }
  MaixCam_SetRecoveryLossFilter(MissionTask_ActiveRecoveryLossEvent());
  if ((state == MISSION_STATE_STOPPED) || (state == MISSION_STATE_FAULT) ||
      (state == MISSION_STATE_WAIT_START))
  {
    s6_align26_pending = false;
    s6_pre_reposition_decision_pending = false;
    s6_obstacle_side_required = false;
    s6_obstacle_side_decision_pending = false;
    s6_obstacle_side_decision_done = false;
    MissionTask_ResetS6XConfirmation();
  }
  if ((state == MISSION_STATE_S7_SEARCH_RED) ||
      (state == MISSION_STATE_S7_WAIT_SECOND_E3) ||
      (state == MISSION_STATE_S7_SEARCH_BLACK))
  {
    s7_last_message_tick = now;
    s7_last_coordinate_sequence = mission_snapshot.target_sequence;
  }

  switch (state)
  {
    case MISSION_STATE_WAIT_START:
      (void)DebugUart_Log("[MISSION] WAIT_START\r\n");
      break;
    case MISSION_STATE_S1_DEPART:
      (void)DebugUart_Log("[MISSION] S1_DEPART\r\n");
      break;
    case MISSION_STATE_S2_CROSS_BUMP:
      (void)DebugUart_Log("[MISSION] S2_CROSS_BUMP\r\n");
      break;
    case MISSION_STATE_S3_ALIGN_GREEN:
      (void)DebugUart_Log("[MISSION] S3_ALIGN_GREEN\r\n");
      break;
    case MISSION_STATE_S3_SEARCH_TURN_CW:
      (void)DebugUart_Log("[MISSION] S3_SEARCH_TURN_CW\r\n");
      break;
    case MISSION_STATE_S3_SEARCH_TURN_CCW:
      (void)DebugUart_Log("[MISSION] S3_SEARCH_TURN_CCW\r\n");
      break;
    case MISSION_STATE_S3_TRACK_GREEN:
      (void)DebugUart_Log("[MISSION] S3_TRACK_GREEN\r\n");
      break;
    case MISSION_STATE_S3_LOWER_FRAME:
      (void)DebugUart_Log("[MISSION] S3_LOWER_FRAME\r\n");
      break;
    case MISSION_STATE_S4_TRACK_CENTER:
      (void)DebugUart_Log("[MISSION] S4_TRACK_CENTER\r\n");
      break;
    case MISSION_STATE_S4_CENTER_FOLLOW_THROUGH:
      (void)DebugUart_Log("[MISSION] S4_CENTER_FOLLOW_THROUGH\r\n");
      break;
    case MISSION_STATE_S4_WAIT_ARRANGE_READY:
      (void)DebugUart_Log("[MISSION] S4_WAIT_RX_02_OR_24\r\n");
      break;
    case MISSION_STATE_S4_TRACK_RIGHT_BLOCK:
      (void)DebugUart_Log("[MISSION] S4_TRACK_RIGHT_TO_LEFT_CORNER\r\n");
      break;
    case MISSION_STATE_S4_REVERSE_RIGHT_BLOCK:
      (void)DebugUart_Log("[MISSION] S4_REVERSE_RIGHT_BLOCK\r\n");
      break;
    case MISSION_STATE_S4_WAIT_LEFT_TARGET:
      (void)DebugUart_Log("[MISSION] S4_WAIT_RX_12\r\n");
      break;
    case MISSION_STATE_S4_WAIT_FINAL_READY:
      (void)DebugUart_Log("[MISSION] S4_WAIT_RX_04_14_24_E4_E2\r\n");
      break;
    case MISSION_STATE_S4_POST22_REVERSE:
    case MISSION_STATE_S4_POST22_TURN_LEFT:
    case MISSION_STATE_S4_POST22_TURN_RIGHT:
      (void)DebugUart_Logf("[MISSION] S4_POST22_E2 state=%u\r\n",
                           (unsigned int)state);
      break;
    case MISSION_STATE_S4_E4_PUSH_RIGHT:
    case MISSION_STATE_S4_E4_REVERSE_RIGHT:
    case MISSION_STATE_S4_E4_PUSH_LEFT:
    case MISSION_STATE_S4_E4_REVERSE_LEFT:
    case MISSION_STATE_S4_E4_TURN_AWAY:
    case MISSION_STATE_S4_E4_FORWARD:
    case MISSION_STATE_S4_E4_TURN_LEFT:
    case MISSION_STATE_S4_E4_TURN_RIGHT:
      (void)DebugUart_Logf("[MISSION] S4_E4_RECOVERY state=%u\r\n",
                           (unsigned int)state);
      break;
    case MISSION_STATE_S4_E4_GREEN_SPIN_360:
      (void)DebugUart_Log("[MISSION] S4_E4_GREEN_SPIN_360\r\n");
      break;
    case MISSION_STATE_S4_E4_RED_SPIN_360:
      (void)DebugUart_Log("[MISSION] S4_E4_RED_SPIN_360\r\n");
      break;
    case MISSION_STATE_S4_TRACK_LEFT_BLOCK:
      (void)DebugUart_Log("[MISSION] S4_TRACK_LEFT_TO_RIGHT_CORNER\r\n");
      break;
    case MISSION_STATE_S4_REVERSE_LEFT_BLOCK:
      (void)DebugUart_Log("[MISSION] S4_REVERSE_LEFT_BLOCK\r\n");
      break;
    case MISSION_STATE_S4_ARRANGE_RECOVERY_REVERSE:
      (void)DebugUart_Log("[MISSION] S4_ARRANGE_RECOVERY_REVERSE\r\n");
      break;
    case MISSION_STATE_S4_ARRANGE_RECOVERY_TURN_LEFT:
      (void)DebugUart_Log("[MISSION] S4_ARRANGE_RECOVERY_TURN_LEFT\r\n");
      break;
    case MISSION_STATE_S4_ARRANGE_RECOVERY_TURN_BACK:
      (void)DebugUart_Log("[MISSION] S4_ARRANGE_RECOVERY_TURN_BACK\r\n");
      break;
    case MISSION_STATE_S4_RAISE_FRAME:
      (void)DebugUart_Log("[MISSION] S4_RAISE_FRAME\r\n");
      break;
    case MISSION_STATE_S4_TRACK_FINAL_BLOCK:
      (void)DebugUart_Log("[MISSION] S4_TRACK_FINAL_TO_FRAME\r\n");
      break;
    case MISSION_STATE_S4_FINAL_RECOVERY_REVERSE:
      (void)DebugUart_Log("[MISSION] S4_FINAL_RECOVERY_REVERSE\r\n");
      break;
    case MISSION_STATE_S4_FINAL_RECOVERY_TURN_270:
      (void)DebugUart_Log("[MISSION] S4_FINAL_RECOVERY_TURN_SIDE1\r\n");
      break;
    case MISSION_STATE_S4_FINAL_RECOVERY_TURN_90:
      (void)DebugUart_Log("[MISSION] S4_FINAL_RECOVERY_TURN_SIDE2\r\n");
      break;
    case MISSION_STATE_S4_FINAL_CENTER_OBJECT:
      (void)DebugUart_Log("[MISSION] S4_FINAL_CENTER_OBJECT\r\n");
      break;
    case MISSION_STATE_S4_FINAL_LOWER_FRAME:
      (void)DebugUart_Log("[MISSION] S4_FINAL_LOWER_FRAME\r\n");
      break;
    case MISSION_STATE_S5_WAIT_SINGLE_GREEN:
      (void)DebugUart_Log("[MISSION] S5_WAIT_SINGLE_GREEN\r\n");
      break;
    case MISSION_STATE_S5_ALIGN_FOR_ARRANGE:
      (void)DebugUart_Log("[MISSION] S5_ALIGN_FOR_ARRANGE\r\n");
      break;
    case MISSION_STATE_S5_RAISE_FOR_ARRANGE:
      (void)DebugUart_Log("[MISSION] S5_RAISE_FOR_ARRANGE\r\n");
      break;
    case MISSION_STATE_S5_REVERSE_FOR_ARRANGE:
      (void)DebugUart_Log("[MISSION] S5_REVERSE_FOR_ARRANGE\r\n");
      break;
    case MISSION_STATE_S5_LOWER_FOR_ARRANGE:
      (void)DebugUart_Log("[MISSION] S5_LOWER_FOR_ARRANGE\r\n");
      break;
    case MISSION_STATE_S6_SEARCH_SAFE_ZONE:
      (void)DebugUart_Log("[MISSION] S6_SEARCH_SAFE_ZONE\r\n");
      break;
    case MISSION_STATE_S6_REPOSITION_TURN_SIDE:
      (void)DebugUart_Log("[MISSION] S6_REPOSITION_TURN_SIDE\r\n");
      break;
    case MISSION_STATE_S6_REPOSITION_FORWARD:
      (void)DebugUart_Log("[MISSION] S6_REPOSITION_FORWARD\r\n");
      break;
    case MISSION_STATE_S6_REPOSITION_FACE_SAFE:
      (void)DebugUart_Log("[MISSION] S6_REPOSITION_FACE_SAFE\r\n");
      break;
    case MISSION_STATE_S6_REPOSITION_TURN_SAFE:
      (void)DebugUart_Log("[MISSION] S6_REPOSITION_TURN_SAFE\r\n");
      break;
    case MISSION_STATE_S6_RECOVERY_START:
      (void)DebugUart_Log("[MISSION] S6_RECOVERY_START\r\n");
      break;
    case MISSION_STATE_S6_RECOVERY_TURN_LEFT:
      (void)DebugUart_Log("[MISSION] S6_RECOVERY_TURN_LEFT\r\n");
      break;
    case MISSION_STATE_S6_RECOVERY_TURN_BACK:
      (void)DebugUart_Log("[MISSION] S6_RECOVERY_TURN_BACK\r\n");
      break;
    case MISSION_STATE_S6_RECOVERY_RETURN_SAFE:
      (void)DebugUart_Log("[MISSION] S6_RECOVERY_RETURN_SAFE\r\n");
      break;
    case MISSION_STATE_S6_TRACK_SAFE_ZONE:
      (void)DebugUart_Log("[MISSION] S6_VISION_APPROACH\r\n");
      break;
    case MISSION_STATE_S6_OBSTACLE_TURN_LEFT:
      (void)DebugUart_Log("[MISSION] S6_OBSTACLE_TURN_LEFT\r\n");
      break;
    case MISSION_STATE_S6_OBSTACLE_FORWARD:
      (void)DebugUart_Log("[MISSION] S6_OBSTACLE_FORWARD\r\n");
      break;
    case MISSION_STATE_S6_OBSTACLE_TURN_RIGHT:
      (void)DebugUart_Log("[MISSION] S6_OBSTACLE_TURN_RIGHT\r\n");
      break;
    case MISSION_STATE_S6_OBSTACLE_FACE_SAFE:
      (void)DebugUart_Log("[MISSION] S6_OBSTACLE_FACE_SAFE\r\n");
      break;
    case MISSION_STATE_S6_TRACK_TIMEOUT_REVERSE:
      (void)DebugUart_Log("[MISSION] S6_TRACK_TIMEOUT_REVERSE\r\n");
      break;
    case MISSION_STATE_S6_FINAL_ALIGN:
      (void)DebugUart_Log("[MISSION] S6_FINAL_ALIGN\r\n");
      break;
    case MISSION_STATE_S6_FINAL_VERIFY:
      (void)DebugUart_Log("[MISSION] S6_FINAL_VERIFY\r\n");
      break;
    case MISSION_STATE_S6_RAISE_FRAME:
      (void)DebugUart_Log("[MISSION] S6_RAISE_FRAME\r\n");
      break;
    case MISSION_STATE_S6_PRE_PUSH_REVERSE:
      (void)DebugUart_Log("[MISSION] S6_PRE_PUSH_REVERSE\r\n");
      break;
    case MISSION_STATE_S6_FINAL_PUSH:
      (void)DebugUart_Log("[MISSION] S6_FINAL_PUSH\r\n");
      break;
    case MISSION_STATE_S6_BORDER_LOCATE:
      (void)DebugUart_Log("[MISSION] S6_BORDER_LOCATE\r\n");
      break;
    case MISSION_STATE_S6_BORDER_BACKOFF:
      (void)DebugUart_Log("[MISSION] S6_BORDER_BACKOFF\r\n");
      break;
    case MISSION_STATE_S6_PARTIAL_RAISE_FRAME:
      (void)DebugUart_Log("[MISSION] S6_PARTIAL_RAISE_FRAME\r\n");
      break;
    case MISSION_STATE_S6_FINAL_REVERSE:
      (void)DebugUart_Log("[MISSION] S6_FINAL_REVERSE\r\n");
      break;
    case MISSION_STATE_S6_EXIT_TURN_180:
      (void)DebugUart_Log("[MISSION] S6_EXIT_TURN_180\r\n");
      break;
    case MISSION_STATE_S6_SAFE_ZONE_REACHED:
      (void)DebugUart_Log("[MISSION] S6_SAFE_ZONE_REACHED\r\n");
      break;
    case MISSION_STATE_S7_SEARCH_RED:
      (void)DebugUart_Log("[MISSION] S7_SEARCH_RED\r\n");
      break;
    case MISSION_STATE_S7_WAIT_SECOND_E3:
      (void)DebugUart_Log("[MISSION] S7_WAIT_SECOND_E3\r\n");
      break;
    case MISSION_STATE_S7_SEARCH_BLACK:
      (void)DebugUart_Log("[MISSION] S7_SEARCH_BLACK\r\n");
      break;
    case MISSION_STATE_S7_SILENT_TURN_LEFT:
      (void)DebugUart_Log("[MISSION] S7_SILENT_TURN_LEFT\r\n");
      break;
    case MISSION_STATE_S7_SILENT_TURN_BACK:
      (void)DebugUart_Log("[MISSION] S7_SILENT_TURN_BACK\r\n");
      break;
    case MISSION_STATE_STOPPED:
      (void)DebugUart_Log("[MISSION] STOPPED\r\n");
      break;
    case MISSION_STATE_FAULT:
      (void)DebugUart_Logf("[MISSION] FAULT flags=0x%08lX\r\n",
                           (unsigned long)mission_snapshot.fault_flags);
      break;
    case MISSION_STATE_BOOT:
    default:
      break;
  }

  /* Ordinary state boundaries decelerate; terminal/fault states bypass the
     slew limiter so an unsafe condition never waits for a smooth ramp. */
  if ((state == MISSION_STATE_FAULT) || (state == MISSION_STATE_STOPPED))
  {
    SpeedLoop_EmergencyStop();
    commanded_left_rpm = 0;
    commanded_right_rpm = 0;
    mission_snapshot.left_target_rpm = 0;
    mission_snapshot.right_target_rpm = 0;
  }
  else
  {
    MissionTask_StopWheels();
  }
  MissionTask_DebugAngles(now, true);
}

static bool MissionTask_IsObjectTrackingPhase(void)
{
  switch (mission_snapshot.state)
  {
    case MISSION_STATE_S3_ALIGN_GREEN:
    case MISSION_STATE_S3_TRACK_GREEN:
    case MISSION_STATE_S3_LOWER_FRAME:
    case MISSION_STATE_S3_SEARCH_TURN_CW:
    case MISSION_STATE_S3_SEARCH_TURN_CCW:
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
    case MISSION_STATE_S4_E4_GREEN_SPIN_360:
    case MISSION_STATE_S4_E4_RED_SPIN_360:
    case MISSION_STATE_S7_SEARCH_RED:
    case MISSION_STATE_S7_WAIT_SECOND_E3:
    case MISSION_STATE_S7_SEARCH_BLACK:
    case MISSION_STATE_S7_SILENT_TURN_LEFT:
    case MISSION_STATE_S7_SILENT_TURN_BACK:
    case MISSION_STATE_S6_EXIT_TURN_180:
      return true;
    default:
      return false;
  }
}

static bool MissionTask_RedCoordinateIsFresh(void)
{
  return mission_snapshot.vision_target_valid &&
      mission_snapshot.target_id == 4U &&
      mission_snapshot.target_age_ms <= MISSION_VISION_TARGET_TIMEOUT_MS &&
      !s7_red_switch_sync_pending &&
      (!s7_red_coordinate_fence_active ||
       (int32_t)(mission_snapshot.target_sequence -
                 s7_red_coordinate_sequence_floor) > 0);
}

static void MissionTask_UpdateVisionInput(uint32_t now)
{
  MaixCam_Object object;
  bool object_present = MaixCam_GetObjectSnapshot(&object);
  mission_snapshot.target_present = object_present;
  mission_snapshot.target_age_ms = object.sequence != 0U
      ? (uint32_t)(now - object.update_tick) : UINT32_MAX;
  mission_snapshot.vision_target_valid = object_present &&
      mission_snapshot.target_age_ms <= MISSION_VISION_TARGET_TIMEOUT_MS;
  if (object.sequence != 0U)
  {
    mission_snapshot.target_id = object.object_id;
    mission_snapshot.vision_x_error_px = object.x_error_px;
    mission_snapshot.vision_y_error_px = object.y_error_px;
    mission_snapshot.target_sequence = object.sequence;
  }

  if (MissionTask_IsObjectTrackingPhase())
  {
    if (!s7_target_switch_enabled)
    {
      /* Before the first delivery, target tracking is locked to green.
         Do not apply this filter to 02/12 corner arrangement or zone tracking. */
      mission_snapshot.vision_target_valid = mission_snapshot.vision_target_valid &&
          object.object_id == 5U;
    }
    else if (s3_target_select_command == MAIXCAM_COMMAND_SELECT_RED)
    {
      mission_snapshot.vision_target_valid = MissionTask_RedCoordinateIsFresh();
    }
    else if (s3_target_select_command == MAIXCAM_COMMAND_SELECT_BLACK_GREEN)
    {
      mission_snapshot.vision_target_valid = mission_snapshot.vision_target_valid &&
          ((object.object_id == 5U) || (object.object_id == 6U));
      if (mission_snapshot.vision_target_valid)
      {
        s51_active_object_id = object.object_id;
      }
    }
  }
}

static void MissionTask_UpdateInputs(uint32_t now)
{
  JY901S_Attitude attitude;

  MissionTask_UpdatePe13();
  MissionTask_UpdatePe14();
  MissionTask_UpdatePe15();

  mission_snapshot.left_motor_online =
      MissionTask_MotorIsOnline(MISSION_LEFT_MOTOR_INDEX, now);
  mission_snapshot.right_motor_online =
      MissionTask_MotorIsOnline(MISSION_RIGHT_MOTOR_INDEX, now);

  mission_snapshot.imu_valid = JY901S_GetAttitude(&attitude) &&
      ((uint32_t)(now - attitude.update_tick) <= MISSION_IMU_TIMEOUT_MS);
  if (mission_snapshot.imu_valid)
  {
    mission_snapshot.yaw_cdeg = attitude.yaw_cdeg;
  }

  MissionTask_UpdateVisionInput(now);
}

static uint32_t MissionTask_GetRequiredFaults(void)
{
  uint32_t faults = MISSION_FAULT_NONE;

  if (!mission_snapshot.left_motor_online)
  {
    faults |= MISSION_FAULT_LEFT_MOTOR;
  }
  if (!mission_snapshot.right_motor_online)
  {
    faults |= MISSION_FAULT_RIGHT_MOTOR;
  }
  if (!mission_snapshot.imu_valid)
  {
    faults |= MISSION_FAULT_IMU;
  }

  return faults;
}

static void MissionTask_StartS1(uint32_t now)
{
  s7_target_switch_enabled = false;
  s7_red_recovery_used = false;
  s51_active_object_id = 0U;
  s6_carried_object_id = 0U;
  s5_load_class_confirmed = false;
  s7_red_switch_sync_pending = false;
  s7_red_coordinate_fence_active = false;
  s3_target_select_command = MAIXCAM_COMMAND_SELECT_GREEN;
  s4_post22_search_active = false;
  s4_arrange_cycle_count = 0U;
  int32_t yaw_delta;

  if (!mission_snapshot.team_locked)
  {
    mission_snapshot.team = pe13_stable == GPIO_PIN_SET
                                ? ROBOT_PE13_HIGH_TEAM
                                : ROBOT_PE13_LOW_TEAM;
    mission_snapshot.team_locked = true;
  }

  if (!mission_snapshot.start_zone_locked)
  {
    mission_snapshot.start_pe14_high = pe14_stable == GPIO_PIN_SET;
    mission_snapshot.start_pe15_high = pe15_stable == GPIO_PIN_SET;
    mission_snapshot.start_zone = MissionTask_DecodeStartZone(pe14_stable,
                                                              pe15_stable);
    mission_snapshot.start_zone_locked = true;
  }

  mission_snapshot.team_selector_high = pe13_stable == GPIO_PIN_SET;
  mission_snapshot.yaw_start_cdeg =
      MissionTask_WrapYaw(mission_snapshot.yaw_cdeg);
  mission_snapshot.safe_zone_yaw_target_cdeg = MissionTask_WrapYaw(
      mission_snapshot.yaw_start_cdeg +
      MissionTask_GetSafeZoneYawOffset(mission_snapshot.start_zone,
                                       mission_snapshot.team));

  /* Start areas 1/4 turn left; areas 2/3 turn right. */
  yaw_delta = (int32_t)ROBOT_YAW_LEFT_SIGN * ROBOT_S1_TURN_ANGLE_CDEG;
  if ((mission_snapshot.start_zone == 2U) ||
      (mission_snapshot.start_zone == 3U))
  {
    yaw_delta = -yaw_delta;
  }
  mission_snapshot.yaw_target_cdeg =
      MissionTask_WrapYaw(mission_snapshot.yaw_start_cdeg + yaw_delta);
  mission_snapshot.yaw_error_cdeg = MissionTask_YawError(
      mission_snapshot.yaw_target_cdeg, mission_snapshot.yaw_cdeg);

  s1_turn_stable_since = 0U;
  s1_next_debug_tick = now;
  MissionTask_SetS1Phase(MISSION_S1_TURN, now);
  MissionTask_EnterState(MISSION_STATE_S1_DEPART, now);
  (void)DebugUart_Logf(
      "[S1] START team=%s zone=%u bits=%u%u turn=%s yaw0=%ld target=%ld safe=%ld\r\n",
      mission_snapshot.team == ROBOT_TEAM_RED ? "RED" : "BLUE",
      (unsigned int)mission_snapshot.start_zone,
      mission_snapshot.start_pe14_high ? 1U : 0U,
      mission_snapshot.start_pe15_high ? 1U : 0U,
      ((mission_snapshot.start_zone == 2U) ||
       (mission_snapshot.start_zone == 3U)) ? "RIGHT" : "LEFT",
      (long)mission_snapshot.yaw_start_cdeg,
      (long)mission_snapshot.yaw_target_cdeg,
      (long)mission_snapshot.safe_zone_yaw_target_cdeg);
}

static void MissionTask_DebugS1(uint32_t now)
{
  int16_t left_actual_rpm;
  int16_t right_actual_rpm;

  if ((int32_t)(now - s1_next_debug_tick) < 0)
  {
    return;
  }
  s1_next_debug_tick = now + ROBOT_S1_DEBUG_PERIOD_MS;

  left_actual_rpm = (int16_t)(
      ((float)moto_chassis[MISSION_LEFT_MOTOR_INDEX].speed_rpm / 36.0f) *
      ROBOT_LEFT_WHEEL_FORWARD_SIGN);
  right_actual_rpm = (int16_t)(
      ((float)moto_chassis[MISSION_RIGHT_MOTOR_INDEX].speed_rpm / 36.0f) *
      ROBOT_RIGHT_WHEEL_FORWARD_SIGN);

  (void)DebugUart_Logf(
      "[S1] p=%u yaw=%ld err=%ld cmd=%d/%d act=%d/%d\r\n",
      (unsigned int)mission_snapshot.s1_phase,
      (long)MissionTask_WrapYaw(mission_snapshot.yaw_cdeg),
      (long)mission_snapshot.yaw_error_cdeg,
      (int)mission_snapshot.left_target_rpm,
      (int)mission_snapshot.right_target_rpm,
      (int)left_actual_rpm, (int)right_actual_rpm);
}

static bool MissionTask_BeginS3Vision(uint32_t now)
{
  s4_e4_protocol_active = false;
  if (s3_target_select_command == MAIXCAM_COMMAND_SELECT_BLACK_GREEN)
  {
    s51_active_object_id = 0U;
  }
  const char *target_name =
      s3_target_select_command == MAIXCAM_COMMAND_SELECT_RED
          ? "RED"
          : (s3_target_select_command == MAIXCAM_COMMAND_SELECT_BLACK
                 ? "BLACK"
                 : (s3_target_select_command == MAIXCAM_COMMAND_SELECT_BLACK_GREEN
                        ? "BLACK/GREEN" : "GREEN"));

  MissionTask_EnterState(MISSION_STATE_S3_TRACK_GREEN, now);

  /* A coordinate received before command 03 belongs to the previous camera
     mode and must never feed the new PID loop. */
  MaixCam_ClearObject();
  MaixCam_ClearEvent();
  mission_snapshot.vision_target_valid = false;
  MissionTask_ResetVisionPid();
  s3_track_stable_since = 0U;
  s3_search_turn_stable_since = 0U;
  s3_search_origin_yaw_cdeg =
      MissionTask_WrapYaw(mission_snapshot.yaw_cdeg);
  s3_search_yaw_target_cdeg = s3_search_origin_yaw_cdeg;
  s3_search_sweep_completed = false;
  s3_search_ccw_midpoint_done = false;
  vision_search_forward_active = false;
  vision_search_resume_state = MISSION_STATE_S3_TRACK_GREEN;
  s3_target_lost_since = now;
  s3_wait_event04_logged = false;
  s3_next_debug_tick = now;

  if (MaixCam_SendCommand(s3_target_select_command) != HAL_OK)
  {
    mission_snapshot.fault_flags |= MISSION_FAULT_VISION_TX;
    MissionTask_EnterState(MISSION_STATE_FAULT, now);
    return false;
  }
  (void)DebugUart_Logf(
      "[S3] TX E1 E2 %02X 1E 2E, SELECT %s\r\n",
      (unsigned int)s3_target_select_command,
      target_name);

  if (MaixCam_SendCommand(MAIXCAM_COMMAND_SEARCH_TARGET) != HAL_OK)
  {
    mission_snapshot.fault_flags |= MISSION_FAULT_VISION_TX;
    MissionTask_EnterState(MISSION_STATE_FAULT, now);
    return false;
  }
  (void)DebugUart_Log("[S3] TX E1 E2 03 1E 2E, START SEARCH\r\n");
  if (s7_red_switch_sync_pending)
  {
    MaixCam_Object last_receipt;
    (void)MaixCam_GetObjectSnapshot(&last_receipt);
    s7_red_coordinate_sequence_floor = last_receipt.sequence;
    s7_red_coordinate_fence_active = true;
    /* Discard coordinates/events received before the 11/03 handshake ends.
       A subsequent ID4 frame must be generated with the new 03 reference. */
    MaixCam_ClearObject();
    MaixCam_ClearEvent();
    mission_snapshot.vision_target_valid = false;
    s7_red_switch_sync_pending = false;
    (void)DebugUart_Logf(
        "[S7-PRIORITY] TX11/03 COMPLETE, WAIT FRESH ID4 AFTER seq=%lu, REF=IMAGE_CENTER\r\n",
        (unsigned long)s7_red_coordinate_sequence_floor);
  }
  return true;
}

static bool MissionTask_StartS3Align(uint32_t now, uint8_t select_command)
{
  if (!s7_target_switch_enabled &&
      select_command != MAIXCAM_COMMAND_SELECT_GREEN)
  {
    (void)DebugUart_Logf(
        "[S3] TARGET CHANGE %02X BLOCKED BEFORE07, KEEP GREEN21\r\n",
        (unsigned int)select_command);
    select_command = MAIXCAM_COMMAND_SELECT_GREEN;
  }
  if ((select_command == MAIXCAM_COMMAND_SELECT_RED) &&
      (s3_target_select_command != MAIXCAM_COMMAND_SELECT_RED))
  {
    /* A newly selected red target gets its own first E4 search. */
    s4_e4_recovery_used = false;
    s7_red_recovery_used = false;
    s7_red_coordinate_fence_active = false;
  }
  s3_target_select_command = select_command;
  if (select_command == MAIXCAM_COMMAND_SELECT_GREEN)
  {
    s4_green_e4_push_cycles = 0U;
    s4_green_e4_spin_exhausted = false;
  }
  s4_final_mode_already_active = false;
  s4_final_event34_pending = false;
  s4_final_missing_since = 0U;
  s4_final_turn_stable_since = 0U;
  s4_final_recovery_count = 0U;
  if ((Actuator_SetFrameLowered() != HAL_OK) ||
      (Actuator_SetCameraWideView() != HAL_OK))
  {
    mission_snapshot.fault_flags |= MISSION_FAULT_ACTUATOR;
    MissionTask_EnterState(MISSION_STATE_FAULT, now);
    return false;
  }

  MissionTask_EnterState(MISSION_STATE_S3_LOWER_FRAME, now);
  (void)DebugUart_Logf(
      "[S3] FRAME DOWN left=%u right=%u settle=%lums, THEN START 03\r\n",
      (unsigned int)ROBOT_LEFT_FRAME_DOWN_DEG,
      (unsigned int)ROBOT_RIGHT_FRAME_DOWN_DEG,
      (unsigned long)ROBOT_S3_FRAME_LOWER_SETTLE_MS);
  return true;
}

static void MissionTask_StartS2CrossBump(uint32_t now)
{
  MissionTask_SetS1Phase(MISSION_S1_DONE, now);
  s2_drive_finished = false;
  s2_next_debug_tick = now;
  MissionTask_EnterState(MISSION_STATE_S2_CROSS_BUMP, now);
  (void)DebugUart_Logf(
      "[S2] FORWARD rpm=%d drive=%lums settle=%lums yaw_target=%ld\r\n",
                       ROBOT_S2_CROSS_BUMP_RPM,
                       (unsigned long)ROBOT_S2_CROSS_BUMP_DRIVE_MS,
                       (unsigned long)ROBOT_S2_CROSS_BUMP_SETTLE_MS,
                       (long)mission_snapshot.yaw_target_cdeg);
}

static void MissionTask_RunS1(uint32_t now)
{
  int32_t absolute_error;
  int16_t turn_rpm;
  bool turn_left;

  if (mission_snapshot.state != MISSION_STATE_S1_DEPART)
  {
    return;
  }

  mission_snapshot.yaw_error_cdeg = MissionTask_YawError(
      mission_snapshot.yaw_target_cdeg, mission_snapshot.yaw_cdeg);

  if ((mission_snapshot.s1_phase == MISSION_S1_TURN) ||
      (mission_snapshot.s1_phase == MISSION_S1_TURN_STABLE))
  {
    if ((uint32_t)(now - mission_snapshot.state_entry_tick) >=
        ROBOT_S1_TURN_TIMEOUT_MS)
    {
      mission_snapshot.fault_flags |= MISSION_FAULT_S1_TIMEOUT;
      MissionTask_EnterState(MISSION_STATE_FAULT, now);
      return;
    }

    absolute_error = MissionTask_Abs32(mission_snapshot.yaw_error_cdeg);
    if (absolute_error > (ROBOT_S1_TURN_ANGLE_CDEG +
                          ROBOT_S1_WRONG_DIRECTION_MARGIN_CDEG))
    {
      mission_snapshot.fault_flags |= MISSION_FAULT_YAW_DIRECTION;
      MissionTask_EnterState(MISSION_STATE_FAULT, now);
      return;
    }

    if (absolute_error <= ROBOT_S1_TURN_TOLERANCE_CDEG)
    {
      MissionTask_StopWheels();
      if (mission_snapshot.s1_phase != MISSION_S1_TURN_STABLE)
      {
        s1_turn_stable_since = now;
        MissionTask_SetS1Phase(MISSION_S1_TURN_STABLE, now);
        (void)DebugUart_Log("[S1] TURN_IN_TOLERANCE\r\n");
      }

      if ((uint32_t)(now - s1_turn_stable_since) >=
          ROBOT_S1_TURN_STABLE_MS)
      {
        MissionTask_StartS2CrossBump(now);
        return;
      }
    }
    else
    {
      if (mission_snapshot.s1_phase == MISSION_S1_TURN_STABLE)
      {
        MissionTask_SetS1Phase(MISSION_S1_TURN, now);
      }
      s1_turn_stable_since = 0U;
      turn_rpm = absolute_error > ROBOT_S1_TURN_SLOW_THRESHOLD_CDEG
                     ? ROBOT_S1_TURN_FAST_RPM
                     : ROBOT_S1_TURN_SLOW_RPM;

      /* ROBOT_YAW_LEFT_SIGN converts yaw-error sign into chassis direction. */
      turn_left = (mission_snapshot.yaw_error_cdeg *
                   (int32_t)ROBOT_YAW_LEFT_SIGN) > 0L;
      MissionTask_SetWheelTargets(turn_left ? -turn_rpm : turn_rpm,
                                  turn_left ? turn_rpm : -turn_rpm);
    }
  }
  MissionTask_DebugS1(now);
}

static void MissionTask_DebugS2(uint32_t now, int16_t correction_rpm)
{
  int16_t left_actual_rpm;
  int16_t right_actual_rpm;

  if ((int32_t)(now - s2_next_debug_tick) < 0)
  {
    return;
  }
  s2_next_debug_tick = now + ROBOT_S2_DEBUG_PERIOD_MS;

  left_actual_rpm = (int16_t)(
      ((float)moto_chassis[MISSION_LEFT_MOTOR_INDEX].speed_rpm / 36.0f) *
      ROBOT_LEFT_WHEEL_FORWARD_SIGN);
  right_actual_rpm = (int16_t)(
      ((float)moto_chassis[MISSION_RIGHT_MOTOR_INDEX].speed_rpm / 36.0f) *
      ROBOT_RIGHT_WHEEL_FORWARD_SIGN);

  (void)DebugUart_Logf(
      "[S2-Y] yaw=%ld target=%ld err=%ld corr=%d cmd=%d/%d act=%d/%d\r\n",
      (long)MissionTask_WrapYaw(mission_snapshot.yaw_cdeg),
      (long)mission_snapshot.yaw_target_cdeg,
      (long)mission_snapshot.yaw_error_cdeg,
      (int)correction_rpm,
      (int)mission_snapshot.left_target_rpm,
      (int)mission_snapshot.right_target_rpm,
      (int)left_actual_rpm,
      (int)right_actual_rpm);
}

static int16_t MissionTask_CalculateS2YawCorrection(void)
{
  float correction;

  mission_snapshot.yaw_error_cdeg = MissionTask_YawError(
      mission_snapshot.yaw_target_cdeg, mission_snapshot.yaw_cdeg);
  if (MissionTask_Abs32(mission_snapshot.yaw_error_cdeg) <=
      ROBOT_S2_YAW_HOLD_TOLERANCE_CDEG)
  {
    return 0;
  }

  /* Positive correction means the chassis must turn left. */
  correction = (float)mission_snapshot.yaw_error_cdeg *
               (float)ROBOT_YAW_LEFT_SIGN * ROBOT_S2_YAW_HOLD_KP;
  correction = MissionTask_ClampFloat(
      correction,
      -ROBOT_S2_YAW_HOLD_MAX_CORRECTION_RPM,
      ROBOT_S2_YAW_HOLD_MAX_CORRECTION_RPM);
  return (int16_t)(correction >= 0.0f
                       ? correction + 0.5f
                       : correction - 0.5f);
}

static void MissionTask_RunS2CrossBump(uint32_t now)
{
  uint32_t elapsed;
  int16_t correction_rpm;
  int16_t left_rpm;
  int16_t right_rpm;

  if (mission_snapshot.state != MISSION_STATE_S2_CROSS_BUMP)
  {
    return;
  }

  elapsed = (uint32_t)(now - mission_snapshot.state_entry_tick);
  if (elapsed < ROBOT_S2_CROSS_BUMP_DRIVE_MS)
  {
    correction_rpm = MissionTask_CalculateS2YawCorrection();
    left_rpm = ROBOT_S2_CROSS_BUMP_RPM;
    right_rpm = ROBOT_S2_CROSS_BUMP_RPM;

    /* Keep one wheel at the requested crossing speed and only slow the inner
       wheel, so yaw correction never raises either wheel above the configured
       crossing rpm. */
    if (correction_rpm > 0)
    {
      left_rpm = (int16_t)(left_rpm - correction_rpm);
    }
    else if (correction_rpm < 0)
    {
      right_rpm = (int16_t)(right_rpm + correction_rpm);
    }

    MissionTask_SetWheelTargets(left_rpm, right_rpm);
    MissionTask_DebugS2(now, correction_rpm);
    return;
  }

  if (!s2_drive_finished)
  {
    s2_drive_finished = true;
    MissionTask_StopWheels();
    (void)DebugUart_Log("[S2] BUMP_CROSSED, SETTLING\r\n");
  }

  if (elapsed >= (ROBOT_S2_CROSS_BUMP_DRIVE_MS +
                  ROBOT_S2_CROSS_BUMP_SETTLE_MS))
  {
    (void)DebugUart_Log("[S2] DONE, ENTER S3\r\n");
    (void)MissionTask_StartS3Align(now, MAIXCAM_COMMAND_SELECT_GREEN);
  }
}

static int16_t MissionTask_CalculateVisionTurn(uint32_t now,
                                               int16_t error_px)
{
  float error = (float)error_px;
  bool final_track =
      mission_snapshot.state == MISSION_STATE_S4_TRACK_FINAL_BLOCK;
  float kp = final_track ? ROBOT_S4_FINAL_X_KP : ROBOT_VISION_X_KP;
  float min_turn_rpm = final_track ? ROBOT_S4_FINAL_MIN_TURN_RPM :
                                    ROBOT_VISION_X_MIN_TURN_RPM;
  float dt_seconds;
  float derivative;
  float magnitude;
  float signed_output;

  if (mission_snapshot.target_sequence != s3_last_target_sequence)
  {
    if (!s3_pid_initialized)
    {
      dt_seconds = (float)MISSION_TASK_PERIOD_TICKS / 1000.0f;
      derivative = 0.0f;
      s3_pid_initialized = true;
    }
    else
    {
      dt_seconds = (float)(now - s3_pid_update_tick) / 1000.0f;
      dt_seconds = MissionTask_ClampFloat(dt_seconds, 0.01f, 0.20f);
      derivative = (error - s3_pid_last_error) / dt_seconds;
    }

    s3_pid_integral += error * dt_seconds;
    s3_pid_integral = MissionTask_ClampFloat(
        s3_pid_integral,
        -ROBOT_VISION_X_INTEGRAL_LIMIT,
        ROBOT_VISION_X_INTEGRAL_LIMIT);
    s3_pid_output = kp * error +
                    ROBOT_VISION_X_KI * s3_pid_integral +
                    ROBOT_VISION_X_KD * derivative;
    s3_pid_output = MissionTask_ClampFloat(
        s3_pid_output,
        -ROBOT_VISION_X_MAX_TURN_RPM,
        ROBOT_VISION_X_MAX_TURN_RPM);

    s3_pid_last_error = error;
    s3_pid_update_tick = now;
    s3_last_target_sequence = mission_snapshot.target_sequence;
  }

  signed_output = s3_pid_output * (float)ROBOT_VISION_X_TURN_SIGN;
  magnitude = signed_output < 0.0f ? -signed_output : signed_output;
  if ((magnitude > 0.0f) && (magnitude < min_turn_rpm))
  {
    signed_output = signed_output < 0.0f
                        ? -min_turn_rpm
                        : min_turn_rpm;
  }

  return (int16_t)(signed_output >= 0.0f
                       ? signed_output + 0.5f
                       : signed_output - 0.5f);
}

static void MissionTask_DebugS3(uint32_t now)
{
  int16_t left_actual_rpm;
  int16_t right_actual_rpm;

  if ((int32_t)(now - s3_next_debug_tick) < 0)
  {
    return;
  }
  s3_next_debug_tick = now + ROBOT_VISION_DEBUG_PERIOD_MS;

  left_actual_rpm = (int16_t)(
      ((float)moto_chassis[MISSION_LEFT_MOTOR_INDEX].speed_rpm / 36.0f) *
      ROBOT_LEFT_WHEEL_FORWARD_SIGN);
  right_actual_rpm = (int16_t)(
      ((float)moto_chassis[MISSION_RIGHT_MOTOR_INDEX].speed_rpm / 36.0f) *
      ROBOT_RIGHT_WHEEL_FORWARD_SIGN);

  if (!mission_snapshot.vision_target_valid)
  {
    (void)DebugUart_Logf("[S3] state=%u target=LOST cmd=0/0\r\n",
                         (unsigned int)mission_snapshot.state);
    return;
  }

  (void)DebugUart_Logf(
      "[S3-T] id=%u xerr=%d yerr=%d turn=%d fwd=%d cmd=%d/%d act=%d/%d\r\n",
      (unsigned int)mission_snapshot.target_id,
      (int)mission_snapshot.vision_x_error_px,
      (int)mission_snapshot.vision_y_error_px,
      (int)mission_snapshot.vision_turn_rpm,
      (int)mission_snapshot.vision_forward_rpm,
      (int)mission_snapshot.left_target_rpm,
      (int)mission_snapshot.right_target_rpm,
      (int)left_actual_rpm,
      (int)right_actual_rpm);
}

static bool MissionTask_ReturnToS4TrackCenter(uint32_t now)
{
  if (mission_snapshot.state == MISSION_STATE_S4_TRACK_CENTER)
  {
    return true;
  }

  /* Synchronize once per recovery return. Keep reacquired coordinates and
     enter 04 before TX so a quick camera reply is preserved. */
  MissionTask_EnterState(MISSION_STATE_S4_TRACK_CENTER, now);
  if (MaixCam_SendCommand(MAIXCAM_COMMAND_TRACK_CENTER_ACK) != HAL_OK)
  {
    mission_snapshot.fault_flags |= MISSION_FAULT_VISION_TX;
    MissionTask_EnterState(MISSION_STATE_FAULT, now);
    (void)DebugUart_Log("[S4] RECOVERY RETURN TX04 FAILED\r\n");
    return false;
  }
  (void)DebugUart_Log(
      "[S4] RECOVERY RETURN04, TX E1 E2 04 1E 2E, SYNC CAMERA\r\n");
  return true;
}

static void MissionTask_StartS4TrackCenterFrom04(uint32_t now)
{
  MissionTask_ResetVisionPid();
  MaixCam_ClearObject();
  mission_snapshot.vision_target_valid = false;
  if (Actuator_SetCameraWideView() != HAL_OK)
  {
    mission_snapshot.fault_flags |= MISSION_FAULT_ACTUATOR;
    MissionTask_EnterState(MISSION_STATE_FAULT, now);
    return;
  }

  s4_next_debug_tick = now;
  s4_center_target_lost_since = now;
  s4_center_search_sweep_completed = false;
  MissionTask_EnterState(MISSION_STATE_S4_TRACK_CENTER, now);
  if (MaixCam_SendCommand(MAIXCAM_COMMAND_TRACK_CENTER_ACK) != HAL_OK)
  {
    mission_snapshot.fault_flags |= MISSION_FAULT_VISION_TX;
    MissionTask_EnterState(MISSION_STATE_FAULT, now);
    return;
  }
  (void)DebugUart_Log("[S4] TX E1 E2 04 1E 2E, ACK RX04\r\n");
  (void)DebugUart_Logf(
      "[S4] RX04 FRAME ALREADY DOWN, CAMERA WIDE=%u, TRACK CENTER\r\n",
      (unsigned int)ROBOT_CAMERA_WIDE_ANGLE_DEG);
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
    /* E3 is repeated while the camera has no target. If a coordinate became
       fresh in the same control cycle, it is newer evidence than a queued E3
       and must be allowed to resume the vision controller. */
    if (mission_snapshot.vision_target_valid)
    {
      (void)DebugUart_Log(
          "[S3] RX E3 WITH FRESH COORDINATES, KEEP TARGET\r\n");
      return false;
    }

    MaixCam_ClearObject();
    mission_snapshot.vision_target_valid = false;
    s3_target_lost_since = now;
    s3_search_sweep_completed = false;

    if ((mission_snapshot.state == MISSION_STATE_S3_SEARCH_TURN_CW) ||
        (mission_snapshot.state == MISSION_STATE_S3_SEARCH_TURN_CCW))
    {
      (void)DebugUart_Log(
          "[S3-S] RX F1 F2 E3 1F 2F, RECOVERY ALREADY ACTIVE\r\n");
      return true;
    }

    (void)DebugUart_Log(
        "[S3] RX F1 F2 E3 1F 2F, START SEARCH RECOVERY\r\n");
    if (s3_target_select_command == MAIXCAM_COMMAND_SELECT_RED)
    {
      /* A transient red coordinate may have moved 07 into 03. Preserve the
         red -> recovery -> 13 -> second E3 -> black decision in that case. */
      (void)DebugUart_Log(
          "[S7] RED LOST IN 03, RECOVER ONCE THEN TX13, NEXT E3 SELECT51 SEARCH03\r\n");
      MissionTask_StartVisionSearchRecovery(
          now, MISSION_STATE_S7_SEARCH_RED, mission_snapshot.yaw_cdeg, false);
      return true;
    }
    MissionTask_StartVisionSearchRecovery(
        now, MISSION_STATE_S3_TRACK_GREEN, s3_search_origin_yaw_cdeg,
        s3_target_select_command == MAIXCAM_COMMAND_SELECT_GREEN);
    return true;
  }

  if (event_code != MAIXCAM_EVENT_OBJECT_IN_FRAME)
  {
    return false;
  }

  (void)DebugUart_Log("[S3] RX F1 F2 04 1F 2F, OBJECT IN FRAME\r\n");
  MissionTask_StartS4TrackCenterFrom04(now);
  return true;
}

static void MissionTask_StartVisionSearchRecovery(
    uint32_t now, MissionState resume_state, int32_t origin_yaw_cdeg,
    bool initial_green_e3)
{
  const char *source = resume_state == MISSION_STATE_S4_TRACK_CENTER
                           ? "04"
                           : (resume_state == MISSION_STATE_S7_SEARCH_RED
                                  ? "07"
                                  : "03");

  if (MissionTask_ActiveRecoveryLossEvent() != 0U) return;

  if (s7_target_switch_enabled &&
      resume_state == MISSION_STATE_S7_SEARCH_RED)
  {
    if (s7_red_recovery_used)
    {
      (void)DebugUart_Log("[S7] RED LOST AGAIN AFTER RECOVERY, SELECT51/03\r\n");
      MissionTask_SwitchS7ToBlackGreen(now);
      return;
    }
    s7_red_recovery_used = true;
    (void)DebugUart_Log("[S7] RED RECOVERY USED=1, RETAIN ACROSS REACQUISITION\r\n");
  }

  MissionTask_StopWheels();
  MissionTask_ResetVisionControllerState();
  vision_search_resume_state = resume_state;
  vision_search_forward_active = true;
  vision_search_green_e3_mode = initial_green_e3 &&
      resume_state == MISSION_STATE_S3_TRACK_GREEN &&
      s3_target_select_command == MAIXCAM_COMMAND_SELECT_GREEN;
  vision_search_e3_mode = resume_state != MISSION_STATE_S4_TRACK_CENTER &&
      !vision_search_green_e3_mode;
  /* Keep legacy 04 coordinate-timeout recovery separate from E3 searches. */
  vision_search_forward_rpm = resume_state == MISSION_STATE_S4_TRACK_CENTER
      ? ROBOT_VISION_SEARCH_FORWARD_RPM : ROBOT_E3_SEARCH_REVERSE_RPM;
  vision_search_forward_ms = resume_state == MISSION_STATE_S4_TRACK_CENTER
      ? ROBOT_VISION_SEARCH_FORWARD_MS : ROBOT_E3_SEARCH_REVERSE_MS;
  vision_search_turn_angle_cdeg = ROBOT_S3_SEARCH_TURN_ANGLE_CDEG;
  s3_search_origin_yaw_cdeg = MissionTask_WrapYaw(origin_yaw_cdeg);
  s3_search_yaw_target_cdeg = MissionTask_WrapYaw(
      s3_search_origin_yaw_cdeg -
      (int32_t)ROBOT_YAW_LEFT_SIGN * vision_search_turn_angle_cdeg);
  s3_search_turn_stable_since = 0U;
  s3_search_ccw_midpoint_done = false;
  s3_next_debug_tick = now;
  MissionTask_EnterState(MISSION_STATE_S3_SEARCH_TURN_CW, now);
  if (vision_search_green_e3_mode)
  {
    s3_green_e3_phase = S3_GREEN_E3_FORWARD;
    s3_green_e3_origin_yaw_cdeg = MissionTask_WrapYaw(mission_snapshot.yaw_cdeg);
    s3_search_yaw_target_cdeg = s3_green_e3_origin_yaw_cdeg;
    vision_search_forward_rpm = ROBOT_S3_GREEN_E3_FORWARD_RPM;
    vision_search_forward_ms = ROBOT_S3_GREEN_E3_FORWARD_MS;
    MissionTask_ResetS6ControllerState();
    (void)DebugUart_Logf(
        "[S3-GE3] FORWARD rpm=%d time=%lums, LEFT45 RIGHT90 LEFT45 origin=%ld\r\n",
        ROBOT_S3_GREEN_E3_FORWARD_RPM,
        (unsigned long)ROBOT_S3_GREEN_E3_FORWARD_MS,
        (long)s3_green_e3_origin_yaw_cdeg);
    return;
  }
  if (vision_search_e3_mode)
  {
    e3_spin_progress_cdeg = 0L;
    s3_search_yaw_target_cdeg = MissionTask_WrapYaw(mission_snapshot.yaw_cdeg);
    (void)DebugUart_Logf(
        "[E3-R] source=%s REVERSE rpm=%d time=%lums THEN CCW360 (ACCUMULATED YAW)\r\n",
        source, ROBOT_E3_SEARCH_REVERSE_RPM,
        (unsigned long)ROBOT_E3_SEARCH_REVERSE_MS);
    return;
  }
  (void)DebugUart_Logf(
      "[V-SEARCH] source=%s FORWARD rpm=%d time=%lums, then CW=%lddeg "
      "origin=%ld target=%ld\r\n",
      source,
      vision_search_forward_rpm,
      (unsigned long)vision_search_forward_ms,
      (long)(vision_search_turn_angle_cdeg / 100L),
      (long)s3_search_origin_yaw_cdeg,
      (long)s3_search_yaw_target_cdeg);
}

static void MissionTask_RunS3GreenE3Recovery(uint32_t now)
{
  uint32_t elapsed = (uint32_t)(now - mission_snapshot.state_entry_tick);
  int32_t target_yaw_cdeg;
  int32_t yaw_error_cdeg;
  int16_t turn_rpm;

  if (s3_green_e3_phase == S3_GREEN_E3_FORWARD)
  {
    if (elapsed < ROBOT_S3_GREEN_E3_FORWARD_MS)
    {
      mission_snapshot.vision_forward_rpm = ROBOT_S3_GREEN_E3_FORWARD_RPM;
      mission_snapshot.vision_turn_rpm = 0;
      MissionTask_SetWheelTargets(ROBOT_S3_GREEN_E3_FORWARD_RPM,
                                  ROBOT_S3_GREEN_E3_FORWARD_RPM);
      return;
    }
    MissionTask_StopWheels();
    MissionTask_ResetS6ControllerState();
    vision_search_forward_active = false;
    s3_green_e3_phase = S3_GREEN_E3_TURN_LEFT;
    s3_search_yaw_target_cdeg = MissionTask_WrapYaw(
        s3_green_e3_origin_yaw_cdeg +
        (int32_t)ROBOT_YAW_LEFT_SIGN * ROBOT_S3_GREEN_E3_TURN_CDEG);
    mission_snapshot.state_entry_tick = now;
    s3_search_turn_stable_since = 0U;
    (void)DebugUart_Logf("[S3-GE3] FORWARD DONE, LEFT45 target=%ld\r\n",
                         (long)s3_search_yaw_target_cdeg);
    return;
  }

  if (elapsed >= ROBOT_S3_SEARCH_TURN_TIMEOUT_MS)
  {
    mission_snapshot.fault_flags |= MISSION_FAULT_S3_TIMEOUT;
    MissionTask_EnterState(MISSION_STATE_FAULT, now);
    (void)DebugUart_Logf("[S3-GE3] TURN TIMEOUT phase=%u yaw=%ld\r\n",
                         (unsigned int)s3_green_e3_phase,
                         (long)MissionTask_WrapYaw(mission_snapshot.yaw_cdeg));
    return;
  }

  if (s3_green_e3_phase == S3_GREEN_E3_TURN_LEFT)
  {
    target_yaw_cdeg = MissionTask_WrapYaw(
        s3_green_e3_origin_yaw_cdeg +
        (int32_t)ROBOT_YAW_LEFT_SIGN * ROBOT_S3_GREEN_E3_TURN_CDEG);
  }
  else if (s3_green_e3_phase == S3_GREEN_E3_TURN_RIGHT)
  {
    target_yaw_cdeg = MissionTask_WrapYaw(
        s3_green_e3_origin_yaw_cdeg -
        (int32_t)ROBOT_YAW_LEFT_SIGN * ROBOT_S3_GREEN_E3_TURN_CDEG);
  }
  else
  {
    target_yaw_cdeg = s3_green_e3_origin_yaw_cdeg;
  }

  s3_search_yaw_target_cdeg = target_yaw_cdeg;
  turn_rpm = MissionTask_CalculateS6YawTurn(now, target_yaw_cdeg);
  yaw_error_cdeg = mission_snapshot.safe_zone_yaw_error_cdeg;
  mission_snapshot.yaw_error_cdeg = yaw_error_cdeg;
  if (MissionTask_Abs32(yaw_error_cdeg) <=
      ROBOT_S3_SEARCH_TURN_TOLERANCE_CDEG)
  {
    MissionTask_StopWheels();
    if (s3_search_turn_stable_since == 0U)
    {
      s3_search_turn_stable_since = now;
    }
    else if ((uint32_t)(now - s3_search_turn_stable_since) >=
             ROBOT_S3_SEARCH_TURN_STABLE_MS)
    {
      MissionTask_ResetS6ControllerState();
      s3_search_turn_stable_since = 0U;
      if (s3_green_e3_phase == S3_GREEN_E3_TURN_BACK)
      {
        vision_search_green_e3_mode = false;
        s3_search_sweep_completed = true;
        s3_target_lost_since = now;
        MissionTask_EnterState(MISSION_STATE_S3_TRACK_GREEN, now);
        (void)DebugUart_Log("[S3-GE3] LEFT45 DONE, WAIT GREEN COORDINATES\r\n");
        return;
      }
      if (s3_green_e3_phase == S3_GREEN_E3_TURN_LEFT)
      {
        s3_green_e3_phase = S3_GREEN_E3_TURN_RIGHT;
        s3_search_yaw_target_cdeg = MissionTask_WrapYaw(
            s3_green_e3_origin_yaw_cdeg -
            (int32_t)ROBOT_YAW_LEFT_SIGN * ROBOT_S3_GREEN_E3_TURN_CDEG);
        (void)DebugUart_Logf("[S3-GE3] LEFT45 DONE, RIGHT90 target=%ld\r\n",
                             (long)s3_search_yaw_target_cdeg);
      }
      else
      {
        s3_green_e3_phase = S3_GREEN_E3_TURN_BACK;
        s3_search_yaw_target_cdeg = s3_green_e3_origin_yaw_cdeg;
        (void)DebugUart_Logf("[S3-GE3] RIGHT90 DONE, LEFT45 target=%ld\r\n",
                             (long)s3_search_yaw_target_cdeg);
      }
      mission_snapshot.state_entry_tick = now;
    }
  }
  else
  {
    s3_search_turn_stable_since = 0U;
    mission_snapshot.vision_forward_rpm = 0;
    mission_snapshot.vision_turn_rpm = turn_rpm;
    MissionTask_SetWheelTargets(turn_rpm, (int16_t)-turn_rpm);
  }
}

static void MissionTask_RunS3SearchTurn(uint32_t now)
{
  int32_t yaw_error;
  int32_t absolute_error;
  int16_t turn_rpm;
  bool turn_left;
  bool clockwise_phase;

  clockwise_phase =
      mission_snapshot.state == MISSION_STATE_S3_SEARCH_TURN_CW;
  if (!clockwise_phase &&
      (mission_snapshot.state != MISSION_STATE_S3_SEARCH_TURN_CCW))
  {
    return;
  }

  if ((vision_search_resume_state == MISSION_STATE_S3_TRACK_GREEN) &&
      MissionTask_TryStartS3Capture(now))
  {
    return;
  }

  /* Do not finish a blind search leg after vision has found a target. */
  if (mission_snapshot.vision_target_valid &&
      ((vision_search_resume_state != MISSION_STATE_S7_SEARCH_RED) ||
       (mission_snapshot.target_id == 4U)))
  {
    vision_search_forward_active = false;
    MissionTask_ResetVisionControllerState();
    if (vision_search_green_e3_mode)
    {
      vision_search_green_e3_mode = false;
      MissionTask_ResetS6ControllerState();
    }
    if (vision_search_resume_state == MISSION_STATE_S4_TRACK_CENTER)
    {
      s4_center_search_sweep_completed = false;
      s4_center_target_lost_since = 0U;
      s4_next_debug_tick = now;
      if (!MissionTask_ReturnToS4TrackCenter(now))
      {
        return;
      }
    }
    else if (vision_search_resume_state == MISSION_STATE_S7_SEARCH_RED)
    {
      (void)DebugUart_Log(
          "[S7] RED COORDINATES FOUND DURING E3 RECOVERY, STOP AND RESUME RED CAPTURE\r\n");
      MissionTask_ResumeRedTracking(now);
    }
    else
    {
      /* Arm one new sweep if this reacquired target is later lost again. */
      s3_search_sweep_completed = false;
      s3_target_lost_since = 0U;
      s3_track_stable_since = 0U;
      MissionTask_EnterState(MISSION_STATE_S3_TRACK_GREEN, now);
    }
    (void)DebugUart_Logf("[V-SEARCH] source=%s TARGET FOUND x=%d y=%d, "
                         "RESUME PID\r\n",
                         vision_search_resume_state ==
                                 MISSION_STATE_S4_TRACK_CENTER
                             ? "04"
                             : (vision_search_resume_state ==
                                        MISSION_STATE_S7_SEARCH_RED
                                    ? "07"
                                    : "03"),
                         (int)mission_snapshot.vision_x_error_px,
                         (int)mission_snapshot.vision_y_error_px);
    return;
  }

  if (vision_search_green_e3_mode)
  {
    MissionTask_RunS3GreenE3Recovery(now);
    return;
  }

  if (vision_search_e3_mode)
  {
    if (vision_search_forward_active)
    {
      if ((uint32_t)(now - mission_snapshot.state_entry_tick) <
          ROBOT_E3_SEARCH_REVERSE_MS)
      {
        mission_snapshot.vision_forward_rpm = -ROBOT_E3_SEARCH_REVERSE_RPM;
        mission_snapshot.vision_turn_rpm = 0;
        MissionTask_SetWheelTargets(-ROBOT_E3_SEARCH_REVERSE_RPM,
                                    -ROBOT_E3_SEARCH_REVERSE_RPM);
        return;
      }
      MissionTask_StopWheels();
      vision_search_forward_active = false;
      e3_spin_previous_yaw_cdeg = MissionTask_WrapYaw(mission_snapshot.yaw_cdeg);
      e3_spin_progress_cdeg = 0L;
      s3_search_yaw_target_cdeg = e3_spin_previous_yaw_cdeg;
      MissionTask_EnterState(MISSION_STATE_S3_SEARCH_TURN_CCW, now);
      (void)DebugUart_Log("[E3-R] REVERSE DONE, START CCW360\r\n");
      return;
    }
    e3_spin_progress_cdeg += (int32_t)ROBOT_YAW_LEFT_SIGN *
        MissionTask_YawError(mission_snapshot.yaw_cdeg, e3_spin_previous_yaw_cdeg);
    e3_spin_previous_yaw_cdeg = MissionTask_WrapYaw(mission_snapshot.yaw_cdeg);
    if (e3_spin_progress_cdeg < 0L)
    {
      e3_spin_progress_cdeg = 0L;
    }
    if (e3_spin_progress_cdeg >= ROBOT_E3_SEARCH_CCW_CDEG)
    {
      MissionTask_StopWheels();
      MissionTask_ResetVisionControllerState();
      if (vision_search_resume_state == MISSION_STATE_S7_SEARCH_RED)
      {
        MissionTask_CompleteS7RedRecovery(now);
      }
      else
      {
        s3_search_sweep_completed = true;
        s3_target_lost_since = now;
        MissionTask_EnterState(MISSION_STATE_S3_TRACK_GREEN, now);
      }
      (void)DebugUart_Logf("[E3-R] CCW360 DONE progress=%ld cdeg\r\n",
                           (long)e3_spin_progress_cdeg);
      return;
    }
    if ((uint32_t)(now - mission_snapshot.state_entry_tick) >=
        ROBOT_E3_SEARCH_SPIN_TIMEOUT_MS)
    {
      mission_snapshot.fault_flags |=
          vision_search_resume_state == MISSION_STATE_S7_SEARCH_RED
              ? MISSION_FAULT_S7_TIMEOUT : MISSION_FAULT_S3_TIMEOUT;
      MissionTask_EnterState(MISSION_STATE_FAULT, now);
      (void)DebugUart_Logf("[E3-R] CCW360 TIMEOUT progress=%ld cdeg\r\n",
                           (long)e3_spin_progress_cdeg);
      return;
    }
    turn_rpm = ROBOT_E3_SEARCH_CCW_CDEG - e3_spin_progress_cdeg >
                   ROBOT_S3_SEARCH_TURN_SLOW_THRESHOLD_CDEG
                   ? ROBOT_S3_SEARCH_TURN_FAST_RPM : ROBOT_S3_SEARCH_TURN_SLOW_RPM;
    mission_snapshot.vision_forward_rpm = 0;
    mission_snapshot.vision_turn_rpm = (int16_t)(-ROBOT_YAW_LEFT_SIGN * turn_rpm);
    MissionTask_SetWheelTargets((int16_t)(-ROBOT_YAW_LEFT_SIGN * turn_rpm),
                                (int16_t)(ROBOT_YAW_LEFT_SIGN * turn_rpm));
    if ((int32_t)(now - s3_next_debug_tick) >= 0)
    {
      s3_next_debug_tick = now + ROBOT_VISION_DEBUG_PERIOD_MS;
      (void)DebugUart_Logf("[E3-R] CCW360 progress=%ld/36000 rpm=%d\r\n",
                           (long)e3_spin_progress_cdeg, (int)turn_rpm);
    }
    return;
  }

  if (vision_search_forward_active)
  {
    if ((uint32_t)(now - mission_snapshot.state_entry_tick) <
        vision_search_forward_ms)
    {
      mission_snapshot.vision_forward_rpm =
          vision_search_forward_rpm;
      mission_snapshot.vision_turn_rpm = 0;
      MissionTask_SetWheelTargets(vision_search_forward_rpm,
                                  vision_search_forward_rpm);
      return;
    }

    MissionTask_StopWheels();
    mission_snapshot.vision_forward_rpm = 0;
    vision_search_forward_active = false;
    mission_snapshot.state_entry_tick = now;
    s3_search_turn_stable_since = 0U;
    (void)DebugUart_Logf(
        "[V-SEARCH] source=%s FORWARD DONE, START CW LEG\r\n",
        vision_search_resume_state == MISSION_STATE_S4_TRACK_CENTER
            ? "04"
            : (vision_search_resume_state == MISSION_STATE_S7_SEARCH_RED
                   ? "07"
                   : "03"));
  }

  if ((uint32_t)(now - mission_snapshot.state_entry_tick) >=
      ROBOT_S3_SEARCH_TURN_TIMEOUT_MS)
  {
    (void)DebugUart_Logf("[V-SEARCH] source=%s TURN TIMEOUT yaw=%ld "
                         "target=%ld\r\n",
                         vision_search_resume_state ==
                                 MISSION_STATE_S4_TRACK_CENTER
                             ? "04"
                             : (vision_search_resume_state ==
                                        MISSION_STATE_S7_SEARCH_RED
                                    ? "07"
                                    : "03"),
                         (long)MissionTask_WrapYaw(mission_snapshot.yaw_cdeg),
                         (long)s3_search_yaw_target_cdeg);
    if (vision_search_resume_state == MISSION_STATE_S4_TRACK_CENTER)
    {
      mission_snapshot.fault_flags |= MISSION_FAULT_S4_TIMEOUT;
    }
    else if (vision_search_resume_state == MISSION_STATE_S7_SEARCH_RED)
    {
      mission_snapshot.fault_flags |= MISSION_FAULT_S7_TIMEOUT;
    }
    else
    {
      mission_snapshot.fault_flags |= MISSION_FAULT_S3_TIMEOUT;
    }
    MissionTask_EnterState(MISSION_STATE_FAULT, now);
    return;
  }

  yaw_error = MissionTask_YawError(s3_search_yaw_target_cdeg,
                                   mission_snapshot.yaw_cdeg);
  mission_snapshot.yaw_error_cdeg = yaw_error;
  absolute_error = MissionTask_Abs32(yaw_error);

  if (absolute_error <= ROBOT_S3_SEARCH_TURN_TOLERANCE_CDEG)
  {
    MissionTask_StopWheels();
    if (s3_search_turn_stable_since == 0U)
    {
      s3_search_turn_stable_since = now;
    }
    else if ((uint32_t)(now - s3_search_turn_stable_since) >=
             ROBOT_S3_SEARCH_TURN_STABLE_MS)
    {
      s3_search_turn_stable_since = 0U;
      if (clockwise_phase)
      {
        /* Split the CCW sweep into two legs via the search origin. */
        s3_search_ccw_midpoint_done = false;
        s3_search_yaw_target_cdeg = s3_search_origin_yaw_cdeg;
        MissionTask_EnterState(MISSION_STATE_S3_SEARCH_TURN_CCW, now);
        (void)DebugUart_Logf(
            "[S3-S] CW DONE, CCW SWEEP midpoint=%ld\r\n",
            (long)s3_search_yaw_target_cdeg);
      }
      else if (!s3_search_ccw_midpoint_done)
      {
        s3_search_ccw_midpoint_done = true;
        s3_search_yaw_target_cdeg = MissionTask_WrapYaw(
            s3_search_origin_yaw_cdeg +
            (int32_t)ROBOT_YAW_LEFT_SIGN *
                vision_search_turn_angle_cdeg);
        (void)DebugUart_Logf(
            "[S3-S] CCW MIDPOINT DONE, CONTINUE CCW target=%ld\r\n",
            (long)s3_search_yaw_target_cdeg);
      }
      else
      {
        if (vision_search_resume_state == MISSION_STATE_S4_TRACK_CENTER)
        {
          s4_center_search_sweep_completed = true;
          s4_center_target_lost_since = now;
          s4_next_debug_tick = now;
          if (!MissionTask_ReturnToS4TrackCenter(now))
          {
            return;
          }
        }
        else if (vision_search_resume_state == MISSION_STATE_S7_SEARCH_RED)
        {
          MissionTask_CompleteS7RedRecovery(now);
        }
        else
        {
          s3_search_sweep_completed = true;
          s3_target_lost_since = now;
          MissionTask_EnterState(MISSION_STATE_S3_TRACK_GREEN, now);
        }
        if (vision_search_resume_state == MISSION_STATE_S7_SEARCH_RED)
        {
          (void)DebugUart_Log(
              "[V-SEARCH] source=07 CCW SWEEP DONE, RED NOT FOUND\r\n");
        }
        else
        {
          (void)DebugUart_Logf(
              "[V-SEARCH] source=%s CCW SWEEP DONE, WAIT COORDINATES\r\n",
              vision_search_resume_state == MISSION_STATE_S4_TRACK_CENTER
                  ? "04"
                  : "03");
        }
      }
      return;
    }
  }
  else
  {
    s3_search_turn_stable_since = 0U;
    turn_rpm = absolute_error >
                       ROBOT_S3_SEARCH_TURN_SLOW_THRESHOLD_CDEG
                   ? ROBOT_S3_SEARCH_TURN_FAST_RPM
                   : ROBOT_S3_SEARCH_TURN_SLOW_RPM;
    turn_left = (yaw_error * (int32_t)ROBOT_YAW_LEFT_SIGN) > 0L;
    MissionTask_SetWheelTargets(turn_left ? -turn_rpm : turn_rpm,
                                turn_left ? turn_rpm : -turn_rpm);
  }

  if ((int32_t)(now - s3_next_debug_tick) >= 0)
  {
    s3_next_debug_tick = now + ROBOT_VISION_DEBUG_PERIOD_MS;
    (void)DebugUart_Logf(
        "[S3-S] leg=%s yaw=%ld target=%ld err=%ld cmd=%d/%d\r\n",
        clockwise_phase ? "CW" :
            (s3_search_ccw_midpoint_done ? "CCW_END" : "CCW_MID"),
        (long)MissionTask_WrapYaw(mission_snapshot.yaw_cdeg),
        (long)s3_search_yaw_target_cdeg,
        (long)yaw_error,
        (int)mission_snapshot.left_target_rpm,
        (int)mission_snapshot.right_target_rpm);
  }
}

static int16_t MissionTask_CalculateVisionForward(uint32_t now,
                                                  int16_t error_px)
{
  float error = (float)error_px;
  bool final_track =
      mission_snapshot.state == MISSION_STATE_S4_TRACK_FINAL_BLOCK;
  float kp = final_track ? ROBOT_S4_FINAL_Y_KP : ROBOT_VISION_Y_KP;
  float min_forward_rpm = final_track
      ? ROBOT_S4_FINAL_MIN_FORWARD_RPM
      : (mission_snapshot.state == MISSION_STATE_S6_TRACK_SAFE_ZONE
             ? ROBOT_S6_APPROACH_MIN_FORWARD_RPM
             : (((mission_snapshot.state == MISSION_STATE_S3_TRACK_GREEN) ||
                 (mission_snapshot.state == MISSION_STATE_S4_TRACK_CENTER))
                    ? ROBOT_VISION_03_04_MIN_FORWARD_RPM
                    : ROBOT_VISION_Y_MIN_FORWARD_RPM));
  float dt_seconds;
  float derivative;
  float magnitude;
  float signed_output;

  if (mission_snapshot.target_sequence != s3_y_last_target_sequence)
  {
    if (!s3_y_pid_initialized)
    {
      dt_seconds = (float)MISSION_TASK_PERIOD_TICKS / 1000.0f;
      derivative = 0.0f;
      s3_y_pid_initialized = true;
    }
    else
    {
      dt_seconds = (float)(now - s3_y_pid_update_tick) / 1000.0f;
      dt_seconds = MissionTask_ClampFloat(dt_seconds, 0.01f, 0.20f);
      derivative = (error - s3_y_pid_last_error) / dt_seconds;
    }

    s3_y_pid_integral += error * dt_seconds;
    s3_y_pid_integral = MissionTask_ClampFloat(
        s3_y_pid_integral,
        -ROBOT_VISION_Y_INTEGRAL_LIMIT,
        ROBOT_VISION_Y_INTEGRAL_LIMIT);
    s3_y_pid_output = kp * error +
                      ROBOT_VISION_Y_KI * s3_y_pid_integral +
                      ROBOT_VISION_Y_KD * derivative;
    s3_y_pid_output = MissionTask_ClampFloat(
        s3_y_pid_output,
        -ROBOT_VISION_Y_MAX_FORWARD_RPM,
        ROBOT_VISION_Y_MAX_FORWARD_RPM);

    s3_y_pid_last_error = error;
    s3_y_pid_update_tick = now;
    s3_y_last_target_sequence = mission_snapshot.target_sequence;
  }

  signed_output = s3_y_pid_output * (float)ROBOT_VISION_Y_FORWARD_SIGN;
  magnitude = signed_output < 0.0f ? -signed_output : signed_output;
  if ((magnitude > 0.0f) &&
      (magnitude < min_forward_rpm))
  {
    signed_output = signed_output < 0.0f
                        ? -min_forward_rpm
                        : min_forward_rpm;
  }

  return (int16_t)(signed_output >= 0.0f
                       ? signed_output + 0.5f
                       : signed_output - 0.5f);
}

static void MissionTask_SetTrackingTargets(int16_t forward_rpm,
                                           int16_t turn_rpm,
                                           int16_t wheel_max_rpm)
{
  int32_t left_rpm = (int32_t)forward_rpm + turn_rpm;
  int32_t right_rpm = (int32_t)forward_rpm - turn_rpm;
  int32_t left_magnitude = MissionTask_Abs32(left_rpm);
  int32_t right_magnitude = MissionTask_Abs32(right_rpm);
  int32_t maximum_magnitude = left_magnitude > right_magnitude
                                  ? left_magnitude
                                  : right_magnitude;

  /* Scale the pair together so curvature is preserved while neither wheel
     exceeds the configured tracking safety limit. */
  if (maximum_magnitude > wheel_max_rpm)
  {
    left_rpm = (left_rpm * wheel_max_rpm) / maximum_magnitude;
    right_rpm = (right_rpm * wheel_max_rpm) / maximum_magnitude;
  }

  MissionTask_SetWheelTargets((int16_t)left_rpm, (int16_t)right_rpm);
}

static float MissionTask_VisionAgeScale(uint32_t age_ms)
{
  if (age_ms >= MISSION_VISION_TARGET_TIMEOUT_MS) return 0.0f;
  if (age_ms <= ROBOT_VISION_COORD_DECEL_START_MS) return 1.0f;
  return (float)(MISSION_VISION_TARGET_TIMEOUT_MS - age_ms) /
      (float)(MISSION_VISION_TARGET_TIMEOUT_MS - ROBOT_VISION_COORD_DECEL_START_MS);
}

static void MissionTask_SetVisionTrackingTargets(int16_t forward_rpm,
                                                int16_t turn_rpm,
                                                int16_t wheel_max_rpm)
{
  float scale = mission_snapshot.vision_target_valid
      ? MissionTask_VisionAgeScale(mission_snapshot.target_age_ms) : 0.0f;
  /* Scale both channels together. Never reapply the PID minimum after fading. */
  forward_rpm = (int16_t)((float)forward_rpm * scale);
  turn_rpm = (int16_t)((float)turn_rpm * scale);
  mission_snapshot.vision_forward_rpm = forward_rpm;
  mission_snapshot.vision_turn_rpm = turn_rpm;
  MissionTask_SetTrackingTargets(forward_rpm, turn_rpm, wheel_max_rpm);
}

static bool MissionTask_CommandS4VisionTracking(uint32_t now,
                                                 bool *xy_in_tolerance)
{
  bool final_track =
      mission_snapshot.state == MISSION_STATE_S4_TRACK_FINAL_BLOCK;
  int16_t turn_max_rpm = final_track ? ROBOT_S4_FINAL_TURN_MAX_RPM :
                                       ROBOT_S4_TRACK_TURN_MAX_RPM;
  int16_t forward_max_rpm = final_track ? ROBOT_S4_FINAL_FORWARD_MAX_RPM :
                                          ROBOT_S4_TRACK_FORWARD_MAX_RPM;
  int16_t wheel_max_rpm = final_track ? ROBOT_S4_FINAL_WHEEL_MAX_RPM :
                                        ROBOT_S4_TRACK_WHEEL_MAX_RPM;
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

  if (!mission_snapshot.vision_target_valid)
  {
    MissionTask_StopWheels();
    /* Keep the pixel PID history until an actual recovery/task change. */
    if ((int32_t)(now - s4_next_debug_tick) >= 0)
    {
      s4_next_debug_tick = now + ROBOT_S4_DEBUG_PERIOD_MS;
      (void)DebugUart_Logf("[S4-V] state=%u TARGET LOST cmd=0/0\r\n",
                           (unsigned int)mission_snapshot.state);
    }
    return false;
  }

  absolute_x_error = MissionTask_Abs32(mission_snapshot.vision_x_error_px);
  absolute_y_error = MissionTask_Abs32(mission_snapshot.vision_y_error_px);
  x_in_tolerance = absolute_x_error <= ROBOT_VISION_X_TOLERANCE_PX;
  y_in_tolerance = absolute_y_error <= ROBOT_VISION_Y_TOLERANCE_PX;

  turn_rpm = x_in_tolerance
                 ? 0
                 : MissionTask_CalculateVisionTurn(
                       now, mission_snapshot.vision_x_error_px);
  forward_rpm = y_in_tolerance
                    ? 0
                    : MissionTask_CalculateVisionForward(
                          now, mission_snapshot.vision_y_error_px);

  if (turn_rpm > turn_max_rpm)
  {
    turn_rpm = turn_max_rpm;
  }
  else if (turn_rpm < -turn_max_rpm)
  {
    turn_rpm = -turn_max_rpm;
  }
  if (forward_rpm > forward_max_rpm)
  {
    forward_rpm = forward_max_rpm;
  }
  else if (forward_rpm < -forward_max_rpm)
  {
    forward_rpm = -forward_max_rpm;
  }

  mission_snapshot.vision_turn_rpm = turn_rpm;
  mission_snapshot.vision_forward_rpm = forward_rpm;
  if (x_in_tolerance && y_in_tolerance)
  {
    MissionTask_StopWheels();
  }
  else
  {
    MissionTask_SetVisionTrackingTargets(forward_rpm, turn_rpm, wheel_max_rpm);
  }

  if (xy_in_tolerance != NULL)
  {
    *xy_in_tolerance = x_in_tolerance && y_in_tolerance;
  }

  if ((int32_t)(now - s4_next_debug_tick) >= 0)
  {
    s4_next_debug_tick = now + ROBOT_S4_DEBUG_PERIOD_MS;
    (void)DebugUart_Logf(
        "[S4-V] state=%u xerr=%d yerr=%d turn=%d fwd=%d cmd=%d/%d\r\n",
        (unsigned int)mission_snapshot.state,
        (int)mission_snapshot.vision_x_error_px,
        (int)mission_snapshot.vision_y_error_px,
        (int)turn_rpm,
        (int)forward_rpm,
        (int)mission_snapshot.left_target_rpm,
        (int)mission_snapshot.right_target_rpm);
  }
  return true;
}

static void MissionTask_AckS4Center(uint32_t now)
{
  MissionTask_StopWheels();
  MissionTask_ResetVisionControllerState();
  MaixCam_ClearObject();
  mission_snapshot.vision_target_valid = false;
  MissionTask_EnterState(MISSION_STATE_S4_WAIT_ARRANGE_READY, now);
  /* RX14 has already been consumed. Do not clear events here: a quick RX02
     or RX24 arriving while the ACK is sent must remain available. */
  if (MaixCam_SendCommand(MAIXCAM_COMMAND_CENTER_ACK) != HAL_OK)
  {
    mission_snapshot.fault_flags |= MISSION_FAULT_VISION_TX;
    MissionTask_EnterState(MISSION_STATE_FAULT, now);
    return;
  }
  (void)DebugUart_Log(
      "[S4] RX14 STOP, TX E1 E2 14 1E 2E IMMEDIATELY, WAIT RX02/24/E4\r\n");
}

static float MissionTask_S4SmoothStep(float fraction)
{
  if (fraction <= 0.0f)
  {
    return 0.0f;
  }
  if (fraction >= 1.0f)
  {
    return 1.0f;
  }
  return fraction * fraction * (3.0f - 2.0f * fraction);
}

/* Both ends approach zero before the next direction is commanded.
   Short actions shorten the ramps so the envelope still reaches its peak. */
static float MissionTask_S4MotionScale(uint32_t elapsed,
                                       uint32_t duration, uint32_t ramp)
{
  uint32_t edge_time;
  if ((duration == 0U) || (elapsed >= duration))
  {
    return 0.0f;
  }
  if (ramp > duration / 2U)
  {
    ramp = duration / 2U;
  }
  if (ramp == 0U)
  {
    return 1.0f;
  }
  edge_time = elapsed < duration - elapsed ? elapsed : duration - elapsed;
  return MissionTask_S4SmoothStep((float)edge_time / (float)ramp);
}

/* Shared forward profile. Reverse replays these wheel commands backwards in
   time, negating both wheels without swapping left/right. */
static void MissionTask_CalculateS4CornerProfile(uint32_t elapsed,
                                                uint32_t duration,
                                                bool push_right,
                                                int16_t *left_rpm,
                                                int16_t *right_rpm)
{
  float scale = MissionTask_S4MotionScale(elapsed, duration, ROBOT_S4_PUSH_RAMP_MS);
  float base_rpm = (float)ROBOT_S4_CORNER_PUSH_RPM;
  if (!push_right)
  {
    float blend = elapsed <= ROBOT_S4_LEFT_CORNER_INITIAL_MS ? 0.0f :
        MissionTask_S4SmoothStep((float)(elapsed - ROBOT_S4_LEFT_CORNER_INITIAL_MS) /
                                (float)ROBOT_S4_LEFT_SPEED_BLEND_MS);
    base_rpm = (float)ROBOT_S4_LEFT_CORNER_INITIAL_RPM +
        ((float)ROBOT_S4_CORNER_PUSH_RPM - (float)ROBOT_S4_LEFT_CORNER_INITIAL_RPM) * blend;
  }
  int16_t forward_rpm = (int16_t)(base_rpm * scale + 0.5f);
  float progress = duration == 0U ? 0.0f : (float)elapsed / (float)duration;
  float middle_blend = MissionTask_S4SmoothStep(
      (progress - ROBOT_S4_CORNER_DIFF_MIDDLE_BEGIN) /
      (ROBOT_S4_CORNER_DIFF_MIDDLE_END -
       ROBOT_S4_CORNER_DIFF_MIDDLE_BEGIN));
  float late_blend = MissionTask_S4SmoothStep(
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

static void MissionTask_CommandS4CornerReverse(uint32_t now, bool push_right)
{
  uint32_t elapsed = (uint32_t)(now - mission_snapshot.state_entry_tick);
  uint32_t duration = MissionTask_S4ReverseDuration(push_right);
  uint32_t push_duration = MissionTask_S4PushDuration(push_right);
  int16_t left_rpm = 0;
  int16_t right_rpm = 0;
  if ((duration > 0U) && (elapsed < duration))
  {
    uint32_t replay_elapsed = (uint32_t)(
        ((uint64_t)(duration - elapsed) * push_duration + duration / 2U) /
        duration);
    float speed_scale = (float)push_duration / (float)duration;
    /* Shortening 12 must reduce distance, not increase speed to compensate. */
    if (!push_right) speed_scale *= (float)ROBOT_S4_LEFT_REVERSE_DISTANCE_PERCENT / 100.0f;
    MissionTask_CalculateS4CornerProfile(replay_elapsed, push_duration,
                                         push_right, &left_rpm, &right_rpm);
    left_rpm = (int16_t)-(int16_t)((float)left_rpm * speed_scale + 0.5f);
    right_rpm = (int16_t)-(int16_t)((float)right_rpm * speed_scale + 0.5f);
  }
  mission_snapshot.vision_forward_rpm = (int16_t)((left_rpm + right_rpm) / 2);
  mission_snapshot.vision_turn_rpm = (int16_t)((left_rpm - right_rpm) / 2);
  MissionTask_SetWheelTargets(left_rpm, right_rpm);
  if ((int32_t)(now - s4_next_debug_tick) >= 0)
  {
    s4_next_debug_tick = now + ROBOT_S4_DEBUG_PERIOD_MS;
    (void)DebugUart_Logf(
        "[S4-CURVE-BACK] phase=%s t=%lu/%lu replay=END_TO_START cmd=%d/%d\r\n",
        push_right ? "02" : "12", (unsigned long)elapsed,
        (unsigned long)duration, (int)left_rpm, (int)right_rpm);
  }
}

static void MissionTask_CommandS4CornerPush(uint32_t now, bool push_right)
{
  uint32_t elapsed = (uint32_t)(now - mission_snapshot.state_entry_tick);
  uint32_t duration = MissionTask_S4PushDuration(push_right);
  int16_t left_rpm;
  int16_t right_rpm;
  MissionTask_CalculateS4CornerProfile(elapsed, duration, push_right,
                                       &left_rpm, &right_rpm);
  mission_snapshot.vision_turn_rpm =
      (int16_t)((left_rpm - right_rpm) / 2);
  mission_snapshot.vision_forward_rpm = (int16_t)((left_rpm + right_rpm) / 2);
  MissionTask_SetWheelTargets(left_rpm, right_rpm);

  if ((int32_t)(now - s4_next_debug_tick) >= 0)
  {
    int16_t actual_left_rpm = (int16_t)(
        ((float)moto_chassis[MISSION_LEFT_MOTOR_INDEX].speed_rpm / 36.0f) *
        ROBOT_LEFT_WHEEL_FORWARD_SIGN);
    int16_t actual_right_rpm = (int16_t)(
        ((float)moto_chassis[MISSION_RIGHT_MOTOR_INDEX].speed_rpm / 36.0f) *
        ROBOT_RIGHT_WHEEL_FORWARD_SIGN);
    s4_next_debug_tick = now + ROBOT_S4_DEBUG_PERIOD_MS;
    (void)DebugUart_Logf(
        "[S4-CURVE] phase=%s t=%lu/%lu smooth_diff=%d cmd=%d/%d act=%d/%d\r\n",
        push_right ? "02" : "12",
        (unsigned long)elapsed,
        (unsigned long)duration,
        (int)MissionTask_Abs32((int32_t)left_rpm - right_rpm),
        (int)mission_snapshot.left_target_rpm,
        (int)mission_snapshot.right_target_rpm,
        (int)actual_left_rpm,
        (int)actual_right_rpm);
  }
}

static void MissionTask_RunS3Track(uint32_t now)
{
  int32_t absolute_x_error;
  int32_t absolute_y_error;
  int16_t turn_rpm;
  int16_t forward_rpm;
  bool x_in_tolerance;
  bool y_in_tolerance;

  if (mission_snapshot.state != MISSION_STATE_S3_TRACK_GREEN)
  {
    return;
  }

  if (MissionTask_TryStartS3Capture(now))
  {
    return;
  }

  if (!mission_snapshot.vision_target_valid)
  {
    MissionTask_StopWheels();
    s3_track_stable_since = 0U;
    s3_wait_event04_logged = false;
    if (s3_target_lost_since == 0U)
    {
      s3_target_lost_since = now;
    }
    if (!s3_search_sweep_completed &&
        ((uint32_t)(now - s3_target_lost_since) >=
         ROBOT_S3_GREEN_WAIT_MS))
    {
      (void)DebugUart_Logf(
          "[S3-T] TARGET LOST %lums, START COMMON RECOVERY\r\n",
          (unsigned long)ROBOT_S3_GREEN_WAIT_MS);
      MissionTask_StartVisionSearchRecovery(
          now,
          s3_target_select_command == MAIXCAM_COMMAND_SELECT_RED
              ? MISSION_STATE_S7_SEARCH_RED : MISSION_STATE_S3_TRACK_GREEN,
          s3_target_select_command == MAIXCAM_COMMAND_SELECT_RED
              ? mission_snapshot.yaw_cdeg : s3_search_origin_yaw_cdeg,
          false);
      return;
    }
    MissionTask_DebugS3(now);
    return;
  }

  s3_target_lost_since = 0U;
  s3_search_sweep_completed = false;

  if ((uint32_t)(now - mission_snapshot.state_entry_tick) >=
      ROBOT_VISION_TRACK_TIMEOUT_MS)
  {
    mission_snapshot.fault_flags |= MISSION_FAULT_S3_TIMEOUT;
    MissionTask_EnterState(MISSION_STATE_FAULT, now);
    return;
  }

  absolute_x_error = MissionTask_Abs32(mission_snapshot.vision_x_error_px);
  absolute_y_error = MissionTask_Abs32(mission_snapshot.vision_y_error_px);
  x_in_tolerance = absolute_x_error <= ROBOT_VISION_X_TOLERANCE_PX;
  y_in_tolerance = absolute_y_error <= ROBOT_VISION_Y_TOLERANCE_PX;

  if (x_in_tolerance && y_in_tolerance)
  {
    MissionTask_StopWheels();
    MissionTask_ResetVisionControllerState();

    if (s3_track_stable_since == 0U)
    {
      s3_track_stable_since = now;
      s3_wait_event04_logged = false;
      (void)DebugUart_Log("[S3-T] XY_IN_TOLERANCE\r\n");
    }
    else if ((uint32_t)(now - s3_track_stable_since) >=
                 ROBOT_VISION_Y_STABLE_MS &&
             !s3_wait_event04_logged)
    {
      s3_wait_event04_logged = true;
      (void)DebugUart_Log("[S3-T] XY_STABLE, WAIT EVENT 04\r\n");
    }

    MissionTask_DebugS3(now);
    return;
  }

  s3_track_stable_since = 0U;
  s3_wait_event04_logged = false;

  if (x_in_tolerance)
  {
    turn_rpm = 0;
    s3_pid_integral = 0.0f;
    s3_pid_output = 0.0f;
    s3_pid_initialized = false;
    s3_last_target_sequence = UINT32_MAX;
  }
  else
  {
    turn_rpm = MissionTask_CalculateVisionTurn(
        now, mission_snapshot.vision_x_error_px);
  }

  if (y_in_tolerance)
  {
    forward_rpm = 0;
    s3_y_pid_integral = 0.0f;
    s3_y_pid_output = 0.0f;
    s3_y_pid_initialized = false;
    s3_y_last_target_sequence = UINT32_MAX;
  }
  else
  {
    forward_rpm = MissionTask_CalculateVisionForward(
        now, mission_snapshot.vision_y_error_px);
  }

  mission_snapshot.vision_turn_rpm = turn_rpm;
  mission_snapshot.vision_forward_rpm = forward_rpm;
  MissionTask_SetVisionTrackingTargets(forward_rpm, turn_rpm,
                                 ROBOT_VISION_TRACK_WHEEL_MAX_RPM);
  MissionTask_DebugS3(now);
}

static void MissionTask_RunS3LowerFrame(uint32_t now)
{
  if (mission_snapshot.state != MISSION_STATE_S3_LOWER_FRAME)
  {
    return;
  }

#if ROBOT_S3_FRAME_LOWER_SETTLE_MS > 0U
  if ((uint32_t)(now - mission_snapshot.state_entry_tick) <
      ROBOT_S3_FRAME_LOWER_SETTLE_MS)
  {
    return;
  }
#endif

  (void)DebugUart_Log("[S3] FRAME DOWN DONE, BEGIN VISION 03\r\n");
  (void)MissionTask_BeginS3Vision(now);
}

/* Estimate distance from the integrated target-speed profile, not total
   duration: keep the ramps and 12's low-speed segment unchanged. */
static uint32_t MissionTask_S4ScalePushDuration(uint32_t duration, bool push_right)
{
  float cruise_rpm = (float)ROBOT_S4_CORNER_PUSH_RPM;
  float area = cruise_rpm * ((float)duration - (float)ROBOT_S4_PUSH_RAMP_MS);
  if (!push_right)
  {
    area -= (cruise_rpm - (float)ROBOT_S4_LEFT_CORNER_INITIAL_RPM) *
        ((float)ROBOT_S4_LEFT_CORNER_INITIAL_MS +
         (float)ROBOT_S4_LEFT_SPEED_BLEND_MS * 0.5f -
         (float)ROBOT_S4_PUSH_RAMP_MS * 0.5f);
  }
  float additional_ms = area *
      ((float)ROBOT_S4_PUSH_DISTANCE_PERCENT - 100.0f) / (100.0f * cruise_rpm);
  return (uint32_t)((float)duration + additional_ms + 0.5f);
}

static uint32_t MissionTask_S4PushDuration(bool push_right)
{
  return MissionTask_S4ScalePushDuration(
      push_right ? ROBOT_S4_RIGHT_CORNER_FOLLOW_MS
                 : ROBOT_S4_LEFT_CORNER_FOLLOW_MS,
      push_right);
}

static uint32_t MissionTask_S4ReverseDuration(bool push_right)
{
  float reverse_rpm = (float)(push_right ? ROBOT_S4_RIGHT_REVERSE_RPM
                                       : ROBOT_S4_LEFT_REVERSE_RPM);
  if (reverse_rpm <= 0.0f)
  {
    return 0U;
  }
  /* Keep the smooth profile, with 12's requested shorter reverse distance. */
  float distance_scale = push_right ? 1.0f :
      (float)ROBOT_S4_LEFT_REVERSE_DISTANCE_PERCENT / 100.0f;
  return (uint32_t)((float)MissionTask_S4PushDuration(push_right) *
                   (float)ROBOT_S4_CORNER_PUSH_RPM / reverse_rpm * distance_scale + 0.5f);
}

static void MissionTask_StartS4TrackState(MissionState state, uint32_t now)
{
  if (state == MISSION_STATE_S4_TRACK_RIGHT_BLOCK)
  {
    if (s4_arrange_cycle_count < UINT16_MAX)
    {
      ++s4_arrange_cycle_count;
    }
    (void)DebugUart_Logf(
        "[S4-REPEAT] RX02 cycle=%u push02=%lums push12=%lums back02=%lums back12=%lums\r\n",
        (unsigned int)s4_arrange_cycle_count,
        (unsigned long)MissionTask_S4PushDuration(true),
        (unsigned long)MissionTask_S4PushDuration(false),
        (unsigned long)MissionTask_S4ReverseDuration(true),
        (unsigned long)MissionTask_S4ReverseDuration(false));
  }
  MaixCam_ClearObject();
  MissionTask_ResetVisionPid();
  if ((state == MISSION_STATE_S4_TRACK_RIGHT_BLOCK) ||
      (state == MISSION_STATE_S4_TRACK_LEFT_BLOCK))
  {
    s4_arrange_recovery_count = 0U;
  }
  s4_next_debug_tick = now;
  MissionTask_EnterState(state, now);
}

static uint8_t MissionTask_GetSelectedObjectId(void)
{
  switch (s3_target_select_command)
  {
    case MAIXCAM_COMMAND_SELECT_BLUE: return 3U;
    case MAIXCAM_COMMAND_SELECT_RED: return 4U;
    case MAIXCAM_COMMAND_SELECT_GREEN: return 5U;
    case MAIXCAM_COMMAND_SELECT_BLACK: return 6U;
    case MAIXCAM_COMMAND_SELECT_BLACK_GREEN:
      return (s51_active_object_id == 5U || s51_active_object_id == 6U)
          ? s51_active_object_id : 0U;
    default: return 0U;
  }
}

static uint8_t MissionTask_GetCarriedObjectId(void)
{
  return s6_carried_object_id != 0U ? s6_carried_object_id
                                   : MissionTask_GetSelectedObjectId();
}

static void MissionTask_UpdateS5CarriedObject(uint32_t now)
{
  MaixCam_Object object;
  if (!s5_close_view_active || !MaixCam_GetObjectSnapshot(&object) ||
      (int32_t)(object.sequence - s5_load_check_sequence_floor) <= 0 ||
      (uint32_t)(now - object.update_tick) > MISSION_VISION_TARGET_TIMEOUT_MS ||
      object.object_id < 3U || object.object_id > 6U)
  {
    return;
  }

  /* In 05 the vision-side ID must describe the confirmed object inside the
     frame, not an unrelated object elsewhere in the image. */
  s5_load_check_sequence_floor = object.sequence;
  if (!s5_load_class_confirmed || s6_carried_object_id != object.object_id)
  {
    s6_carried_object_id = object.object_id;
    s5_load_class_confirmed = true;
    (void)DebugUart_Logf(
        "[S5-CARGO] CONFIRMED ID=%u kind=%s cmd=%02X seq=%lu age=%lums\r\n",
        (unsigned int)s6_carried_object_id,
        MissionTask_S6IsSupplyTarget() ? "SUPPLY" : "CASUALTY",
        (unsigned int)s3_target_select_command,
        (unsigned long)object.sequence,
        (unsigned long)(now - object.update_tick));
  }
}

static void MissionTask_EnterS5(uint32_t now)
{
  uint8_t analyze_command = mission_snapshot.team == ROBOT_TEAM_RED
                                ? MAIXCAM_COMMAND_ANALYZE_LOAD_RED
                                : MAIXCAM_COMMAND_ANALYZE_LOAD_BLUE;

  /* Start a new load record. Never carry the previous red task's identity
     into a 51 capture; prefer the actual black/green tracking ID. */
  s6_carried_object_id = MissionTask_GetSelectedObjectId();
  s5_load_class_confirmed = false;
  s5_load_check_sequence_floor = 0U;
  MaixCam_ClearEvent();
  (void)MaixCam_TakeSafeZoneAlignRequest();
  s6_align26_pending = false;
  MaixCam_ClearObject();
  s5_next_debug_tick = now;
  s5_close_view_active = false;
  MissionTask_EnterState(MISSION_STATE_S5_WAIT_SINGLE_GREEN, now);
  if (MaixCam_SendCommand(analyze_command) != HAL_OK)
  {
    mission_snapshot.fault_flags |= MISSION_FAULT_VISION_TX;
    MissionTask_EnterState(MISSION_STATE_FAULT, now);
    return;
  }
  (void)DebugUart_Logf(
      "[S5] TX E1 E2 %02X 1E 2E, WAIT RX05 LOAD CHECK\r\n",
      (unsigned int)analyze_command);
  (void)DebugUart_Logf("[S5-CARGO] INIT ID=%u cmd=%02X kind=%s source=TASK\r\n",
      (unsigned int)s6_carried_object_id, (unsigned int)s3_target_select_command,
      MissionTask_S6IsSupplyTarget() ? "SUPPLY" :
          (MissionTask_S6IsCasualtyTarget() ? "CASUALTY" : "UNKNOWN"));
}

static void MissionTask_SkipToS4FinalTrack(uint32_t now)
{
  s4_arrange_cycle_count = 0U;
  MissionTask_StopWheels();
  MaixCam_ClearObject();
  MissionTask_ResetVisionPid();
  if (Actuator_SetFrameRaised() != HAL_OK)
  {
    mission_snapshot.fault_flags |= MISSION_FAULT_ACTUATOR;
    MissionTask_EnterState(MISSION_STATE_FAULT, now);
    return;
  }

  s4_final_mode_already_active = true;
  s4_final_event34_pending = false;
  MissionTask_EnterState(MISSION_STATE_S4_RAISE_FRAME, now);
  (void)DebugUart_Logf(
      "[S4] RX24 SKIP REMAINING ARRANGE, FRAME UP settle=%lums\r\n",
      (unsigned long)ROBOT_S4_FRAME_RAISE_SETTLE_MS);
}

static void MissionTask_StartS4ArrangeRecovery(uint32_t now,
                                                MissionState resume_state)
{
  MissionTask_StopWheels();
  MaixCam_ClearObject();
  mission_snapshot.vision_target_valid = false;
  MissionTask_ResetVisionPid();
  MissionTask_ResetS6ControllerState();
  s6_phase_stable_since = 0U;
  s4_arrange_recovery_resume_state = resume_state;
  s4_arrange_recovery_base_yaw_cdeg = MissionTask_WrapYaw(
      mission_snapshot.yaw_cdeg);
  s4_arrange_recovery_left_yaw_cdeg = MissionTask_WrapYaw(
      s4_arrange_recovery_base_yaw_cdeg +
      (int32_t)ROBOT_YAW_LEFT_SIGN *
          ROBOT_S4_ARRANGE_RECOVERY_LEFT_CDEG);
  s4_arrange_recovery_right_yaw_cdeg = MissionTask_WrapYaw(
      s4_arrange_recovery_left_yaw_cdeg -
      (int32_t)ROBOT_YAW_LEFT_SIGN *
          ROBOT_S4_ARRANGE_RECOVERY_RIGHT_CDEG);
  ++s4_arrange_recovery_count;
  s4_next_debug_tick = now;
  MissionTask_EnterState(
      MISSION_STATE_S4_ARRANGE_RECOVERY_REVERSE, now);
  (void)DebugUart_Logf(
      "[S4-E2] START #%u resume=%s base=%ld left45=%ld right90=%ld reverse=%d/%lums\r\n",
      (unsigned int)s4_arrange_recovery_count,
      resume_state == MISSION_STATE_S4_TRACK_RIGHT_BLOCK ? "02" : "12",
      (long)s4_arrange_recovery_base_yaw_cdeg,
      (long)s4_arrange_recovery_left_yaw_cdeg,
      (long)s4_arrange_recovery_right_yaw_cdeg,
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
      "[S4] RX EVENT %02X, EXPECT 24 OR COORDS (E2 ONLY AFTER TX22)\r\n",
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
          "[S4-E2] RX EVENT %02X, EXPECT COORDS OR 24\r\n",
          (unsigned int)event_code);
    }
  }

  if (!mission_snapshot.vision_target_valid)
  {
    return false;
  }

  recovered_x_error = mission_snapshot.vision_x_error_px;
  recovered_y_error = mission_snapshot.vision_y_error_px;
  MissionTask_StopWheels();
  MissionTask_ResetVisionPid();
  MissionTask_ResetS6ControllerState();
  s4_next_debug_tick = now;
  MissionTask_EnterState(s4_arrange_recovery_resume_state, now);
  (void)DebugUart_Logf(
      "[S4-E2] TARGET RECOVERED x=%d y=%d, RESUME %s\r\n",
      (int)recovered_x_error,
      (int)recovered_y_error,
      s4_arrange_recovery_resume_state ==
              MISSION_STATE_S4_TRACK_RIGHT_BLOCK
          ? "02"
          : "12");
  return true;
}

static void MissionTask_EnterS4FinalCenter(uint32_t now)
{
  MissionTask_StopWheels();
  MissionTask_ResetVisionPid();
  MaixCam_ClearObject();
  s4_final_missing_since = 0U;
  s4_final_frame_lowered = false;
  MissionTask_EnterState(MISSION_STATE_S4_FINAL_CENTER_OBJECT, now);
  (void)DebugUart_Logf(
      "[S4] RX34 FINAL OBJECT IN FRAME, FORWARD rpm=%d time=%lums\r\n",
      ROBOT_S4_FINAL_CENTER_RPM,
      (unsigned long)(s3_target_select_command == MAIXCAM_COMMAND_SELECT_RED
                          ? ROBOT_S4_RED_FINAL_CENTER_MS
                          : ROBOT_S4_FINAL_CENTER_MS));
}

static void MissionTask_StartS4FinalRecovery(uint32_t now)
{
  if (MissionTask_ActiveRecoveryLossEvent() != 0U) return;
  if (s7_target_switch_enabled && s4_final_recovery_count >= 1U)
  {
    MissionTask_StopWheels();
    MaixCam_ClearEvent();
    MaixCam_ClearObject();
    mission_snapshot.vision_target_valid = false;
    (void)DebugUart_Log(
        "[S4-24R] SECOND LOSS, SKIP RECOVERY, SELECT51 BLACK/GREEN AND SEARCH03\r\n");
    (void)MissionTask_StartS3Align(now, MAIXCAM_COMMAND_SELECT_BLACK_GREEN);
    return;
  }
  if (!s7_target_switch_enabled && s4_final_recovery_count >= 1U)
  {
    (void)DebugUart_Log(
        "[S4-24R] INITIAL GREEN LOST AGAIN, KEEP GREEN21 AND REPEAT24 RECOVERY\r\n");
  }
  MissionTask_StopWheels();
  MissionTask_ResetVisionPid();
  s4_final_turn_stable_since = 0U;
  s4_final_first_yaw_cdeg = MissionTask_WrapYaw(
      mission_snapshot.safe_zone_yaw_target_cdeg +
      ROBOT_S4_FINAL_RECOVERY_FIRST_SAFE_OFFSET_CDEG);
  s4_final_second_yaw_cdeg = MissionTask_WrapYaw(
      mission_snapshot.safe_zone_yaw_target_cdeg +
      ROBOT_S4_FINAL_RECOVERY_SECOND_SAFE_OFFSET_CDEG);
  if (s4_final_recovery_count < UINT8_MAX)
  {
    ++s4_final_recovery_count;
  }
  MissionTask_EnterState(
      MISSION_STATE_S4_FINAL_RECOVERY_REVERSE, now);
  (void)DebugUart_Logf(
      "[S4-24R] START #%u NO COORD %lums, REVERSE rpm=%d time=%lums\r\n",
      (unsigned int)s4_final_recovery_count,
      (unsigned long)ROBOT_S4_FINAL_MISSING_TIMEOUT_MS,
      ROBOT_S4_FINAL_RECOVERY_REVERSE_RPM,
      (unsigned long)ROBOT_S4_FINAL_RECOVERY_REVERSE_MS);
  (void)DebugUart_Logf(
      "[S4-24R] zone=%u team=%u safe_field=%ld side1_field=%ld side2_field=%ld (cdeg)\r\n",
      (unsigned int)mission_snapshot.start_zone,
      (unsigned int)mission_snapshot.team,
      (long)MissionTask_WrapYaw(mission_snapshot.safe_zone_yaw_target_cdeg -
                               mission_snapshot.yaw_start_cdeg),
      (long)MissionTask_WrapYaw(s4_final_first_yaw_cdeg - mission_snapshot.yaw_start_cdeg),
      (long)MissionTask_WrapYaw(s4_final_second_yaw_cdeg - mission_snapshot.yaw_start_cdeg));
}

static bool MissionTask_TryResumeS4FinalRecovery(uint32_t now)
{
  uint8_t event_code;

  if (MaixCam_TakeEvent(&event_code))
  {
    if (event_code == MAIXCAM_EVENT_FINAL_OBJECT_IN_FRAME)
    {
      MissionTask_EnterS4FinalCenter(now);
      return true;
    }
    (void)DebugUart_Logf(
        "[S4-24R] RX EVENT %02X, EXPECT 34 OR COORDS\r\n",
        (unsigned int)event_code);
  }

  if (!mission_snapshot.vision_target_valid)
  {
    return false;
  }

  MissionTask_StopWheels();
  MissionTask_ResetVisionPid();
  s4_final_missing_since = 0U;
  s4_next_debug_tick = now;
  MissionTask_EnterState(MISSION_STATE_S4_TRACK_FINAL_BLOCK, now);
  (void)DebugUart_Logf(
      "[S4-24R] TARGET RECOVERED x=%d y=%d, RESUME 24 TRACK\r\n",
      (int)mission_snapshot.vision_x_error_px,
      (int)mission_snapshot.vision_y_error_px);
  return true;
}

static bool MissionTask_RunS4ForcedLeftTurn(uint32_t now,
                                             int32_t target_yaw_cdeg)
{
  int32_t yaw_error = MissionTask_YawError(
      target_yaw_cdeg, mission_snapshot.yaw_cdeg);
  int32_t absolute_error = MissionTask_Abs32(yaw_error);
  int16_t turn_rpm;

  mission_snapshot.safe_zone_yaw_command_cdeg = target_yaw_cdeg;
  mission_snapshot.safe_zone_yaw_error_cdeg = yaw_error;
  mission_snapshot.vision_forward_rpm = 0;

  if ((uint32_t)(now - mission_snapshot.state_entry_tick) >=
      ROBOT_S4_FINAL_RECOVERY_TURN_TIMEOUT_MS)
  {
    mission_snapshot.fault_flags |= MISSION_FAULT_S4_TIMEOUT;
    MissionTask_EnterState(MISSION_STATE_FAULT, now);
    return false;
  }

  if (absolute_error <= ROBOT_S6_TURN_TOLERANCE_CDEG)
  {
    MissionTask_StopWheels();
    mission_snapshot.vision_turn_rpm = 0;
    if (s4_final_turn_stable_since == 0U)
    {
      s4_final_turn_stable_since = now;
    }
    else if ((uint32_t)(now - s4_final_turn_stable_since) >=
             ROBOT_S6_TURN_STABLE_MS)
    {
      return true;
    }
  }
  else
  {
    s4_final_turn_stable_since = 0U;
    turn_rpm = absolute_error >
                       ROBOT_S4_FINAL_RECOVERY_TURN_SLOW_CDEG
                   ? ROBOT_S4_FINAL_RECOVERY_TURN_FAST_RPM
                   : ROBOT_S4_FINAL_RECOVERY_TURN_SLOW_RPM;
    mission_snapshot.vision_turn_rpm = -turn_rpm;
    MissionTask_SetWheelTargets(-turn_rpm, turn_rpm);
  }

  if ((int32_t)(now - s4_next_debug_tick) >= 0)
  {
    s4_next_debug_tick = now + ROBOT_S4_DEBUG_PERIOD_MS;
    (void)DebugUart_Logf(
        "[S4-24R] LEFT yaw=%ld target=%ld err=%ld cmd=%d/%d\r\n",
        (long)MissionTask_WrapYaw(mission_snapshot.yaw_cdeg),
        (long)target_yaw_cdeg,
        (long)yaw_error,
        (int)mission_snapshot.left_target_rpm,
        (int)mission_snapshot.right_target_rpm);
  }
  return false;
}

static void MissionTask_ResumeS4AfterE4(uint32_t now)
{
  MissionTask_StopWheels();
  MissionTask_ResetVisionControllerState();
  MissionTask_ResetS6ControllerState();
  MaixCam_ClearEvent();
  s4_center_search_sweep_completed = false;
  s4_center_target_lost_since = mission_snapshot.vision_target_valid ? 0U : now;
  s4_next_debug_tick = now;
  (void)MissionTask_ReturnToS4TrackCenter(now);
}

static void MissionTask_StartS4E4Spin(uint32_t now, bool red_search)
{
  MissionTask_StopWheels();
  MissionTask_ResetVisionControllerState();
  MaixCam_ClearEvent();
  MaixCam_ClearObject();
  mission_snapshot.vision_target_valid = false;
  s4_e4_spin_origin_yaw_cdeg =
      MissionTask_WrapYaw(mission_snapshot.yaw_cdeg);
  s4_e4_spin_previous_yaw_cdeg = s4_e4_spin_origin_yaw_cdeg;
  s4_e4_spin_progress_cdeg = 0L;
  s4_next_debug_tick = now;
  if (red_search)
  {
    MissionTask_EnterState(MISSION_STATE_S4_E4_RED_SPIN_360, now);
    (void)DebugUart_Logf(
        "[S4-E4-RED360] START CCW360 yaw=%ld rpm=%d, RED COORDINATES INTERRUPT; NO TARGET THEN TX51/03\r\n",
        (long)s4_e4_spin_origin_yaw_cdeg,
        ROBOT_S4_E4_RED_SPIN_FAST_RPM);
    return;
  }
  MissionTask_EnterState(MISSION_STATE_S4_E4_GREEN_SPIN_360, now);
  (void)DebugUart_Logf(
      "[S4-E4-360] START after green push cycle=%u/%u yaw=%ld, wait fresh coordinates\r\n",
      (unsigned int)s4_green_e4_push_cycles,
      (unsigned int)ROBOT_S4_E4_GREEN_PUSH_MAX_CYCLES,
      (long)s4_e4_spin_origin_yaw_cdeg);
}

static void MissionTask_RunS4E4Spin(uint32_t now)
{
  uint8_t event_code;
  int16_t turn_rpm;
  bool red_search = mission_snapshot.state == MISSION_STATE_S4_E4_RED_SPIN_360;
  const char *log_tag = red_search ? "S4-E4-RED360" : "S4-E4-360";
  int32_t sweep_cdeg = red_search ? ROBOT_S4_E4_RED_SPIN_CDEG
                                  : ROBOT_S4_E4_GREEN_SPIN_CDEG;
  uint32_t timeout_ms = red_search ? ROBOT_S4_E4_RED_SPIN_TIMEOUT_MS
                                   : ROBOT_S4_E4_GREEN_SPIN_TIMEOUT_MS;
  int32_t current_yaw_cdeg = MissionTask_WrapYaw(mission_snapshot.yaw_cdeg);

  if (MaixCam_TakeEvent(&event_code) &&
      event_code == MAIXCAM_EVENT_CENTER_REACHED)
  {
    s4_green_e4_spin_exhausted = false;
    (void)DebugUart_Logf("[%s] RX14, STOP SEARCH AND ACK\r\n", log_tag);
    MissionTask_AckS4Center(now);
    return;
  }
  if (mission_snapshot.vision_target_valid &&
      (!red_search || mission_snapshot.target_id == 4U))
  {
    s4_green_e4_spin_exhausted = false;
    (void)DebugUart_Logf(
        "[%s] TARGET FOUND x=%d y=%d, RESUME04\r\n",
        log_tag,
        (int)mission_snapshot.vision_x_error_px,
        (int)mission_snapshot.vision_y_error_px);
    MissionTask_ResumeS4AfterE4(now);
    return;
  }

  s4_e4_spin_progress_cdeg += (int32_t)ROBOT_YAW_LEFT_SIGN *
      MissionTask_YawError(current_yaw_cdeg,
                           s4_e4_spin_previous_yaw_cdeg);
  s4_e4_spin_previous_yaw_cdeg = current_yaw_cdeg;
  if (s4_e4_spin_progress_cdeg < 0L)
  {
    s4_e4_spin_progress_cdeg = 0L;
  }
  if (s4_e4_spin_progress_cdeg >= sweep_cdeg)
  {
    MissionTask_StopWheels();
    if (red_search)
    {
      s3_target_select_command = MAIXCAM_COMMAND_SELECT_BLACK_GREEN;
      (void)DebugUart_Logf(
          "[S4-E4-RED360] DONE progress=%ld, NO RED TARGET; SELECT51 BLACK/GREEN AND SEARCH03\r\n",
          (long)s4_e4_spin_progress_cdeg);
      (void)MissionTask_BeginS3Vision(now);
      return;
    }
    s4_green_e4_spin_exhausted = true;
    (void)DebugUart_Logf(
        "[S4-E4-360] DONE progress=%ld, NO TARGET; WAIT COORDINATES WITHOUT MORE PUSHES\r\n",
        (long)s4_e4_spin_progress_cdeg);
    MissionTask_ResumeS4AfterE4(now);
    return;
  }
  if ((uint32_t)(now - mission_snapshot.state_entry_tick) >=
      timeout_ms)
  {
    mission_snapshot.fault_flags |= MISSION_FAULT_S4_TIMEOUT;
    MissionTask_EnterState(MISSION_STATE_FAULT, now);
    (void)DebugUart_Logf("[%s] TIMEOUT progress=%ld\r\n", log_tag,
                         (long)s4_e4_spin_progress_cdeg);
    return;
  }

  turn_rpm = sweep_cdeg - s4_e4_spin_progress_cdeg >
                 ROBOT_S3_SEARCH_TURN_SLOW_THRESHOLD_CDEG
                 ? (red_search ? ROBOT_S4_E4_RED_SPIN_FAST_RPM
                               : ROBOT_S4_E4_GREEN_SPIN_FAST_RPM)
                 : (red_search ? ROBOT_S4_E4_RED_SPIN_SLOW_RPM
                               : ROBOT_S4_E4_GREEN_SPIN_SLOW_RPM);
  mission_snapshot.vision_forward_rpm = 0;
  mission_snapshot.vision_turn_rpm =
      (int16_t)(-ROBOT_YAW_LEFT_SIGN * turn_rpm);
  MissionTask_SetWheelTargets((int16_t)(-ROBOT_YAW_LEFT_SIGN * turn_rpm),
                              (int16_t)(ROBOT_YAW_LEFT_SIGN * turn_rpm));
  if ((int32_t)(now - s4_next_debug_tick) >= 0)
  {
    s4_next_debug_tick = now + ROBOT_ANGLE_DEBUG_PERIOD_MS;
    (void)DebugUart_Logf(
        "[%s] progress=%ld/%ld yaw=%ld rpm=%d\r\n",
        log_tag, (long)s4_e4_spin_progress_cdeg, (long)sweep_cdeg,
        (long)current_yaw_cdeg, (int)turn_rpm);
  }
}

static void MissionTask_StartS4E4Recovery(uint32_t now)
{
  if (MissionTask_ActiveRecoveryLossEvent() != 0U) return;
  s4_e4_protocol_active = true;
  MissionTask_StopWheels();
  MissionTask_ResetVisionControllerState();
  MaixCam_ClearObject();
  mission_snapshot.vision_target_valid = false;
  if (!s7_target_switch_enabled ||
      s3_target_select_command == MAIXCAM_COMMAND_SELECT_GREEN)
  {
    if (s4_green_e4_push_cycles >= ROBOT_S4_E4_GREEN_PUSH_MAX_CYCLES)
    {
      if (!s4_green_e4_spin_exhausted)
      {
        MissionTask_StartS4E4Spin(now, false);
      }
      else
      {
        /* An E4 reply to 14/44 must also leave the handshake wait after the
           recovery allowance is exhausted. Keep the existing no-more-pushes
           policy and wait for coordinates in 04 instead. */
        MissionTask_ResumeS4AfterE4(now);
        (void)DebugUart_Log(
            "[S4-E4] GREEN RECOVERY LIMIT REACHED, WAIT04 COORDINATES WITHOUT MORE PUSHES\r\n");
      }
      return;
    }
    ++s4_green_e4_push_cycles;
    /* Initial green: complete both timed pushes even if coordinates arrive. */
    MissionTask_EnterState(MISSION_STATE_S4_E4_PUSH_RIGHT, now);
    (void)DebugUart_Logf(
        "[S4-E4] INITIAL GREEN PUSH cycle=%u/%u, RIGHT/BACK/LEFT/BACK THEN TX44\r\n",
        (unsigned int)s4_green_e4_push_cycles,
        (unsigned int)ROBOT_S4_E4_GREEN_PUSH_MAX_CYCLES);
    return;
  }
  if (s7_target_switch_enabled && s4_e4_recovery_used &&
      (s3_target_select_command != MAIXCAM_COMMAND_SELECT_BLACK_GREEN))
  {
    s3_target_select_command = MAIXCAM_COMMAND_SELECT_BLACK_GREEN;
    (void)DebugUart_Log("[S4-E4] SECOND LOSS, SELECT51 BLACK/GREEN AND SEARCH03\r\n");
    (void)MissionTask_BeginS3Vision(now);
    return;
  }
  s4_e4_recovery_used = true;
  if (s7_target_switch_enabled &&
      s3_target_select_command == MAIXCAM_COMMAND_SELECT_RED)
  {
    MissionTask_StartS4E4Spin(now, true);
    return;
  }
  s4_e4_away_yaw_cdeg = MissionTask_WrapYaw(
      mission_snapshot.safe_zone_yaw_target_cdeg + 18000L);
  s4_e4_left_yaw_cdeg = MissionTask_WrapYaw(
      s4_e4_away_yaw_cdeg + (int32_t)ROBOT_YAW_LEFT_SIGN *
          ROBOT_S4_E4_LEFT_CDEG);
  s4_e4_right_yaw_cdeg = MissionTask_WrapYaw(
      s4_e4_left_yaw_cdeg - (int32_t)ROBOT_YAW_LEFT_SIGN *
          ROBOT_S4_E4_RIGHT_CDEG);
  MissionTask_ResetS6ControllerState();
  s6_phase_stable_since = 0U;
  MissionTask_EnterState(MISSION_STATE_S4_E4_TURN_AWAY, now);
  (void)DebugUart_Logf(
      "[S4-E4] TURN OPPOSITE SAFE raw=%ld field=%ld, FORWARD80/600ms LEFT45 RIGHT90, COORDINATES INTERRUPT\r\n",
      (long)s4_e4_away_yaw_cdeg,
      (long)MissionTask_WrapYaw(s4_e4_away_yaw_cdeg - mission_snapshot.yaw_start_cdeg));
}

static void MissionTask_RunS4E4Recovery(uint32_t now)
{
  uint32_t elapsed = (uint32_t)(now - mission_snapshot.state_entry_tick);
  MissionState state = mission_snapshot.state;
  bool empty_recovery = s5_empty_recovery_active;

  if ((state == MISSION_STATE_S4_E4_GREEN_SPIN_360) ||
      (state == MISSION_STATE_S4_E4_RED_SPIN_360))
  {
    MissionTask_RunS4E4Spin(now);
    return;
  }
  if ((state < MISSION_STATE_S4_E4_PUSH_RIGHT) ||
      (state > MISSION_STATE_S4_E4_TURN_RIGHT))
  {
    return;
  }
  /* Discard repeated E4/other old events throughout the maneuver. */
  MaixCam_ClearEvent();
  if ((state >= MISSION_STATE_S4_E4_TURN_AWAY) &&
      mission_snapshot.vision_target_valid)
  {
    (void)DebugUart_Log("[S4-E4] COORDINATES FOUND, RESUME04\r\n");
    MissionTask_ResumeS4AfterE4(now);
    return;
  }
  switch (state)
  {
    case MISSION_STATE_S4_E4_PUSH_RIGHT:
      if (elapsed < MissionTask_S4ScalePushDuration(ROBOT_S4_RIGHT_CORNER_FOLLOW_MS, true))
      {
        MissionTask_CommandS4CornerPush(now, true);
        return;
      }
      MissionTask_EnterState(MISSION_STATE_S4_E4_REVERSE_RIGHT, now);
      break;
    case MISSION_STATE_S4_E4_REVERSE_RIGHT:
      if (elapsed < MissionTask_S4ReverseDuration(true))
      {
        MissionTask_CommandS4CornerReverse(now, true);
        return;
      }
      MissionTask_EnterState(MISSION_STATE_S4_E4_PUSH_LEFT, now);
      break;
    case MISSION_STATE_S4_E4_PUSH_LEFT:
      if (elapsed < MissionTask_S4ScalePushDuration(ROBOT_S4_LEFT_CORNER_FOLLOW_MS, false))
      {
        MissionTask_CommandS4CornerPush(now, false);
        return;
      }
      MissionTask_StopWheels();
      MissionTask_EnterState(MISSION_STATE_S4_E4_REVERSE_LEFT, now);
      (void)DebugUart_Logf("[S4-E4] LEFT PUSH DONE, RETRACE BACK12 time=%lums\r\n",
          (unsigned long)MissionTask_S4ReverseDuration(false));
      break;
    case MISSION_STATE_S4_E4_REVERSE_LEFT:
      if (elapsed < MissionTask_S4ReverseDuration(false))
      {
        MissionTask_CommandS4CornerReverse(now, false);
        return;
      }
      MissionTask_StopWheels();
      if (empty_recovery)
      {
        MissionTask_ResetVisionPid();
        MissionTask_ResetS6ControllerState();
        mission_snapshot.target_id = 0U;
        mission_snapshot.target_present = false;
        mission_snapshot.target_age_ms = UINT32_MAX;
      }
      else
      {
        MissionTask_ResetVisionControllerState();
      }
      MaixCam_ClearObject();
      mission_snapshot.vision_target_valid = false;
      /* TX44 makes the camera reply with 02, 24 or E4. Enter the
         receiving state before TX44 and preserve events arriving during TX. */
      MissionTask_EnterState(MISSION_STATE_S4_WAIT_ARRANGE_READY, now);
      if (empty_recovery)
      {
        /* Clear as the last step before TX44; do not clear after TX, since
           the camera's immediate02/24 reply belongs to the new handshake. */
        MaixCam_ClearPendingInput();
      }
      if (MaixCam_SendCommand(MAIXCAM_COMMAND_CENTER_RECOVERY_DONE) != HAL_OK)
      {
        mission_snapshot.fault_flags |= MISSION_FAULT_VISION_TX;
        MissionTask_EnterState(MISSION_STATE_FAULT, now);
        return;
      }
      (void)DebugUart_Log(empty_recovery
          ? "[S5-E5] OLD INPUT/PID CLEARED, TX E1 E2 44 1E 2E, WAIT RX02/24/E4\r\n"
          : "[S4-E4] PUSHES DONE, TX E1 E2 44 1E 2E, WAIT RX02/24/E4\r\n");
      break;
    case MISSION_STATE_S4_E4_TURN_AWAY:
      if (!MissionTask_RunS6TurnToHeading(now, s4_e4_away_yaw_cdeg))
      {
        return;
      }
      MissionTask_ResetS6ControllerState();
      s6_phase_stable_since = 0U;
      MissionTask_EnterState(MISSION_STATE_S4_E4_FORWARD, now);
      (void)DebugUart_Log("[S4-E4] OPPOSITE SAFE ALIGNED, START FORWARD\r\n");
      break;
    case MISSION_STATE_S4_E4_FORWARD:
      if (elapsed < ROBOT_S4_E4_FORWARD_MS)
      {
        MissionTask_CommandS6Straight(now, s4_e4_away_yaw_cdeg,
                                     ROBOT_S4_E4_FORWARD_RPM,
                                     ROBOT_S4_E4_FORWARD_WHEEL_MAX_RPM);
        return;
      }
      MissionTask_ResetS6ControllerState();
      s6_phase_stable_since = 0U;
      MissionTask_EnterState(MISSION_STATE_S4_E4_TURN_LEFT, now);
      break;
    case MISSION_STATE_S4_E4_TURN_LEFT:
      if (!MissionTask_RunS6TurnToHeading(now, s4_e4_left_yaw_cdeg))
      {
        return;
      }
      MissionTask_ResetS6ControllerState();
      s6_phase_stable_since = 0U;
      MissionTask_EnterState(MISSION_STATE_S4_E4_TURN_RIGHT, now);
      break;
    case MISSION_STATE_S4_E4_TURN_RIGHT:
      if (MissionTask_RunS6TurnToHeading(now, s4_e4_right_yaw_cdeg))
      {
        MissionTask_ResumeS4AfterE4(now);
        (void)DebugUart_Log("[S4-E4] SWEEP DONE, WAIT04 COORDINATES OR NEXT E4\r\n");
      }
      break;
    default:
      break;
  }
}

static void MissionTask_StartS4Post22Recovery(uint32_t now)
{
  MissionTask_StopWheels();
  MissionTask_ResetVisionControllerState();
  MissionTask_ResetS6ControllerState();
  MaixCam_ClearObject();
  mission_snapshot.vision_target_valid = false;
  s4_post22_left_yaw_cdeg = MissionTask_WrapYaw(
      mission_snapshot.yaw_cdeg + (int32_t)ROBOT_YAW_LEFT_SIGN *
          ROBOT_S4_POST22_LEFT_CDEG);
  s4_post22_right_yaw_cdeg = MissionTask_WrapYaw(
      s4_post22_left_yaw_cdeg - (int32_t)ROBOT_YAW_LEFT_SIGN *
          ROBOT_S4_POST22_RIGHT_CDEG);
  s6_phase_stable_since = 0U;
  MissionTask_EnterState(MISSION_STATE_S4_POST22_REVERSE, now);
  (void)DebugUart_Logf(
      "[S4-22E2] REVERSE rpm=%d time=%lums LEFT45=%ld RIGHT90=%ld, %s CAN INTERRUPT\r\n",
      ROBOT_S4_POST22_REVERSE_RPM,
      (unsigned long)ROBOT_S4_POST22_REVERSE_MS,
      (long)s4_post22_left_yaw_cdeg, (long)s4_post22_right_yaw_cdeg,
      s4_post22_search_active ? "RX04/24 OR 03 COORDINATES" : "RX04/24");
}

static void MissionTask_ResumeS3FromPost22Search(uint32_t now)
{
  MissionTask_StopWheels();
  MissionTask_ResetVisionControllerState();
  MissionTask_ResetS6ControllerState();
  /* Keep the fresh error and a possibly queued 04 handshake for S3. */
  s4_post22_search_active = false;
  s4_e4_protocol_active = false;
  s3_track_stable_since = 0U;
  s3_search_sweep_completed = false;
  s3_search_origin_yaw_cdeg = MissionTask_WrapYaw(mission_snapshot.yaw_cdeg);
  vision_search_resume_state = MISSION_STATE_S3_TRACK_GREEN;
  s3_target_lost_since = 0U;
  s3_wait_event04_logged = false;
  s3_next_debug_tick = now;
  MissionTask_EnterState(MISSION_STATE_S3_TRACK_GREEN, now);
  (void)DebugUart_Logf(
      "[S4-22E2] 03 TARGET FOUND id=%u x=%d y=%d, STOP RECOVERY AND TRACK03\r\n",
      (unsigned int)mission_snapshot.target_id,
      (int)mission_snapshot.vision_x_error_px,
      (int)mission_snapshot.vision_y_error_px);
}

static bool MissionTask_StartPost22BlackGreenSearch(uint32_t now)
{
  MissionTask_StopWheels();
  MaixCam_ClearEvent();
  MaixCam_ClearObject();
  mission_snapshot.vision_target_valid = false;
  if ((MaixCam_SendCommand(MAIXCAM_COMMAND_SELECT_BLACK_GREEN) != HAL_OK) ||
      (MaixCam_SendCommand(MAIXCAM_COMMAND_SEARCH_TARGET) != HAL_OK))
  {
    mission_snapshot.fault_flags |= MISSION_FAULT_VISION_TX;
    MissionTask_EnterState(MISSION_STATE_FAULT, now);
    return false;
  }
  s3_target_select_command = MAIXCAM_COMMAND_SELECT_BLACK_GREEN;
  s4_post22_search_active = true;
  (void)DebugUart_Log("[S4-22E2] TX51 THEN TX03, SEARCH BLACK/GREEN DURING RECOVERY\r\n");
  return true;
}

static void MissionTask_RunS4Post22Recovery(uint32_t now)
{
  uint8_t event_code;
  uint32_t elapsed = (uint32_t)(now - mission_snapshot.state_entry_tick);
  if ((mission_snapshot.state < MISSION_STATE_S4_POST22_REVERSE) ||
      (mission_snapshot.state > MISSION_STATE_S4_POST22_TURN_RIGHT))
  {
    return;
  }
  if (MaixCam_TakeEvent(&event_code))
  {
    if (event_code == MAIXCAM_EVENT_SKIP_TO_FINAL_TRACK)
    {
      MissionTask_StopWheels();
      MissionTask_ResetS6ControllerState();
      s4_post22_search_active = false;
      (void)DebugUart_Log("[S4-22E2] RX24, STOP RECOVERY AND RAISE FRAME\r\n");
      MissionTask_SkipToS4FinalTrack(now);
      return;
    }
    if (event_code == MAIXCAM_EVENT_OBJECT_IN_FRAME)
    {
      MissionTask_StopWheels();
      MissionTask_ResetS6ControllerState();
      s4_post22_search_active = false;
      (void)DebugUart_Log("[S4-22E2] RX04, STOP RECOVERY AND TRACK CENTER\r\n");
      MissionTask_StartS4TrackCenterFrom04(now);
      return;
    }
    /* Repeated E2 must not restart a running recovery. */
    if (event_code != MAIXCAM_EVENT_ARRANGE_TARGET_LOST)
    {
      (void)DebugUart_Logf("[S4-22E2] RX%02X, WAIT04/24\r\n",
                           (unsigned int)event_code);
    }
  }
  if (s4_post22_search_active && mission_snapshot.vision_target_valid &&
      ((mission_snapshot.target_id == 5U) ||
       (mission_snapshot.target_id == 6U)))
  {
    MissionTask_ResumeS3FromPost22Search(now);
    return;
  }
  switch (mission_snapshot.state)
  {
    case MISSION_STATE_S4_POST22_REVERSE:
      if (elapsed < ROBOT_S4_POST22_REVERSE_MS)
      {
        MissionTask_SetWheelTargets(-ROBOT_S4_POST22_REVERSE_RPM,
                                    -ROBOT_S4_POST22_REVERSE_RPM);
        return;
      }
      MissionTask_ResetS6ControllerState();
      s6_phase_stable_since = 0U;
      MissionTask_EnterState(MISSION_STATE_S4_POST22_TURN_LEFT, now);
      break;
    case MISSION_STATE_S4_POST22_TURN_LEFT:
      if (!MissionTask_RunS6TurnToHeading(now, s4_post22_left_yaw_cdeg))
      {
        return;
      }
      MissionTask_ResetS6ControllerState();
      s6_phase_stable_since = 0U;
      MissionTask_EnterState(MISSION_STATE_S4_POST22_TURN_RIGHT, now);
      break;
    case MISSION_STATE_S4_POST22_TURN_RIGHT:
      if (MissionTask_RunS6TurnToHeading(now, s4_post22_right_yaw_cdeg))
      {
        MissionTask_StopWheels();
        MissionTask_ResetS6ControllerState();
        MissionTask_EnterState(MISSION_STATE_S4_WAIT_FINAL_READY, now);
        (void)DebugUart_Log(s4_post22_search_active
            ? "[S4-22E2] RECOVERY DONE, FRAME DOWN WAIT03 TARGET\r\n"
            : "[S4-22E2] RECOVERY DONE, FRAME DOWN WAIT04/24 OR NEXT E2\r\n");
      }
      break;
    default:
      break;
  }
}

static void MissionTask_RunS4Arrange(uint32_t now)
{
  uint32_t elapsed = (uint32_t)(now - mission_snapshot.state_entry_tick);
  uint8_t event_code;
  bool xy_in_tolerance;
  bool camera_already_in_final_mode;

  switch (mission_snapshot.state)
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
            "[S4] RX EVENT %02X, EXPECT 14\r\n",
            (unsigned int)event_code);
      }
      if (!mission_snapshot.vision_target_valid)
      {
        MissionTask_StopWheels();
        if (s4_center_target_lost_since == 0U)
        {
          s4_center_target_lost_since = now;
        }
        if (!s4_e4_protocol_active && !s4_center_search_sweep_completed &&
            ((uint32_t)(now - s4_center_target_lost_since) >=
             ROBOT_S3_GREEN_WAIT_MS))
        {
          (void)DebugUart_Logf(
              "[S4] TARGET LOST %lums, START SAME RECOVERY AS 03\r\n",
              (unsigned long)ROBOT_S3_GREEN_WAIT_MS);
          MissionTask_StartVisionSearchRecovery(
              now, MISSION_STATE_S4_TRACK_CENTER, mission_snapshot.yaw_cdeg,
              false);
          return;
        }
        (void)MissionTask_CommandS4VisionTracking(now, &xy_in_tolerance);
        break;
      }
      else
      {
        s4_center_target_lost_since = 0U;
        s4_center_search_sweep_completed = false;
        s4_green_e4_spin_exhausted = false;
      }
      if (elapsed >= ROBOT_S4_TRACK_TIMEOUT_MS)
      {
        mission_snapshot.fault_flags |= MISSION_FAULT_S4_TIMEOUT;
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
        else if (event_code == MAIXCAM_EVENT_ARRANGE_READY)
        {
          MissionTask_StartS4TrackState(
              MISSION_STATE_S4_TRACK_RIGHT_BLOCK, now);
          (void)DebugUart_Log(
              "[S4] RX02, TRACK RIGHTMOST TO LEFT FRAME CORNER\r\n");
          return;
        }
        else if (event_code == MAIXCAM_EVENT_CENTER_TARGET_LOST)
        {
          /* MaixCAM owns the reliable-target observation after 14/44. Even
             fresh coordinates here do not override its E4 decision. */
          (void)DebugUart_Log(
              "[S4] RX E4 WHILE WAITING02/24 AFTER TX14/44, START RECOVERY\r\n");
          MissionTask_StartS4E4Recovery(now);
          return;
        }
        (void)DebugUart_Logf(
            "[S4] RX EVENT %02X, EXPECT 02/24/E4\r\n",
            (unsigned int)event_code);
      }
      if (elapsed >= ROBOT_S4_HANDSHAKE_TIMEOUT_MS)
      {
        mission_snapshot.fault_flags |= MISSION_FAULT_S4_TIMEOUT;
        MissionTask_EnterState(MISSION_STATE_FAULT, now);
        (void)DebugUart_Log("[S4] WAIT RX02/24/E4 AFTER TX14/44 TIMEOUT\r\n");
      }
      break;

    case MISSION_STATE_S4_TRACK_RIGHT_BLOCK:
      if (MissionTask_TryS4ArrangeEvent(
              now, MISSION_STATE_S4_TRACK_RIGHT_BLOCK))
      {
        return;
      }
      if (elapsed < MissionTask_S4PushDuration(true))
      {
        MissionTask_CommandS4CornerPush(now, true);
        return;
      }
      MissionTask_ResetVisionPid();
      MissionTask_EnterState(MISSION_STATE_S4_REVERSE_RIGHT_BLOCK, now);
      (void)DebugUart_Logf(
          "[S4] RIGHT X-FOLLOW DONE, REVERSE rpm=%d time=%lums\r\n",
          ROBOT_S4_RIGHT_REVERSE_RPM,
          (unsigned long)MissionTask_S4ReverseDuration(true));
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
        mission_snapshot.fault_flags |= MISSION_FAULT_VISION_TX;
        MissionTask_EnterState(MISSION_STATE_FAULT, now);
        return;
      }
      (void)DebugUart_Log(
          "[S4] TX E1 E2 12 1E 2E, WAIT RX12 LEFT TARGET READY\r\n");
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
              "[S4] RX12, TRACK LEFTMOST TO RIGHT FRAME CORNER\r\n");
          return;
        }
        (void)DebugUart_Logf(
            "[S4] RX EVENT %02X, EXPECT 12 OR 24\r\n",
            (unsigned int)event_code);
      }
      if (elapsed >= ROBOT_S4_HANDSHAKE_TIMEOUT_MS)
      {
        mission_snapshot.fault_flags |= MISSION_FAULT_S4_TIMEOUT;
        MissionTask_EnterState(MISSION_STATE_FAULT, now);
      }
      break;

    case MISSION_STATE_S4_TRACK_LEFT_BLOCK:
      if (MissionTask_TryS4ArrangeEvent(
              now, MISSION_STATE_S4_TRACK_LEFT_BLOCK))
      {
        return;
      }
      if (elapsed < MissionTask_S4PushDuration(false))
      {
        MissionTask_CommandS4CornerPush(now, false);
        return;
      }
      MissionTask_ResetVisionPid();
      MissionTask_StopWheels();
      MissionTask_EnterState(MISSION_STATE_S4_REVERSE_LEFT_BLOCK, now);
      (void)DebugUart_Logf("[S4] LEFT PUSH DONE, RETRACE BACK12 rpm=%d time=%lums\r\n",
          ROBOT_S4_LEFT_REVERSE_RPM,
          (unsigned long)MissionTask_S4ReverseDuration(false));
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
      mission_snapshot.vision_target_valid = false;
      MissionTask_EnterState(MISSION_STATE_S4_WAIT_FINAL_READY, now);
      if (MaixCam_SendCommand(MAIXCAM_COMMAND_LEFT_DONE_ACK) != HAL_OK)
      {
        mission_snapshot.fault_flags |= MISSION_FAULT_VISION_TX;
        MissionTask_EnterState(MISSION_STATE_FAULT, now);
        return;
      }
      (void)DebugUart_Log(
          "[S4] 12 COMPLETE, TX E1 E2 22 1E 2E, FRAME DOWN WAIT RX04/14/24/E4/E2\r\n");
      break;

    case MISSION_STATE_S4_WAIT_FINAL_READY:
      MissionTask_StopWheels();
      if (MaixCam_TakeEvent(&event_code))
      {
        if (event_code == MAIXCAM_EVENT_OBJECT_IN_FRAME)
        {
          s4_post22_search_active = false;
          MissionTask_StartS4TrackCenterFrom04(now);
          return;
        }
        if (event_code == MAIXCAM_EVENT_SKIP_TO_FINAL_TRACK)
        {
          s4_post22_search_active = false;
          MissionTask_SkipToS4FinalTrack(now);
          return;
        }
        if (!s4_post22_search_active)
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
            if (s7_target_switch_enabled &&
                !MissionTask_StartPost22BlackGreenSearch(now))
            {
              return;
            }
            MissionTask_StartS4Post22Recovery(now);
            return;
          }
        }
        (void)DebugUart_Logf(
            "[S4] RX EVENT %02X, WAIT RX04/14/24/E4/E2 AFTER TX22\r\n",
            (unsigned int)event_code);
      }
      if (s4_post22_search_active && mission_snapshot.vision_target_valid &&
          ((mission_snapshot.target_id == 5U) ||
           (mission_snapshot.target_id == 6U)))
      {
        MissionTask_ResumeS3FromPost22Search(now);
        return;
      }
      if (elapsed >= ROBOT_S4_HANDSHAKE_TIMEOUT_MS)
      {
        if (s4_post22_search_active || !s7_target_switch_enabled)
        {
          MissionTask_StartS4Post22Recovery(now);
          return;
        }
        mission_snapshot.fault_flags |= MISSION_FAULT_S4_TIMEOUT;
        MissionTask_EnterState(MISSION_STATE_FAULT, now);
        (void)DebugUart_Log("[S4] WAIT RX04/14/24/E4/E2 AFTER TX22 TIMEOUT\r\n");
      }
      break;

    case MISSION_STATE_S4_ARRANGE_RECOVERY_REVERSE:
      if (MissionTask_TryResumeS4ArrangeRecovery(now))
      {
        return;
      }
      if (elapsed < ROBOT_S4_ARRANGE_RECOVERY_REVERSE_MS)
      {
        MissionTask_SetWheelTargets(
            -ROBOT_S4_ARRANGE_RECOVERY_REVERSE_RPM,
            -ROBOT_S4_ARRANGE_RECOVERY_REVERSE_RPM);
        return;
      }
      MissionTask_StopWheels();
      MissionTask_ResetS6ControllerState();
      s6_phase_stable_since = 0U;
      s4_next_debug_tick = now;
      MissionTask_EnterState(
          MISSION_STATE_S4_ARRANGE_RECOVERY_TURN_LEFT, now);
      (void)DebugUart_Logf(
          "[S4-E2] REVERSE DONE, LEFT45 target=%ld\r\n",
          (long)s4_arrange_recovery_left_yaw_cdeg);
      break;

    case MISSION_STATE_S4_ARRANGE_RECOVERY_TURN_LEFT:
      if (MissionTask_TryResumeS4ArrangeRecovery(now))
      {
        return;
      }
      if (MissionTask_RunS6TurnToHeading(
              now, s4_arrange_recovery_left_yaw_cdeg))
      {
        MissionTask_ResetS6ControllerState();
        s6_phase_stable_since = 0U;
        s4_next_debug_tick = now;
        MissionTask_EnterState(
            MISSION_STATE_S4_ARRANGE_RECOVERY_TURN_BACK, now);
        (void)DebugUart_Logf(
            "[S4-E2] LEFT45 DONE, RIGHT90 target=%ld\r\n",
            (long)s4_arrange_recovery_right_yaw_cdeg);
      }
      break;

    case MISSION_STATE_S4_ARRANGE_RECOVERY_TURN_BACK:
      if (MissionTask_TryResumeS4ArrangeRecovery(now))
      {
        return;
      }
      if (!MissionTask_RunS6TurnToHeading(
              now, s4_arrange_recovery_right_yaw_cdeg))
      {
        return;
      }
      (void)DebugUart_Log(
          "[S4-E2] SWEEP DONE, TARGET STILL LOST, REPEAT RECOVERY\r\n");
      MissionTask_StartS4ArrangeRecovery(
          now, s4_arrange_recovery_resume_state);
      break;

    case MISSION_STATE_S4_RAISE_FRAME:
      /* When MaixCam initiated the 24 transition it may report 34 before the
         frame has finished rising. Latch that one-shot event instead of
         clearing it at the end of the actuator settle delay. */
      if (s4_final_mode_already_active &&
          MaixCam_TakeEvent(&event_code))
      {
        if (event_code == MAIXCAM_EVENT_FINAL_OBJECT_IN_FRAME)
        {
          if (!s4_final_event34_pending)
          {
            (void)DebugUart_Log(
                "[S4] RX34 DURING FRAME UP, LATCH UNTIL SETTLED\r\n");
          }
          s4_final_event34_pending = true;
        }
        else
        {
          (void)DebugUart_Logf(
              "[S4] RX EVENT %02X DURING FRAME UP, EXPECT 34\r\n",
              (unsigned int)event_code);
        }
      }
#if ROBOT_S4_FRAME_RAISE_SETTLE_MS > 0U
      if (elapsed < ROBOT_S4_FRAME_RAISE_SETTLE_MS)
      {
        return;
      }
#endif
      camera_already_in_final_mode = s4_final_mode_already_active;
      s4_final_mode_already_active = false;
      if (camera_already_in_final_mode && s4_final_event34_pending)
      {
        s4_final_event34_pending = false;
        (void)DebugUart_Log(
            "[S4] FRAME UP, APPLY LATCHED RX34\r\n");
        MissionTask_EnterS4FinalCenter(now);
        return;
      }
      s4_final_event34_pending = false;
      if (camera_already_in_final_mode)
      {
        /* SkipToS4FinalTrack cleared the old object at entry, so any object
           received during the raise delay is already valid 24-mode data. */
        MissionTask_ResetVisionControllerState();
        s4_next_debug_tick = now;
        MissionTask_EnterState(MISSION_STATE_S4_TRACK_FINAL_BLOCK, now);
        s4_final_missing_since =
            mission_snapshot.vision_target_valid ? 0U : now;
      }
      else
      {
        MaixCam_ClearEvent();
        MissionTask_StartS4TrackState(
            MISSION_STATE_S4_TRACK_FINAL_BLOCK, now);
        s4_final_missing_since = now;
      }
      if (!camera_already_in_final_mode &&
          (MaixCam_SendCommand(MAIXCAM_COMMAND_FINAL_CAPTURE) != HAL_OK))
      {
        mission_snapshot.fault_flags |= MISSION_FAULT_VISION_TX;
        MissionTask_EnterState(MISSION_STATE_FAULT, now);
        return;
      }
      if (camera_already_in_final_mode)
      {
        (void)DebugUart_Log(
            "[S4] FRAME UP, RX24 MODE ACTIVE, TRACK TARGET TO FRAME CENTER\r\n");
      }
      else
      {
        (void)DebugUart_Log(
            "[S4] TX E1 E2 24 1E 2E, TRACK TARGET TO FRAME CENTER\r\n");
      }
      break;

    case MISSION_STATE_S4_TRACK_FINAL_BLOCK:
      if (MaixCam_TakeEvent(&event_code))
      {
        if (event_code == MAIXCAM_EVENT_FINAL_OBJECT_IN_FRAME)
        {
          MissionTask_EnterS4FinalCenter(now);
          return;
        }
        (void)DebugUart_Logf(
            "[S4] RX EVENT %02X, EXPECT 34\r\n",
            (unsigned int)event_code);
      }
      if (elapsed >= ROBOT_S4_TRACK_TIMEOUT_MS)
      {
        mission_snapshot.fault_flags |= MISSION_FAULT_S4_TIMEOUT;
        MissionTask_EnterState(MISSION_STATE_FAULT, now);
        return;
      }
      if (mission_snapshot.vision_target_valid)
      {
        s4_final_missing_since = 0U;
      }
      else
      {
        if (s4_final_missing_since == 0U)
        {
          s4_final_missing_since = now;
        }
        else if ((uint32_t)(now - s4_final_missing_since) >=
                 ROBOT_S4_FINAL_MISSING_TIMEOUT_MS)
        {
          MissionTask_StartS4FinalRecovery(now);
          return;
        }
      }
      (void)MissionTask_CommandS4VisionTracking(now, &xy_in_tolerance);
      break;

    case MISSION_STATE_S4_FINAL_RECOVERY_REVERSE:
      if (MissionTask_TryResumeS4FinalRecovery(now))
      {
        return;
      }
      if (elapsed < ROBOT_S4_FINAL_RECOVERY_REVERSE_MS)
      {
        MissionTask_SetWheelTargets(
            -ROBOT_S4_FINAL_RECOVERY_REVERSE_RPM,
            -ROBOT_S4_FINAL_RECOVERY_REVERSE_RPM);
        return;
      }
      MissionTask_StopWheels();
      s4_final_turn_stable_since = 0U;
      s4_next_debug_tick = now;
      MissionTask_EnterState(
          MISSION_STATE_S4_FINAL_RECOVERY_TURN_270, now);
      (void)DebugUart_Logf(
          "[S4-24R] REVERSE DONE, FORCE LEFT TO SIDE1 raw_cdeg=%ld\r\n",
          (long)s4_final_first_yaw_cdeg);
      break;

    case MISSION_STATE_S4_FINAL_RECOVERY_TURN_270:
      if (MissionTask_TryResumeS4FinalRecovery(now))
      {
        return;
      }
      if (MissionTask_RunS4ForcedLeftTurn(
              now, s4_final_first_yaw_cdeg))
      {
        MissionTask_StopWheels();
        MissionTask_ResetS6ControllerState();
        s6_phase_stable_since = 0U;
        s6_next_debug_tick = now;
        MissionTask_EnterState(
            MISSION_STATE_S4_FINAL_RECOVERY_TURN_90, now);
        (void)DebugUart_Logf(
            "[S4-24R] SIDE1 DONE, TURN TO SIDE2 raw_cdeg=%ld\r\n",
            (long)s4_final_second_yaw_cdeg);
      }
      break;

    case MISSION_STATE_S4_FINAL_RECOVERY_TURN_90:
      if (MissionTask_TryResumeS4FinalRecovery(now))
      {
        return;
      }
      if (MissionTask_RunS6TurnToHeading(
              now, s4_final_second_yaw_cdeg))
      {
        MissionTask_StopWheels();
        MissionTask_ResetS6ControllerState();
        MaixCam_ClearObject();
        mission_snapshot.vision_target_valid = false;
        MissionTask_ResetVisionPid();
        if (MaixCam_SendCommand(MAIXCAM_COMMAND_FINAL_RECOVERY_DONE) != HAL_OK)
        {
          mission_snapshot.fault_flags |= MISSION_FAULT_VISION_TX;
          MissionTask_EnterState(MISSION_STATE_FAULT, now);
          return;
        }
        (void)DebugUart_Log(
            "[S4-24R] RECOVERY COMPLETE, TX E1 E2 F1 1E 2E\r\n");
        s4_final_missing_since = now;
        s4_next_debug_tick = now;
        MissionTask_EnterState(
            MISSION_STATE_S4_TRACK_FINAL_BLOCK, now);
        (void)DebugUart_Log(
            "[S4-24R] SIDE2 DONE, RESUME 24 WAIT COORDS\r\n");
      }
      break;

    case MISSION_STATE_S4_FINAL_CENTER_OBJECT:
      if ((s3_target_select_command == MAIXCAM_COMMAND_SELECT_RED) &&
          !s4_final_frame_lowered &&
          (elapsed >= ROBOT_S4_RED_FRAME_LOWER_DELAY_MS))
      {
        if (Actuator_SetFrameLowered() != HAL_OK)
        {
          mission_snapshot.fault_flags |= MISSION_FAULT_ACTUATOR;
          MissionTask_EnterState(MISSION_STATE_FAULT, now);
          return;
        }
        s4_final_frame_lowered = true;
        (void)DebugUart_Logf(
            "[S4] RED34 FRAME DOWN AT %lums, CONTINUE FORWARD\r\n",
            (unsigned long)elapsed);
      }
      if (elapsed < (s3_target_select_command == MAIXCAM_COMMAND_SELECT_RED
                         ? ROBOT_S4_RED_FINAL_CENTER_MS
                         : ROBOT_S4_FINAL_CENTER_MS))
      {
        MissionTask_SetWheelTargets(ROBOT_S4_FINAL_CENTER_RPM,
                                    ROBOT_S4_FINAL_CENTER_RPM);
        return;
      }
      MissionTask_StopWheels();
      if (!s4_final_frame_lowered &&
          (Actuator_SetFrameLowered() != HAL_OK))
      {
        mission_snapshot.fault_flags |= MISSION_FAULT_ACTUATOR;
        MissionTask_EnterState(MISSION_STATE_FAULT, now);
        return;
      }
      MissionTask_EnterState(MISSION_STATE_S4_FINAL_LOWER_FRAME, now);
      (void)DebugUart_Logf(
          "[S4] FINAL CENTER DONE, FRAME DOWN settle=%lums, "
          "THEN REVERSE rpm=%d time=%lums\r\n",
          (unsigned long)ROBOT_S4_FRAME_LOWER_SETTLE_MS,
          ROBOT_S4_POST_LOWER_REVERSE_RPM,
          (unsigned long)ROBOT_S4_POST_LOWER_REVERSE_MS);
      break;

    case MISSION_STATE_S4_FINAL_LOWER_FRAME:
      if (elapsed < ROBOT_S4_FRAME_LOWER_SETTLE_MS)
      {
        return;
      }
      if (elapsed < (ROBOT_S4_FRAME_LOWER_SETTLE_MS +
                     ROBOT_S4_POST_LOWER_REVERSE_MS))
      {
        MissionTask_SetWheelTargets(-ROBOT_S4_POST_LOWER_REVERSE_RPM,
                                    -ROBOT_S4_POST_LOWER_REVERSE_RPM);
        return;
      }
      MissionTask_StopWheels();
      (void)DebugUart_Log(
          "[S4] FRAME DOWN REVERSE DONE, ENTER 05\r\n");
      MissionTask_EnterS5(now);
      break;

    default:
      break;
  }
}

static void MissionTask_RunS5WaitSingleGreen(uint32_t now)
{
  uint8_t event_code;
  MaixCam_Object load_check_object;

  if (mission_snapshot.state != MISSION_STATE_S5_WAIT_SINGLE_GREEN)
  {
    return;
  }

  /* Latch a fresh 05 class before RX06 clears the coordinate mailbox. */
  MissionTask_UpdateS5CarriedObject(now);
  if (!MaixCam_TakeEvent(&event_code))
  {
    if ((int32_t)(now - s5_next_debug_tick) >= 0)
    {
      s5_next_debug_tick = now + ROBOT_S5_DEBUG_PERIOD_MS;
      (void)DebugUart_Logf(
          "[S5] WAIT RX %s elapsed=%lums team=%s camera=%s\r\n",
          s5_close_view_active ? "02/06/24/E5" : "05/24/E5",
          (unsigned long)(now - mission_snapshot.state_entry_tick),
          mission_snapshot.team == ROBOT_TEAM_RED ? "RED" : "BLUE",
          s5_close_view_active ? "NEAR" : "WIDE");
    }
    return;
  }

  if (event_code == MAIXCAM_EVENT_LOAD_CHECK_REQUEST)
  {
    if (Actuator_SetCameraNearView() != HAL_OK)
    {
      mission_snapshot.fault_flags |= MISSION_FAULT_ACTUATOR;
      MissionTask_EnterState(MISSION_STATE_FAULT, now);
      return;
    }
    if (!s5_close_view_active)
    {
      /* Arm a new confirmation window once; duplicate05 must not discard
         the confirmed class or fence out a class frame already received. */
      s5_load_class_confirmed = false;
      (void)MaixCam_GetObjectSnapshot(&load_check_object);
      s5_load_check_sequence_floor = load_check_object.sequence;
    }
    s5_close_view_active = true;
    s5_next_debug_tick = now;
    (void)DebugUart_Logf(
        "[S5] RX05 LOAD CHECK, CAMERA NEAR angle=%u, WAIT RX02/06/24/E5\r\n",
        (unsigned int)ROBOT_CAMERA_NEAR_ANGLE_DEG);
    return;
  }

  if (event_code == MAIXCAM_EVENT_SKIP_TO_FINAL_TRACK)
  {
    if (Actuator_SetCameraWideView() != HAL_OK)
    {
      mission_snapshot.fault_flags |= MISSION_FAULT_ACTUATOR;
      MissionTask_EnterState(MISSION_STATE_FAULT, now);
      return;
    }
    s5_close_view_active = false;
    s6_carried_object_id = 0U;
    s5_load_class_confirmed = false;
    (void)DebugUart_Logf(
        "[S5] RX24 NO OBJECT IN 05, CAMERA WIDE=%u, ENTER 24 TRACK\r\n",
        (unsigned int)ROBOT_CAMERA_WIDE_ANGLE_DEG);
    MissionTask_SkipToS4FinalTrack(now);
    return;
  }

  if (event_code == MAIXCAM_EVENT_LOAD_EMPTY_RECOVERY)
  {
    if (s7_target_switch_enabled ||
        s3_target_select_command != MAIXCAM_COMMAND_SELECT_GREEN)
    {
      (void)DebugUart_Log(
          "[S5-E5] IGNORE: ONLY INITIAL GREEN BEFORE07\r\n");
      return;
    }
    MissionTask_StopWheels();
    if (Actuator_SetCameraWideView() != HAL_OK)
    {
      mission_snapshot.fault_flags |= MISSION_FAULT_ACTUATOR;
      MissionTask_EnterState(MISSION_STATE_FAULT, now);
      return;
    }
    s5_close_view_active = false;
    s6_carried_object_id = 0U;
    s5_load_class_confirmed = false;
    MissionTask_ResetVisionPid();
    MaixCam_ClearObject();
    mission_snapshot.vision_target_valid = false;
    /* Reuse all four uninterruptible E4 motion legs and TX44 handshake.
       E5 is requested by the camera's empty05 loop counter, not E4's loss
       budget: neither reset nor consume the existing E4 push allowance. */
    s4_e4_protocol_active = true;
    s5_empty_recovery_active = true;
    MissionTask_EnterState(MISSION_STATE_S4_E4_PUSH_RIGHT, now);
    (void)DebugUart_Logf(
        "[S5-E5] INITIAL GREEN EMPTY, CAMERA WIDE=%u, FRAME DOWN, "
        "RIGHT/BACK/LEFT/BACK THEN TX44; E4 cycles=%u UNCHANGED\r\n",
        (unsigned int)ROBOT_CAMERA_WIDE_ANGLE_DEG,
        (unsigned int)s4_green_e4_push_cycles);
    return;
  }

  if (event_code == MAIXCAM_EVENT_ARRANGE_READY)
  {
    if (Actuator_SetCameraWideView() != HAL_OK)
    {
      mission_snapshot.fault_flags |= MISSION_FAULT_ACTUATOR;
      MissionTask_EnterState(MISSION_STATE_FAULT, now);
      return;
    }
    s5_close_view_active = false;
    s4_final_mode_already_active = false;
    MissionTask_StopWheels();
    MaixCam_ClearObject();
    MissionTask_ResetVisionPid();
    MissionTask_ResetS6ControllerState();
    s6_phase_stable_since = 0U;
    s6_next_debug_tick = now;
    mission_snapshot.safe_zone_fixed_heading_active = true;
    MissionTask_EnterState(MISSION_STATE_S5_ALIGN_FOR_ARRANGE, now);
    (void)DebugUart_Logf(
        "[S5] RX02 ARRANGE, CAMERA WIDE=%u, ALIGN SAFE yaw=%ld err=%ld\r\n",
        (unsigned int)ROBOT_CAMERA_WIDE_ANGLE_DEG,
        (long)mission_snapshot.safe_zone_yaw_target_cdeg,
        (long)MissionTask_YawError(
            mission_snapshot.safe_zone_yaw_target_cdeg,
            mission_snapshot.yaw_cdeg));
    return;
  }

  if (event_code != MAIXCAM_EVENT_NO_ARRANGE_REQUIRED)
  {
    (void)DebugUart_Logf(
        "[S5] RX EVENT %02X, EXPECT 05/02/06/24/E5, KEEP WAITING\r\n",
        (unsigned int)event_code);
    return;
  }

  if (!s7_target_switch_enabled &&
      (!s5_close_view_active || !s5_load_class_confirmed ||
       s6_carried_object_id != 5U))
  {
    /* A green search command is not proof of green cargo. Require the fresh
       ID5 latched in this05 window; a non-green/unknown load stays parked.
       Vision may send02 to arrange,24 to recapture, or a new ID5 then06. */
    MissionTask_StopWheels();
    (void)DebugUart_Logf(
        "[S5-GREEN] RX06 BLOCKED before07 ID=%u confirmed=%u near=%u, WAIT ID5 THEN06 OR02/24\r\n",
        (unsigned int)s6_carried_object_id,
        s5_load_class_confirmed ? 1U : 0U,
        s5_close_view_active ? 1U : 0U);
    return;
  }

  (void)DebugUart_Log(
      "[S5] RX F1 F2 06 1F 2F, NO ARRANGE, START SAFE-ZONE SEARCH\r\n");
  if (Actuator_SetCameraWideView() != HAL_OK)
  {
    mission_snapshot.fault_flags |= MISSION_FAULT_ACTUATOR;
    MissionTask_EnterState(MISSION_STATE_FAULT, now);
    return;
  }
  s5_close_view_active = false;
  (void)DebugUart_Logf("[S6] CAMERA WIDE angle=%u\r\n",
                       (unsigned int)ROBOT_CAMERA_WIDE_ANGLE_DEG);
  (void)DebugUart_Logf("[S6-CARGO] LOCK ID=%u cmd=%02X kind=%s source=%s\r\n",
      (unsigned int)MissionTask_GetCarriedObjectId(),
      (unsigned int)s3_target_select_command,
      MissionTask_S6IsSupplyTarget() ? "SUPPLY" :
          (MissionTask_S6IsCasualtyTarget() ? "CASUALTY" : "UNKNOWN"),
      s5_load_class_confirmed ? "VISION05" : "TASK_FALLBACK");

  MaixCam_ClearObject();
  mission_snapshot.vision_target_valid = false;
  MissionTask_ResetVisionPid();
  MissionTask_ResetS6ControllerState();
  s6_phase_stable_since = 0U;
  s6_next_debug_tick = now;
  s6_missing_since = 0U;
  s6_search_start_tick = now;
  s6_track_elapsed_ms = 0U;
  s6_track_accounting_tick = now;
  s6_align26_pending = false;
  s6_pre_reposition_decision_pending = false;
  s6_supply_y_started = false;
  s6_obstacle_side_required = false;
  s6_obstacle_side_decision_pending = false;
  s6_obstacle_side_decision_done = false;
  MissionTask_ResetS6XConfirmation();
  s6_recovery_count = 0U;
  MissionTask_EnterState(MISSION_STATE_S6_SEARCH_SAFE_ZONE, now);
  (void)DebugUart_Logf(
      "[S6] SHORTEST TURN TO SAFE yaw=%ld err=%ld max=%d timeout=%lums\r\n",
      (long)mission_snapshot.safe_zone_yaw_target_cdeg,
      (long)MissionTask_YawError(mission_snapshot.safe_zone_yaw_target_cdeg,
                                 mission_snapshot.yaw_cdeg),
      (int)ROBOT_S6_SEARCH_TURN_MAX_RPM,
      (unsigned long)ROBOT_S6_SEARCH_TIMEOUT_MS);
}

static bool MissionTask_S6IsSupplyTarget(void)
{
  uint8_t object_id = MissionTask_GetCarriedObjectId();
  return object_id == 3U || object_id == 5U || object_id == 6U ||
      (object_id == 0U &&
       s3_target_select_command == MAIXCAM_COMMAND_SELECT_BLACK_GREEN);
}

static bool MissionTask_S6IsCasualtyTarget(void)
{
  return MissionTask_GetCarriedObjectId() == 4U;
}

typedef struct
{
  uint8_t start_zone;
  uint8_t team;
  uint8_t target_kind;
  uint8_t enabled;
  int32_t sector_start_cdeg;
  int32_t sector_end_cdeg;
  int32_t side_heading_cdeg;
  uint8_t include_start;
  uint8_t include_end;
} MissionSideRule;

static const MissionSideRule s6_side_rules[] = { ROBOT_S6_SIDE_RULE_ROWS };

static const MissionSideRule *MissionTask_GetS6SideRule(
    int32_t decision_yaw_cdeg, bool *has_calibrated_rule)
{
  uint32_t i;
  uint8_t kind;
  int32_t yaw_from_start = MissionTask_WrapYaw(
      decision_yaw_cdeg - mission_snapshot.yaw_start_cdeg);

  *has_calibrated_rule = false;

  if (MissionTask_S6IsSupplyTarget())
  {
    kind = 0U;
  }
  else if (MissionTask_S6IsCasualtyTarget())
  {
    kind = 1U;
  }
  else
  {
    return NULL;
  }
  for (i = 0U; i < sizeof(s6_side_rules) / sizeof(s6_side_rules[0]); ++i)
  {
    const MissionSideRule *rule = &s6_side_rules[i];
    if (rule->enabled && (rule->start_zone == mission_snapshot.start_zone) &&
        (rule->team == mission_snapshot.team) && (rule->target_kind == kind))
    {
      int32_t span = MissionTask_WrapYaw(
          rule->sector_end_cdeg - rule->sector_start_cdeg);
      int32_t position = MissionTask_WrapYaw(
          yaw_from_start - rule->sector_start_cdeg);
      *has_calibrated_rule = true;
      if (((position > 0L) || rule->include_start) &&
          ((position < span) || ((position == span) && rule->include_end)))
      {
        return rule;
      }
    }
  }
  return NULL;
}

static bool MissionTask_S6NeedsSideReposition(int32_t decision_yaw_cdeg)
{
  bool has_calibrated_rule;
  const MissionSideRule *rule = MissionTask_GetS6SideRule(
      decision_yaw_cdeg, &has_calibrated_rule);
  int32_t yaw_error = MissionTask_YawError(
      mission_snapshot.safe_zone_yaw_target_cdeg,
      decision_yaw_cdeg);

  if (has_calibrated_rule) return rule != NULL;
  /* Compatibility fallback until this zone/team has been calibrated. */
  if (MissionTask_S6IsSupplyTarget())
  {
    return (yaw_error > 0L) &&
           (yaw_error <= ROBOT_S6_REPOSITION_SECTOR_CDEG);
  }
  if (MissionTask_S6IsCasualtyTarget())
  {
    return (yaw_error < 0L) &&
           (yaw_error >= -ROBOT_S6_REPOSITION_SECTOR_CDEG);
  }
  /* Unclassified targets retain the earlier sector/tolerance policy. */
  return (yaw_error < -ROBOT_S6_REPOSITION_MIN_OFFSET_CDEG) &&
         (yaw_error >= -ROBOT_S6_REPOSITION_SECTOR_CDEG);
}

static void MissionTask_StartS6SideReposition(uint32_t now,
                                               int32_t decision_yaw_cdeg)
{
  bool has_calibrated_rule;
  const MissionSideRule *rule = MissionTask_GetS6SideRule(
      decision_yaw_cdeg, &has_calibrated_rule);
  int32_t current_relative_cdeg;
  int32_t target_relative_cdeg;
  int32_t safe_relative_cdeg;
  MissionTask_StopWheels();
  MaixCam_ClearObject();
  mission_snapshot.vision_target_valid = false;
  MissionTask_ResetVisionPid();
  MissionTask_ResetS6ControllerState();
  s6_phase_stable_since = 0U;
  MissionTask_ResetS6XConfirmation();
  s6_approach_start_tick = now;
  s6_reposition_side_yaw_cdeg = rule != NULL
      ? MissionTask_WrapYaw(mission_snapshot.yaw_start_cdeg +
                           rule->side_heading_cdeg)
      : MissionTask_WrapYaw(
      mission_snapshot.safe_zone_yaw_target_cdeg +
      (MissionTask_S6IsSupplyTarget() ? -1L : 1L) *
          ROBOT_S6_REPOSITION_SECTOR_CDEG);
  ++s6_reposition_count;
  s6_next_debug_tick = now;
  MissionTask_EnterState(MISSION_STATE_S6_REPOSITION_TURN_SIDE, now);
  (void)DebugUart_Logf(
      "[S6-P] START #%u yaw=%ld safe=%ld side=%ld target=%02X cargo=%u kind=%s "
      "zone=%u team=%u rule=%s\r\n",
      (unsigned int)s6_reposition_count,
      (long)MissionTask_WrapYaw(mission_snapshot.yaw_cdeg),
      (long)mission_snapshot.safe_zone_yaw_target_cdeg,
      (long)s6_reposition_side_yaw_cdeg,
      (unsigned int)s3_target_select_command,
      (unsigned int)MissionTask_GetCarriedObjectId(),
      MissionTask_S6IsSupplyTarget() ? "SUPPLY" :
          (MissionTask_S6IsCasualtyTarget()
               ? "CASUALTY" : "LEGACY"),
      (unsigned int)mission_snapshot.start_zone,
      (unsigned int)mission_snapshot.team,
      rule != NULL ? "CALIBRATED" : "FALLBACK");
  current_relative_cdeg = MissionTask_WrapYaw(
      mission_snapshot.yaw_cdeg - mission_snapshot.yaw_start_cdeg);
  target_relative_cdeg = MissionTask_WrapYaw(
      s6_reposition_side_yaw_cdeg - mission_snapshot.yaw_start_cdeg);
  safe_relative_cdeg = MissionTask_WrapYaw(
      mission_snapshot.safe_zone_yaw_target_cdeg - mission_snapshot.yaw_start_cdeg);
  (void)DebugUart_Logf(
      "[S6-P-ANGLE] relative_deg decision=%ld.%02ld current=%ld.%02ld target=%ld.%02ld safe=%ld.%02ld\r\n",
      (long)(MissionTask_WrapYaw(decision_yaw_cdeg -
          mission_snapshot.yaw_start_cdeg) / 100L),
      (long)(MissionTask_WrapYaw(decision_yaw_cdeg -
          mission_snapshot.yaw_start_cdeg) % 100L),
      (long)(current_relative_cdeg / 100L),
      (long)(current_relative_cdeg % 100L),
      (long)(target_relative_cdeg / 100L),
      (long)(target_relative_cdeg % 100L),
      (long)(safe_relative_cdeg / 100L),
      (long)(safe_relative_cdeg % 100L));
  (void)DebugUart_Logf(
      "[S6-P-ANGLE] raw_cdeg current=%ld target=%ld zero=%ld\r\n",
      (long)MissionTask_WrapYaw(mission_snapshot.yaw_cdeg),
      (long)s6_reposition_side_yaw_cdeg,
      (long)mission_snapshot.yaw_start_cdeg);
}

static void MissionTask_ResetS6PreDecisionTracking(uint32_t now)
{
  s6_pre_reposition_track_elapsed_ms = 0U;
  s6_pre_reposition_track_tick = now;
  s6_pre_reposition_track_active = false;
}

static bool MissionTask_S6PreDecisionTrackingReady(void)
{
  return s6_pre_reposition_track_elapsed_ms >= ROBOT_S6_PRE_REPOSITION_TRACK_MS;
}

static void MissionTask_UpdateS6PreDecisionTracking(uint32_t now)
{
  uint32_t interval_ms = (uint32_t)(now - s6_pre_reposition_track_tick);
  bool valid_tracking = mission_snapshot.vision_target_valid &&
      mission_snapshot.target_age_ms <= MISSION_VISION_TARGET_TIMEOUT_MS;
  s6_pre_reposition_track_tick = now;
  if (valid_tracking && s6_pre_reposition_track_active &&
      interval_ms <= MISSION_VISION_TARGET_TIMEOUT_MS &&
      !MissionTask_S6PreDecisionTrackingReady())
  {
    uint32_t remaining_ms = ROBOT_S6_PRE_REPOSITION_TRACK_MS -
        s6_pre_reposition_track_elapsed_ms;
    s6_pre_reposition_track_elapsed_ms +=
        interval_ms < remaining_ms ? interval_ms : remaining_ms;
    if (MissionTask_S6PreDecisionTrackingReady())
      (void)DebugUart_Logf("[S6-PRE] TRACK %lums COMPLETE, SIDE ANGLE DECISION ALLOWED\r\n",
          (unsigned long)s6_pre_reposition_track_elapsed_ms);
  }
  /* Missing coordinates pause the interval; they do not erase prior tracking. */
  s6_pre_reposition_track_active = valid_tracking;
}

static void MissionTask_ResetS6XConfirmation(void)
{
  s6_x_align_frames = 0U;
  s6_x_align_first_tick = 0U;
  s6_side_x_align_frames = 0U;
  s6_side_x_align_first_tick = 0U;
  s6_x_align_last_tick = 0U;
  s6_x_align_has_sequence = false;
}

static bool MissionTask_S6SideXConfirmed(void)
{
  return mission_snapshot.vision_target_valid &&
      mission_snapshot.target_age_ms <= ROBOT_VISION_COORD_DECEL_START_MS &&
      s6_x_align_has_sequence &&
      mission_snapshot.target_sequence == s6_x_align_last_sequence &&
      mission_snapshot.target_id == s6_x_align_id &&
      MissionTask_Abs32(mission_snapshot.vision_x_error_px) <= ROBOT_S6_SIDE_X_TOLERANCE_PX &&
      s6_side_x_align_frames >= ROBOT_S6_X_ALIGN_MIN_FRAMES &&
      (uint32_t)(s6_x_align_last_tick - s6_side_x_align_first_tick) >= ROBOT_S6_X_ALIGN_STABLE_MS;
}

static bool MissionTask_S6XConfirmed(void)
{
  return mission_snapshot.vision_target_valid &&
      mission_snapshot.target_age_ms <= ROBOT_VISION_COORD_DECEL_START_MS &&
      s6_x_align_has_sequence &&
      mission_snapshot.target_sequence == s6_x_align_last_sequence &&
      mission_snapshot.target_id == s6_x_align_id &&
      MissionTask_Abs32(mission_snapshot.vision_x_error_px) <= ROBOT_VISION_X_TOLERANCE_PX &&
      s6_x_align_frames >= ROBOT_S6_X_ALIGN_MIN_FRAMES &&
      (uint32_t)(s6_x_align_last_tick - s6_x_align_first_tick) >= ROBOT_S6_X_ALIGN_STABLE_MS;
}

static bool MissionTask_UpdateS6XConfirmation(uint32_t now)
{
  uint32_t receipt_tick = now - mission_snapshot.target_age_ms;
  bool consecutive;
  if (!mission_snapshot.vision_target_valid)
  {
    MissionTask_ResetS6XConfirmation();
    s6_pre_reposition_decision_pending = !s6_obstacle_side_decision_done;
    return false;
  }
  /* An aging cached frame may drive deceleration, but cannot prove alignment. */
  if (mission_snapshot.target_age_ms > ROBOT_VISION_COORD_DECEL_START_MS) return false;
  if (s6_x_align_has_sequence &&
      mission_snapshot.target_sequence == s6_x_align_last_sequence)
    return MissionTask_S6SideXConfirmed();

  consecutive = s6_x_align_has_sequence &&
      (uint32_t)(mission_snapshot.target_sequence - s6_x_align_last_sequence) == 1U &&
      mission_snapshot.target_id == s6_x_align_id &&
      (uint32_t)(receipt_tick - s6_x_align_last_tick) <= ROBOT_S6_X_ALIGN_MAX_GAP_MS;
  s6_x_align_last_sequence = mission_snapshot.target_sequence;
  s6_x_align_id = mission_snapshot.target_id;
  s6_x_align_last_tick = receipt_tick;
  s6_x_align_has_sequence = true;
  if (MissionTask_Abs32(mission_snapshot.vision_x_error_px) > ROBOT_S6_SIDE_X_TOLERANCE_PX)
    s6_side_x_align_frames = 0U;
  else
  {
    if (!consecutive || s6_side_x_align_frames == 0U)
    {
      s6_side_x_align_frames = 0U;
      s6_side_x_align_first_tick = receipt_tick;
    }
    if (s6_side_x_align_frames < UINT8_MAX) ++s6_side_x_align_frames;
  }
  if (MissionTask_Abs32(mission_snapshot.vision_x_error_px) > ROBOT_VISION_X_TOLERANCE_PX)
    s6_x_align_frames = 0U;
  else
  {
    if (!consecutive || s6_x_align_frames == 0U)
    {
      s6_x_align_frames = 0U;
      s6_x_align_first_tick = receipt_tick;
    }
    if (s6_x_align_frames < UINT8_MAX) ++s6_x_align_frames;
  }
  if (s6_pre_reposition_decision_pending || s6_align26_pending)
    (void)DebugUart_Logf("[S6-X] seq=%lu x=%d side=%u span=%lums ready=%u final=%u ready=%u\r\n",
        (unsigned long)s6_x_align_last_sequence, (int)mission_snapshot.vision_x_error_px,
        (unsigned int)s6_side_x_align_frames,
        (unsigned long)(s6_side_x_align_frames ? s6_x_align_last_tick - s6_side_x_align_first_tick : 0U),
        MissionTask_S6SideXConfirmed() ? 1U : 0U,
        (unsigned int)s6_x_align_frames,
        MissionTask_S6XConfirmed() ? 1U : 0U);
  return MissionTask_S6SideXConfirmed();
}

static void MissionTask_StartS6VisionApproach(uint32_t now)
{
  MissionTask_StopWheels();
  /* Preserve the freshly recovered errors; only reset controller history. */
  MissionTask_ResetVisionControllerState();
  MissionTask_ResetS6ControllerState();
  MissionTask_ResetS6XConfirmation();
  MissionTask_ResetS6PreDecisionTracking(now);
  s6_pre_reposition_decision_pending = !s6_obstacle_side_decision_done;
  s6_approach_start_tick = now;
  s6_missing_since = mission_snapshot.vision_target_valid ? 0U : now;
  s6_recovery_count = 0U;
  s6_next_debug_tick = now;
  MissionTask_EnterState(MISSION_STATE_S6_TRACK_SAFE_ZONE, now);
  (void)DebugUart_Logf(
      "[S6] VISION APPROACH yaw=%ld safe=%ld reposition=%u supply_y_started=%u, TRACK TIMER RESET 0/%lums\r\n",
      (long)MissionTask_WrapYaw(mission_snapshot.yaw_cdeg),
      (long)mission_snapshot.safe_zone_yaw_target_cdeg,
      (unsigned int)s6_reposition_count,
      s6_supply_y_started ? 1U : 0U,
      (unsigned long)ROBOT_S6_PRE_REPOSITION_TRACK_MS);
}

static void MissionTask_EnterS6TrackingFrom16(uint32_t now)
{
  (void)DebugUart_Log("[S6] RX F1 F2 16 1F 2F, SAFE ZONE FOUND\r\n");
  MissionTask_StopWheels();
  /* Discard coordinates from the preceding search reference. MaixCam must
     publish a fresh safe-zone coordinate frame after event 16. */
  MaixCam_ClearObject();
  mission_snapshot.vision_target_valid = false;
  mission_snapshot.vision_x_error_px = 0;
  mission_snapshot.vision_y_error_px = 0;
  MissionTask_ResetVisionPid();
  MissionTask_ResetS6ControllerState();
  s6_approach_start_tick = now;
  s6_reposition_count = 0U;
  s6_supply_y_started = false;
  s6_recovery_count = 0U;
  s6_next_debug_tick = now;
  MissionTask_StartS6VisionApproach(now);
  (void)DebugUart_Logf(
      "[S6] SIDE AFTER %lums (ANGLE ONLY), Y CAP %d/%d rpm, ALIGN X +/- %dpx, %u NEW FRAMES OVER %lums\r\n",
      (unsigned long)ROBOT_S6_PRE_REPOSITION_TRACK_MS,
      ROBOT_S6_APPROACH_UNALIGNED_RPM, ROBOT_S6_APPROACH_RPM,
      ROBOT_S6_SIDE_X_TOLERANCE_PX,
      (unsigned int)ROBOT_S6_X_ALIGN_MIN_FRAMES,
      (unsigned long)ROBOT_S6_X_ALIGN_STABLE_MS);
}

static bool MissionTask_DecideS6Side(uint32_t now, const char *source)
{
  if (!MissionTask_S6PreDecisionTrackingReady() ||
      !mission_snapshot.vision_target_valid ||
      mission_snapshot.target_age_ms > MISSION_VISION_TARGET_TIMEOUT_MS) return false;
  bool needs_side = MissionTask_S6NeedsSideReposition(
      mission_snapshot.yaw_cdeg);
  s6_pre_reposition_decision_pending = false;
  (void)DebugUart_Logf(
      "[S6-DEC] src=%s target=%02X id=%u zone=%u team=%u track_ms=%lu x=%d (ANGLE ONLY)\r\n",
      source, (unsigned int)s3_target_select_command,
      (unsigned int)mission_snapshot.target_id,
      (unsigned int)mission_snapshot.start_zone,
      (unsigned int)mission_snapshot.team,
      (unsigned long)s6_pre_reposition_track_elapsed_ms,
      (int)mission_snapshot.vision_x_error_px);
  (void)DebugUart_Logf(
      "[S6-DEC] field=%ld safe=%ld result=%s pending26=%u\r\n",
      (long)MissionTask_WrapYaw(mission_snapshot.yaw_cdeg - mission_snapshot.yaw_start_cdeg),
      (long)MissionTask_WrapYaw(mission_snapshot.safe_zone_yaw_target_cdeg -
                               mission_snapshot.yaw_start_cdeg),
      needs_side ? "SIDE" : "NO_SIDE", s6_align26_pending ? 1U : 0U);
  if (needs_side) MissionTask_StartS6SideReposition(
      now, mission_snapshot.yaw_cdeg);
  return needs_side;
}

static void MissionTask_StartS6FinalFrom26(uint32_t now)
{
  MissionTask_StopWheels();
  s6_align26_pending = false;
  s6_pre_reposition_decision_pending = false;
  s6_obstacle_side_required = false;
  s6_obstacle_side_decision_pending = false;
  s6_obstacle_side_decision_done = false;
  s6_push_yaw_target_cdeg = MissionTask_WrapYaw(mission_snapshot.safe_zone_yaw_target_cdeg);
  MaixCam_ClearObject();
  mission_snapshot.vision_target_valid = false;
  MissionTask_ResetVisionPid();
  MissionTask_ResetS6ControllerState();
  s6_phase_stable_since = 0U;
  s6_next_debug_tick = now;
  MissionTask_EnterState(MISSION_STATE_S6_FINAL_ALIGN, now);
  (void)DebugUart_Logf("[S6-26] RX26, GYRO ALIGN field=%ld THEN BORDER LOCATE/BACKOFF/PARTIAL RAISE/PUSH, NO VISION X\r\n",
      (long)MissionTask_WrapYaw(s6_push_yaw_target_cdeg - mission_snapshot.yaw_start_cdeg));
}

static void MissionTask_HandleS6AlignRequest(uint32_t now)
{
  MissionState state = mission_snapshot.state;
  bool side_move = (state == MISSION_STATE_S6_REPOSITION_TURN_SIDE) ||
      (state == MISSION_STATE_S6_REPOSITION_FORWARD) ||
      (state == MISSION_STATE_S6_REPOSITION_TURN_SAFE) ||
      (state == MISSION_STATE_S6_REPOSITION_FACE_SAFE);
  bool recovery = state >= MISSION_STATE_S6_RECOVERY_START &&
      state <= MISSION_STATE_S6_RECOVERY_RETURN_SAFE;
  bool obstacle = state >= MISSION_STATE_S6_OBSTACLE_TURN_LEFT &&
      state <= MISSION_STATE_S6_OBSTACLE_FACE_SAFE;

  if (MaixCam_TakeSafeZoneAlignRequest())
  {
    if ((state == MISSION_STATE_S6_SEARCH_SAFE_ZONE) ||
        (state == MISSION_STATE_S6_TRACK_SAFE_ZONE) || side_move || obstacle ||
        (state == MISSION_STATE_S6_TRACK_TIMEOUT_REVERSE) || recovery)
    {
      if (!s6_align26_pending)
        (void)DebugUart_Logf("[S6-26] LATCH st=%u pending_decision=%u\r\n",
            (unsigned int)state, s6_pre_reposition_decision_pending ? 1U : 0U);
      s6_align26_pending = true;
      /* 26 has priority over tracking, side moves, recovery, and the 1s gate. */
      MissionTask_StartS6FinalFrom26(now);
    }
    else
    {
      (void)DebugUart_Logf("[S6-26] IGNORE OUTSIDE SAFE APPROACH st=%u\r\n",
                          (unsigned int)state);
    }
  }
}

static void MissionTask_StartS6Recovery(uint32_t now,
                                         bool from_tracking)
{
  bool use_long_forward;

  /* This is confirmed long loss, not a single missing frame. */
  if (from_tracking)
  {
    MissionTask_ResetS6XConfirmation();
    s6_pre_reposition_decision_pending = true;
  }

  ++s6_recovery_count;
  use_long_forward =
      s6_recovery_count >= ROBOT_S6_RECOVERY_LONG_START_COUNT;
  /* First recovery is centered on the field's known safe-zone heading,
     even if the vehicle lost tracking while pointed elsewhere. */
  s6_recovery_base_yaw_cdeg = MissionTask_WrapYaw(
      use_long_forward ? mission_snapshot.yaw_cdeg
                       : mission_snapshot.safe_zone_yaw_target_cdeg);
  s6_recovery_left_yaw_cdeg = MissionTask_WrapYaw(
      s6_recovery_base_yaw_cdeg +
      (int32_t)ROBOT_YAW_LEFT_SIGN *
          ROBOT_S6_RECOVERY_LEFT_ANGLE_CDEG);
  s6_recovery_right_yaw_cdeg = MissionTask_WrapYaw(
      s6_recovery_left_yaw_cdeg -
      (int32_t)ROBOT_YAW_LEFT_SIGN *
          ROBOT_S6_RECOVERY_RIGHT_ANGLE_CDEG);
  s6_recovery_from_tracking = from_tracking;
  s6_missing_since = 0U;
  s6_phase_stable_since = 0U;
  MissionTask_ResetVisionPid();
  MissionTask_ResetS6ControllerState();
  s6_next_debug_tick = now;
  MissionTask_EnterState(MISSION_STATE_S6_RECOVERY_START, now);
  (void)DebugUart_Logf(
      "[S6-R] START #%u source=%s base=%ld left45=%ld right90=%ld mode=%s\r\n",
      (unsigned int)s6_recovery_count,
      from_tracking ? "TRACK_LOST" : "WAIT16",
      (long)s6_recovery_base_yaw_cdeg,
      (long)s6_recovery_left_yaw_cdeg,
      (long)s6_recovery_right_yaw_cdeg,
      use_long_forward ? "LONG_FORWARD_70RPM_2000MS" :
                         "SAFE_TURN_NO_FORWARD");
}

static void MissionTask_RunS6SearchSafeZone(uint32_t now)
{
  uint8_t event_code;
  int32_t absolute_yaw_error;
  int32_t heading_tolerance_cdeg;
  int16_t turn_rpm;

  if (mission_snapshot.state != MISSION_STATE_S6_SEARCH_SAFE_ZONE)
  {
    return;
  }

  if (MaixCam_TakeEvent(&event_code))
  {
    if (event_code == MAIXCAM_EVENT_SAFE_ZONE_FOUND)
    {
      MissionTask_EnterS6TrackingFrom16(now);
      return;
    }
    (void)DebugUart_Logf("[S6] RX EVENT %02X, EXPECT 16\r\n",
                         (unsigned int)event_code);
  }

  if ((uint32_t)(now - s6_search_start_tick) >=
      ROBOT_S6_SEARCH_TIMEOUT_MS)
  {
    (void)DebugUart_Logf(
        "[S6] SEARCH TIMEOUT yaw=%ld target=%ld\r\n",
        (long)MissionTask_WrapYaw(mission_snapshot.yaw_cdeg),
        (long)mission_snapshot.safe_zone_yaw_target_cdeg);
    mission_snapshot.fault_flags |= MISSION_FAULT_S6_TIMEOUT;
    MissionTask_EnterState(MISSION_STATE_FAULT, now);
    return;
  }

  turn_rpm = MissionTask_CalculateS6YawTurn(
      now, mission_snapshot.safe_zone_yaw_target_cdeg);
  absolute_yaw_error = MissionTask_Abs32(
      mission_snapshot.safe_zone_yaw_error_cdeg);
  /* The wait timestamp also marks that the heading has already been reached.
     Use a wider release threshold to avoid stop/start chatter near 2 degrees. */
  heading_tolerance_cdeg = s6_missing_since == 0U
      ? ROBOT_S6_TURN_TOLERANCE_CDEG
      : ROBOT_S6_SEARCH_HOLD_TOLERANCE_CDEG;
  mission_snapshot.safe_zone_fixed_heading_active = true;
  mission_snapshot.vision_forward_rpm = 0;
  if (absolute_yaw_error <= heading_tolerance_cdeg)
  {
    turn_rpm = 0;
    MissionTask_StopWheels();
#if ROBOT_S6_SEARCH_WAIT16_MS == 0U
    (void)DebugUart_Log("[S6] SAFE HEADING REACHED, NO WAIT16 DELAY, START RECOVERY\r\n");
    MissionTask_StartS6Recovery(now, false);
    return;
#else
    if (s6_missing_since == 0U)
    {
      s6_missing_since = now;
      (void)DebugUart_Logf(
          "[S6] SAFE HEADING REACHED, WAIT RX16 %lums HOLD=%ldcdeg\r\n",
          (unsigned long)ROBOT_S6_SEARCH_WAIT16_MS,
          (long)ROBOT_S6_SEARCH_HOLD_TOLERANCE_CDEG);
    }
    else if ((uint32_t)(now - s6_missing_since) >=
             ROBOT_S6_SEARCH_WAIT16_MS)
    {
      MissionTask_StartS6Recovery(now, false);
      return;
    }
#endif
  }
  else
  {
    s6_missing_since = 0U;
    MissionTask_SetWheelTargets(turn_rpm, (int16_t)-turn_rpm);
  }
  mission_snapshot.vision_turn_rpm = turn_rpm;

  if ((int32_t)(now - s6_next_debug_tick) >= 0)
  {
    s6_next_debug_tick = now + ROBOT_S6_DEBUG_PERIOD_MS;
    (void)DebugUart_Logf(
        "[S6-S] yaw=%ld target=%ld err=%ld turn=%d cmd=%d/%d\r\n",
        (long)MissionTask_WrapYaw(mission_snapshot.yaw_cdeg),
        (long)mission_snapshot.safe_zone_yaw_target_cdeg,
        (long)mission_snapshot.safe_zone_yaw_error_cdeg,
        (int)turn_rpm,
        (int)mission_snapshot.left_target_rpm,
        (int)mission_snapshot.right_target_rpm);
  }
}

static int16_t MissionTask_CalculateS6YawTurn(uint32_t now,
                                              int32_t yaw_command_cdeg)
{
  bool recovery_scan = mission_snapshot.state == MISSION_STATE_S6_RECOVERY_TURN_LEFT ||
      mission_snapshot.state == MISSION_STATE_S6_RECOVERY_TURN_BACK;
  bool recovery_safe_turn = mission_snapshot.state == MISSION_STATE_S6_RECOVERY_START ||
      mission_snapshot.state == MISSION_STATE_S6_RECOVERY_RETURN_SAFE;
  float turn_min_rpm = recovery_scan ? ROBOT_S6_RECOVERY_SCAN_MIN_RPM
      : (recovery_safe_turn ? ROBOT_S6_RECOVERY_SAFE_MIN_RPM : ROBOT_S6_YAW_MIN_TURN_RPM);
  float turn_max_rpm = mission_snapshot.state == MISSION_STATE_S6_SEARCH_SAFE_ZONE
      ? (float)ROBOT_S6_SEARCH_TURN_MAX_RPM
      : (mission_snapshot.state == MISSION_STATE_S6_EXIT_TURN_180
          ? (float)ROBOT_S6_EXIT_TURN_MAX_RPM
          : (recovery_scan ? ROBOT_S6_RECOVERY_SCAN_MAX_RPM
              : (recovery_safe_turn ? ROBOT_S6_RECOVERY_SAFE_MAX_RPM : ROBOT_S6_YAW_MAX_TURN_RPM)));

  /* State-specific limits leave other turns and straight yaw correction alone. */
  switch (mission_snapshot.state)
  {
    case MISSION_STATE_S6_REPOSITION_TURN_SIDE:
    case MISSION_STATE_S6_REPOSITION_TURN_SAFE:
      turn_max_rpm = ROBOT_S6_REPOSITION_TURN_MAX_RPM;
      break;
    case MISSION_STATE_S6_OBSTACLE_TURN_LEFT:
    case MISSION_STATE_S6_OBSTACLE_TURN_RIGHT:
    case MISSION_STATE_S6_OBSTACLE_FACE_SAFE:
      turn_max_rpm = ROBOT_S6_OBSTACLE_TURN_MAX_RPM;
      break;
    case MISSION_STATE_S4_E4_TURN_AWAY:
    case MISSION_STATE_S4_E4_TURN_LEFT:
    case MISSION_STATE_S4_E4_TURN_RIGHT:
      turn_max_rpm = ROBOT_S4_E4_TURN_MAX_RPM;
      break;
    case MISSION_STATE_S4_POST22_TURN_LEFT:
    case MISSION_STATE_S4_POST22_TURN_RIGHT:
    case MISSION_STATE_S4_ARRANGE_RECOVERY_TURN_LEFT:
    case MISSION_STATE_S4_ARRANGE_RECOVERY_TURN_BACK:
      turn_max_rpm = ROBOT_S4_E2_TURN_MAX_RPM;
      break;
    default:
      break;
  }

  return MissionTask_CalculateS6YawPid(now, yaw_command_cdeg,
                                      turn_min_rpm, turn_max_rpm);
}

static int16_t MissionTask_CalculateS6YawPid(uint32_t now,
                                             int32_t yaw_command_cdeg,
                                             float turn_min_rpm,
                                             float turn_max_rpm)
{
  float error;
  float dt_seconds;
  float derivative;
  float output;
  float magnitude;

  mission_snapshot.safe_zone_yaw_command_cdeg = yaw_command_cdeg;
  mission_snapshot.safe_zone_yaw_error_cdeg = MissionTask_YawError(
      yaw_command_cdeg, mission_snapshot.yaw_cdeg);
  error = (float)mission_snapshot.safe_zone_yaw_error_cdeg;

  if (!s6_yaw_pid_initialized)
  {
    dt_seconds = (float)MISSION_TASK_PERIOD_TICKS / 1000.0f;
    derivative = 0.0f;
    s6_yaw_pid_initialized = true;
  }
  else
  {
    dt_seconds = (float)(now - s6_yaw_pid_update_tick) / 1000.0f;
    dt_seconds = MissionTask_ClampFloat(dt_seconds, 0.01f, 0.20f);
    derivative = (error - s6_yaw_pid_last_error) / dt_seconds;
  }

  s6_yaw_pid_integral += error * dt_seconds;
  s6_yaw_pid_integral = MissionTask_ClampFloat(
      s6_yaw_pid_integral,
      -ROBOT_S6_YAW_INTEGRAL_LIMIT,
      ROBOT_S6_YAW_INTEGRAL_LIMIT);
  output = ROBOT_S6_YAW_KP * error +
           ROBOT_S6_YAW_KI * s6_yaw_pid_integral +
           ROBOT_S6_YAW_KD * derivative;
  output = MissionTask_ClampFloat(output,
                                  -turn_max_rpm, turn_max_rpm);

  s6_yaw_pid_last_error = error;
  s6_yaw_pid_update_tick = now;

  /* Positive yaw error means physical-left when ROBOT_YAW_LEFT_SIGN is +1,
     while a negative wheel-differential command turns this chassis left. */
  output *= -(float)ROBOT_YAW_LEFT_SIGN;
  magnitude = output < 0.0f ? -output : output;
  if ((magnitude > 0.0f) && (magnitude < turn_min_rpm))
  {
    output = output < 0.0f ? -turn_min_rpm : turn_min_rpm;
  }

  return (int16_t)(output >= 0.0f ? output + 0.5f : output - 0.5f);
}

static void MissionTask_DebugS6(uint32_t now)
{
  int16_t left_actual_rpm;
  int16_t right_actual_rpm;

  if ((int32_t)(now - s6_next_debug_tick) < 0)
  {
    return;
  }
  s6_next_debug_tick = now + ROBOT_S6_DEBUG_PERIOD_MS;

  left_actual_rpm = (int16_t)(
      ((float)moto_chassis[MISSION_LEFT_MOTOR_INDEX].speed_rpm / 36.0f) *
      ROBOT_LEFT_WHEEL_FORWARD_SIGN);
  right_actual_rpm = (int16_t)(
      ((float)moto_chassis[MISSION_RIGHT_MOTOR_INDEX].speed_rpm / 36.0f) *
      ROBOT_RIGHT_WHEEL_FORWARD_SIGN);

  (void)DebugUart_Logf(
      "[S6D] s=%u q=%u n=%u x=%d y=%d a=%ld t=%ld e=%ld u=%d v=%d c=%d/%d r=%d/%d\r\n",
      (unsigned int)mission_snapshot.state,
      mission_snapshot.vision_target_valid ? 1U : 0U,
      (unsigned int)s6_recovery_count,
      (int)mission_snapshot.vision_x_error_px,
      (int)mission_snapshot.vision_y_error_px,
      (long)mission_snapshot.yaw_cdeg,
      (long)mission_snapshot.safe_zone_yaw_command_cdeg,
      (long)mission_snapshot.safe_zone_yaw_error_cdeg,
      (int)mission_snapshot.vision_turn_rpm,
      (int)mission_snapshot.vision_forward_rpm,
      (int)mission_snapshot.left_target_rpm,
      (int)mission_snapshot.right_target_rpm,
      (int)left_actual_rpm,
      (int)right_actual_rpm);
}

static bool MissionTask_RunS6TurnToHeading(uint32_t now,
                                           int32_t target_yaw_cdeg)
{
  int32_t absolute_yaw_error;
  int16_t turn_rpm;
  bool exit_turn = mission_snapshot.state == MISSION_STATE_S6_EXIT_TURN_180;
  bool recovery_turn = mission_snapshot.state >= MISSION_STATE_S6_RECOVERY_START &&
      mission_snapshot.state <= MISSION_STATE_S6_RECOVERY_RETURN_SAFE;
  uint32_t stable_ms = recovery_turn ? ROBOT_S6_RECOVERY_TURN_STABLE_MS
                                    : ROBOT_S6_TURN_STABLE_MS;
  uint32_t timeout_ms = exit_turn ? ROBOT_S6_EXIT_TURN_TIMEOUT_MS
                                  : ROBOT_S6_TURN_TIMEOUT_MS;

  if ((uint32_t)(now - mission_snapshot.state_entry_tick) >=
      timeout_ms)
  {
    mission_snapshot.fault_flags |= MISSION_FAULT_S6_TIMEOUT;
    MissionTask_EnterState(MISSION_STATE_FAULT, now);
    return false;
  }

  turn_rpm = MissionTask_CalculateS6YawTurn(now, target_yaw_cdeg);
  if (exit_turn)
  {
    if (turn_rpm > ROBOT_S6_EXIT_TURN_MAX_RPM)
    {
      turn_rpm = ROBOT_S6_EXIT_TURN_MAX_RPM;
    }
    else if (turn_rpm < -ROBOT_S6_EXIT_TURN_MAX_RPM)
    {
      turn_rpm = -ROBOT_S6_EXIT_TURN_MAX_RPM;
    }
  }
  absolute_yaw_error = MissionTask_Abs32(
      mission_snapshot.safe_zone_yaw_error_cdeg);

  if (absolute_yaw_error <= ROBOT_S6_TURN_TOLERANCE_CDEG)
  {
    MissionTask_StopWheels();
    mission_snapshot.vision_turn_rpm = 0;
    mission_snapshot.vision_forward_rpm = 0;
    s6_yaw_pid_integral = 0.0f;
    if (stable_ms == 0U)
    {
      return true;
    }
    if (s6_phase_stable_since == 0U)
    {
      s6_phase_stable_since = now;
    }
    else if ((uint32_t)(now - s6_phase_stable_since) >=
             stable_ms)
    {
      return true;
    }
  }
  else
  {
    s6_phase_stable_since = 0U;
    mission_snapshot.vision_turn_rpm = turn_rpm;
    mission_snapshot.vision_forward_rpm = 0;
    MissionTask_SetWheelTargets(turn_rpm, (int16_t)-turn_rpm);
  }

  MissionTask_DebugS6(now);
  return false;
}

static void MissionTask_RunS5PrepareArrange(uint32_t now)
{
  uint32_t elapsed = (uint32_t)(now - mission_snapshot.state_entry_tick);

  switch (mission_snapshot.state)
  {
    case MISSION_STATE_S5_ALIGN_FOR_ARRANGE:
      mission_snapshot.safe_zone_fixed_heading_active = true;
      if (!MissionTask_RunS6TurnToHeading(
              now, mission_snapshot.safe_zone_yaw_target_cdeg))
      {
        return;
      }
      MissionTask_StopWheels();
      MissionTask_ResetS6ControllerState();
      if (Actuator_SetFrameRaised() != HAL_OK)
      {
        mission_snapshot.fault_flags |= MISSION_FAULT_ACTUATOR;
        MissionTask_EnterState(MISSION_STATE_FAULT, now);
        return;
      }
      MissionTask_EnterState(MISSION_STATE_S5_RAISE_FOR_ARRANGE, now);
      (void)DebugUart_Logf(
          "[S5] SAFE HEADING ALIGNED yaw=%ld, FRAME UP settle=%lums\r\n",
          (long)MissionTask_WrapYaw(mission_snapshot.yaw_cdeg),
          (unsigned long)ROBOT_S5_ARRANGE_FRAME_RAISE_SETTLE_MS);
      break;

    case MISSION_STATE_S5_RAISE_FOR_ARRANGE:
#if ROBOT_S5_ARRANGE_FRAME_RAISE_SETTLE_MS > 0U
      if (elapsed < ROBOT_S5_ARRANGE_FRAME_RAISE_SETTLE_MS)
      {
        return;
      }
#endif
      MissionTask_ResetS6ControllerState();
      s6_next_debug_tick = now;
      MissionTask_EnterState(MISSION_STATE_S5_REVERSE_FOR_ARRANGE, now);
      (void)DebugUart_Logf(
          "[S5] FRAME UP, REVERSE rpm=%d time=%lums yaw=%ld\r\n",
          ROBOT_S5_ARRANGE_REVERSE_RPM,
          (unsigned long)ROBOT_S5_ARRANGE_REVERSE_MS,
          (long)mission_snapshot.safe_zone_yaw_target_cdeg);
      break;

    case MISSION_STATE_S5_REVERSE_FOR_ARRANGE:
      if (elapsed < ROBOT_S5_ARRANGE_REVERSE_MS)
      {
        MissionTask_CommandS6Straight(
            now,
            mission_snapshot.safe_zone_yaw_target_cdeg,
            -ROBOT_S5_ARRANGE_REVERSE_RPM,
            ROBOT_S5_ARRANGE_REVERSE_WHEEL_MAX_RPM);
        return;
      }
      MissionTask_StopWheels();
      MissionTask_ResetS6ControllerState();
      if (Actuator_SetFrameLowered() != HAL_OK)
      {
        mission_snapshot.fault_flags |= MISSION_FAULT_ACTUATOR;
        MissionTask_EnterState(MISSION_STATE_FAULT, now);
        return;
      }
      MissionTask_EnterState(MISSION_STATE_S5_LOWER_FOR_ARRANGE, now);
      (void)DebugUart_Logf(
          "[S5] REVERSE DONE, FRAME DOWN settle=%lums\r\n",
          (unsigned long)ROBOT_S5_ARRANGE_FRAME_LOWER_SETTLE_MS);
      break;

    case MISSION_STATE_S5_LOWER_FOR_ARRANGE:
      if (elapsed < ROBOT_S5_ARRANGE_FRAME_LOWER_SETTLE_MS)
      {
        return;
      }
      mission_snapshot.safe_zone_fixed_heading_active = false;
      MissionTask_StartS4TrackState(
          MISSION_STATE_S4_TRACK_RIGHT_BLOCK, now);
      (void)DebugUart_Log(
          "[S5] FRAME DOWN, START FIRST RIGHT PUSH\r\n");
      break;

    default:
      break;
  }
}

static void MissionTask_CommandS6Straight(uint32_t now,
                                           int32_t target_yaw_cdeg,
                                           int16_t forward_rpm,
                                           int16_t wheel_max_rpm)
{
  int32_t absolute_yaw_error = MissionTask_Abs32(MissionTask_YawError(
      target_yaw_cdeg, mission_snapshot.yaw_cdeg));
  int16_t turn_rpm;
  bool recovery_forward =
      mission_snapshot.state == MISSION_STATE_S6_RECOVERY_START &&
      s6_recovery_count >= ROBOT_S6_RECOVERY_LONG_START_COUNT;

  if (recovery_forward &&
      absolute_yaw_error <= ROBOT_S6_YAW_FORWARD_ENABLE_CDEG)
  {
    /* Small moving corrections stay proportional, without a turn-speed floor.
       Large heading errors still stop forward travel and use the normal turn. */
    turn_rpm = MissionTask_CalculateS6YawPid(
        now, target_yaw_cdeg, 0.0f,
        ROBOT_S6_RECOVERY_FORWARD_MAX_CORRECTION_RPM);
  }
  else
  {
    turn_rpm = MissionTask_CalculateS6YawTurn(now, target_yaw_cdeg);
  }

  if (absolute_yaw_error <= ROBOT_S6_YAW_TOLERANCE_CDEG)
  {
    turn_rpm = 0;
    s6_yaw_pid_integral = 0.0f;
  }
  if (absolute_yaw_error > ROBOT_S6_YAW_FORWARD_ENABLE_CDEG)
  {
    forward_rpm = 0;
  }

  mission_snapshot.vision_turn_rpm = turn_rpm;
  mission_snapshot.vision_forward_rpm = forward_rpm;
  MissionTask_SetTrackingTargets(forward_rpm, turn_rpm, wheel_max_rpm);
  MissionTask_DebugS6(now);
}

static void MissionTask_RunS6SideReposition(uint32_t now)
{
  uint32_t elapsed = (uint32_t)(now - mission_snapshot.state_entry_tick);

  if ((mission_snapshot.state != MISSION_STATE_S6_REPOSITION_TURN_SIDE) &&
      (mission_snapshot.state != MISSION_STATE_S6_REPOSITION_FORWARD) &&
      (mission_snapshot.state != MISSION_STATE_S6_REPOSITION_TURN_SAFE) &&
      (mission_snapshot.state != MISSION_STATE_S6_REPOSITION_FACE_SAFE))
  {
    return;
  }

  if ((uint32_t)(now - s6_approach_start_tick) >=
      ROBOT_S6_APPROACH_TIMEOUT_MS)
  {
    mission_snapshot.fault_flags |= MISSION_FAULT_S6_TIMEOUT;
    MissionTask_EnterState(MISSION_STATE_FAULT, now);
    return;
  }

  switch (mission_snapshot.state)
  {
    case MISSION_STATE_S6_REPOSITION_TURN_SIDE:
      if (MissionTask_RunS6TurnToHeading(
              now, s6_reposition_side_yaw_cdeg))
      {
        MissionTask_StopWheels();
        MissionTask_ResetS6ControllerState();
        s6_next_debug_tick = now;
        MissionTask_EnterState(
            MISSION_STATE_S6_REPOSITION_FORWARD, now);
        (void)DebugUart_Logf(
            "[S6-P] SIDE ALIGNED, FORWARD rpm=%d time=%lums\r\n",
            ROBOT_S6_REPOSITION_FORWARD_RPM,
            (unsigned long)ROBOT_S6_REPOSITION_FORWARD_MS);
        (void)DebugUart_Logf(
            "[S6-P-ANGLE] TURN DONE current_relative_cdeg=%ld target_relative_cdeg=%ld\r\n",
            (long)MissionTask_WrapYaw(
                mission_snapshot.yaw_cdeg - mission_snapshot.yaw_start_cdeg),
            (long)MissionTask_WrapYaw(
                s6_reposition_side_yaw_cdeg - mission_snapshot.yaw_start_cdeg));
      }
      break;

    case MISSION_STATE_S6_REPOSITION_FORWARD:
      if (elapsed < ROBOT_S6_REPOSITION_FORWARD_MS)
      {
        MissionTask_CommandS6Straight(
            now,
            s6_reposition_side_yaw_cdeg,
            ROBOT_S6_REPOSITION_FORWARD_RPM,
            ROBOT_S6_REPOSITION_WHEEL_MAX_RPM);
        return;
      }
      MissionTask_StopWheels();
      MaixCam_ClearObject();
      mission_snapshot.vision_target_valid = false;
      MissionTask_ResetVisionPid();
      MissionTask_ResetS6ControllerState();
      s6_phase_stable_since = 0U;
      s6_next_debug_tick = now;
      MissionTask_EnterState(
          MISSION_STATE_S6_REPOSITION_TURN_SAFE, now);
      (void)DebugUart_Logf(
          "[S6-P] FORWARD DONE, YAW TURN TO SAFE raw_cdeg=%ld field_cdeg=%ld\r\n",
          (long)mission_snapshot.safe_zone_yaw_target_cdeg,
          (long)MissionTask_WrapYaw(mission_snapshot.safe_zone_yaw_target_cdeg -
                                   mission_snapshot.yaw_start_cdeg));
      break;

    case MISSION_STATE_S6_REPOSITION_TURN_SAFE:
      if (!MissionTask_RunS6TurnToHeading(now, mission_snapshot.safe_zone_yaw_target_cdeg))
        return;
      MaixCam_ClearObject();
      mission_snapshot.vision_target_valid = false;
      MissionTask_ResetS6PreDecisionTracking(now);
      MissionTask_StartS6VisionApproach(now);
      (void)DebugUart_Log("[S6-P] RETURNED TO SAFE HEADING, RESTART 1S TRACKING AND RECONFIRM X\r\n");
      break;

    case MISSION_STATE_S6_REPOSITION_FACE_SAFE:
      /* Reserved legacy state: use exactly the same acquisition gate as RX16. */
      MissionTask_StartS6VisionApproach(now);
      break;

    default:
      break;
  }
}

static bool MissionTask_TryFinishS6Recovery(uint32_t now)
{
  uint8_t event_code;

  if (MaixCam_TakeEvent(&event_code))
  {
    if (event_code == MAIXCAM_EVENT_SAFE_ZONE_FOUND)
    {
      MissionTask_EnterS6TrackingFrom16(now);
      return true;
    }
    if (s6_recovery_from_tracking &&
        (event_code == MAIXCAM_EVENT_SAFE_ZONE_OBSTACLE))
    {
      MissionTask_StartS6ObstacleAvoidance(now);
      return true;
    }
    (void)DebugUart_Logf("[S6-R] RX EVENT %02X, EXPECT 16\r\n",
                         (unsigned int)event_code);
  }

  if (s6_recovery_from_tracking &&
      mission_snapshot.vision_target_valid)
  {
    MissionTask_StartS6VisionApproach(now);
    (void)DebugUart_Logf(
        "[S6-R] COORDINATES RECOVERED x=%d y=%d, RECONFIRM X BEFORE ANGLE\r\n",
        (int)mission_snapshot.vision_x_error_px,
        (int)mission_snapshot.vision_y_error_px);
    return true;
  }
  return false;
}

static void MissionTask_RunS6Recovery(uint32_t now)
{
  bool recovery_done;
  bool use_long_forward;

  if ((mission_snapshot.state != MISSION_STATE_S6_RECOVERY_START) &&
      (mission_snapshot.state != MISSION_STATE_S6_RECOVERY_TURN_LEFT) &&
      (mission_snapshot.state != MISSION_STATE_S6_RECOVERY_TURN_BACK) &&
      (mission_snapshot.state != MISSION_STATE_S6_RECOVERY_RETURN_SAFE))
  {
    return;
  }

  if (MissionTask_TryFinishS6Recovery(now))
  {
    return;
  }

  use_long_forward =
      s6_recovery_count >= ROBOT_S6_RECOVERY_LONG_START_COUNT;

  if (!s6_recovery_from_tracking &&
      ((uint32_t)(now - s6_search_start_tick) >=
       ROBOT_S6_SEARCH_TIMEOUT_MS))
  {
    (void)DebugUart_Logf(
        "[S6-R] OVERALL TIMEOUT source=%s count=%u\r\n",
        s6_recovery_from_tracking ? "TRACK_LOST" : "WAIT16",
        (unsigned int)s6_recovery_count);
    mission_snapshot.fault_flags |= MISSION_FAULT_S6_TIMEOUT;
    MissionTask_EnterState(MISSION_STATE_FAULT, now);
    return;
  }

  switch (mission_snapshot.state)
  {
    case MISSION_STATE_S6_RECOVERY_START:
      if (!use_long_forward)
      {
        if (!MissionTask_RunS6TurnToHeading(
                now, mission_snapshot.safe_zone_yaw_target_cdeg))
        {
          return;
        }
        (void)DebugUart_Log("[S6-R] SAFE HEADING REACHED, LEFT45 NEXT\r\n");
      }
      else
      {
        if ((uint32_t)(now - mission_snapshot.state_entry_tick) <
            ROBOT_S6_RECOVERY_LONG_FORWARD_MS)
        {
          MissionTask_CommandS6Straight(
              now, s6_recovery_base_yaw_cdeg,
              ROBOT_S6_RECOVERY_LONG_FORWARD_RPM,
              ROBOT_S6_RECOVERY_LONG_WHEEL_MAX_RPM);
          return;
        }
        (void)DebugUart_Logf(
            "[S6-R] FORWARD %dRPM/%lums DONE, LEFT45 NEXT\r\n",
            ROBOT_S6_RECOVERY_LONG_FORWARD_RPM,
            (unsigned long)ROBOT_S6_RECOVERY_LONG_FORWARD_MS);
      }
      MissionTask_StopWheels();
      MissionTask_ResetS6ControllerState();
      s6_phase_stable_since = 0U;
      s6_next_debug_tick = now;
      MissionTask_EnterState(MISSION_STATE_S6_RECOVERY_TURN_LEFT, now);
      (void)DebugUart_Logf("[S6-R] LEFT45 target=%ld max=%d min=%d stable=%lums\r\n",
          (long)s6_recovery_left_yaw_cdeg,
          (int)ROBOT_S6_RECOVERY_SCAN_MAX_RPM, (int)ROBOT_S6_RECOVERY_SCAN_MIN_RPM,
          (unsigned long)ROBOT_S6_RECOVERY_TURN_STABLE_MS);
      break;

    case MISSION_STATE_S6_RECOVERY_TURN_LEFT:
      if (MissionTask_RunS6TurnToHeading(now,
                                         s6_recovery_left_yaw_cdeg))
      {
        MissionTask_ResetS6ControllerState();
        s6_phase_stable_since = 0U;
        s6_next_debug_tick = now;
        MissionTask_EnterState(MISSION_STATE_S6_RECOVERY_TURN_BACK, now);
        (void)DebugUart_Logf(
            "[S6-R] LEFT45 DONE, RIGHT90 target=%ld max=%d min=%d\r\n",
            (long)s6_recovery_right_yaw_cdeg,
            (int)ROBOT_S6_RECOVERY_SCAN_MAX_RPM, (int)ROBOT_S6_RECOVERY_SCAN_MIN_RPM);
      }
      break;

    case MISSION_STATE_S6_RECOVERY_TURN_BACK:
      recovery_done = MissionTask_RunS6TurnToHeading(
          now, s6_recovery_right_yaw_cdeg);
      if (!recovery_done)
      {
        return;
      }
      MissionTask_StopWheels();
      MissionTask_ResetS6ControllerState();
      s6_phase_stable_since = 0U;
      s6_next_debug_tick = now;
      MissionTask_EnterState(
          MISSION_STATE_S6_RECOVERY_RETURN_SAFE, now);
      (void)DebugUart_Logf(
          "[S6-R] SWEEP DONE, RETURN SAFE HEADING target=%ld\r\n",
          (long)mission_snapshot.safe_zone_yaw_target_cdeg);
      break;

    case MISSION_STATE_S6_RECOVERY_RETURN_SAFE:
      recovery_done = MissionTask_RunS6TurnToHeading(
          now, mission_snapshot.safe_zone_yaw_target_cdeg);
      if (!recovery_done)
      {
        return;
      }
      MissionTask_StopWheels();
      MissionTask_ResetS6ControllerState();
      s6_missing_since = now;
      s6_next_debug_tick = now;
      if (s6_recovery_from_tracking)
      {
        MissionTask_StartS6VisionApproach(now);
        (void)DebugUart_Logf(
            "[S6-R] SAFE HEADING RESTORED, WAIT COORDINATES %lums\r\n",
            (unsigned long)ROBOT_S6_RECOVERY_WAIT_MS);
      }
      else
      {
        MissionTask_EnterState(MISSION_STATE_S6_SEARCH_SAFE_ZONE, now);
        (void)DebugUart_Logf(
            "[S6-R] SAFE HEADING RESTORED, WAIT RX16 %lums\r\n",
            (unsigned long)ROBOT_S6_SEARCH_WAIT16_MS);
      }
      break;

    default:
      break;
  }
}

static void MissionTask_StartS6ObstacleAvoidance(uint32_t now)
{
  /* Keep the pre-36 heading, but evaluate its side sector only after a new
     one-second tracking interval on return, never while starting the move. */
  s6_obstacle_decision_yaw_cdeg = MissionTask_WrapYaw(
      mission_snapshot.yaw_cdeg);
  s6_obstacle_side_required = false;
  s6_obstacle_side_decision_pending = true;
  s6_obstacle_side_decision_done = false;
  (void)DebugUart_Logf(
      "[S6-O] SAVE pre36_yaw=%ld, DEFER SIDE ANGLE UNTIL TRACK %lums pending26=%u\r\n",
      (long)s6_obstacle_decision_yaw_cdeg,
      (unsigned long)ROBOT_S6_PRE_REPOSITION_TRACK_MS,
      s6_align26_pending ? 1U : 0U);
  MissionTask_StopWheels();
  s6_obstacle_left_yaw_cdeg = MissionTask_WrapYaw(
      mission_snapshot.yaw_cdeg + (int32_t)ROBOT_YAW_LEFT_SIGN *
          ROBOT_S6_OBSTACLE_LEFT_ANGLE_CDEG);
  MaixCam_ClearObject();
  mission_snapshot.vision_target_valid = false;
  MissionTask_ResetVisionPid();
  MissionTask_ResetS6ControllerState();
  s6_phase_stable_since = 0U;
  s6_next_debug_tick = now;
  MissionTask_EnterState(MISSION_STATE_S6_OBSTACLE_TURN_LEFT, now);
  (void)DebugUart_Logf(
      "[S6-O] RX36 LEFT45 target=%ld safe=%ld, turn_max=%d fwd=%d/%lums THEN RIGHT45 (NO REVERSE)\r\n",
      (long)s6_obstacle_left_yaw_cdeg,
      (long)mission_snapshot.safe_zone_yaw_target_cdeg,
      (int)ROBOT_S6_OBSTACLE_TURN_MAX_RPM,
      ROBOT_S6_OBSTACLE_DRIVE_RPM,
      (unsigned long)ROBOT_S6_OBSTACLE_FORWARD_MS);
}

static void MissionTask_RunS6ObstacleAvoidance(uint32_t now)
{
  uint32_t elapsed = (uint32_t)(now - mission_snapshot.state_entry_tick);

  switch (mission_snapshot.state)
  {
    case MISSION_STATE_S6_OBSTACLE_TURN_LEFT:
      if (!MissionTask_RunS6TurnToHeading(now, s6_obstacle_left_yaw_cdeg))
      {
        return;
      }
      MissionTask_StopWheels();
      MissionTask_ResetS6ControllerState();
      s6_phase_stable_since = 0U;
      MissionTask_EnterState(MISSION_STATE_S6_OBSTACLE_FORWARD, now);
      break;

    case MISSION_STATE_S6_OBSTACLE_FORWARD:
      if (elapsed < ROBOT_S6_OBSTACLE_FORWARD_MS)
      {
        MissionTask_SetWheelTargets(ROBOT_S6_OBSTACLE_DRIVE_RPM,
                                    ROBOT_S6_OBSTACLE_DRIVE_RPM);
        return;
      }
      MissionTask_StopWheels();
      s6_obstacle_right_yaw_cdeg = MissionTask_WrapYaw(
          mission_snapshot.yaw_cdeg - (int32_t)ROBOT_YAW_LEFT_SIGN *
              ROBOT_S6_OBSTACLE_RIGHT_ANGLE_CDEG);
      MissionTask_ResetS6ControllerState();
      s6_phase_stable_since = 0U;
      MissionTask_EnterState(MISSION_STATE_S6_OBSTACLE_TURN_RIGHT, now);
      (void)DebugUart_Logf("[S6-O] FORWARD DONE, RIGHT45 target=%ld\r\n",
                           (long)s6_obstacle_right_yaw_cdeg);
      break;

    case MISSION_STATE_S6_OBSTACLE_TURN_RIGHT:
      if (!MissionTask_RunS6TurnToHeading(now, s6_obstacle_right_yaw_cdeg))
      {
        return;
      }
      MissionTask_StopWheels();
      MissionTask_ResetS6ControllerState();
      s6_phase_stable_since = 0U;
      MissionTask_EnterState(MISSION_STATE_S6_OBSTACLE_FACE_SAFE, now);
      (void)DebugUart_Logf("[S6-O] RIGHT45 DONE, FACE SAFE yaw=%ld\r\n",
          (long)mission_snapshot.safe_zone_yaw_target_cdeg);
      break;

    case MISSION_STATE_S6_OBSTACLE_FACE_SAFE:
      if (!MissionTask_RunS6TurnToHeading(
              now, mission_snapshot.safe_zone_yaw_target_cdeg))
      {
        return;
      }
      MissionTask_StopWheels();
      /* Drop observations/events from the maneuver before acknowledging it. */
      MaixCam_ClearEvent();
      MaixCam_ClearObject();
      mission_snapshot.vision_target_valid = false;
      if (MaixCam_SendCommand(MAIXCAM_COMMAND_OBSTACLE_DONE_ACK) != HAL_OK)
      {
        mission_snapshot.fault_flags |= MISSION_FAULT_VISION_TX;
        MissionTask_EnterState(MISSION_STATE_FAULT, now);
        return;
      }
      MissionTask_StartS6VisionApproach(now);
      (void)DebugUart_Log(
          "[S6-O] DONE TX E1 E2 36 1E 2E, WAIT FRESH SAFE COORDINATES\r\n");
      break;

    default:
      break;
  }
}

static void MissionTask_RunS6TrackTimeoutReverse(uint32_t now)
{
  if (mission_snapshot.state != MISSION_STATE_S6_TRACK_TIMEOUT_REVERSE)
  {
    return;
  }
  if ((uint32_t)(now - mission_snapshot.state_entry_tick) <
      ROBOT_S6_TRACK_REVERSE_MS)
  {
    MissionTask_SetWheelTargets(-ROBOT_S6_TRACK_REVERSE_RPM,
                                -ROBOT_S6_TRACK_REVERSE_RPM);
    return;
  }
  MissionTask_StopWheels();
  MaixCam_ClearObject();
  mission_snapshot.vision_target_valid = false;
  s6_track_elapsed_ms = 0U;
  s6_track_accounting_tick = now;
  MissionTask_StartS6VisionApproach(now);
  (void)DebugUart_Log("[S6-8S] REVERSE DONE, RESUME 16 TRACK\r\n");
}

static void MissionTask_RunS6TrackSafeZone(uint32_t now)
{
  uint8_t event_code;
  bool side_x_ready;
  int32_t absolute_x_error;
  int32_t absolute_y_error;
  int16_t turn_rpm;
  int16_t forward_rpm;
  int16_t forward_max_rpm;

  if (mission_snapshot.state != MISSION_STATE_S6_TRACK_SAFE_ZONE)
  {
    return;
  }

  if (s6_align26_pending)
  {
    MissionTask_StartS6FinalFrom26(now);
    return;
  }
  MissionTask_UpdateS6PreDecisionTracking(now);

  if (!MaixCam_TakeEvent(&event_code))
  {
    event_code = 0U;
  }
  if (event_code == MAIXCAM_EVENT_SAFE_ZONE_OBSTACLE)
  {
    MissionTask_StartS6ObstacleAvoidance(now);
    return;
  }

  if (s6_track_elapsed_ms >= ROBOT_S6_TRACK_REVERSE_INTERVAL_MS)
  {
    MissionTask_StopWheels();
    MissionTask_ResetVisionControllerState();
    MissionTask_ResetS6ControllerState();
    MaixCam_ClearObject();
    mission_snapshot.vision_target_valid = false;
    MissionTask_EnterState(MISSION_STATE_S6_TRACK_TIMEOUT_REVERSE, now);
    (void)DebugUart_Logf(
        "[S6-8S] TRACK TOTAL=%lums (RECOVERY EXCLUDED), REVERSE rpm=%d time=%lums\r\n",
        (unsigned long)s6_track_elapsed_ms,
        ROBOT_S6_TRACK_REVERSE_RPM,
        (unsigned long)ROBOT_S6_TRACK_REVERSE_MS);
    return;
  }

  if ((uint32_t)(now - s6_approach_start_tick) >=
      ROBOT_S6_APPROACH_TIMEOUT_MS)
  {
    mission_snapshot.fault_flags |= MISSION_FAULT_S6_TIMEOUT;
    MissionTask_EnterState(MISSION_STATE_FAULT, now);
    return;
  }

  if (!mission_snapshot.vision_target_valid)
  {
    MissionTask_StopWheels();
    (void)MissionTask_UpdateS6XConfirmation(now);
    if (s6_missing_since == 0U)
    {
      s6_missing_since = now;
      (void)DebugUart_Logf(
          "[S6] COORDINATES LOST, WAIT %lums BEFORE RECOVERY\r\n",
          (unsigned long)ROBOT_S6_RECOVERY_WAIT_MS);
    }
    else if ((uint32_t)(now - s6_missing_since) >=
             ROBOT_S6_RECOVERY_WAIT_MS)
    {
      MissionTask_StartS6Recovery(now, true);
      return;
    }
    MissionTask_DebugS6(now);
    return;
  }

  s6_missing_since = 0U;
  s6_recovery_count = 0U;

  side_x_ready = MissionTask_UpdateS6XConfirmation(now);
  /* Side decisions depend on tracking time and yaw only, never pixel error
     or whether Y-axis approach has already started. */
  if (s6_obstacle_side_decision_pending &&
      MissionTask_S6PreDecisionTrackingReady())
  {
    /* Honor the saved pre-36 heading after the resumed tracking interval. */
    s6_obstacle_side_required = MissionTask_S6NeedsSideReposition(
        s6_obstacle_decision_yaw_cdeg);
    s6_obstacle_side_decision_pending = false;
    s6_obstacle_side_decision_done = true;
    s6_pre_reposition_decision_pending = false;
    (void)DebugUart_Logf(
        "[S6-O] TRACK 1S DONE, saved_side=%u pending26=%u (ANGLE ONLY)\r\n",
        s6_obstacle_side_required ? 1U : 0U,
        s6_align26_pending ? 1U : 0U);
    if (s6_obstacle_side_required)
    {
      s6_obstacle_side_required = false;
      MissionTask_StartS6SideReposition(
          now, s6_obstacle_decision_yaw_cdeg);
      return;
    }
  }
  if (!s6_obstacle_side_decision_done &&
      s6_pre_reposition_decision_pending &&
      MissionTask_DecideS6Side(now, "TRACK_1S")) return;

  if (side_x_ready && MissionTask_S6IsSupplyTarget() && !s6_supply_y_started)
  {
    s6_supply_y_started = true;
    (void)DebugUart_Logf(
        "[S6] SUPPLY X ALIGNED +/- %dpx, Y CAP=%d rpm\r\n",
        ROBOT_S6_SIDE_X_TOLERANCE_PX, ROBOT_S6_APPROACH_RPM);
  }

  absolute_x_error = MissionTask_Abs32(
      mission_snapshot.vision_x_error_px);
  absolute_y_error = MissionTask_Abs32(
      mission_snapshot.vision_y_error_px);

  if (absolute_x_error <= ROBOT_VISION_X_TOLERANCE_PX)
  {
    turn_rpm = 0;
    s3_pid_integral = 0.0f;
    s3_pid_output = 0.0f;
    s3_pid_initialized = false;
    s3_last_target_sequence = UINT32_MAX;
  }
  else
  {
    turn_rpm = MissionTask_CalculateVisionTurn(
        now, mission_snapshot.vision_x_error_px);
    if (turn_rpm > ROBOT_S6_APPROACH_TURN_MAX_RPM)
    {
      turn_rpm = ROBOT_S6_APPROACH_TURN_MAX_RPM;
    }
    else if (turn_rpm < -ROBOT_S6_APPROACH_TURN_MAX_RPM)
    {
      turn_rpm = -ROBOT_S6_APPROACH_TURN_MAX_RPM;
    }
  }

  /* Keep Y tracking while turning toward X, but use the reduced speed limit
     until fresh coordinate frames confirm alignment. */
  forward_max_rpm = side_x_ready ? ROBOT_S6_APPROACH_RPM
                               : ROBOT_S6_APPROACH_UNALIGNED_RPM;
  if (absolute_y_error <= ROBOT_VISION_Y_TOLERANCE_PX)
  {
    forward_rpm = 0;
    s3_y_pid_integral = 0.0f;
    s3_y_pid_output = 0.0f;
    s3_y_pid_initialized = false;
    s3_y_last_target_sequence = UINT32_MAX;
  }
  else
  {
    forward_rpm = MissionTask_CalculateVisionForward(
        now, mission_snapshot.vision_y_error_px);
    if (forward_rpm > forward_max_rpm)
    {
      forward_rpm = forward_max_rpm;
    }
    else if (forward_rpm < -forward_max_rpm)
    {
      forward_rpm = -forward_max_rpm;
    }
  }

  mission_snapshot.safe_zone_fixed_heading_active = false;
  mission_snapshot.vision_turn_rpm = turn_rpm;
  mission_snapshot.vision_forward_rpm = forward_rpm;
  MissionTask_SetVisionTrackingTargets(forward_rpm, turn_rpm,
                                  ROBOT_S6_APPROACH_WHEEL_MAX_RPM);
  MissionTask_DebugS6(now);
}

static void MissionTask_RunS6FinalAlign(uint32_t now)
{
  if (mission_snapshot.state != MISSION_STATE_S6_FINAL_ALIGN)
  {
    return;
  }

  if (!MissionTask_RunS6TurnToHeading(now, s6_push_yaw_target_cdeg))
  {
    return;
  }

  MissionTask_StopWheels();
  MissionTask_ResetS6ControllerState();
  mission_snapshot.safe_zone_fixed_heading_active = true;
  s6_next_debug_tick = now;
  MissionTask_EnterState(MISSION_STATE_S6_BORDER_LOCATE, now);
  (void)DebugUart_Logf("[S6-26] GYRO ALIGNED, BORDER LOCATE rpm=%d time=%lums, FRAME DOWN\r\n",
      ROBOT_S6_BORDER_LOCATE_RPM,
      (unsigned long)ROBOT_S6_BORDER_LOCATE_MS);
}

static void MissionTask_RunS6BorderLocate(uint32_t now)
{
  uint32_t elapsed;

  if (mission_snapshot.state != MISSION_STATE_S6_BORDER_LOCATE)
  {
    return;
  }

  elapsed = (uint32_t)(now - mission_snapshot.state_entry_tick);
  if (elapsed >= ROBOT_S6_BORDER_LOCATE_TIMEOUT_MS)
  {
    mission_snapshot.fault_flags |= MISSION_FAULT_S6_TIMEOUT;
    MissionTask_EnterState(MISSION_STATE_FAULT, now);
    return;
  }
  if (elapsed >= ROBOT_S6_BORDER_LOCATE_MS)
  {
    MissionTask_StopWheels();
    MissionTask_ResetS6ControllerState();
    mission_snapshot.safe_zone_fixed_heading_active = true;
    s6_next_debug_tick = now;
    MissionTask_EnterState(MISSION_STATE_S6_BORDER_BACKOFF, now);
    (void)DebugUart_Logf(
        "[S6-26] BORDER LOCATE DONE, BACKOFF rpm=%d time=%lums, FRAME DOWN\r\n",
        ROBOT_S6_BORDER_BACKOFF_RPM,
        (unsigned long)ROBOT_S6_BORDER_BACKOFF_MS);
    return;
  }

  mission_snapshot.safe_zone_fixed_heading_active = true;
  MissionTask_CommandS6Straight(now, s6_push_yaw_target_cdeg,
                                ROBOT_S6_BORDER_LOCATE_RPM,
                                ROBOT_S6_BORDER_LOCATE_WHEEL_MAX_RPM);
}

static void MissionTask_RunS6BorderBackoff(uint32_t now)
{
  uint32_t elapsed;

  if (mission_snapshot.state != MISSION_STATE_S6_BORDER_BACKOFF)
  {
    return;
  }

  elapsed = (uint32_t)(now - mission_snapshot.state_entry_tick);
  if (elapsed >= ROBOT_S6_BORDER_BACKOFF_TIMEOUT_MS)
  {
    mission_snapshot.fault_flags |= MISSION_FAULT_S6_TIMEOUT;
    MissionTask_EnterState(MISSION_STATE_FAULT, now);
    return;
  }
  if (elapsed >= ROBOT_S6_BORDER_BACKOFF_MS)
  {
    MissionTask_StopWheels();
    if (Actuator_SetFramePartiallyRaised(ROBOT_S6_PARTIAL_FRAME_RAISE_DEG) != HAL_OK)
    {
      mission_snapshot.fault_flags |= MISSION_FAULT_ACTUATOR;
      MissionTask_EnterState(MISSION_STATE_FAULT, now);
      return;
    }
    MissionTask_ResetS6ControllerState();
    mission_snapshot.safe_zone_fixed_heading_active = true;
    s6_next_debug_tick = now;
    MissionTask_EnterState(MISSION_STATE_S6_PARTIAL_RAISE_FRAME, now);
    (void)DebugUart_Logf(
        "[S6-26] BACKOFF DONE, FRAME PARTIAL UP delta=%u settle=%lums\r\n",
        (unsigned int)ROBOT_S6_PARTIAL_FRAME_RAISE_DEG,
        (unsigned long)ROBOT_S6_PARTIAL_FRAME_SETTLE_MS);
    return;
  }

  mission_snapshot.safe_zone_fixed_heading_active = true;
  MissionTask_CommandS6Straight(now, s6_push_yaw_target_cdeg,
                                -ROBOT_S6_BORDER_BACKOFF_RPM,
                                ROBOT_S6_BORDER_BACKOFF_WHEEL_MAX_RPM);
}

static void MissionTask_RunS6PartialRaiseFrame(uint32_t now)
{
  if (mission_snapshot.state != MISSION_STATE_S6_PARTIAL_RAISE_FRAME)
  {
    return;
  }

  MissionTask_StopWheels();
  if ((uint32_t)(now - mission_snapshot.state_entry_tick) <
      ROBOT_S6_PARTIAL_FRAME_SETTLE_MS)
  {
    return;
  }

  MissionTask_ResetS6ControllerState();
  mission_snapshot.safe_zone_fixed_heading_active = true;
  s6_next_debug_tick = now;
  MissionTask_EnterState(MISSION_STATE_S6_FINAL_PUSH, now);
  (void)DebugUart_Logf(
      "[S6-26] PARTIAL FRAME UP DONE, DELIVERY PUSH rpm=%d time=%lums\r\n",
      ROBOT_S6_FINAL_PUSH_RPM, (unsigned long)ROBOT_S6_FINAL_PUSH_MS);
}

static void MissionTask_RunS6RaiseFrame(uint32_t now)
{
  if (mission_snapshot.state != MISSION_STATE_S6_RAISE_FRAME)
  {
    return;
  }

  MissionTask_StopWheels();
  if ((uint32_t)(now - mission_snapshot.state_entry_tick) <
      ROBOT_S6_FRAME_RAISE_SETTLE_MS)
  {
    return;
  }

  MissionTask_ResetS6ControllerState();
  mission_snapshot.safe_zone_fixed_heading_active = true;
  s6_next_debug_tick = now;
  MissionTask_EnterState(MISSION_STATE_S6_FINAL_REVERSE, now);
  (void)DebugUart_Logf(
      "[S6] FRAME UP DONE, FINAL REVERSE rpm=%d time=%lums\r\n",
      ROBOT_S6_REVERSE_RPM,
      (unsigned long)ROBOT_S6_REVERSE_DRIVE_MS);
}

static void MissionTask_RunS6PrePushReverse(uint32_t now)
{
  uint32_t elapsed;

  if (mission_snapshot.state != MISSION_STATE_S6_PRE_PUSH_REVERSE)
  {
    return;
  }

  elapsed = (uint32_t)(now - mission_snapshot.state_entry_tick);
  if (elapsed >= ROBOT_S6_PRE_PUSH_TIMEOUT_MS)
  {
    mission_snapshot.fault_flags |= MISSION_FAULT_S6_TIMEOUT;
    MissionTask_EnterState(MISSION_STATE_FAULT, now);
    return;
  }

  if (elapsed < ROBOT_S6_PRE_PUSH_REVERSE_MS)
  {
    mission_snapshot.safe_zone_fixed_heading_active = true;
    MissionTask_CommandS6Straight(
        now, s6_push_yaw_target_cdeg,
        -ROBOT_S6_PRE_PUSH_REVERSE_RPM,
        ROBOT_S6_PRE_PUSH_REVERSE_WHEEL_MAX_RPM);
    return;
  }

  if (elapsed < (ROBOT_S6_PRE_PUSH_REVERSE_MS +
                 ROBOT_S6_PRE_PUSH_BRAKE_MS))
  {
    MissionTask_StopWheels();
    return;
  }

  MissionTask_StopWheels();
  MissionTask_ResetS6ControllerState();
  mission_snapshot.safe_zone_fixed_heading_active = true;
  s6_next_debug_tick = now;
  /* Compatibility for the reserved pre-reverse state: no visual verification. */
  MissionTask_EnterState(MISSION_STATE_S6_FINAL_PUSH, now);
}

static void MissionTask_RunS6FinalPush(uint32_t now)
{
  uint32_t elapsed;

  if (mission_snapshot.state != MISSION_STATE_S6_FINAL_PUSH)
  {
    return;
  }

  elapsed = (uint32_t)(now - mission_snapshot.state_entry_tick);
  if (elapsed >= ROBOT_S6_FINAL_PUSH_TIMEOUT_MS)
  {
    mission_snapshot.fault_flags |= MISSION_FAULT_S6_TIMEOUT;
    MissionTask_EnterState(MISSION_STATE_FAULT, now);
    return;
  }
  if (elapsed >= ROBOT_S6_FINAL_PUSH_MS)
  {
    MissionTask_StopWheels();
    if (Actuator_SetFrameRaised() != HAL_OK)
    {
      mission_snapshot.fault_flags |= MISSION_FAULT_ACTUATOR;
      MissionTask_EnterState(MISSION_STATE_FAULT, now);
      return;
    }

    (void)DebugUart_Logf(
        "[S6] FINAL PUSH DONE rpm=%d time=%lums yaw=%ld, FRAME UP left=%u right=%u settle=%lums\r\n",
        ROBOT_S6_FINAL_PUSH_RPM,
        (unsigned long)ROBOT_S6_FINAL_PUSH_MS,
        (long)s6_push_yaw_target_cdeg,
        (unsigned int)ROBOT_LEFT_FRAME_UP_DEG,
        (unsigned int)ROBOT_RIGHT_FRAME_UP_DEG,
        (unsigned long)ROBOT_S6_FRAME_RAISE_SETTLE_MS);
    MissionTask_ResetS6ControllerState();
    mission_snapshot.safe_zone_fixed_heading_active = true;
    s6_next_debug_tick = now;
    MissionTask_EnterState(MISSION_STATE_S6_RAISE_FRAME, now);
    return;
  }

  mission_snapshot.safe_zone_fixed_heading_active = true;
  MissionTask_CommandS6Straight(now, s6_push_yaw_target_cdeg,
                                ROBOT_S6_FINAL_PUSH_RPM,
                                ROBOT_S6_FINAL_PUSH_WHEEL_MAX_RPM);
}

static void MissionTask_RunS6FinalReverse(uint32_t now)
{
  uint32_t elapsed;

  if (mission_snapshot.state != MISSION_STATE_S6_FINAL_REVERSE)
  {
    return;
  }

  elapsed = (uint32_t)(now - mission_snapshot.state_entry_tick);
  if (elapsed >= ROBOT_S6_REVERSE_TIMEOUT_MS)
  {
    mission_snapshot.fault_flags |= MISSION_FAULT_S6_TIMEOUT;
    MissionTask_EnterState(MISSION_STATE_FAULT, now);
    return;
  }

#if ROBOT_S6_REVERSE_BRAKE_MS > 0U
  if (elapsed < ROBOT_S6_REVERSE_BRAKE_MS)
  {
    MissionTask_StopWheels();
    return;
  }
#endif

  if (elapsed >= (ROBOT_S6_REVERSE_BRAKE_MS +
                  ROBOT_S6_REVERSE_DRIVE_MS))
  {
    MissionTask_StopWheels();
    s6_exit_yaw_target_cdeg = MissionTask_WrapYaw(
        s6_push_yaw_target_cdeg +
        (int32_t)ROBOT_YAW_LEFT_SIGN * ROBOT_S6_EXIT_TURN_ANGLE_CDEG);
    /* Begin the next vision search before turning so coordinates can interrupt. */
    MissionTask_StartS7Decision(now);
    if (mission_snapshot.state == MISSION_STATE_FAULT)
    {
      return;
    }
    MissionTask_ResetS6ControllerState();
    s6_phase_stable_since = 0U;
    s6_next_debug_tick = now;
    MissionTask_EnterState(MISSION_STATE_S6_EXIT_TURN_180, now);
    (void)DebugUart_Logf(
        "[S6] REVERSE DONE rpm=%d time=%lums, 07 SEARCH ACTIVE, "
        "TURN180 target=%ld max=%d, COORDINATES INTERRUPT\r\n",
        ROBOT_S6_REVERSE_RPM,
        (unsigned long)ROBOT_S6_REVERSE_DRIVE_MS,
        (long)s6_exit_yaw_target_cdeg,
        ROBOT_S6_EXIT_TURN_MAX_RPM);
    return;
  }

  mission_snapshot.safe_zone_fixed_heading_active = true;
  MissionTask_CommandS6Straight(now, s6_push_yaw_target_cdeg,
                                -ROBOT_S6_REVERSE_RPM,
                                ROBOT_S6_REVERSE_WHEEL_MAX_RPM);
}

static void MissionTask_CompleteS7RedRecovery(uint32_t now)
{
  MissionTask_StopWheels();
  MissionTask_ResetVisionControllerState();
  MaixCam_ClearEvent();
  MaixCam_ClearObject();
  mission_snapshot.vision_target_valid = false;

  if (MaixCam_SendCommand(MAIXCAM_COMMAND_RED_SEARCH_DONE) != HAL_OK)
  {
    mission_snapshot.fault_flags |= MISSION_FAULT_VISION_TX;
    MissionTask_EnterState(MISSION_STATE_FAULT, now);
    return;
  }

  MissionTask_EnterState(MISSION_STATE_S7_WAIT_SECOND_E3, now);
  (void)DebugUart_Log(
      "[S7] RED RECOVERY DONE, TX E1 E2 13 1E 2E, WAIT SECOND E3\r\n");
}

static void MissionTask_StartS7Decision(uint32_t now)
{
  s7_target_switch_enabled = true;
  s7_red_recovery_used = false;
  s51_active_object_id = 0U;
  s6_carried_object_id = 0U;
  s5_load_class_confirmed = false;
  s7_red_switch_sync_pending = false;
  s7_red_coordinate_fence_active = false;
  s3_target_select_command = MAIXCAM_COMMAND_SELECT_RED;
  s4_arrange_cycle_count = 0U;
  s4_e4_recovery_used = false;
  MissionTask_StopWheels();
  MissionTask_ResetVisionControllerState();
  MaixCam_ClearEvent();
  MaixCam_ClearObject();
  mission_snapshot.vision_target_valid = false;

  if ((MaixCam_SendCommand(MAIXCAM_COMMAND_SELECT_RED) != HAL_OK) ||
      (MaixCam_SendCommand(MAIXCAM_COMMAND_SEARCH_TARGET) != HAL_OK))
  {
    mission_snapshot.fault_flags |= MISSION_FAULT_VISION_TX;
    MissionTask_EnterState(MISSION_STATE_FAULT, now);
    return;
  }

  MissionTask_EnterState(MISSION_STATE_S7_SEARCH_RED, now);
  (void)DebugUart_Log(
      "[S7] TX E1 E2 11 1E 2E, TX E1 E2 03 1E 2E, SEARCH RED\r\n");
}

static void MissionTask_StartBlackFinalTrack(uint32_t now)
{
  s4_arrange_cycle_count = 0U;
  MissionTask_StopWheels();
  MissionTask_ResetVisionControllerState();
  MaixCam_ClearEvent();
  MaixCam_ClearObject();
  mission_snapshot.vision_target_valid = false;
  s3_target_select_command = MAIXCAM_COMMAND_SELECT_BLACK;
  s4_final_mode_already_active = false;
  s4_final_event34_pending = false;
  s4_final_missing_since = 0U;
  s4_final_turn_stable_since = 0U;
  s4_final_recovery_count = 0U;

  if ((Actuator_SetFrameRaised() != HAL_OK) ||
      (Actuator_SetCameraWideView() != HAL_OK))
  {
    mission_snapshot.fault_flags |= MISSION_FAULT_ACTUATOR;
    MissionTask_EnterState(MISSION_STATE_FAULT, now);
    return;
  }
  if (MaixCam_SendCommand(MAIXCAM_COMMAND_SELECT_BLACK) != HAL_OK)
  {
    mission_snapshot.fault_flags |= MISSION_FAULT_VISION_TX;
    MissionTask_EnterState(MISSION_STATE_FAULT, now);
    return;
  }

  /* S4_RAISE_FRAME sends 24 after settling when final mode is not yet active. */
  MissionTask_EnterState(MISSION_STATE_S4_RAISE_FRAME, now);
  (void)DebugUart_Logf(
      "[S7-BLACK] TX E1 E2 31 1E 2E, FRAME UP %lums, "
      "THEN TX24 DIRECT TRACK (SKIP03/04)\r\n",
      (unsigned long)ROBOT_S4_FRAME_RAISE_SETTLE_MS);
}

static void MissionTask_SwitchS7ToBlackGreen(uint32_t now)
{
  MissionTask_StopWheels();
  (void)DebugUart_Log("[S7] RX SECOND E3, SELECT51 BLACK/GREEN AND SEARCH03\r\n");
  (void)MissionTask_StartS3Align(now, MAIXCAM_COMMAND_SELECT_BLACK_GREEN);
}

static void MissionTask_HandleRedPriorityRequest(uint32_t now)
{
  bool can_switch = false;
  if (!MaixCam_TakeRedPriorityRequest())
  {
    return;
  }
  if (s3_target_select_command == MAIXCAM_COMMAND_SELECT_RED)
  {
    (void)DebugUart_Log("[S7-PRIORITY] RX11 REPEAT RED, KEEP PID/LOCK/RECOVERY COUNT\r\n");
    return;
  }
  if ((mission_snapshot.state == MISSION_STATE_S7_SEARCH_BLACK) ||
      (((mission_snapshot.state == MISSION_STATE_S7_SILENT_TURN_LEFT) ||
        (mission_snapshot.state == MISSION_STATE_S7_SILENT_TURN_BACK)) &&
       (s7_silent_resume_state == MISSION_STATE_S7_SEARCH_BLACK)))
  {
    can_switch = true;
  }
  else if ((s3_target_select_command == MAIXCAM_COMMAND_SELECT_BLACK) ||
           ((s3_target_select_command == MAIXCAM_COMMAND_SELECT_BLACK_GREEN) &&
            (s51_active_object_id == 6U)))
  {
    switch (mission_snapshot.state)
    {
      case MISSION_STATE_S3_ALIGN_GREEN:
      case MISSION_STATE_S3_TRACK_GREEN:
      case MISSION_STATE_S3_LOWER_FRAME:
      case MISSION_STATE_S3_SEARCH_TURN_CW:
      case MISSION_STATE_S3_SEARCH_TURN_CCW:
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
        can_switch = true;
        break;
      default:
        break;
    }
  }
  if (!s7_target_switch_enabled || !can_switch)
  {
    (void)DebugUart_Logf("[S7-PRIORITY] RX11 IGNORED state=%u target=%02X actual51=%u\r\n",
        (unsigned int)mission_snapshot.state, (unsigned int)s3_target_select_command,
        (unsigned int)s51_active_object_id);
    return;
  }
  MissionTask_StopWheels();
  MissionTask_ResetVisionControllerState();
  MissionTask_ResetS6ControllerState();
  MaixCam_ClearEvent();
  MaixCam_ClearObject();
  mission_snapshot.vision_target_valid = false;
  vision_search_forward_active = false;
  s7_red_switch_sync_pending = true;
  (void)DebugUart_Logf(
      "[S7-PRIORITY] RX11 target=%02X actual51=%u -> RED11, STOP AND SYNC11/03\r\n",
      (unsigned int)s3_target_select_command, (unsigned int)s51_active_object_id);
  (void)MissionTask_StartS3Align(now, MAIXCAM_COMMAND_SELECT_RED);
}

static void MissionTask_ResumeRedTracking(uint32_t now)
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
      (Actuator_SetCameraWideView() != HAL_OK))
  {
    mission_snapshot.fault_flags |= MISSION_FAULT_ACTUATOR;
    MissionTask_EnterState(MISSION_STATE_FAULT, now);
    return;
  }
  s3_target_select_command = MAIXCAM_COMMAND_SELECT_RED;
  vision_search_forward_active = false;
  vision_search_e3_mode = false;
  vision_search_green_e3_mode = false;
  vision_search_resume_state = MISSION_STATE_S3_TRACK_GREEN;
  s3_search_sweep_completed = false;
  s3_target_lost_since = 0U;
  s3_track_stable_since = 0U;
  s3_wait_event04_logged = false;
  s3_next_debug_tick = now;
  MissionTask_EnterState(MISSION_STATE_S3_TRACK_GREEN, now);
  (void)DebugUart_Logf("[S7] RED RESUME PID, recovery_used=%u, KEEP LOCK NO TX11/03\r\n",
                       (unsigned int)s7_red_recovery_used);
}

static void MissionTask_RunS7Decision(uint32_t now)
{
  uint8_t event_code;

  if ((mission_snapshot.state != MISSION_STATE_S7_SEARCH_RED) &&
      (mission_snapshot.state != MISSION_STATE_S7_WAIT_SECOND_E3) &&
      (mission_snapshot.state != MISSION_STATE_S7_SEARCH_BLACK))
  {
    return;
  }

  MissionTask_StopWheels();
  if (mission_snapshot.vision_target_valid &&
      (mission_snapshot.target_sequence != s7_last_coordinate_sequence))
  {
    s7_last_coordinate_sequence = mission_snapshot.target_sequence;
    s7_last_message_tick = now;
  }

  if (((mission_snapshot.state == MISSION_STATE_S7_SEARCH_RED) ||
       (mission_snapshot.state == MISSION_STATE_S7_WAIT_SECOND_E3) ||
       (mission_snapshot.state == MISSION_STATE_S7_SEARCH_BLACK)) &&
      mission_snapshot.vision_target_valid &&
      (mission_snapshot.target_id ==
       (mission_snapshot.state == MISSION_STATE_S7_SEARCH_BLACK ? 6U : 4U)))
  {
    uint8_t select_command =
        mission_snapshot.state == MISSION_STATE_S7_SEARCH_BLACK
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
        "[S7] %s TARGET FOUND id=%u x=%d y=%d, "
        "REUSE GREEN CAPTURE FLOW\r\n",
        target_name,
        (unsigned int)mission_snapshot.target_id,
        (int)mission_snapshot.vision_x_error_px,
        (int)mission_snapshot.vision_y_error_px);
    MissionTask_ResumeRedTracking(now);
    return;
  }

  if (!MaixCam_TakeEvent(&event_code))
  {
    if ((uint32_t)(now - s7_last_message_tick) >= ROBOT_S7_SILENT_TIMEOUT_MS)
    {
      if ((mission_snapshot.state == MISSION_STATE_S7_SEARCH_RED) ||
          (mission_snapshot.state == MISSION_STATE_S7_WAIT_SECOND_E3))
      {
        (void)DebugUart_Log("[S7-SILENT] RED SEARCH SILENT, SELECT51 AND SEARCH03\r\n");
        MissionTask_SwitchS7ToBlackGreen(now);
        return;
      }
      s7_silent_resume_state = mission_snapshot.state;
      s7_silent_left_yaw_cdeg = MissionTask_WrapYaw(
          mission_snapshot.yaw_cdeg + (int32_t)ROBOT_YAW_LEFT_SIGN *
              ROBOT_S7_SILENT_LEFT_CDEG);
      s7_silent_right_yaw_cdeg = MissionTask_WrapYaw(
          s7_silent_left_yaw_cdeg - (int32_t)ROBOT_YAW_LEFT_SIGN *
              ROBOT_S7_SILENT_RIGHT_CDEG);
      MissionTask_ResetVisionControllerState();
      MissionTask_ResetS6ControllerState();
      s6_phase_stable_since = 0U;
      s6_next_debug_tick = now;
      MissionTask_EnterState(MISSION_STATE_S7_SILENT_TURN_LEFT, now);
      (void)DebugUart_Logf(
          "[S7-SILENT] NO MESSAGE %lums, LEFT45=%ld THEN RIGHT90=%ld resume=%u\r\n",
          (unsigned long)ROBOT_S7_SILENT_TIMEOUT_MS,
          (long)s7_silent_left_yaw_cdeg, (long)s7_silent_right_yaw_cdeg,
          (unsigned int)s7_silent_resume_state);
    }
    return;
  }
  s7_last_message_tick = now;

  if (event_code != MAIXCAM_EVENT_SEARCH_TARGET_LOST)
  {
    (void)DebugUart_Logf("[S7] RX EVENT %02X, EXPECT E3\r\n",
                         (unsigned int)event_code);
    return;
  }

  if (mission_snapshot.state == MISSION_STATE_S7_SEARCH_RED)
  {
    if (mission_snapshot.vision_target_valid)
    {
      (void)DebugUart_Log(
          "[S7] RX E3 WITH FRESH RED COORDINATES, KEEP TARGET\r\n");
      return;
    }

    MaixCam_ClearObject();
    mission_snapshot.vision_target_valid = false;
    (void)DebugUart_Log(
        "[S7] RX FIRST E3, START ONE SEARCH RECOVERY\r\n");
    MissionTask_StartVisionSearchRecovery(
        now, MISSION_STATE_S7_SEARCH_RED, mission_snapshot.yaw_cdeg, false);
    return;
  }

  if (mission_snapshot.state == MISSION_STATE_S7_WAIT_SECOND_E3)
  {
    MissionTask_SwitchS7ToBlackGreen(now);
    return;
  }

  (void)DebugUart_Log(
      "[S7] RX E3 WHILE SEARCHING BLACK, WAIT NEXT BLACK POLICY\r\n");
}

static void MissionTask_RunS7SilentSearch(uint32_t now)
{
  uint8_t expected_id;
  bool turning_left;
  if ((mission_snapshot.state != MISSION_STATE_S7_SILENT_TURN_LEFT) &&
      (mission_snapshot.state != MISSION_STATE_S7_SILENT_TURN_BACK))
  {
    return;
  }
  expected_id = s7_silent_resume_state == MISSION_STATE_S7_SEARCH_BLACK ? 6U : 4U;
  if (mission_snapshot.vision_target_valid &&
      (mission_snapshot.target_id == expected_id))
  {
    MissionTask_StopWheels();
    MissionTask_ResetS6ControllerState();
    (void)DebugUart_Logf("[S7-SILENT] TARGET FOUND id=%u, STOP TURN AND CAPTURE\r\n",
                         (unsigned int)expected_id);
    MissionTask_EnterState(s7_silent_resume_state, now);
    return;
  }
  turning_left = mission_snapshot.state == MISSION_STATE_S7_SILENT_TURN_LEFT;
  if (!MissionTask_RunS6TurnToHeading(
          now, turning_left ? s7_silent_left_yaw_cdeg : s7_silent_right_yaw_cdeg))
  {
    return;
  }
  MissionTask_StopWheels();
  MissionTask_ResetS6ControllerState();
  s6_phase_stable_since = 0U;
  if (turning_left)
  {
    MissionTask_EnterState(MISSION_STATE_S7_SILENT_TURN_BACK, now);
    (void)DebugUart_Log("[S7-SILENT] LEFT45 DONE, START RIGHT90\r\n");
  }
  else
  {
    MissionTask_EnterState(s7_silent_resume_state, now);
    (void)DebugUart_Log("[S7-SILENT] RIGHT90 DONE, RESUME 07 WAIT\r\n");
  }
}

static void MissionTask_RunS6ExitTurn180(uint32_t now)
{
  if (mission_snapshot.state != MISSION_STATE_S6_EXIT_TURN_180)
  {
    return;
  }

  if (mission_snapshot.vision_target_valid &&
      (mission_snapshot.target_id == 4U))
  {
    MissionTask_StopWheels();
    MissionTask_ResetS6ControllerState();
    s6_phase_stable_since = 0U;
    (void)DebugUart_Logf(
        "[S7] RED COORDINATES DURING TURN180 yaw=%ld x=%d y=%d, "
        "STOP TURN AND CAPTURE\r\n",
        (long)MissionTask_WrapYaw(mission_snapshot.yaw_cdeg),
        (int)mission_snapshot.vision_x_error_px,
        (int)mission_snapshot.vision_y_error_px);
    MissionTask_EnterState(MISSION_STATE_S7_SEARCH_RED, now);
    return;
  }

  if (!MissionTask_RunS6TurnToHeading(now, s6_exit_yaw_target_cdeg))
  {
    return;
  }

  MissionTask_StopWheels();
  MaixCam_ClearEvent();
  MissionTask_EnterState(MISSION_STATE_S7_SEARCH_RED, now);
  (void)DebugUart_Logf(
      "[S7] TURN180 DONE yaw=%ld, CONTINUE RED SEARCH, WAIT COORDINATES/E3\r\n",
      (long)s6_exit_yaw_target_cdeg);
}

static void MissionTask_ProcessEvents(uint32_t events, uint32_t now)
{
  if ((events & MISSION_EVENT_EMERGENCY_STOP) != 0U)
  {
    mission_snapshot.fault_flags |= MISSION_FAULT_EMERGENCY;
    MissionTask_EnterState(MISSION_STATE_FAULT, now);
    return;
  }

  if ((events & MISSION_EVENT_STOP) != 0U)
  {
    MissionTask_EnterState(MISSION_STATE_STOPPED, now);
    return;
  }

  if ((events & MISSION_EVENT_RESET) != 0U)
  {
    s7_target_switch_enabled = false;
    s7_red_recovery_used = false;
    s51_active_object_id = 0U;
    s6_carried_object_id = 0U;
    s5_load_class_confirmed = false;
    s7_red_switch_sync_pending = false;
    s7_red_coordinate_fence_active = false;
    s3_target_select_command = MAIXCAM_COMMAND_SELECT_GREEN;
    s4_post22_search_active = false;
    mission_snapshot.fault_flags = MISSION_FAULT_NONE;
    MissionTask_SetS1Phase(MISSION_S1_IDLE, now);
    MissionTask_ResetVisionPid();
    MissionTask_ResetS6ControllerState();
    MaixCam_ClearObject();
    MaixCam_ClearEvent();
    mission_snapshot.vision_target_valid = false;
    MissionTask_EnterState(MISSION_STATE_WAIT_START, now);
    return;
  }

  if (((events & MISSION_EVENT_START) != 0U) &&
      ((mission_snapshot.state == MISSION_STATE_WAIT_START) ||
       (mission_snapshot.state == MISSION_STATE_STOPPED)))
  {
    uint32_t required_faults = MissionTask_GetRequiredFaults();
    if (required_faults == MISSION_FAULT_NONE)
    {
      MissionTask_StartS1(now);
    }
    else
    {
      mission_snapshot.fault_flags |= required_faults;
      MissionTask_EnterState(MISSION_STATE_FAULT, now);
    }
  }
}

static void MissionTask_CheckRunningHealth(uint32_t now)
{
  uint32_t required_faults;

  if ((mission_snapshot.state != MISSION_STATE_S1_DEPART) &&
      (mission_snapshot.state != MISSION_STATE_S2_CROSS_BUMP) &&
      (mission_snapshot.state != MISSION_STATE_S3_ALIGN_GREEN) &&
      (mission_snapshot.state != MISSION_STATE_S3_SEARCH_TURN_CW) &&
      (mission_snapshot.state != MISSION_STATE_S3_SEARCH_TURN_CCW) &&
      (mission_snapshot.state != MISSION_STATE_S3_TRACK_GREEN) &&
      ((mission_snapshot.state < MISSION_STATE_S4_TRACK_CENTER) ||
       (mission_snapshot.state > MISSION_STATE_S4_FINAL_LOWER_FRAME)) &&
      (mission_snapshot.state != MISSION_STATE_S5_ALIGN_FOR_ARRANGE) &&
      (mission_snapshot.state != MISSION_STATE_S5_RAISE_FOR_ARRANGE) &&
      (mission_snapshot.state != MISSION_STATE_S5_REVERSE_FOR_ARRANGE) &&
      (mission_snapshot.state != MISSION_STATE_S5_LOWER_FOR_ARRANGE) &&
      (mission_snapshot.state != MISSION_STATE_S6_SEARCH_SAFE_ZONE) &&
      (mission_snapshot.state != MISSION_STATE_S6_REPOSITION_TURN_SIDE) &&
      (mission_snapshot.state != MISSION_STATE_S6_REPOSITION_FORWARD) &&
      (mission_snapshot.state != MISSION_STATE_S6_REPOSITION_TURN_SAFE) &&
      (mission_snapshot.state != MISSION_STATE_S6_REPOSITION_FACE_SAFE) &&
      (mission_snapshot.state != MISSION_STATE_S6_RECOVERY_START) &&
      (mission_snapshot.state != MISSION_STATE_S6_RECOVERY_TURN_LEFT) &&
      (mission_snapshot.state != MISSION_STATE_S6_RECOVERY_TURN_BACK) &&
      (mission_snapshot.state != MISSION_STATE_S6_RECOVERY_RETURN_SAFE) &&
      (mission_snapshot.state != MISSION_STATE_S6_TRACK_SAFE_ZONE) &&
      (mission_snapshot.state != MISSION_STATE_S6_OBSTACLE_TURN_LEFT) &&
      (mission_snapshot.state != MISSION_STATE_S6_OBSTACLE_FORWARD) &&
      (mission_snapshot.state != MISSION_STATE_S6_OBSTACLE_TURN_RIGHT) &&
      (mission_snapshot.state != MISSION_STATE_S6_OBSTACLE_FACE_SAFE) &&
      (mission_snapshot.state != MISSION_STATE_S6_TRACK_TIMEOUT_REVERSE) &&
      (mission_snapshot.state != MISSION_STATE_S6_FINAL_ALIGN) &&
      (mission_snapshot.state != MISSION_STATE_S6_FINAL_VERIFY) &&
      (mission_snapshot.state != MISSION_STATE_S6_RAISE_FRAME) &&
      (mission_snapshot.state != MISSION_STATE_S6_PRE_PUSH_REVERSE) &&
      (mission_snapshot.state != MISSION_STATE_S6_FINAL_PUSH) &&
      (mission_snapshot.state != MISSION_STATE_S6_BORDER_LOCATE) &&
      (mission_snapshot.state != MISSION_STATE_S6_BORDER_BACKOFF) &&
      (mission_snapshot.state != MISSION_STATE_S6_PARTIAL_RAISE_FRAME) &&
      (mission_snapshot.state != MISSION_STATE_S6_FINAL_REVERSE) &&
      (mission_snapshot.state != MISSION_STATE_S6_EXIT_TURN_180) &&
      (mission_snapshot.state != MISSION_STATE_S7_SEARCH_RED) &&
      (mission_snapshot.state != MISSION_STATE_S7_WAIT_SECOND_E3) &&
      (mission_snapshot.state != MISSION_STATE_S7_SEARCH_BLACK) &&
      (mission_snapshot.state != MISSION_STATE_S7_SILENT_TURN_LEFT) &&
       (mission_snapshot.state != MISSION_STATE_S7_SILENT_TURN_BACK) &&
       (mission_snapshot.state != MISSION_STATE_S4_E4_GREEN_SPIN_360) &&
       (mission_snapshot.state != MISSION_STATE_S4_E4_RED_SPIN_360))
  {
    return;
  }

  required_faults = MissionTask_GetRequiredFaults();
  if (required_faults != MISSION_FAULT_NONE)
  {
    mission_snapshot.fault_flags |= required_faults;
    MissionTask_EnterState(MISSION_STATE_FAULT, now);
  }
}

bool MissionTask_Create(void)
{
  if (mission_task_handle != NULL)
  {
    return true;
  }

  mission_event_handle = osEventFlagsNew(NULL);
  mission_snapshot_mutex = osMutexNew(NULL);
  if ((mission_event_handle == NULL) || (mission_snapshot_mutex == NULL))
  {
    return false;
  }

  mission_task_handle = osThreadNew(StartMissionTask, NULL,
                                    &mission_task_attributes);
  return mission_task_handle != NULL;
}

static void MissionTask_SetEvent(uint32_t event)
{
  if (mission_event_handle != NULL)
  {
    (void)osEventFlagsSet(mission_event_handle, event);
  }
}

void MissionTask_RequestStart(void)
{
  MissionTask_SetEvent(MISSION_EVENT_START);
}

void MissionTask_RequestStop(void)
{
  MissionTask_SetEvent(MISSION_EVENT_STOP);
}

void MissionTask_RequestReset(void)
{
  MissionTask_SetEvent(MISSION_EVENT_RESET);
}

void MissionTask_RequestEmergencyStop(void)
{
  MissionTask_SetEvent(MISSION_EVENT_EMERGENCY_STOP);
}

bool MissionTask_GetSnapshot(MissionSnapshot *snapshot)
{
  if ((snapshot == NULL) || (mission_snapshot_mutex == NULL))
  {
    return false;
  }

  if (osMutexAcquire(mission_snapshot_mutex, 0U) != osOK)
  {
    return false;
  }
  *snapshot = mission_snapshot;
  (void)osMutexRelease(mission_snapshot_mutex);
  return true;
}

static void MissionTask_DebugVisionAge(uint32_t now)
{
  const char *reason;
  MissionState state = mission_snapshot.state;
  bool tracking = state == MISSION_STATE_S3_TRACK_GREEN ||
      state == MISSION_STATE_S4_TRACK_CENTER ||
      state == MISSION_STATE_S4_TRACK_FINAL_BLOCK ||
      state == MISSION_STATE_S6_TRACK_SAFE_ZONE ||
      state == MISSION_STATE_S6_REPOSITION_FACE_SAFE;
  if ((int32_t)(now - vision_age_next_debug_tick) < 0) return;
  vision_age_next_debug_tick = now + ROBOT_VISION_COORD_DEBUG_PERIOD_MS;
  if (tracking)
  {
    if (!mission_snapshot.vision_target_valid)
      reason = mission_snapshot.target_present ? "AGE_STOP" : "NO_COORD";
    else if (mission_snapshot.target_age_ms >= MISSION_VISION_TARGET_TIMEOUT_MS)
      reason = "AGE_STOP";
    else if (mission_snapshot.target_age_ms > ROBOT_VISION_COORD_DECEL_START_MS)
      reason = "AGE_DECEL";
    else if (commanded_left_rpm == 0 && commanded_right_rpm == 0)
      reason = "XY_HOLD";
    else reason = "PID_RUN";
  }
  else if (state == MISSION_STATE_S5_WAIT_SINGLE_GREEN) reason = "WAIT_CAMERA";
  else if (state == MISSION_STATE_S4_WAIT_ARRANGE_READY ||
           state == MISSION_STATE_S4_WAIT_LEFT_TARGET ||
           state == MISSION_STATE_S4_WAIT_FINAL_READY) reason = "WAIT_ACK";
  else if (state == MISSION_STATE_S3_LOWER_FRAME ||
           state == MISSION_STATE_S4_RAISE_FRAME ||
           state == MISSION_STATE_S6_PARTIAL_RAISE_FRAME ||
           state == MISSION_STATE_S5_LOWER_FOR_ARRANGE ||
           (state == MISSION_STATE_S4_FINAL_LOWER_FRAME &&
            (uint32_t)(now - mission_snapshot.state_entry_tick) < ROBOT_S4_FRAME_LOWER_SETTLE_MS))
    reason = "FRAME_SETTLE";
  else if (state == MISSION_STATE_FAULT) reason = "FAULT_STOP";
  else if (state == MISSION_STATE_STOPPED || state == MISSION_STATE_WAIT_START)
    reason = "IDLE_STOP";
  else reason = commanded_left_rpm == 0 && commanded_right_rpm == 0
      ? "PHASE_HOLD" : "TIMED_OR_YAW";

  (void)DebugUart_Logf(
      "[VDBG] st=%u ID=%u seq=%lu age_ms=%lu valid=%u why=%s cmd=%d/%d\r\n",
      (unsigned int)state, (unsigned int)mission_snapshot.target_id,
      (unsigned long)mission_snapshot.target_sequence,
      (unsigned long)mission_snapshot.target_age_ms,
      mission_snapshot.vision_target_valid ? 1U : 0U, reason,
      (int)commanded_left_rpm, (int)commanded_right_rpm);
  if (s6_pre_reposition_decision_pending || s6_align26_pending)
    (void)DebugUart_Logf("[S6-PRE] track_ms=%lu/%lu side_frames=%u/%u span_ms=%lu final_frames=%u pending_decision=%u pending26=%u\r\n",
        (unsigned long)s6_pre_reposition_track_elapsed_ms,
        (unsigned long)ROBOT_S6_PRE_REPOSITION_TRACK_MS,
        (unsigned int)s6_side_x_align_frames, (unsigned int)ROBOT_S6_X_ALIGN_MIN_FRAMES,
        (unsigned long)(s6_side_x_align_frames ? s6_x_align_last_tick - s6_side_x_align_first_tick : 0U),
        (unsigned int)s6_x_align_frames,
        s6_pre_reposition_decision_pending ? 1U : 0U, s6_align26_pending ? 1U : 0U);
}

static void MissionTask_DebugAngles(uint32_t now, bool force)
{
  int32_t raw_current;
  int32_t raw_zero;
  int32_t field_current;
  int32_t raw_goal = mission_snapshot.safe_zone_yaw_target_cdeg;
  int32_t field_goal;
  const char *role = "SAFE_REF";

  if (!force && ((int32_t)(now - angle_next_debug_tick) < 0))
  {
    return;
  }
  if (!force && ((mission_snapshot.state == MISSION_STATE_STOPPED) ||
                 (mission_snapshot.state == MISSION_STATE_FAULT)))
  {
    return;
  }
  angle_next_debug_tick = now + ROBOT_ANGLE_DEBUG_PERIOD_MS;

  if (mission_snapshot.state == MISSION_STATE_WAIT_START)
  {
    if (!mission_snapshot.imu_valid)
    {
      (void)DebugUart_Log("[YAW] WAIT_START imu=0\r\n");
      return;
    }
    raw_current = MissionTask_WrapYaw(mission_snapshot.yaw_cdeg);
    (void)DebugUart_Logf("[YAW] WAIT_START deg=%ld.%02ld imu=1\r\n",
        (long)(raw_current / 100L), (long)(raw_current % 100L));
    return;
  }

  switch (mission_snapshot.state)
  {
    case MISSION_STATE_S1_DEPART:
    case MISSION_STATE_S2_CROSS_BUMP:
      raw_goal = mission_snapshot.yaw_target_cdeg;
      role = "DEPART";
      break;
    case MISSION_STATE_S3_SEARCH_TURN_CW:
    case MISSION_STATE_S3_SEARCH_TURN_CCW:
      raw_goal = s3_search_yaw_target_cdeg;
      role = vision_search_forward_active ? "SEARCH_REF" : "SEARCH_TURN";
      break;
    case MISSION_STATE_S4_ARRANGE_RECOVERY_TURN_LEFT:
      raw_goal = s4_arrange_recovery_left_yaw_cdeg;
      role = "ARRANGE_LEFT";
      break;
    case MISSION_STATE_S4_E4_TURN_LEFT:
      raw_goal = s4_e4_left_yaw_cdeg;
      role = "E4_LEFT";
      break;
    case MISSION_STATE_S4_E4_TURN_AWAY:
    case MISSION_STATE_S4_E4_FORWARD:
      raw_goal = s4_e4_away_yaw_cdeg;
      role = "E4_AWAY";
      break;
    case MISSION_STATE_S4_E4_TURN_RIGHT:
      raw_goal = s4_e4_right_yaw_cdeg;
      role = "E4_RIGHT";
      break;
    case MISSION_STATE_S4_E4_GREEN_SPIN_360:
      raw_goal = s4_e4_spin_origin_yaw_cdeg;
      role = "E4_360";
      break;
    case MISSION_STATE_S4_E4_RED_SPIN_360:
      raw_goal = s4_e4_spin_origin_yaw_cdeg;
      role = "E4_RED360";
      break;
    case MISSION_STATE_S4_POST22_TURN_LEFT:
      raw_goal = s4_post22_left_yaw_cdeg;
      role = "22E2_LEFT";
      break;
    case MISSION_STATE_S4_POST22_TURN_RIGHT:
      raw_goal = s4_post22_right_yaw_cdeg;
      role = "22E2_RIGHT";
      break;
    case MISSION_STATE_S4_ARRANGE_RECOVERY_TURN_BACK:
      raw_goal = s4_arrange_recovery_right_yaw_cdeg;
      role = "ARRANGE_BACK";
      break;
    case MISSION_STATE_S4_FINAL_RECOVERY_TURN_270:
      raw_goal = s4_final_first_yaw_cdeg;
      role = "24_SIDE1";
      break;
    case MISSION_STATE_S4_FINAL_RECOVERY_TURN_90:
      raw_goal = s4_final_second_yaw_cdeg;
      role = "24_SIDE2";
      break;
    case MISSION_STATE_S6_REPOSITION_TURN_SIDE:
    case MISSION_STATE_S6_REPOSITION_FORWARD:
      raw_goal = s6_reposition_side_yaw_cdeg;
      role = "SIDE";
      break;
    case MISSION_STATE_S6_RECOVERY_START:
      raw_goal = s6_recovery_base_yaw_cdeg;
      role = s6_recovery_count < ROBOT_S6_RECOVERY_LONG_START_COUNT
                 ? "RECOVER_SAFE" : "RECOVER_FWD";
      break;
    case MISSION_STATE_S6_RECOVERY_TURN_LEFT:
      raw_goal = s6_recovery_left_yaw_cdeg;
      role = "RECOVER_LEFT";
      break;
    case MISSION_STATE_S6_RECOVERY_TURN_BACK:
      raw_goal = s6_recovery_right_yaw_cdeg;
      role = "RECOVER_BACK";
      break;
    case MISSION_STATE_S6_OBSTACLE_TURN_LEFT:
    case MISSION_STATE_S6_OBSTACLE_FORWARD:
      raw_goal = s6_obstacle_left_yaw_cdeg;
      role = "36_SIDE";
      break;
    case MISSION_STATE_S6_OBSTACLE_TURN_RIGHT:
      raw_goal = s6_obstacle_right_yaw_cdeg;
      role = "36_RIGHT";
      break;
    case MISSION_STATE_S6_EXIT_TURN_180:
      raw_goal = s6_exit_yaw_target_cdeg;
      role = "EXIT180";
      break;
    case MISSION_STATE_S7_SILENT_TURN_LEFT:
      raw_goal = s7_silent_left_yaw_cdeg;
      role = "07_LEFT45";
      break;
    case MISSION_STATE_S7_SILENT_TURN_BACK:
      raw_goal = s7_silent_right_yaw_cdeg;
      role = "07_RIGHT90";
      break;
    case MISSION_STATE_S5_ALIGN_FOR_ARRANGE:
    case MISSION_STATE_S5_REVERSE_FOR_ARRANGE:
    case MISSION_STATE_S6_SEARCH_SAFE_ZONE:
    case MISSION_STATE_S6_RECOVERY_RETURN_SAFE:
    case MISSION_STATE_S6_REPOSITION_TURN_SAFE:
    case MISSION_STATE_S6_OBSTACLE_FACE_SAFE:
    case MISSION_STATE_S6_FINAL_ALIGN:
    case MISSION_STATE_S6_FINAL_VERIFY:
    case MISSION_STATE_S6_PRE_PUSH_REVERSE:
    case MISSION_STATE_S6_FINAL_PUSH:
    case MISSION_STATE_S6_FINAL_REVERSE:
      role = "SAFE_HOLD";
      break;
    case MISSION_STATE_S6_BORDER_LOCATE:
      raw_goal = s6_push_yaw_target_cdeg;
      role = "BORDER_LOCATE";
      break;
    case MISSION_STATE_S6_BORDER_BACKOFF:
      raw_goal = s6_push_yaw_target_cdeg;
      role = "BORDER_BACKOFF";
      break;
    case MISSION_STATE_S6_PARTIAL_RAISE_FRAME:
      raw_goal = s6_push_yaw_target_cdeg;
      role = "PARTIAL_FRAME";
      break;
    default:
      /* Vision PID states use safe direction as a reference, not a yaw goal. */
      break;
  }

  raw_current = MissionTask_WrapYaw(mission_snapshot.yaw_cdeg);
  raw_zero = MissionTask_WrapYaw(mission_snapshot.yaw_start_cdeg);
  field_current = MissionTask_WrapYaw(raw_current - raw_zero);
  raw_goal = MissionTask_WrapYaw(raw_goal);
  field_goal = MissionTask_WrapYaw(raw_goal - raw_zero);
  (void)DebugUart_Logf(
      "[ANGLE] s=%u raw=%ld.%02ld zero=%ld.%02ld field=%ld.%02ld zone=%u team=%u imu=%u\r\n",
      (unsigned int)mission_snapshot.state,
      (long)(raw_current / 100L), (long)(raw_current % 100L),
      (long)(raw_zero / 100L), (long)(raw_zero % 100L),
      (long)(field_current / 100L), (long)(field_current % 100L),
      (unsigned int)mission_snapshot.start_zone,
      (unsigned int)mission_snapshot.team,
      mission_snapshot.imu_valid ? 1U : 0U);
  (void)DebugUart_Logf(
      "[ANGLE-T] role=%s raw=%ld.%02ld field=%ld.%02ld err_cdeg=%ld\r\n",
      role, (long)(raw_goal / 100L), (long)(raw_goal % 100L),
      (long)(field_goal / 100L), (long)(field_goal % 100L),
      (long)MissionTask_YawError(raw_goal, raw_current));
}

void StartMissionTask(void *argument)
{
  uint32_t events;
  uint32_t now;

  (void)argument;
  mission_snapshot.state = MISSION_STATE_BOOT;
  mission_snapshot.s1_phase = MISSION_S1_IDLE;
  mission_snapshot.fault_flags = MISSION_FAULT_NONE;
  mission_snapshot.state_entry_tick = osKernelGetTickCount();
  commanded_left_rpm = 0;
  commanded_right_rpm = 0;
  MissionTask_ResetVisionPid();
  MissionTask_ResetS6ControllerState();
  MissionTask_StopWheels();

  if (osMutexAcquire(mission_snapshot_mutex, osWaitForever) == osOK)
  {
    MissionTask_UpdateInputs(HAL_GetTick());
    MissionTask_EnterState(MISSION_STATE_WAIT_START, HAL_GetTick());
    (void)osMutexRelease(mission_snapshot_mutex);
  }

  for (;;)
  {
    events = osEventFlagsWait(mission_event_handle, MISSION_EVENT_MASK,
                              osFlagsWaitAny, MISSION_TASK_PERIOD_TICKS);
    if ((events & osFlagsError) != 0U)
    {
      events = 0U;
    }

    now = HAL_GetTick();
    if (osMutexAcquire(mission_snapshot_mutex, osWaitForever) == osOK)
    {
      /* Only time in the 16 PID state counts; recovery/side moves/36 pause it. */
      if (mission_snapshot.state == MISSION_STATE_S6_TRACK_SAFE_ZONE)
      {
        s6_track_elapsed_ms += (uint32_t)(now - s6_track_accounting_tick);
      }
      s6_track_accounting_tick = now;
      MissionTask_UpdateInputs(now);
      MissionTask_ProcessEvents(events, now);
      MissionTask_CheckRunningHealth(now);
      MissionTask_DropUnexpectedS5E5();
      MissionTask_HandleRedPriorityRequest(now);
      MissionTask_DropDuplicateRecoveryEvent(now);
      MissionTask_HandleS6AlignRequest(now);
      MissionTask_RunS1(now);
      MissionTask_RunS2CrossBump(now);
      MissionTask_RunS3SearchTurn(now);
      MissionTask_RunS3Track(now);
      MissionTask_RunS3LowerFrame(now);
      MissionTask_RunS4Arrange(now);
      MissionTask_RunS4Post22Recovery(now);
      MissionTask_RunS4E4Recovery(now);
      MissionTask_RunS5WaitSingleGreen(now);
      MissionTask_RunS5PrepareArrange(now);
      MissionTask_RunS6SearchSafeZone(now);
      MissionTask_RunS6SideReposition(now);
      MissionTask_RunS6Recovery(now);
      MissionTask_RunS6TrackSafeZone(now);
      MissionTask_RunS6TrackTimeoutReverse(now);
      MissionTask_RunS6ObstacleAvoidance(now);
      MissionTask_RunS6FinalAlign(now);
      MissionTask_RunS6BorderLocate(now);
      MissionTask_RunS6BorderBackoff(now);
      MissionTask_RunS6PartialRaiseFrame(now);
      MissionTask_RunS6RaiseFrame(now);
      MissionTask_RunS6PrePushReverse(now);
      MissionTask_RunS6FinalPush(now);
      MissionTask_RunS6FinalReverse(now);
      MissionTask_RunS6ExitTurn180(now);
      MissionTask_RunS7SilentSearch(now);
      MissionTask_RunS7Decision(now);
      MissionTask_DebugAngles(now, false);
      MissionTask_DebugVisionAge(now);
      (void)osMutexRelease(mission_snapshot_mutex);
    }
  }
}
