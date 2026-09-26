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
#define MISSION_VISION_TARGET_TIMEOUT_MS 150U
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
static uint32_t s1_turn_stable_since;
static uint32_t s1_next_debug_tick;
static bool s2_drive_finished;
static uint32_t s2_next_debug_tick;
static uint32_t s3_align_stable_since;
static uint32_t s3_search_turn_stable_since;
static int32_t s3_search_origin_yaw_cdeg;
static int32_t s3_search_yaw_target_cdeg;
static bool s3_green_error_seen;
static bool s3_search_sweep_completed;
static bool s3_search_ccw_midpoint_done;
static bool vision_search_forward_active;
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
static uint32_t s4_center_target_lost_since;
static bool s4_center_search_sweep_completed;
static uint32_t s4_final_missing_since;
static uint32_t s4_final_turn_stable_since;
static uint8_t s4_final_recovery_count;
static MissionState s4_arrange_recovery_resume_state;
static int32_t s4_arrange_recovery_base_yaw_cdeg;
static int32_t s4_arrange_recovery_left_yaw_cdeg;
static int32_t s4_arrange_recovery_right_yaw_cdeg;
static uint8_t s4_arrange_recovery_count;
static uint32_t s5_next_debug_tick;
static bool s5_close_view_active;
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
static uint32_t s6_missing_since;
static uint32_t s6_search_start_tick;
static uint32_t s6_approach_start_tick;
static int32_t s6_reposition_side_yaw_cdeg;
static uint32_t s6_reposition_x_stable_since;
static uint32_t s6_pre_reposition_track_start_tick;
static bool s6_pre_reposition_decision_pending;
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

static int16_t MissionTask_CalculateS6YawTurn(uint32_t now,
                                              int32_t yaw_command_cdeg);
static bool MissionTask_RunS6TurnToHeading(uint32_t now,
                                            int32_t target_yaw_cdeg);
static void MissionTask_CommandS6Straight(uint32_t now,
                                           int32_t target_yaw_cdeg,
                                           int16_t forward_rpm,
                                           int16_t wheel_max_rpm);

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
      (void)DebugUart_Log("[MISSION] S4_WAIT_RX_02\r\n");
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
      (void)DebugUart_Log("[MISSION] S4_FINAL_RECOVERY_TURN_270\r\n");
      break;
    case MISSION_STATE_S4_FINAL_RECOVERY_TURN_90:
      (void)DebugUart_Log("[MISSION] S4_FINAL_RECOVERY_TURN_90\r\n");
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
    case MISSION_STATE_S6_RECOVERY_FORWARD:
      (void)DebugUart_Log("[MISSION] S6_RECOVERY_FORWARD\r\n");
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
    case MISSION_STATE_S6_FINAL_ALIGN:
      (void)DebugUart_Log("[MISSION] S6_FINAL_ALIGN\r\n");
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
    case MISSION_STATE_S6_FINAL_REVERSE:
      (void)DebugUart_Log("[MISSION] S6_FINAL_REVERSE\r\n");
      break;
    case MISSION_STATE_S6_EXIT_TURN_180:
      (void)DebugUart_Log("[MISSION] S6_EXIT_TURN_180\r\n");
      break;
    case MISSION_STATE_S6_SAFE_ZONE_REACHED:
      (void)DebugUart_Log("[MISSION] S6_SAFE_ZONE_REACHED\r\n");
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

  /* Every main-state boundary explicitly removes the previous motor command.
     The active state's runner publishes its own command afterwards. */
  MissionTask_StopWheels();
}

static void MissionTask_UpdateInputs(uint32_t now)
{
  JY901S_Attitude attitude;
  MaixCam_Object object;

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

  /* A missing target is not a camera-offline indication. The current UART
     protocol has no heartbeat/no-target frame, so only target freshness is
     reported here. */
  mission_snapshot.vision_target_valid = MaixCam_GetObject(&object) &&
      ((uint32_t)(now - object.update_tick) <=
       MISSION_VISION_TARGET_TIMEOUT_MS);
  if (mission_snapshot.vision_target_valid)
  {
    mission_snapshot.target_id = object.object_id;
    mission_snapshot.vision_x_error_px = object.x_error_px;
    mission_snapshot.vision_y_error_px = object.y_error_px;
    mission_snapshot.target_sequence = object.sequence;
  }
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
  MissionTask_EnterState(MISSION_STATE_S3_ALIGN_GREEN, now);

  /* A coordinate received before command 03 belongs to the previous camera
     mode and must never feed the new PID loop. */
  MaixCam_ClearObject();
  MaixCam_ClearEvent();
  mission_snapshot.vision_target_valid = false;
  MissionTask_ResetVisionPid();
  s3_align_stable_since = 0U;
  s3_search_turn_stable_since = 0U;
  s3_search_origin_yaw_cdeg = mission_snapshot.yaw_target_cdeg;
  s3_search_yaw_target_cdeg = s3_search_origin_yaw_cdeg;
  s3_green_error_seen = false;
  s3_search_sweep_completed = false;
  s3_search_ccw_midpoint_done = false;
  vision_search_forward_active = false;
  vision_search_resume_state = MISSION_STATE_S3_ALIGN_GREEN;
  s3_target_lost_since = now;
  s3_wait_event04_logged = false;
  s3_next_debug_tick = now;

  if (MaixCam_SendCommand(MAIXCAM_COMMAND_SELECT_GREEN) != HAL_OK)
  {
    mission_snapshot.fault_flags |= MISSION_FAULT_VISION_TX;
    MissionTask_EnterState(MISSION_STATE_FAULT, now);
    return false;
  }
  (void)DebugUart_Logf(
      "[S3] TX E1 E2 %02X 1E 2E, SELECT GREEN ID=5\r\n",
      (unsigned int)MAIXCAM_COMMAND_SELECT_GREEN);

  if (MaixCam_SendCommand(MAIXCAM_COMMAND_SEARCH_TARGET) != HAL_OK)
  {
    mission_snapshot.fault_flags |= MISSION_FAULT_VISION_TX;
    MissionTask_EnterState(MISSION_STATE_FAULT, now);
    return false;
  }
  (void)DebugUart_Log("[S3] TX E1 E2 03 1E 2E, START SEARCH\r\n");
  return true;
}

static bool MissionTask_StartS3Align(uint32_t now)
{
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
    (void)MissionTask_StartS3Align(now);
  }
}

static int16_t MissionTask_CalculateVisionTurn(uint32_t now,
                                               int16_t error_px)
{
  float error = (float)error_px;
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
    s3_pid_output = ROBOT_VISION_X_KP * error +
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
  if ((magnitude > 0.0f) && (magnitude < ROBOT_VISION_X_MIN_TURN_RPM))
  {
    signed_output = signed_output < 0.0f
                        ? -ROBOT_VISION_X_MIN_TURN_RPM
                        : ROBOT_VISION_X_MIN_TURN_RPM;
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

  if (mission_snapshot.state == MISSION_STATE_S3_TRACK_GREEN)
  {
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
  else
  {
    (void)DebugUart_Logf(
        "[S3-X] id=%u xerr=%d yerr=%d turn=%d cmd=%d/%d act=%d/%d\r\n",
        (unsigned int)mission_snapshot.target_id,
        (int)mission_snapshot.vision_x_error_px,
        (int)mission_snapshot.vision_y_error_px,
        (int)mission_snapshot.vision_turn_rpm,
        (int)mission_snapshot.left_target_rpm,
        (int)mission_snapshot.right_target_rpm,
        (int)left_actual_rpm,
        (int)right_actual_rpm);
  }
}

static bool MissionTask_TryStartS3Capture(uint32_t now)
{
  uint8_t event_code;

  if (!MaixCam_TakeEvent(&event_code) ||
      (event_code != MAIXCAM_EVENT_OBJECT_IN_FRAME))
  {
    return false;
  }

  (void)DebugUart_Log("[S3] RX F1 F2 04 1F 2F, OBJECT IN FRAME\r\n");
  MissionTask_ResetVisionPid();
  MaixCam_ClearObject();
  mission_snapshot.vision_target_valid = false;
  if (Actuator_SetCameraWideView() != HAL_OK)
  {
    mission_snapshot.fault_flags |= MISSION_FAULT_ACTUATOR;
    MissionTask_EnterState(MISSION_STATE_FAULT, now);
    return true;
  }

  s4_next_debug_tick = now;
  s4_center_target_lost_since = now;
  s4_center_search_sweep_completed = false;
  MissionTask_EnterState(MISSION_STATE_S4_TRACK_CENTER, now);
  (void)DebugUart_Logf(
      "[S4] RX04 FRAME ALREADY DOWN, CAMERA WIDE=%u, TRACK CENTER\r\n",
      (unsigned int)ROBOT_CAMERA_WIDE_ANGLE_DEG);
  return true;
}

static void MissionTask_StartVisionSearchRecovery(
    uint32_t now, MissionState resume_state, int32_t origin_yaw_cdeg)
{
  MissionTask_StopWheels();
  MissionTask_ResetVisionControllerState();
  vision_search_resume_state = resume_state;
  vision_search_forward_active = true;
  s3_search_origin_yaw_cdeg = MissionTask_WrapYaw(origin_yaw_cdeg);
  s3_search_yaw_target_cdeg = MissionTask_WrapYaw(
      s3_search_origin_yaw_cdeg -
      (int32_t)ROBOT_YAW_LEFT_SIGN * ROBOT_S3_SEARCH_TURN_ANGLE_CDEG);
  s3_search_turn_stable_since = 0U;
  s3_search_ccw_midpoint_done = false;
  s3_next_debug_tick = now;
  MissionTask_EnterState(MISSION_STATE_S3_SEARCH_TURN_CW, now);
  (void)DebugUart_Logf(
      "[V-SEARCH] source=%s FORWARD rpm=%d time=%lums, then CW90 "
      "origin=%ld target=%ld\r\n",
      resume_state == MISSION_STATE_S4_TRACK_CENTER ? "04" : "03",
      ROBOT_VISION_SEARCH_FORWARD_RPM,
      (unsigned long)ROBOT_VISION_SEARCH_FORWARD_MS,
      (long)s3_search_origin_yaw_cdeg,
      (long)s3_search_yaw_target_cdeg);
}

static void MissionTask_RunS3Align(uint32_t now)
{
  int32_t absolute_error;
  int16_t turn_rpm;

  if (mission_snapshot.state != MISSION_STATE_S3_ALIGN_GREEN)
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
    MissionTask_ResetVisionPid();
    s3_align_stable_since = 0U;

    if (s3_target_lost_since == 0U)
    {
      s3_target_lost_since = now;
    }

    if (!s3_search_sweep_completed &&
        ((uint32_t)(now - s3_target_lost_since) >=
         ROBOT_S3_GREEN_WAIT_MS))
    {
      (void)DebugUart_Logf(
          "[S3-S] %s %lums, START COMMON RECOVERY\r\n",
          s3_green_error_seen ? "GREEN LOST" : "NO GREEN",
          (unsigned long)ROBOT_S3_GREEN_WAIT_MS);
      MissionTask_StartVisionSearchRecovery(
          now, MISSION_STATE_S3_ALIGN_GREEN, s3_search_origin_yaw_cdeg);
      return;
    }

    MissionTask_DebugS3(now);
    return;
  }

  s3_green_error_seen = true;
  s3_search_sweep_completed = false;
  s3_target_lost_since = 0U;

  absolute_error = MissionTask_Abs32(mission_snapshot.vision_x_error_px);

  if (absolute_error <= ROBOT_VISION_X_TOLERANCE_PX)
  {
    MissionTask_StopWheels();
    mission_snapshot.vision_turn_rpm = 0;
    s3_pid_integral = 0.0f;
    s3_pid_output = 0.0f;

    if (s3_align_stable_since == 0U)
    {
      s3_align_stable_since = now;
      (void)DebugUart_Log("[S3] X_IN_TOLERANCE\r\n");
    }
    else if ((uint32_t)(now - s3_align_stable_since) >=
             ROBOT_VISION_X_STABLE_MS)
    {
      (void)DebugUart_Logf("[S3] X_ALIGNED xerr=%d yerr=%d\r\n",
                           (int)mission_snapshot.vision_x_error_px,
                           (int)mission_snapshot.vision_y_error_px);
      MissionTask_ResetVisionControllerState();
      s3_track_stable_since = 0U;
      MissionTask_EnterState(MISSION_STATE_S3_TRACK_GREEN, now);
      return;
    }
  }
  else
  {
    s3_align_stable_since = 0U;
    turn_rpm = MissionTask_CalculateVisionTurn(
        now, mission_snapshot.vision_x_error_px);
    mission_snapshot.vision_turn_rpm = turn_rpm;

    /* Positive image error means target right: left wheel forward and right
       wheel backward turns the chassis toward it. */
    MissionTask_SetWheelTargets(turn_rpm, (int16_t)-turn_rpm);
  }

  MissionTask_DebugS3(now);
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

  if ((vision_search_resume_state != MISSION_STATE_S4_TRACK_CENTER) &&
      MissionTask_TryStartS3Capture(now))
  {
    return;
  }

  /* Do not finish a blind search leg after vision has already found green. */
  if (mission_snapshot.vision_target_valid)
  {
    vision_search_forward_active = false;
    MissionTask_ResetVisionControllerState();
    if (vision_search_resume_state == MISSION_STATE_S4_TRACK_CENTER)
    {
      s4_center_search_sweep_completed = false;
      s4_center_target_lost_since = 0U;
      s4_next_debug_tick = now;
      MissionTask_EnterState(MISSION_STATE_S4_TRACK_CENTER, now);
    }
    else
    {
      s3_green_error_seen = true;
      /* Arm one new sweep if this reacquired target is later lost again. */
      s3_search_sweep_completed = false;
      s3_target_lost_since = 0U;
      s3_align_stable_since = 0U;
      MissionTask_EnterState(MISSION_STATE_S3_ALIGN_GREEN, now);
    }
    (void)DebugUart_Logf("[V-SEARCH] source=%s TARGET FOUND x=%d y=%d, "
                         "RESUME PID\r\n",
                         vision_search_resume_state ==
                                 MISSION_STATE_S4_TRACK_CENTER
                             ? "04"
                             : "03",
                         (int)mission_snapshot.vision_x_error_px,
                         (int)mission_snapshot.vision_y_error_px);
    return;
  }

  if (vision_search_forward_active)
  {
    if ((uint32_t)(now - mission_snapshot.state_entry_tick) <
        ROBOT_VISION_SEARCH_FORWARD_MS)
    {
      mission_snapshot.vision_forward_rpm =
          ROBOT_VISION_SEARCH_FORWARD_RPM;
      mission_snapshot.vision_turn_rpm = 0;
      MissionTask_SetWheelTargets(ROBOT_VISION_SEARCH_FORWARD_RPM,
                                  ROBOT_VISION_SEARCH_FORWARD_RPM);
      return;
    }

    MissionTask_StopWheels();
    mission_snapshot.vision_forward_rpm = 0;
    vision_search_forward_active = false;
    mission_snapshot.state_entry_tick = now;
    s3_search_turn_stable_since = 0U;
    (void)DebugUart_Logf(
        "[V-SEARCH] source=%s FORWARD DONE, START CW90\r\n",
        vision_search_resume_state == MISSION_STATE_S4_TRACK_CENTER
            ? "04"
            : "03");
  }

  if ((uint32_t)(now - mission_snapshot.state_entry_tick) >=
      ROBOT_S3_SEARCH_TURN_TIMEOUT_MS)
  {
    (void)DebugUart_Logf("[V-SEARCH] source=%s TURN TIMEOUT yaw=%ld "
                         "target=%ld\r\n",
                         vision_search_resume_state ==
                                 MISSION_STATE_S4_TRACK_CENTER
                             ? "04"
                             : "03",
                         (long)MissionTask_WrapYaw(mission_snapshot.yaw_cdeg),
                         (long)s3_search_yaw_target_cdeg);
    mission_snapshot.fault_flags |=
        vision_search_resume_state == MISSION_STATE_S4_TRACK_CENTER
            ? MISSION_FAULT_S4_TIMEOUT
            : MISSION_FAULT_S3_TIMEOUT;
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
        /* Split the requested counter-clockwise 180-degree sweep into two
           90-degree heading legs. A single target exactly 180 degrees away
           is direction-ambiguous after even a small CW90 overshoot. */
        s3_search_ccw_midpoint_done = false;
        s3_search_yaw_target_cdeg = s3_search_origin_yaw_cdeg;
        MissionTask_EnterState(MISSION_STATE_S3_SEARCH_TURN_CCW, now);
        (void)DebugUart_Logf(
            "[S3-S] CW90 DONE, CCW180 midpoint=%ld\r\n",
            (long)s3_search_yaw_target_cdeg);
      }
      else if (!s3_search_ccw_midpoint_done)
      {
        s3_search_ccw_midpoint_done = true;
        s3_search_yaw_target_cdeg = MissionTask_WrapYaw(
            s3_search_origin_yaw_cdeg +
            (int32_t)ROBOT_YAW_LEFT_SIGN *
                ROBOT_S3_SEARCH_TURN_ANGLE_CDEG);
        (void)DebugUart_Logf(
            "[S3-S] CCW90 DONE, CONTINUE CCW target=%ld\r\n",
            (long)s3_search_yaw_target_cdeg);
      }
      else
      {
        if (vision_search_resume_state == MISSION_STATE_S4_TRACK_CENTER)
        {
          s4_center_search_sweep_completed = true;
          s4_center_target_lost_since = now;
          s4_next_debug_tick = now;
          MissionTask_EnterState(MISSION_STATE_S4_TRACK_CENTER, now);
        }
        else
        {
          s3_search_sweep_completed = true;
          s3_target_lost_since = now;
          MissionTask_EnterState(MISSION_STATE_S3_ALIGN_GREEN, now);
        }
        (void)DebugUart_Logf(
            "[V-SEARCH] source=%s CCW180 DONE, WAIT COORDINATES\r\n",
            vision_search_resume_state == MISSION_STATE_S4_TRACK_CENTER
                ? "04"
                : "03");
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
        clockwise_phase ? "CW90" :
            (s3_search_ccw_midpoint_done ? "CCW180" : "CCW90"),
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
    s3_y_pid_output = ROBOT_VISION_Y_KP * error +
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
      (magnitude < ROBOT_VISION_Y_MIN_FORWARD_RPM))
  {
    signed_output = signed_output < 0.0f
                        ? -ROBOT_VISION_Y_MIN_FORWARD_RPM
                        : ROBOT_VISION_Y_MIN_FORWARD_RPM;
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

static bool MissionTask_CommandS4VisionTracking(uint32_t now,
                                                 bool *xy_in_tolerance)
{
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
    MissionTask_ResetVisionControllerState();
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

  if (turn_rpm > ROBOT_S4_TRACK_TURN_MAX_RPM)
  {
    turn_rpm = ROBOT_S4_TRACK_TURN_MAX_RPM;
  }
  else if (turn_rpm < -ROBOT_S4_TRACK_TURN_MAX_RPM)
  {
    turn_rpm = -ROBOT_S4_TRACK_TURN_MAX_RPM;
  }
  if (forward_rpm > ROBOT_S4_TRACK_FORWARD_MAX_RPM)
  {
    forward_rpm = ROBOT_S4_TRACK_FORWARD_MAX_RPM;
  }
  else if (forward_rpm < -ROBOT_S4_TRACK_FORWARD_MAX_RPM)
  {
    forward_rpm = -ROBOT_S4_TRACK_FORWARD_MAX_RPM;
  }

  mission_snapshot.vision_turn_rpm = turn_rpm;
  mission_snapshot.vision_forward_rpm = forward_rpm;
  if (x_in_tolerance && y_in_tolerance)
  {
    MissionTask_StopWheels();
  }
  else
  {
    MissionTask_SetTrackingTargets(forward_rpm, turn_rpm,
                                   ROBOT_S4_TRACK_WHEEL_MAX_RPM);
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

static void MissionTask_CommandS4CenterFollow(uint32_t now)
{
  int16_t turn_rpm = 0;

  if (mission_snapshot.vision_target_valid &&
      (MissionTask_Abs32(mission_snapshot.vision_x_error_px) >
       ROBOT_VISION_X_TOLERANCE_PX))
  {
    turn_rpm = MissionTask_CalculateVisionTurn(
        now, mission_snapshot.vision_x_error_px);
    if (turn_rpm > ROBOT_S4_CENTER_FOLLOW_TURN_MAX_RPM)
    {
      turn_rpm = ROBOT_S4_CENTER_FOLLOW_TURN_MAX_RPM;
    }
    else if (turn_rpm < -ROBOT_S4_CENTER_FOLLOW_TURN_MAX_RPM)
    {
      turn_rpm = -ROBOT_S4_CENTER_FOLLOW_TURN_MAX_RPM;
    }
  }

  mission_snapshot.vision_turn_rpm = turn_rpm;
  mission_snapshot.vision_forward_rpm = ROBOT_S4_CENTER_FOLLOW_RPM;
  MissionTask_SetTrackingTargets(ROBOT_S4_CENTER_FOLLOW_RPM, turn_rpm,
                                 ROBOT_S4_TRACK_WHEEL_MAX_RPM);

  if ((int32_t)(now - s4_next_debug_tick) >= 0)
  {
    s4_next_debug_tick = now + ROBOT_S4_DEBUG_PERIOD_MS;
    (void)DebugUart_Logf(
        "[S4-X] target=%s xerr=%d turn=%d fwd=%d cmd=%d/%d\r\n",
        mission_snapshot.vision_target_valid ? "OK" : "LOST",
        (int)mission_snapshot.vision_x_error_px,
        (int)turn_rpm,
        ROBOT_S4_CENTER_FOLLOW_RPM,
        (int)mission_snapshot.left_target_rpm,
        (int)mission_snapshot.right_target_rpm);
  }
}

static void MissionTask_CommandS4CornerPush(uint32_t now,
                                             bool push_right)
{
  int16_t wheel_diff_rpm =
      push_right ? ROBOT_S4_RIGHT_CORNER_WHEEL_DIFF_RPM
                 : ROBOT_S4_LEFT_CORNER_WHEEL_DIFF_RPM;
  int16_t faster_rpm =
      (int16_t)(ROBOT_S4_CORNER_PUSH_RPM +
                ((wheel_diff_rpm + 1) / 2));
  int16_t slower_rpm =
      (int16_t)(ROBOT_S4_CORNER_PUSH_RPM -
                (wheel_diff_rpm / 2));
  int16_t left_rpm = push_right ? faster_rpm : slower_rpm;
  int16_t right_rpm = push_right ? slower_rpm : faster_rpm;

  mission_snapshot.vision_turn_rpm =
      push_right ? (int16_t)(wheel_diff_rpm / 2)
                 : (int16_t)-(wheel_diff_rpm / 2);
  mission_snapshot.vision_forward_rpm = ROBOT_S4_CORNER_PUSH_RPM;
  MissionTask_SetWheelTargets(left_rpm, right_rpm);

  if ((int32_t)(now - s4_next_debug_tick) >= 0)
  {
    s4_next_debug_tick = now + ROBOT_S4_DEBUG_PERIOD_MS;
    (void)DebugUart_Logf(
        "[S4-CURVE] phase=%s fixed_diff=%d cmd=%d/%d\r\n",
        push_right ? "02" : "12",
        (int)wheel_diff_rpm,
        (int)mission_snapshot.left_target_rpm,
        (int)mission_snapshot.right_target_rpm);
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
    MissionTask_ResetVisionControllerState();
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
          now, MISSION_STATE_S3_ALIGN_GREEN, s3_search_origin_yaw_cdeg);
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
  MissionTask_SetTrackingTargets(forward_rpm, turn_rpm,
                                 ROBOT_VISION_TRACK_WHEEL_MAX_RPM);
  MissionTask_DebugS3(now);
}

static void MissionTask_RunS3LowerFrame(uint32_t now)
{
  if (mission_snapshot.state != MISSION_STATE_S3_LOWER_FRAME)
  {
    return;
  }

  if ((uint32_t)(now - mission_snapshot.state_entry_tick) <
      ROBOT_S3_FRAME_LOWER_SETTLE_MS)
  {
    return;
  }

  (void)DebugUart_Log("[S3] FRAME DOWN DONE, BEGIN VISION 03\r\n");
  (void)MissionTask_BeginS3Vision(now);
}

static void MissionTask_StartS4TrackState(MissionState state, uint32_t now)
{
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

static void MissionTask_EnterS5(uint32_t now)
{
  uint8_t analyze_command = mission_snapshot.team == ROBOT_TEAM_RED
                                ? MAIXCAM_COMMAND_ANALYZE_LOAD_RED
                                : MAIXCAM_COMMAND_ANALYZE_LOAD_BLUE;

  MaixCam_ClearEvent();
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
}

static void MissionTask_SkipToS4FinalTrack(uint32_t now)
{
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

static void MissionTask_Send24AndEnterS4FinalTrack(uint32_t now)
{
  MissionTask_StopWheels();
  MaixCam_ClearObject();
  mission_snapshot.vision_target_valid = false;
  MissionTask_ResetVisionPid();
  if (Actuator_SetFrameRaised() != HAL_OK)
  {
    mission_snapshot.fault_flags |= MISSION_FAULT_ACTUATOR;
    MissionTask_EnterState(MISSION_STATE_FAULT, now);
    return;
  }
  if (MaixCam_SendCommand(MAIXCAM_COMMAND_FINAL_CAPTURE) != HAL_OK)
  {
    mission_snapshot.fault_flags |= MISSION_FAULT_VISION_TX;
    MissionTask_EnterState(MISSION_STATE_FAULT, now);
    return;
  }

  /* Command 24 is already active while the frame rises, so latch an early
     event 34 in the same way as a 24 initiated by MaixCam. */
  s4_final_mode_already_active = true;
  s4_final_event34_pending = false;
  MissionTask_EnterState(MISSION_STATE_S4_RAISE_FRAME, now);
  (void)DebugUart_Logf(
      "[S4-E2] TX E1 E2 24 1E 2E, FRAME UP settle=%lums, ENTER 24\r\n",
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

  if (!MaixCam_TakeEvent(&event_code))
  {
    return false;
  }
  if (event_code == MAIXCAM_EVENT_SKIP_TO_FINAL_TRACK)
  {
    MissionTask_SkipToS4FinalTrack(now);
    return true;
  }
  if (event_code == MAIXCAM_EVENT_ARRANGE_TARGET_LOST)
  {
    (void)resume_state;
    MissionTask_Send24AndEnterS4FinalTrack(now);
    return true;
  }
  (void)DebugUart_Logf(
      "[S4] RX EVENT %02X, EXPECT E2/24 OR COORDS\r\n",
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
  MissionTask_EnterState(MISSION_STATE_S4_FINAL_CENTER_OBJECT, now);
  (void)DebugUart_Logf(
      "[S4] RX34 FINAL OBJECT IN FRAME, FORWARD rpm=%d time=%lums\r\n",
      ROBOT_S4_FINAL_CENTER_RPM,
      (unsigned long)ROBOT_S4_FINAL_CENTER_MS);
}

static void MissionTask_StartS4FinalRecovery(uint32_t now)
{
  MissionTask_StopWheels();
  MissionTask_ResetVisionPid();
  s4_final_turn_stable_since = 0U;
  ++s4_final_recovery_count;
  MissionTask_EnterState(
      MISSION_STATE_S4_FINAL_RECOVERY_REVERSE, now);
  (void)DebugUart_Logf(
      "[S4-24R] START #%u NO COORD %lums, REVERSE rpm=%d time=%lums\r\n",
      (unsigned int)s4_final_recovery_count,
      (unsigned long)ROBOT_S4_FINAL_MISSING_TIMEOUT_MS,
      ROBOT_S4_FINAL_RECOVERY_REVERSE_RPM,
      (unsigned long)ROBOT_S4_FINAL_RECOVERY_REVERSE_MS);
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
        if (event_code == MAIXCAM_EVENT_CENTER_REACHED)
        {
          MissionTask_StopWheels();
          MissionTask_ResetVisionPid();
          MissionTask_EnterState(
              MISSION_STATE_S4_CENTER_FOLLOW_THROUGH, now);
          (void)DebugUart_Logf(
              "[S4] RX14 CENTER, X TRACK fwd=%d time=%lums\r\n",
              ROBOT_S4_CENTER_FOLLOW_RPM,
              (unsigned long)ROBOT_S4_CENTER_FOLLOW_MS);
          return;
        }
        (void)DebugUart_Logf(
            "[S4] RX EVENT %02X, EXPECT 14\r\n",
            (unsigned int)event_code);
      }
      if (!mission_snapshot.vision_target_valid)
      {
        MissionTask_StopWheels();
        MissionTask_ResetVisionControllerState();
        if (s4_center_target_lost_since == 0U)
        {
          s4_center_target_lost_since = now;
        }
        if (!s4_center_search_sweep_completed &&
            ((uint32_t)(now - s4_center_target_lost_since) >=
             ROBOT_S3_GREEN_WAIT_MS))
        {
          (void)DebugUart_Logf(
              "[S4] TARGET LOST %lums, START SAME RECOVERY AS 03\r\n",
              (unsigned long)ROBOT_S3_GREEN_WAIT_MS);
          MissionTask_StartVisionSearchRecovery(
              now, MISSION_STATE_S4_TRACK_CENTER, mission_snapshot.yaw_cdeg);
          return;
        }
        (void)MissionTask_CommandS4VisionTracking(now, &xy_in_tolerance);
        break;
      }
      else
      {
        s4_center_target_lost_since = 0U;
        s4_center_search_sweep_completed = false;
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
      if (elapsed < ROBOT_S4_CENTER_FOLLOW_MS)
      {
        MissionTask_CommandS4CenterFollow(now);
        return;
      }
      MissionTask_StopWheels();
      MaixCam_ClearEvent();
      MaixCam_ClearObject();
      MissionTask_EnterState(MISSION_STATE_S4_WAIT_ARRANGE_READY, now);
      if (MaixCam_SendCommand(MAIXCAM_COMMAND_CENTER_ACK) != HAL_OK)
      {
        mission_snapshot.fault_flags |= MISSION_FAULT_VISION_TX;
        MissionTask_EnterState(MISSION_STATE_FAULT, now);
        return;
      }
      (void)DebugUart_Log(
          "[S4] TX E1 E2 14 1E 2E, WAIT RX02 ARRANGE READY\r\n");
      break;

    case MISSION_STATE_S4_WAIT_ARRANGE_READY:
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
        (void)DebugUart_Logf(
            "[S4] RX EVENT %02X, EXPECT 02 OR 24\r\n",
            (unsigned int)event_code);
      }
      if (elapsed >= ROBOT_S4_HANDSHAKE_TIMEOUT_MS)
      {
        mission_snapshot.fault_flags |= MISSION_FAULT_S4_TIMEOUT;
        MissionTask_EnterState(MISSION_STATE_FAULT, now);
      }
      break;

    case MISSION_STATE_S4_TRACK_RIGHT_BLOCK:
      if (MissionTask_TryS4ArrangeEvent(
              now, MISSION_STATE_S4_TRACK_RIGHT_BLOCK))
      {
        return;
      }
      if (elapsed < ROBOT_S4_RIGHT_CORNER_FOLLOW_MS)
      {
        MissionTask_CommandS4CornerPush(now, true);
        return;
      }
      MissionTask_ResetVisionPid();
      MissionTask_EnterState(MISSION_STATE_S4_REVERSE_RIGHT_BLOCK, now);
      (void)DebugUart_Logf(
          "[S4] RIGHT X-FOLLOW DONE, REVERSE rpm=%d time=%lums\r\n",
          ROBOT_S4_RIGHT_REVERSE_RPM,
          (unsigned long)ROBOT_S4_RIGHT_REVERSE_MS);
      break;

    case MISSION_STATE_S4_REVERSE_RIGHT_BLOCK:
      if (MissionTask_TryS4ArrangeEvent(
              now, MISSION_STATE_S4_TRACK_RIGHT_BLOCK))
      {
        return;
      }
      if (elapsed < ROBOT_S4_RIGHT_REVERSE_MS)
      {
        MissionTask_SetWheelTargets(-ROBOT_S4_RIGHT_REVERSE_RPM,
                                    -ROBOT_S4_RIGHT_REVERSE_RPM);
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
      if (elapsed < ROBOT_S4_LEFT_CORNER_FOLLOW_MS)
      {
        MissionTask_CommandS4CornerPush(now, false);
        return;
      }
      MissionTask_ResetVisionPid();
      MissionTask_EnterState(MISSION_STATE_S4_REVERSE_LEFT_BLOCK, now);
      (void)DebugUart_Logf(
          "[S4] LEFT X-FOLLOW DONE, REVERSE rpm=%d time=%lums\r\n",
          ROBOT_S4_LEFT_REVERSE_RPM,
          (unsigned long)ROBOT_S4_LEFT_REVERSE_MS);
      break;

    case MISSION_STATE_S4_REVERSE_LEFT_BLOCK:
      if (MissionTask_TryS4ArrangeEvent(
              now, MISSION_STATE_S4_TRACK_LEFT_BLOCK))
      {
        return;
      }
      if (elapsed < ROBOT_S4_LEFT_REVERSE_MS)
      {
        MissionTask_SetWheelTargets(-ROBOT_S4_LEFT_REVERSE_RPM,
                                    -ROBOT_S4_LEFT_REVERSE_RPM);
        return;
      }
      MissionTask_StopWheels();
      if (Actuator_SetFrameRaised() != HAL_OK)
      {
        mission_snapshot.fault_flags |= MISSION_FAULT_ACTUATOR;
        MissionTask_EnterState(MISSION_STATE_FAULT, now);
        return;
      }
      s4_final_mode_already_active = false;
      s4_final_event34_pending = false;
      MissionTask_EnterState(MISSION_STATE_S4_RAISE_FRAME, now);
      (void)DebugUart_Logf(
          "[S4] LEFT REVERSE DONE, FRAME UP settle=%lums\r\n",
          (unsigned long)ROBOT_S4_FRAME_RAISE_SETTLE_MS);
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
      if (elapsed < ROBOT_S4_FRAME_RAISE_SETTLE_MS)
      {
        return;
      }
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
      (void)DebugUart_Log(
          "[S4-24R] REVERSE DONE, FORCE LEFT TO YAW 270\r\n");
      break;

    case MISSION_STATE_S4_FINAL_RECOVERY_TURN_270:
      if (MissionTask_TryResumeS4FinalRecovery(now))
      {
        return;
      }
      if (MissionTask_RunS4ForcedLeftTurn(
              now, ROBOT_S4_FINAL_RECOVERY_TURN_270_CDEG))
      {
        MissionTask_StopWheels();
        MissionTask_ResetS6ControllerState();
        s6_phase_stable_since = 0U;
        s6_next_debug_tick = now;
        MissionTask_EnterState(
            MISSION_STATE_S4_FINAL_RECOVERY_TURN_90, now);
        (void)DebugUart_Log(
            "[S4-24R] YAW270 DONE, TURN TO YAW90\r\n");
      }
      break;

    case MISSION_STATE_S4_FINAL_RECOVERY_TURN_90:
      if (MissionTask_TryResumeS4FinalRecovery(now))
      {
        return;
      }
      if (MissionTask_RunS6TurnToHeading(
              now, ROBOT_S4_FINAL_RECOVERY_TURN_90_CDEG))
      {
        MissionTask_StopWheels();
        MissionTask_ResetS6ControllerState();
        MaixCam_ClearObject();
        mission_snapshot.vision_target_valid = false;
        MissionTask_ResetVisionPid();
        s4_final_missing_since = now;
        s4_next_debug_tick = now;
        MissionTask_EnterState(
            MISSION_STATE_S4_TRACK_FINAL_BLOCK, now);
        (void)DebugUart_Log(
            "[S4-24R] YAW90 DONE, RESUME 24 WAIT COORDS\r\n");
      }
      break;

    case MISSION_STATE_S4_FINAL_CENTER_OBJECT:
      if (elapsed < ROBOT_S4_FINAL_CENTER_MS)
      {
        MissionTask_SetWheelTargets(ROBOT_S4_FINAL_CENTER_RPM,
                                    ROBOT_S4_FINAL_CENTER_RPM);
        return;
      }
      MissionTask_StopWheels();
      if (Actuator_SetFrameLowered() != HAL_OK)
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

  if (mission_snapshot.state != MISSION_STATE_S5_WAIT_SINGLE_GREEN)
  {
    return;
  }

  if (!MaixCam_TakeEvent(&event_code))
  {
    if ((int32_t)(now - s5_next_debug_tick) >= 0)
    {
      s5_next_debug_tick = now + ROBOT_S5_DEBUG_PERIOD_MS;
      (void)DebugUart_Logf(
          "[S5] WAIT RX %s elapsed=%lums team=%s camera=%s\r\n",
          s5_close_view_active ? "02/06/24" : "05",
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
    s5_close_view_active = true;
    s5_next_debug_tick = now;
    (void)DebugUart_Logf(
        "[S5] RX05 LOAD CHECK, CAMERA NEAR angle=%u, WAIT RX02/06\r\n",
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
    (void)DebugUart_Logf(
        "[S5] RX24 NO OBJECT IN 05, CAMERA WIDE=%u, ENTER 24 TRACK\r\n",
        (unsigned int)ROBOT_CAMERA_WIDE_ANGLE_DEG);
    MissionTask_SkipToS4FinalTrack(now);
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
        "[S5] RX EVENT %02X, EXPECT 05/02/06/24, KEEP WAITING\r\n",
        (unsigned int)event_code);
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

  MaixCam_ClearObject();
  mission_snapshot.vision_target_valid = false;
  MissionTask_ResetVisionPid();
  MissionTask_ResetS6ControllerState();
  s6_phase_stable_since = 0U;
  s6_next_debug_tick = now;
  s6_missing_since = 0U;
  s6_search_start_tick = now;
  s6_recovery_count = 0U;
  MissionTask_EnterState(MISSION_STATE_S6_SEARCH_SAFE_ZONE, now);
  (void)DebugUart_Logf(
      "[S6] SHORTEST TURN TO SAFE yaw=%ld err=%ld max=%d timeout=%lums\r\n",
      (long)mission_snapshot.safe_zone_yaw_target_cdeg,
      (long)MissionTask_YawError(mission_snapshot.safe_zone_yaw_target_cdeg,
                                 mission_snapshot.yaw_cdeg),
      (int)ROBOT_S6_YAW_MAX_TURN_RPM,
      (unsigned long)ROBOT_S6_SEARCH_TIMEOUT_MS);
}

static bool MissionTask_S6NeedsSideReposition(void)
{
  int32_t yaw_error = MissionTask_YawError(
      mission_snapshot.safe_zone_yaw_target_cdeg,
      mission_snapshot.yaw_cdeg);

  /* Example: start zone 4 / blue has safe heading 180 degrees. Yaw in
     (180, 270] produces an error in [-90, 0), so reposition sideways first.
     The same relative sector applies to every mapped safe-zone heading. */
  return (yaw_error < -ROBOT_S6_REPOSITION_MIN_OFFSET_CDEG) &&
         (yaw_error >= -ROBOT_S6_REPOSITION_SECTOR_CDEG);
}

static void MissionTask_StartS6SideReposition(uint32_t now)
{
  MissionTask_StopWheels();
  MaixCam_ClearObject();
  mission_snapshot.vision_target_valid = false;
  MissionTask_ResetVisionPid();
  MissionTask_ResetS6ControllerState();
  s6_phase_stable_since = 0U;
  s6_reposition_x_stable_since = 0U;
  s6_approach_start_tick = now;
  s6_reposition_side_yaw_cdeg = MissionTask_WrapYaw(
      mission_snapshot.safe_zone_yaw_target_cdeg +
      (int32_t)ROBOT_YAW_LEFT_SIGN *
          ROBOT_S6_REPOSITION_SECTOR_CDEG);
  ++s6_reposition_count;
  s6_next_debug_tick = now;
  MissionTask_EnterState(MISSION_STATE_S6_REPOSITION_TURN_SIDE, now);
  (void)DebugUart_Logf(
      "[S6-P] START #%u yaw=%ld safe=%ld side=%ld\r\n",
      (unsigned int)s6_reposition_count,
      (long)MissionTask_WrapYaw(mission_snapshot.yaw_cdeg),
      (long)mission_snapshot.safe_zone_yaw_target_cdeg,
      (long)s6_reposition_side_yaw_cdeg);
}

static void MissionTask_StartS6VisionApproach(uint32_t now)
{
  MissionTask_StopWheels();
  MissionTask_ResetVisionPid();
  MissionTask_ResetS6ControllerState();
  s6_approach_start_tick = now;
  s6_missing_since = mission_snapshot.vision_target_valid ? 0U : now;
  s6_recovery_count = 0U;
  s6_next_debug_tick = now;
  MissionTask_EnterState(MISSION_STATE_S6_TRACK_SAFE_ZONE, now);
  (void)DebugUart_Logf(
      "[S6] VISION PID APPROACH yaw=%ld safe=%ld reposition=%u\r\n",
      (long)MissionTask_WrapYaw(mission_snapshot.yaw_cdeg),
      (long)mission_snapshot.safe_zone_yaw_target_cdeg,
      (unsigned int)s6_reposition_count);
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
  s6_reposition_x_stable_since = 0U;
  s6_pre_reposition_track_start_tick = 0U;
  s6_pre_reposition_decision_pending = true;
  s6_recovery_count = 0U;
  s6_next_debug_tick = now;
  MissionTask_StartS6VisionApproach(now);
  (void)DebugUart_Logf(
      "[S6] TRACK %lums BEFORE SIDE REPOSITION DECISION\r\n",
      (unsigned long)ROBOT_S6_PRE_REPOSITION_TRACK_MS);
}

static void MissionTask_StartS6Recovery(uint32_t now,
                                         bool from_tracking)
{
  bool use_long_forward;
  int16_t forward_rpm;
  uint32_t forward_ms;

  s6_recovery_base_yaw_cdeg = MissionTask_WrapYaw(
      mission_snapshot.yaw_cdeg);
  s6_recovery_left_yaw_cdeg = MissionTask_WrapYaw(
      s6_recovery_base_yaw_cdeg +
      (int32_t)ROBOT_YAW_LEFT_SIGN *
          ROBOT_S6_RECOVERY_LEFT_ANGLE_CDEG);
  s6_recovery_right_yaw_cdeg = MissionTask_WrapYaw(
      s6_recovery_left_yaw_cdeg -
      (int32_t)ROBOT_YAW_LEFT_SIGN *
          ROBOT_S6_RECOVERY_RIGHT_ANGLE_CDEG);
  s6_recovery_from_tracking = from_tracking;
  ++s6_recovery_count;
  use_long_forward =
      s6_recovery_count >= ROBOT_S6_RECOVERY_LONG_START_COUNT;
  forward_rpm = use_long_forward
                    ? ROBOT_S6_RECOVERY_LONG_FORWARD_RPM
                    : ROBOT_S6_RECOVERY_FORWARD_RPM;
  forward_ms = use_long_forward
                   ? ROBOT_S6_RECOVERY_LONG_FORWARD_MS
                   : ROBOT_S6_RECOVERY_FORWARD_MS;
  s6_missing_since = 0U;
  s6_phase_stable_since = 0U;
  MissionTask_ResetVisionPid();
  MissionTask_ResetS6ControllerState();
  s6_next_debug_tick = now;
  MissionTask_EnterState(MISSION_STATE_S6_RECOVERY_FORWARD, now);
  (void)DebugUart_Logf(
      "[S6-R] START #%u source=%s base=%ld left45=%ld right90=%ld fwd=%d/%lums\r\n",
      (unsigned int)s6_recovery_count,
      from_tracking ? "TRACK_LOST" : "WAIT16",
      (long)s6_recovery_base_yaw_cdeg,
      (long)s6_recovery_left_yaw_cdeg,
      (long)s6_recovery_right_yaw_cdeg,
      (int)forward_rpm,
      (unsigned long)forward_ms);
}

static void MissionTask_RunS6SearchSafeZone(uint32_t now)
{
  uint8_t event_code;
  int32_t absolute_yaw_error;
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
  mission_snapshot.safe_zone_fixed_heading_active = true;
  mission_snapshot.vision_forward_rpm = 0;
  if (absolute_yaw_error <= ROBOT_S6_TURN_TOLERANCE_CDEG)
  {
    turn_rpm = 0;
    MissionTask_StopWheels();
    if (s6_missing_since == 0U)
    {
      s6_missing_since = now;
      (void)DebugUart_Logf(
          "[S6] SAFE HEADING REACHED, WAIT RX16 %lums\r\n",
          (unsigned long)ROBOT_S6_RECOVERY_WAIT_MS);
    }
    else if ((uint32_t)(now - s6_missing_since) >=
             ROBOT_S6_RECOVERY_WAIT_MS)
    {
      MissionTask_StartS6Recovery(now, false);
      return;
    }
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
                                  -ROBOT_S6_YAW_MAX_TURN_RPM,
                                  ROBOT_S6_YAW_MAX_TURN_RPM);

  s6_yaw_pid_last_error = error;
  s6_yaw_pid_update_tick = now;

  /* Positive yaw error means physical-left when ROBOT_YAW_LEFT_SIGN is +1,
     while a negative wheel-differential command turns this chassis left. */
  output *= -(float)ROBOT_YAW_LEFT_SIGN;
  magnitude = output < 0.0f ? -output : output;
  if ((magnitude > 0.0f) && (magnitude < ROBOT_S6_YAW_MIN_TURN_RPM))
  {
    output = output < 0.0f ? -ROBOT_S6_YAW_MIN_TURN_RPM
                           : ROBOT_S6_YAW_MIN_TURN_RPM;
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

  if ((uint32_t)(now - mission_snapshot.state_entry_tick) >=
      ROBOT_S6_TURN_TIMEOUT_MS)
  {
    mission_snapshot.fault_flags |= MISSION_FAULT_S6_TIMEOUT;
    MissionTask_EnterState(MISSION_STATE_FAULT, now);
    return false;
  }

  turn_rpm = MissionTask_CalculateS6YawTurn(now, target_yaw_cdeg);
  absolute_yaw_error = MissionTask_Abs32(
      mission_snapshot.safe_zone_yaw_error_cdeg);

  if (absolute_yaw_error <= ROBOT_S6_TURN_TOLERANCE_CDEG)
  {
    MissionTask_StopWheels();
    mission_snapshot.vision_turn_rpm = 0;
    mission_snapshot.vision_forward_rpm = 0;
    s6_yaw_pid_integral = 0.0f;
    if (s6_phase_stable_since == 0U)
    {
      s6_phase_stable_since = now;
    }
    else if ((uint32_t)(now - s6_phase_stable_since) >=
             ROBOT_S6_TURN_STABLE_MS)
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
      if (elapsed < ROBOT_S5_ARRANGE_FRAME_RAISE_SETTLE_MS)
      {
        return;
      }
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
  int32_t absolute_yaw_error;
  int16_t turn_rpm = MissionTask_CalculateS6YawTurn(now, target_yaw_cdeg);

  absolute_yaw_error = MissionTask_Abs32(
      mission_snapshot.safe_zone_yaw_error_cdeg);
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
  int32_t absolute_x_error;
  int16_t turn_rpm;

  if ((mission_snapshot.state != MISSION_STATE_S6_REPOSITION_TURN_SIDE) &&
      (mission_snapshot.state != MISSION_STATE_S6_REPOSITION_FORWARD) &&
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
      s6_reposition_x_stable_since = 0U;
      s6_missing_since = now;
      s6_next_debug_tick = now;
      MissionTask_EnterState(
          MISSION_STATE_S6_REPOSITION_FACE_SAFE, now);
      (void)DebugUart_Logf(
          "[S6-P] FORWARD DONE, WAIT SAFE-ZONE COORDINATES %lums\r\n",
          (unsigned long)ROBOT_S6_RECOVERY_WAIT_MS);
      break;

    case MISSION_STATE_S6_REPOSITION_FACE_SAFE:
      if (!mission_snapshot.vision_target_valid)
      {
        MissionTask_StopWheels();
        MissionTask_ResetVisionControllerState();
        s6_reposition_x_stable_since = 0U;
        if ((int32_t)(now - s6_next_debug_tick) >= 0)
        {
          s6_next_debug_tick = now + ROBOT_S6_DEBUG_PERIOD_MS;
          (void)DebugUart_Log(
              "[S6-P] WAIT FRESH SAFE-ZONE COORDINATES\r\n");
        }
        if ((uint32_t)(now - s6_missing_since) >=
            ROBOT_S6_RECOVERY_WAIT_MS)
        {
          (void)DebugUart_Logf(
              "[S6-P] COORDINATES MISSING %lums, START RECOVERY\r\n",
              (unsigned long)ROBOT_S6_RECOVERY_WAIT_MS);
          MissionTask_StartS6Recovery(now, true);
        }
        return;
      }

      s6_missing_since = 0U;
      absolute_x_error = MissionTask_Abs32(
          mission_snapshot.vision_x_error_px);
      if (absolute_x_error <= ROBOT_VISION_X_TOLERANCE_PX)
      {
        MissionTask_StopWheels();
        MissionTask_ResetVisionControllerState();
        if (s6_reposition_x_stable_since == 0U)
        {
          s6_reposition_x_stable_since = now;
        }
        else if ((uint32_t)(now - s6_reposition_x_stable_since) >=
                 ROBOT_S6_REPOSITION_X_STABLE_MS)
        {
          (void)DebugUart_Logf(
              "[S6-P] X ALIGNED yaw=%ld safe=%ld sector=%s\r\n",
              (long)MissionTask_WrapYaw(mission_snapshot.yaw_cdeg),
              (long)mission_snapshot.safe_zone_yaw_target_cdeg,
              MissionTask_S6NeedsSideReposition() ? "REPEAT" : "TRACK");
          if (MissionTask_S6NeedsSideReposition())
          {
            MissionTask_StartS6SideReposition(now);
          }
          else
          {
            MissionTask_StartS6VisionApproach(now);
          }
          return;
        }
      }
      else
      {
        s6_reposition_x_stable_since = 0U;
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
        mission_snapshot.vision_turn_rpm = turn_rpm;
        mission_snapshot.vision_forward_rpm = 0;
        MissionTask_SetTrackingTargets(
            0, turn_rpm, ROBOT_S6_APPROACH_WHEEL_MAX_RPM);
      }
      MissionTask_DebugS6(now);
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
        (event_code == MAIXCAM_EVENT_SAFE_ZONE_ALIGN_READY))
    {
      s6_push_yaw_target_cdeg = MissionTask_WrapYaw(
          mission_snapshot.safe_zone_yaw_target_cdeg);
      MissionTask_StopWheels();
      MaixCam_ClearObject();
      mission_snapshot.vision_target_valid = false;
      MissionTask_ResetVisionPid();
      MissionTask_ResetS6ControllerState();
      s6_phase_stable_since = 0U;
      s6_next_debug_tick = now;
      MissionTask_EnterState(MISSION_STATE_S6_FINAL_ALIGN, now);
      (void)DebugUart_Logf(
          "[S6-R] RX26 DURING RECOVERY, ALIGN NORMAL yaw=%ld\r\n",
          (long)s6_push_yaw_target_cdeg);
      return true;
    }
    (void)DebugUart_Logf("[S6-R] RX EVENT %02X, EXPECT 16\r\n",
                         (unsigned int)event_code);
  }

  if (s6_recovery_from_tracking &&
      mission_snapshot.vision_target_valid)
  {
    MissionTask_StopWheels();
    MissionTask_ResetVisionControllerState();
    MissionTask_ResetS6ControllerState();
    s6_approach_start_tick = now;
    s6_missing_since = 0U;
    s6_next_debug_tick = now;
    MissionTask_EnterState(MISSION_STATE_S6_TRACK_SAFE_ZONE, now);
    (void)DebugUart_Logf(
        "[S6-R] COORDINATES RECOVERED x=%d y=%d, RESUME PID\r\n",
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
  int16_t forward_rpm;
  int16_t wheel_max_rpm;
  uint32_t forward_ms;

  if ((mission_snapshot.state != MISSION_STATE_S6_RECOVERY_FORWARD) &&
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
  forward_rpm = use_long_forward
                    ? ROBOT_S6_RECOVERY_LONG_FORWARD_RPM
                    : ROBOT_S6_RECOVERY_FORWARD_RPM;
  forward_ms = use_long_forward
                   ? ROBOT_S6_RECOVERY_LONG_FORWARD_MS
                   : ROBOT_S6_RECOVERY_FORWARD_MS;
  wheel_max_rpm = use_long_forward
                      ? ROBOT_S6_RECOVERY_LONG_WHEEL_MAX_RPM
                      : ROBOT_S6_RECOVERY_WHEEL_MAX_RPM;

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
    case MISSION_STATE_S6_RECOVERY_FORWARD:
      if ((uint32_t)(now - mission_snapshot.state_entry_tick) <
          forward_ms)
      {
        MissionTask_CommandS6Straight(
            now, s6_recovery_base_yaw_cdeg,
            forward_rpm, wheel_max_rpm);
        return;
      }
      MissionTask_StopWheels();
      MissionTask_ResetS6ControllerState();
      s6_phase_stable_since = 0U;
      s6_next_debug_tick = now;
      MissionTask_EnterState(MISSION_STATE_S6_RECOVERY_TURN_LEFT, now);
      (void)DebugUart_Logf("[S6-R] FORWARD DONE, LEFT45 target=%ld\r\n",
                           (long)s6_recovery_left_yaw_cdeg);
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
            "[S6-R] LEFT45 DONE, RIGHT90 target=%ld\r\n",
            (long)s6_recovery_right_yaw_cdeg);
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
        s6_approach_start_tick = now;
        MissionTask_EnterState(MISSION_STATE_S6_TRACK_SAFE_ZONE, now);
        (void)DebugUart_Logf(
            "[S6-R] SAFE HEADING RESTORED, WAIT COORDINATES %lums\r\n",
            (unsigned long)ROBOT_S6_RECOVERY_WAIT_MS);
      }
      else
      {
        MissionTask_EnterState(MISSION_STATE_S6_SEARCH_SAFE_ZONE, now);
        (void)DebugUart_Logf(
            "[S6-R] SAFE HEADING RESTORED, WAIT RX16 %lums\r\n",
            (unsigned long)ROBOT_S6_RECOVERY_WAIT_MS);
      }
      break;

    default:
      break;
  }
}

static void MissionTask_RunS6TrackSafeZone(uint32_t now)
{
  uint8_t event_code;
  int32_t absolute_x_error;
  int32_t absolute_y_error;
  int16_t turn_rpm;
  int16_t forward_rpm;

  if (mission_snapshot.state != MISSION_STATE_S6_TRACK_SAFE_ZONE)
  {
    return;
  }

  if (MaixCam_TakeEvent(&event_code) &&
      (event_code == MAIXCAM_EVENT_SAFE_ZONE_ALIGN_READY))
  {
    s6_push_yaw_target_cdeg =
        MissionTask_WrapYaw(mission_snapshot.safe_zone_yaw_target_cdeg);
    (void)DebugUart_Logf(
        "[S6] RX F1 F2 26 1F 2F, ALIGN NORMAL yaw=%ld\r\n",
        (long)s6_push_yaw_target_cdeg);
    MissionTask_StopWheels();
    MaixCam_ClearObject();
    mission_snapshot.vision_target_valid = false;
    MissionTask_ResetVisionPid();
    MissionTask_ResetS6ControllerState();
    s6_phase_stable_since = 0U;
    s6_next_debug_tick = now;
    MissionTask_EnterState(MISSION_STATE_S6_FINAL_ALIGN, now);
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
    MissionTask_ResetVisionControllerState();
    if (s6_pre_reposition_decision_pending)
    {
      s6_pre_reposition_track_start_tick = 0U;
    }
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

  if (s6_pre_reposition_decision_pending)
  {
    if (s6_pre_reposition_track_start_tick == 0U)
    {
      s6_pre_reposition_track_start_tick = now;
      (void)DebugUart_Logf(
          "[S6] FRESH COORDINATES, START %lums PRE-DECISION TRACK\r\n",
          (unsigned long)ROBOT_S6_PRE_REPOSITION_TRACK_MS);
    }
    else if ((uint32_t)(now - s6_pre_reposition_track_start_tick) >=
             ROBOT_S6_PRE_REPOSITION_TRACK_MS)
    {
      bool needs_side_reposition = MissionTask_S6NeedsSideReposition();
      s6_pre_reposition_decision_pending = false;
      s6_pre_reposition_track_start_tick = 0U;
      (void)DebugUart_Logf(
          "[S6] PRE-DECISION TRACK DONE yaw=%ld safe=%ld action=%s\r\n",
          (long)MissionTask_WrapYaw(mission_snapshot.yaw_cdeg),
          (long)mission_snapshot.safe_zone_yaw_target_cdeg,
          needs_side_reposition ? "SIDE" : "CONTINUE_PID");
      if (needs_side_reposition)
      {
        MissionTask_StartS6SideReposition(now);
        return;
      }
    }
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
    if (forward_rpm > ROBOT_S6_APPROACH_RPM)
    {
      forward_rpm = ROBOT_S6_APPROACH_RPM;
    }
    else if (forward_rpm < -ROBOT_S6_APPROACH_RPM)
    {
      forward_rpm = -ROBOT_S6_APPROACH_RPM;
    }
  }

  mission_snapshot.safe_zone_fixed_heading_active = false;
  mission_snapshot.vision_turn_rpm = turn_rpm;
  mission_snapshot.vision_forward_rpm = forward_rpm;
  MissionTask_SetTrackingTargets(forward_rpm, turn_rpm,
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
  MissionTask_EnterState(MISSION_STATE_S6_PRE_PUSH_REVERSE, now);
  (void)DebugUart_Logf(
      "[S6] NORMAL ALIGNED yaw=%ld, PRE-REVERSE rpm=%d time=%lums\r\n",
      (long)s6_push_yaw_target_cdeg,
      ROBOT_S6_PRE_PUSH_REVERSE_RPM,
      (unsigned long)ROBOT_S6_PRE_PUSH_REVERSE_MS);
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
  MissionTask_EnterState(MISSION_STATE_S6_FINAL_PUSH, now);
  (void)DebugUart_Logf(
      "[S6] PRE-REVERSE DONE, FINAL PUSH rpm=%d time=%lums\r\n",
      ROBOT_S6_FINAL_PUSH_RPM,
      (unsigned long)ROBOT_S6_FINAL_PUSH_MS);
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

  if (elapsed < ROBOT_S6_REVERSE_BRAKE_MS)
  {
    MissionTask_StopWheels();
    return;
  }

  if (elapsed >= (ROBOT_S6_REVERSE_BRAKE_MS +
                  ROBOT_S6_REVERSE_DRIVE_MS))
  {
    MissionTask_StopWheels();
    s6_exit_yaw_target_cdeg = MissionTask_WrapYaw(
        s6_push_yaw_target_cdeg +
        (int32_t)ROBOT_YAW_LEFT_SIGN * ROBOT_S6_EXIT_TURN_ANGLE_CDEG);
    MissionTask_ResetS6ControllerState();
    s6_phase_stable_since = 0U;
    s6_next_debug_tick = now;
    MissionTask_EnterState(MISSION_STATE_S6_EXIT_TURN_180, now);
    (void)DebugUart_Logf(
        "[S6] REVERSE DONE rpm=%d time=%lums, TURN180 target=%ld\r\n",
        ROBOT_S6_REVERSE_RPM,
        (unsigned long)ROBOT_S6_REVERSE_DRIVE_MS,
        (long)s6_exit_yaw_target_cdeg);
    return;
  }

  mission_snapshot.safe_zone_fixed_heading_active = true;
  MissionTask_CommandS6Straight(now, s6_push_yaw_target_cdeg,
                                -ROBOT_S6_REVERSE_RPM,
                                ROBOT_S6_REVERSE_WHEEL_MAX_RPM);
}

static void MissionTask_RunS6ExitTurn180(uint32_t now)
{
  if (mission_snapshot.state != MISSION_STATE_S6_EXIT_TURN_180)
  {
    return;
  }

  if (!MissionTask_RunS6TurnToHeading(now, s6_exit_yaw_target_cdeg))
  {
    return;
  }

  MissionTask_StopWheels();
  (void)DebugUart_Logf("[S6] TURN180 DONE yaw=%ld, STOP\r\n",
                       (long)s6_exit_yaw_target_cdeg);
  MissionTask_EnterState(MISSION_STATE_S6_SAFE_ZONE_REACHED, now);
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
      (mission_snapshot.state != MISSION_STATE_S6_REPOSITION_FACE_SAFE) &&
      (mission_snapshot.state != MISSION_STATE_S6_RECOVERY_FORWARD) &&
      (mission_snapshot.state != MISSION_STATE_S6_RECOVERY_TURN_LEFT) &&
      (mission_snapshot.state != MISSION_STATE_S6_RECOVERY_TURN_BACK) &&
      (mission_snapshot.state != MISSION_STATE_S6_RECOVERY_RETURN_SAFE) &&
      (mission_snapshot.state != MISSION_STATE_S6_TRACK_SAFE_ZONE) &&
      (mission_snapshot.state != MISSION_STATE_S6_FINAL_ALIGN) &&
      (mission_snapshot.state != MISSION_STATE_S6_RAISE_FRAME) &&
      (mission_snapshot.state != MISSION_STATE_S6_PRE_PUSH_REVERSE) &&
      (mission_snapshot.state != MISSION_STATE_S6_FINAL_PUSH) &&
      (mission_snapshot.state != MISSION_STATE_S6_FINAL_REVERSE) &&
      (mission_snapshot.state != MISSION_STATE_S6_EXIT_TURN_180))
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
      MissionTask_UpdateInputs(now);
      MissionTask_ProcessEvents(events, now);
      MissionTask_CheckRunningHealth(now);
      MissionTask_RunS1(now);
      MissionTask_RunS2CrossBump(now);
      MissionTask_RunS3Align(now);
      MissionTask_RunS3SearchTurn(now);
      MissionTask_RunS3Track(now);
      MissionTask_RunS3LowerFrame(now);
      MissionTask_RunS4Arrange(now);
      MissionTask_RunS5WaitSingleGreen(now);
      MissionTask_RunS5PrepareArrange(now);
      MissionTask_RunS6SearchSafeZone(now);
      MissionTask_RunS6SideReposition(now);
      MissionTask_RunS6Recovery(now);
      MissionTask_RunS6TrackSafeZone(now);
      MissionTask_RunS6FinalAlign(now);
      MissionTask_RunS6RaiseFrame(now);
      MissionTask_RunS6PrePushReverse(now);
      MissionTask_RunS6FinalPush(now);
      MissionTask_RunS6FinalReverse(now);
      MissionTask_RunS6ExitTurn180(now);
      (void)osMutexRelease(mission_snapshot_mutex);
    }
  }
}
