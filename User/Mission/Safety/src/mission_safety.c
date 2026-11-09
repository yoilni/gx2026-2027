#include "mission_safety.h"
#include "mission_internal.h"
#include "mission_state.h"
#include "mission_vision.h"
#include "mission_safe_zone.h"

#include "M2006.h"
#include "chassis_motion.h"
#include "maixcam_task.h"
#include "robot_config.h"

void MissionTask_SetWheelTargets(int16_t left_rpm,
                                        int16_t right_rpm)
{
  if ((left_rpm == mission_ctx.commanded_left_rpm) &&
      (right_rpm == mission_ctx.commanded_right_rpm))
  {
    return;
  }

  mission_ctx.commanded_left_rpm = left_rpm;
  mission_ctx.commanded_right_rpm = right_rpm;
  mission_ctx.mission_snapshot.left_target_rpm = left_rpm;
  mission_ctx.mission_snapshot.right_target_rpm = right_rpm;

  ChassisMotion_SetWheelRpm(left_rpm, right_rpm, ROBOT_S1_CURRENT_LIMIT);
}

void MissionTask_StopWheels(void)
{
  ChassisMotion_Stop();
  mission_ctx.commanded_left_rpm = 0;
  mission_ctx.commanded_right_rpm = 0;
  mission_ctx.mission_snapshot.left_target_rpm = 0;
  mission_ctx.mission_snapshot.right_target_rpm = 0;
}

bool MissionTask_MotorIsOnline(uint8_t motor_index, uint32_t now)
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

static uint32_t MissionTask_GetRequiredFaults(void)
{
  uint32_t faults = MISSION_FAULT_NONE;

  if (!mission_ctx.mission_snapshot.left_motor_online)
  {
    faults |= MISSION_FAULT_LEFT_MOTOR;
  }
  if (!mission_ctx.mission_snapshot.right_motor_online)
  {
    faults |= MISSION_FAULT_RIGHT_MOTOR;
  }
  if (!mission_ctx.mission_snapshot.imu_valid)
  {
    faults |= MISSION_FAULT_IMU;
  }

  return faults;
}

void MissionTask_ProcessEvents(uint32_t events, uint32_t now)
{
  if ((events & MISSION_EVENT_EMERGENCY_STOP) != 0U)
  {
    mission_ctx.mission_snapshot.fault_flags |= MISSION_FAULT_EMERGENCY;
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
    mission_ctx.s7_material_delivery_count = 0U;
    mission_ctx.s7_casualty_delivery_count = 0U;
    mission_ctx.s7_delivery_counted = false;
    mission_ctx.s7_material_only_reselect_pending = false;
    mission_ctx.s7_target_switch_enabled = false;
    mission_ctx.s7_red_recovery_used = false;
    mission_ctx.s51_active_object_id = 0U;
    mission_ctx.s6_carried_object_id = 0U;
    mission_ctx.s5_load_class_confirmed = false;
    mission_ctx.s7_red_switch_sync_pending = false;
    mission_ctx.s7_red_coordinate_fence_active = false;
    mission_ctx.s3_target_select_command = MAIXCAM_COMMAND_SELECT_GREEN;
    mission_ctx.s4_post22_search_active = false;
    mission_ctx.mission_snapshot.fault_flags = MISSION_FAULT_NONE;
    MissionTask_SetS1Phase(MISSION_S1_IDLE, now);
    MissionTask_ResetVisionPid();
    MissionTask_ResetS6ControllerState();
    MaixCam_ClearObject();
    MaixCam_ClearEvent();
    mission_ctx.mission_snapshot.vision_target_valid = false;
    MissionTask_EnterState(MISSION_STATE_WAIT_START, now);
    return;
  }

  if (((events & MISSION_EVENT_START) != 0U) &&
      ((mission_ctx.mission_snapshot.state == MISSION_STATE_WAIT_START) ||
       (mission_ctx.mission_snapshot.state == MISSION_STATE_STOPPED)))
  {
    uint32_t required_faults = MissionTask_GetRequiredFaults();
    if (required_faults == MISSION_FAULT_NONE)
    {
      MissionTask_StartS1(now);
    }
    else
    {
      mission_ctx.mission_snapshot.fault_flags |= required_faults;
      MissionTask_EnterState(MISSION_STATE_FAULT, now);
    }
  }
}

void MissionTask_CheckRunningHealth(uint32_t now)
{
  uint32_t required_faults;

  if ((mission_ctx.mission_snapshot.state != MISSION_STATE_S1_DEPART) &&
      (mission_ctx.mission_snapshot.state != MISSION_STATE_S2_CROSS_BUMP) &&
      (mission_ctx.mission_snapshot.state != MISSION_STATE_S3_ALIGN_GREEN) &&
      (mission_ctx.mission_snapshot.state != MISSION_STATE_S3_SEARCH_TURN_CW) &&
      (mission_ctx.mission_snapshot.state != MISSION_STATE_S3_SEARCH_TURN_CCW) &&
      (mission_ctx.mission_snapshot.state != MISSION_STATE_S3_E3_TURN_AWAY) &&
      (mission_ctx.mission_snapshot.state != MISSION_STATE_S3_TRACK_GREEN) &&
      ((mission_ctx.mission_snapshot.state < MISSION_STATE_S4_TRACK_CENTER) ||
       (mission_ctx.mission_snapshot.state > MISSION_STATE_S4_FINAL_LOWER_FRAME)) &&
      (mission_ctx.mission_snapshot.state != MISSION_STATE_S4_FINAL_TURN_SAFE) &&
      (mission_ctx.mission_snapshot.state != MISSION_STATE_S4_CASUALTY_REGRIP) &&
      (mission_ctx.mission_snapshot.state != MISSION_STATE_S4_FINAL_TIMEOUT_BACKOFF) &&
      ((mission_ctx.mission_snapshot.state < MISSION_STATE_SUPPLEMENT_WIDE_SETTLE) ||
       (mission_ctx.mission_snapshot.state > MISSION_STATE_SUPPLEMENT_CENTER_OBJECT)) &&
      (mission_ctx.mission_snapshot.state != MISSION_STATE_SUPPLEMENT_LOAD_CHECK) &&
      (mission_ctx.mission_snapshot.state != MISSION_STATE_S6_RECHECK_WAIT_ACK) &&
      (mission_ctx.mission_snapshot.state != MISSION_STATE_S6_RECHECK_WAIT_RESULT) &&
      (mission_ctx.mission_snapshot.state != MISSION_STATE_S6_RECHECK_WIDE_SETTLE) &&
      (mission_ctx.mission_snapshot.state != MISSION_STATE_SUPPLEMENT_WAIT_RECHECK) &&
      (mission_ctx.mission_snapshot.state != MISSION_STATE_SUPPLEMENT_E3_SPIN_CCW) &&
      (mission_ctx.mission_snapshot.state != MISSION_STATE_VISION_AVOID_START_TURN) &&
      (mission_ctx.mission_snapshot.state != MISSION_STATE_VISION_AVOID_START_WAIT_ACK) &&
      ((mission_ctx.mission_snapshot.state < MISSION_STATE_S6_BAR_REVERSE) ||
       (mission_ctx.mission_snapshot.state > MISSION_STATE_S6_BAR_WAIT_ACK)) &&
      ((mission_ctx.mission_snapshot.state < MISSION_STATE_S4_FINE32_TRACK) ||
       (mission_ctx.mission_snapshot.state > MISSION_STATE_S4_FINE42_FINISH_TURN_LEFT)) &&
      ((mission_ctx.mission_snapshot.state < MISSION_STATE_S6_CORNER_TURN_AWAY) ||
       (mission_ctx.mission_snapshot.state > MISSION_STATE_S6_CORNER_TURN_SAFE)) &&
      (mission_ctx.mission_snapshot.state != MISSION_STATE_S5_ALIGN_FOR_ARRANGE) &&
      (mission_ctx.mission_snapshot.state != MISSION_STATE_S5_RAISE_FOR_ARRANGE) &&
      (mission_ctx.mission_snapshot.state != MISSION_STATE_S5_REVERSE_FOR_ARRANGE) &&
      (mission_ctx.mission_snapshot.state != MISSION_STATE_S5_LOWER_FOR_ARRANGE) &&
      (mission_ctx.mission_snapshot.state != MISSION_STATE_S6_SEARCH_SAFE_ZONE) &&
      (mission_ctx.mission_snapshot.state != MISSION_STATE_S6_REPOSITION_TURN_SIDE) &&
      (mission_ctx.mission_snapshot.state != MISSION_STATE_S6_REPOSITION_FORWARD) &&
      (mission_ctx.mission_snapshot.state != MISSION_STATE_S6_REPOSITION_TURN_SAFE) &&
      (mission_ctx.mission_snapshot.state != MISSION_STATE_S6_REPOSITION_FACE_SAFE) &&
      (mission_ctx.mission_snapshot.state != MISSION_STATE_S6_RECOVERY_START) &&
      (mission_ctx.mission_snapshot.state != MISSION_STATE_S6_RECOVERY_TURN_LEFT) &&
      (mission_ctx.mission_snapshot.state != MISSION_STATE_S6_RECOVERY_TURN_BACK) &&
      (mission_ctx.mission_snapshot.state != MISSION_STATE_S6_RECOVERY_RETURN_SAFE) &&
      (mission_ctx.mission_snapshot.state != MISSION_STATE_S6_TRACK_SAFE_ZONE) &&
      (mission_ctx.mission_snapshot.state != MISSION_STATE_S6_OBSTACLE_TURN_OUT) &&
      (mission_ctx.mission_snapshot.state != MISSION_STATE_S6_OBSTACLE_FORWARD) &&
      (mission_ctx.mission_snapshot.state != MISSION_STATE_S6_OBSTACLE_TURN_BACK) &&
      (mission_ctx.mission_snapshot.state != MISSION_STATE_S6_OBSTACLE_FACE_SAFE) &&
      (mission_ctx.mission_snapshot.state != MISSION_STATE_S6_TRACK_TIMEOUT_REVERSE) &&
      (mission_ctx.mission_snapshot.state != MISSION_STATE_S6_FINAL_ALIGN) &&
      (mission_ctx.mission_snapshot.state != MISSION_STATE_S6_FINAL_VERIFY) &&
      (mission_ctx.mission_snapshot.state != MISSION_STATE_S6_BORDER_LOCATE) &&
      (mission_ctx.mission_snapshot.state != MISSION_STATE_S6_BORDER_BACKOFF) &&
      (mission_ctx.mission_snapshot.state != MISSION_STATE_S6_FINAL_PUSH) &&
      (mission_ctx.mission_snapshot.state != MISSION_STATE_S6_FINAL_REVERSE) &&
      (mission_ctx.mission_snapshot.state != MISSION_STATE_S6_EXIT_TURN_180) &&
      (mission_ctx.mission_snapshot.state != MISSION_STATE_S7_SEARCH_RED) &&
      (mission_ctx.mission_snapshot.state != MISSION_STATE_S7_WAIT_SECOND_E3) &&
      (mission_ctx.mission_snapshot.state != MISSION_STATE_S7_SEARCH_BLACK) &&
      (mission_ctx.mission_snapshot.state != MISSION_STATE_S7_SILENT_TURN_LEFT) &&
       (mission_ctx.mission_snapshot.state != MISSION_STATE_S7_SILENT_TURN_BACK) &&
       (mission_ctx.mission_snapshot.state != MISSION_STATE_S4_E4_GREEN_SPIN_360) &&
       (mission_ctx.mission_snapshot.state != MISSION_STATE_S4_E4_RED_SPIN_360) &&
       (mission_ctx.mission_snapshot.state != MISSION_STATE_S4_E4_MATERIAL_SPIN_CW_360))
  {
    return;
  }

  required_faults = MissionTask_GetRequiredFaults();
  if (required_faults != MISSION_FAULT_NONE)
  {
    mission_ctx.mission_snapshot.fault_flags |= required_faults;
    MissionTask_EnterState(MISSION_STATE_FAULT, now);
  }
}
