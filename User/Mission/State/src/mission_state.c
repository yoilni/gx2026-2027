#include "mission_state.h"
#include "mission_internal.h"
#include "mission_safety.h"
#include "mission_vision.h"
#include "mission_arrange.h"
#include "mission_load.h"
#include "mission_supplement.h"
#include "mission_safe_zone.h"
#include "mission_delivery.h"
#include "mission_strategy.h"

#include "M2006.h"
#include "actuator_task.h"
#include "app_rtos_task_config.h"
#include "chassis_motion.h"
#include "departure_flow.h"
#include "cmsis_os.h"
#include "debug_uart_task.h"
#include "heading_control.h"
#include "imu_task.h"
#include "maixcam_task.h"
#include "robot_config.h"
#include "stm32f4xx_hal.h"
#include "switch_input.h"
#include "flow_motion.h"

static osThreadId_t mission_task_handle;
static osEventFlagsId_t mission_event_handle;
static osMutexId_t mission_snapshot_mutex;
static uint32_t angle_next_debug_tick;
static uint32_t yaw_next_debug_tick;
static uint32_t s1_turn_stable_since;
static uint32_t s1_next_debug_tick;
static bool s2_drive_finished;
static uint32_t s2_baffle_down_tick;
static uint32_t s2_next_debug_tick;
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

MissionContext mission_ctx = {
  .s3_target_select_command = MAIXCAM_COMMAND_SELECT_GREEN,
  .s4_fine_next_left = true,
  .s4_fixed_motion_ack = MAIXCAM_COMMAND_SIDE_PUSH_DONE_ACK,
};

static void MissionTask_DebugAngles(uint32_t now, bool force);

int32_t MissionTask_WrapYaw(int32_t angle_cdeg)
{
  return HeadingControl_WrapYaw(angle_cdeg);
}

int32_t MissionTask_YawError(int32_t target_cdeg,
                                    int32_t current_cdeg)
{
  return HeadingControl_YawError(target_cdeg, current_cdeg);
}

int32_t MissionTask_Abs32(int32_t value)
{
  return value < 0L ? -value : value;
}

float MissionTask_ClampFloat(float value, float minimum, float maximum)
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

void MissionTask_SetS1Phase(MissionS1Phase phase, uint32_t now)
{
  mission_ctx.mission_snapshot.s1_phase = phase;
  mission_ctx.mission_snapshot.phase_entry_tick = now;
}

static void MissionTask_UpdatePe13(void)
{
  GPIO_PinState sample = SwitchInput_ReadTeam();

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
  GPIO_PinState sample = SwitchInput_ReadZoneMsb();

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
  GPIO_PinState sample = SwitchInput_ReadZoneLsb();

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

void MissionTask_EnterState(MissionState state, uint32_t now)
{
  if (mission_ctx.mission_snapshot.state == state)
  {
    return;
  }

  if (state != MISSION_STATE_S6_RECHECK_WAIT_RESULT)
  {
    if (mission_ctx.s5_casualty_adjust_active) MaixCam_SetLoadRecheckEnabled(false);
    mission_ctx.s5_casualty_adjust_active = false;
    mission_ctx.s5_casualty_adjust_done = false;
  }

  /* Keep PA2 driven through all three pre-release legs. On an abort,
     stop the temporary output without attempting to release the load. */
  bool keep_delivery_pwm = state == MISSION_STATE_S6_BORDER_BACKOFF ||
                           state == MISSION_STATE_S6_FINAL_PUSH;
  if (((mission_ctx.delivery_flow.baffle_held && !keep_delivery_pwm) ||
       state == MISSION_STATE_STOPPED || state == MISSION_STATE_FAULT ||
       state == MISSION_STATE_WAIT_START || state == MISSION_STATE_S6_RECHECK_WAIT_ACK ||
       state == MISSION_STATE_S6_RECHECK_WAIT_RESULT) &&
      !MissionTask_StopS6DeliveryBafflePwm())
  {
    state = MISSION_STATE_FAULT;
  }
  mission_ctx.mission_snapshot.state = state;
  mission_ctx.mission_snapshot.state_entry_tick = now;
  MaixCam_SetCasualtyRegripEnabled(MissionTask_CanLatchCasualtyRegrip(state));
  if (state == MISSION_STATE_S6_BORDER_LOCATE || state == MISSION_STATE_S6_FINAL_PUSH)
    MissionTask_ResetS6BorderProgress(now);
  if (state != MISSION_STATE_S3_LOWER_FRAME) mission_ctx.s3_entry_pose_ready = false;
  if (state < MISSION_STATE_S4_FINE32_TRACK ||
      state > MISSION_STATE_S4_FINE42_FINISH_TURN_LEFT)
    mission_ctx.s4_initial_green02_active = false;
  if (state != MISSION_STATE_S4_WAIT_ARRANGE_READY)
  {
    mission_ctx.s4_fine_waiting_decision = false;
    mission_ctx.s4_direct_capture_wait24 = false;
  }
  if (state != MISSION_STATE_S4_E4_PUSH_RIGHT)
    mission_ctx.s4_green_e4_half_turn_active = false;
  if (state != MISSION_STATE_S6_RECOVERY_START)
    mission_ctx.s6_recovery_first_forward_active = false;
  MaixCam_SetSupplementDirectSearch(mission_ctx.supplement_capture_active &&
      !mission_ctx.supplement_budget_exhausted &&
      (state == MISSION_STATE_SUPPLEMENT_WIDE_SETTLE ||
       state == MISSION_STATE_SUPPLEMENT_TRACK ||
       state == MISSION_STATE_SUPPLEMENT_TURN_LEFT ||
       state == MISSION_STATE_SUPPLEMENT_TURN_RIGHT ||
       state == MISSION_STATE_SUPPLEMENT_E3_SPIN_CCW ||
       ((state == MISSION_STATE_VISION_AVOID_START_TURN ||
         state == MISSION_STATE_VISION_AVOID_START_WAIT_ACK) &&
        mission_ctx.ae_resume_state == MISSION_STATE_SUPPLEMENT_TRACK)));
  /* Do not close a05 received by ISR during a phase transition. Its task
     handler closes the old motion only after preserving the new report. */
  MaixCam_SetLoadCheckInterruptEnabled(MissionTask_CanPauseForLoadCheck());
  MaixCam_SetCornerEvacuationActive(
      state >= MISSION_STATE_S6_CORNER_TURN_AWAY &&
      state <= MISSION_STATE_S6_CORNER_TURN_SAFE);
  MaixCam_SetBarShiftActive(state >= MISSION_STATE_S6_BAR_REVERSE &&
      state <= MISSION_STATE_S6_BAR_WAIT_ACK);
  MaixCam_SetStartZoneAvoidanceActive(state == MISSION_STATE_VISION_AVOID_START_TURN ||
      state == MISSION_STATE_VISION_AVOID_START_WAIT_ACK);
  if (state != MISSION_STATE_SUPPLEMENT_E3_SPIN_CCW)
    mission_ctx.supplement_e3_first_hold_active = false;
  MaixCam_SetSupplementE3TurnActive(state == MISSION_STATE_SUPPLEMENT_E3_SPIN_CCW &&
      mission_ctx.supplement_e3_spin_goal_cdeg == ROBOT_SUPPLEMENT_E3_FIRST_SPIN_CDEG &&
      !mission_ctx.supplement_e3_first_hold_active);
  if (state != MISSION_STATE_VISION_AVOID_START_TURN &&
      state != MISSION_STATE_VISION_AVOID_START_WAIT_ACK &&
      !MissionTask_CanAvoidStartZone())
    mission_ctx.ae_ack_completed = mission_ctx.ae_reply_pending = false;
  MaixCam_SetFineArrangeActive(state == MISSION_STATE_S4_TRACK_RIGHT_BLOCK ||
      (state >= MISSION_STATE_S4_FINE32_TRACK &&
       state <= MISSION_STATE_S4_FINE42_FINISH_TURN_LEFT));
  MaixCam_SetArrange02Active(state == MISSION_STATE_S5_REVERSE_FOR_ARRANGE ||
      state == MISSION_STATE_S4_TRACK_RIGHT_BLOCK ||
      (state >= MISSION_STATE_S4_FINE32_TRACK &&
       state <= MISSION_STATE_S4_FINE42_FINISH_TURN_LEFT &&
       mission_ctx.s4_fixed_motion_ack == MAIXCAM_COMMAND_RIGHT_DONE_ACK));
  MaixCam_SetLoadEmptyRecoveryEnabled(
      (state == MISSION_STATE_S5_WAIT_SINGLE_GREEN ||
       state == MISSION_STATE_S6_RECHECK_WAIT_RESULT) &&
      !mission_ctx.s5_recheck_resume_safe_tracking &&
      !mission_ctx.s7_target_switch_enabled &&
      mission_ctx.s3_target_select_command == MAIXCAM_COMMAND_SELECT_GREEN);
  MaixCam_SetSupplementLoadCheckEnabled(
      (state == MISSION_STATE_S5_WAIT_SINGLE_GREEN && mission_ctx.s5_close_view_active) ||
      state == MISSION_STATE_SUPPLEMENT_LOAD_CHECK);
  if ((state < MISSION_STATE_S4_E4_PUSH_RIGHT) ||
      (state > MISSION_STATE_S4_E4_REVERSE_LEFT))
  {
    mission_ctx.s5_empty_recovery_active = false;
  }
  if (state != MISSION_STATE_S6_TRACK_SAFE_ZONE)
  {
    mission_ctx.s6_pre_reposition_track_active = false;
    mission_ctx.s6_pre_reposition_track_tick = now;
  }
  MaixCam_SetRecoveryLossFilter(MissionTask_ActiveRecoveryLossEvent());
  if ((state == MISSION_STATE_STOPPED) || (state == MISSION_STATE_FAULT) ||
      (state == MISSION_STATE_WAIT_START))
  {
    Actuator_CancelDeferredClawClose();
    MissionTask_ResetSupplement();
    mission_ctx.s4_fine_next_left = true;
    mission_ctx.s4_fixed_motion_ack = MAIXCAM_COMMAND_SIDE_PUSH_DONE_ACK;
    mission_ctx.s51_e3_recovery_count = 0U;
    mission_ctx.s51_second_e3_pending = false;
    mission_ctx.s4_material_e4_recovery_count = 0U;
    mission_ctx.s6_recovery_found16_pending = false;
    mission_ctx.s6_align26_pending = false;
    mission_ctx.s6_pre_reposition_decision_pending = false;
    mission_ctx.s6_obstacle_side_required = false;
    mission_ctx.s6_obstacle_side_decision_pending = false;
    mission_ctx.s6_obstacle_side_decision_done = false;
    mission_ctx.s6_corner_completed = false;
    mission_ctx.s6_bar_shift_completed = false;
    mission_ctx.s6_bar_shift_ack_pending = false;
    mission_ctx.s6_bar_initial_align_pending = false;
    mission_ctx.ae_ack_completed = mission_ctx.ae_reply_pending = false;
    mission_ctx.s5_recheck_resume_supplement = false;
    mission_ctx.s5_recheck_resume_safe_tracking = false;
    MissionTask_ResetS6XConfirmation();
  }
  if ((state == MISSION_STATE_S7_SEARCH_RED) ||
      (state == MISSION_STATE_S7_WAIT_SECOND_E3) ||
      (state == MISSION_STATE_S7_SEARCH_BLACK))
  {
    mission_ctx.s7_last_message_tick = now;
    mission_ctx.s7_last_coordinate_sequence = mission_ctx.mission_snapshot.target_sequence;
  }

  switch (state)
  {
    case MISSION_STATE_WAIT_START:
      (void)DebugUart_Log("[ST] WAIT PE12\r\n");
      break;
    case MISSION_STATE_S1_DEPART:
      (void)DebugUart_Log("[ST] DEPART\r\n");
      break;
    case MISSION_STATE_S2_CROSS_BUMP:
      (void)DebugUart_Log("[ST] BUMP\r\n");
      break;
    case MISSION_STATE_S3_ALIGN_GREEN:
      (void)DebugUart_Log("[ST] 03 ALIGN\r\n");
      break;
    case MISSION_STATE_S3_SEARCH_TURN_CW:
      (void)DebugUart_Log("[ST] E3 CW\r\n");
      break;
    case MISSION_STATE_S3_SEARCH_TURN_CCW:
      (void)DebugUart_Log("[ST] E3 CCW\r\n");
      break;
    case MISSION_STATE_S3_E3_TURN_AWAY:
      (void)DebugUart_Log("[ST] E3 AWAY\r\n");
      break;
    case MISSION_STATE_SUPPLEMENT_WAIT_RECHECK:
      (void)DebugUart_Log("[ST] SUPP WAIT05\r\n");
      break;
    case MISSION_STATE_S3_TRACK_GREEN:
      (void)DebugUart_Log("[ST] 03 TRACK\r\n");
      break;
    case MISSION_STATE_S3_LOWER_FRAME:
      (void)DebugUart_Log("[ST] 03 DOWN\r\n");
      break;
    case MISSION_STATE_S4_TRACK_CENTER:
      (void)DebugUart_Log("[ST] 04 TRACK\r\n");
      break;
    case MISSION_STATE_S4_CENTER_FOLLOW_THROUGH:
      (void)DebugUart_Log("[ST] 14 FOLLOW\r\n");
      break;
    case MISSION_STATE_S4_WAIT_ARRANGE_READY:
      (void)DebugUart_Log("[ST] WAIT02/24\r\n");
      break;
    case MISSION_STATE_S4_TRACK_RIGHT_BLOCK:
      (void)DebugUart_Log("[ST] 02 PUSH-R\r\n");
      break;
    case MISSION_STATE_S4_REVERSE_RIGHT_BLOCK:
      (void)DebugUart_Log("[ST] 02 BACK\r\n");
      break;
    case MISSION_STATE_S4_WAIT_LEFT_TARGET:
      (void)DebugUart_Log("[ST] WAIT12\r\n");
      break;
    case MISSION_STATE_S4_WAIT_FINAL_READY:
      (void)DebugUart_Log("[ST] WAIT04/14/24/E4/E2\r\n");
      break;
    case MISSION_STATE_S4_POST22_REVERSE:
    case MISSION_STATE_S4_POST22_TURN_LEFT:
    case MISSION_STATE_S4_POST22_TURN_RIGHT:
      (void)DebugUart_Logf("[ST] 22E2 s=%u\r\n",
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
      (void)DebugUart_Logf("[ST] E4 REC s=%u\r\n",
                           (unsigned int)state);
      break;
    case MISSION_STATE_S4_E4_GREEN_SPIN_360:
      (void)DebugUart_Log("[ST] E4 GREEN360\r\n");
      break;
    case MISSION_STATE_S4_E4_RED_SPIN_360:
      (void)DebugUart_Log("[ST] E4 RED SCAN\r\n");
      break;
    case MISSION_STATE_S4_E4_MATERIAL_SPIN_CW_360:
      (void)DebugUart_Log("[ST] E4 MAT CW360\r\n");
      break;
    case MISSION_STATE_S4_TRACK_LEFT_BLOCK:
      (void)DebugUart_Log("[ST] 12 ACK22 ONLY\r\n");
      break;
    case MISSION_STATE_S4_REVERSE_LEFT_BLOCK:
      (void)DebugUart_Log("[ST] 12 BACK\r\n");
      break;
    case MISSION_STATE_S4_ARRANGE_RECOVERY_REVERSE:
      (void)DebugUart_Log("[ST] E2 BACK\r\n");
      break;
    case MISSION_STATE_S4_ARRANGE_RECOVERY_TURN_LEFT:
      (void)DebugUart_Log("[ST] E2 TURN LEFT\r\n");
      break;
    case MISSION_STATE_S4_ARRANGE_RECOVERY_TURN_BACK:
      (void)DebugUart_Log("[ST] E2 TURN BACK\r\n");
      break;
    case MISSION_STATE_S4_RAISE_FRAME:
      (void)DebugUart_Log("[ST] 24 CLAW OPEN\r\n");
      break;
    case MISSION_STATE_S4_TRACK_FINAL_BLOCK:
      (void)DebugUart_Log("[ST] 24 TRACK\r\n");
      break;
    case MISSION_STATE_S4_FINAL_TIMEOUT_BACKOFF:
      (void)DebugUart_Log("[ST] 24 TMO BACK\r\n");
      break;
    case MISSION_STATE_S4_FINAL_CENTER_OBJECT:
      (void)DebugUart_Log("[ST] 34 WAIT-CLAW\r\n");
      break;
    case MISSION_STATE_S4_FINAL_TURN_SAFE:
      (void)DebugUart_Log("[ST] 34 TURN SAFE\r\n");
      break;
    case MISSION_STATE_S4_CASUALTY_REGRIP:
      (void)DebugUart_Log("[ST] 54 REGRIP\r\n");
      break;
    case MISSION_STATE_S4_FINAL_RECOVERY_REVERSE:
      (void)DebugUart_Log("[ST] EE BACK\r\n");
      break;
    case MISSION_STATE_S4_FINAL_RECOVERY_TURN_270:
      (void)DebugUart_Log("[ST] EE TURN SIDE1\r\n");
      break;
    case MISSION_STATE_S4_FINAL_RECOVERY_TURN_90:
      (void)DebugUart_Log("[ST] EE TURN SIDE2\r\n");
      break;
    case MISSION_STATE_S5_WAIT_SINGLE_GREEN:
      (void)DebugUart_Log("[ST] 05 WAIT\r\n");
      break;
    case MISSION_STATE_S5_ALIGN_FOR_ARRANGE:
      (void)DebugUart_Log("[ST] 05 SAFE\r\n");
      break;
    case MISSION_STATE_S5_RAISE_FOR_ARRANGE:
      (void)DebugUart_Log("[ST] 05 UP\r\n");
      break;
    case MISSION_STATE_S5_REVERSE_FOR_ARRANGE:
      (void)DebugUart_Log("[ST] 05/02 BACK\r\n");
      break;
    case MISSION_STATE_S5_LOWER_FOR_ARRANGE:
      (void)DebugUart_Log("[ST] 05 DOWN\r\n");
      break;
    case MISSION_STATE_S6_SEARCH_SAFE_ZONE:
      (void)DebugUart_Log("[ST] 06 SEARCH\r\n");
      break;
    case MISSION_STATE_S6_REPOSITION_TURN_SIDE:
      (void)DebugUart_Log("[ST] SIDE TURN SIDE\r\n");
      break;
    case MISSION_STATE_S6_REPOSITION_FORWARD:
      (void)DebugUart_Log("[ST] SIDE FWD\r\n");
      break;
    case MISSION_STATE_S6_REPOSITION_FACE_SAFE:
      (void)DebugUart_Log("[ST] SIDE FACE SAFE\r\n");
      break;
    case MISSION_STATE_S6_REPOSITION_TURN_SAFE:
      (void)DebugUart_Log("[ST] SIDE TURN SAFE\r\n");
      break;
    case MISSION_STATE_S6_RECOVERY_START:
      (void)DebugUart_Log("[ST] 6E START\r\n");
      break;
    case MISSION_STATE_S6_RECOVERY_TURN_LEFT:
      (void)DebugUart_Log("[ST] 6E TURN LEFT\r\n");
      break;
    case MISSION_STATE_S6_RECOVERY_TURN_BACK:
      (void)DebugUart_Log("[ST] 6E TURN BACK\r\n");
      break;
    case MISSION_STATE_S6_RECOVERY_RETURN_SAFE:
      (void)DebugUart_Log("[ST] 6E RETURN SAFE\r\n");
      break;
    case MISSION_STATE_S6_TRACK_SAFE_ZONE:
      (void)DebugUart_Log("[ST] 16 TRACK\r\n");
      break;
    case MISSION_STATE_S6_CORNER_TURN_AWAY:
      (void)DebugUart_Log("[ST] CORNER TURN AWAY\r\n");
      break;
    case MISSION_STATE_S6_CORNER_FORWARD_AWAY:
      (void)DebugUart_Log("[ST] CORNER FWD AWAY\r\n");
      break;
    case MISSION_STATE_S6_CORNER_TURN_SIDE:
      (void)DebugUart_Log("[ST] CORNER TURN SIDE\r\n");
      break;
    case MISSION_STATE_S6_CORNER_FORWARD_SIDE:
      (void)DebugUart_Log("[ST] CORNER FWD SIDE\r\n");
      break;
    case MISSION_STATE_S6_CORNER_TURN_SAFE:
      (void)DebugUart_Log("[ST] CORNER TURN SAFE\r\n");
      break;
    case MISSION_STATE_S6_OBSTACLE_TURN_OUT:
      (void)DebugUart_Log(mission_ctx.s6_obstacle_turn_right
          ? "[ST] 36 TURN OUT RIGHT\r\n"
          : "[ST] 36 TURN OUT LEFT\r\n");
      break;
    case MISSION_STATE_S6_OBSTACLE_FORWARD:
      (void)DebugUart_Log("[ST] 36 FWD\r\n");
      break;
    case MISSION_STATE_S6_OBSTACLE_TURN_BACK:
      (void)DebugUart_Log(mission_ctx.s6_obstacle_turn_right
          ? "[ST] 36 TURN BACK LEFT\r\n"
          : "[ST] 36 TURN BACK RIGHT\r\n");
      break;
    case MISSION_STATE_S6_OBSTACLE_FACE_SAFE:
      (void)DebugUart_Log("[ST] 36 FACE SAFE\r\n");
      break;
    case MISSION_STATE_S6_TRACK_TIMEOUT_REVERSE:
      (void)DebugUart_Log("[ST] 16 BACK8S\r\n");
      break;
    case MISSION_STATE_S6_FINAL_ALIGN:
      (void)DebugUart_Log("[ST] 26 ALIGN\r\n");
      break;
    case MISSION_STATE_S6_FINAL_VERIFY:
      (void)DebugUart_Log("[ST] 26 CHECK\r\n");
      break;
    case MISSION_STATE_S6_BORDER_LOCATE:
      (void)DebugUart_Log("[ST] 26 BORDER\r\n");
      break;
    case MISSION_STATE_S6_BORDER_BACKOFF:
      (void)DebugUart_Log("[ST] 26 BORDER-BACK\r\n");
      break;
    case MISSION_STATE_S6_FINAL_PUSH:
      (void)DebugUart_Log("[ST] 26 PUSH-IN\r\n");
      break;
    case MISSION_STATE_S6_FINAL_REVERSE:
      (void)DebugUart_Log("[ST] 26 EXIT-BACK\r\n");
      break;
    case MISSION_STATE_S6_EXIT_TURN_180:
      (void)DebugUart_Log("[ST] 07 TURN180\r\n");
      break;
    case MISSION_STATE_S6_SAFE_ZONE_REACHED:
      (void)DebugUart_Log("[ST] 06 DONE\r\n");
      break;
    case MISSION_STATE_S7_SEARCH_RED:
      (void)DebugUart_Log("[ST] 07 RED\r\n");
      break;
    case MISSION_STATE_S7_WAIT_SECOND_E3:
      (void)DebugUart_Log("[ST] 07 WAIT E3\r\n");
      break;
    case MISSION_STATE_S7_SEARCH_BLACK:
      (void)DebugUart_Log("[ST] 07 BLACK\r\n");
      break;
    case MISSION_STATE_S7_SILENT_TURN_LEFT:
      (void)DebugUart_Log("[ST] 07 L45\r\n");
      break;
    case MISSION_STATE_S7_SILENT_TURN_BACK:
      (void)DebugUart_Log("[ST] 07 R90\r\n");
      break;
    case MISSION_STATE_SUPPLEMENT_WIDE_SETTLE:
    case MISSION_STATE_SUPPLEMENT_TRACK:
    case MISSION_STATE_SUPPLEMENT_WAIT_FINAL:
    case MISSION_STATE_SUPPLEMENT_TURN_LEFT:
    case MISSION_STATE_SUPPLEMENT_TURN_RIGHT:
    case MISSION_STATE_SUPPLEMENT_CENTER_OBJECT:
    case MISSION_STATE_SUPPLEMENT_E3_SPIN_CCW:
    case MISSION_STATE_SUPPLEMENT_LOAD_CHECK:
      (void)DebugUart_Logf("[ST] SUPPLEMENT s=%u n=%u\r\n",
          (unsigned int)state, (unsigned int)mission_ctx.supplement_capture_count);
      break;
    case MISSION_STATE_S6_BAR_REVERSE:
    case MISSION_STATE_S6_BAR_TURN_SIDE:
    case MISSION_STATE_S6_BAR_FORWARD:
    case MISSION_STATE_S6_BAR_TURN_SAFE:
    case MISSION_STATE_S6_BAR_WAIT_ACK:
      (void)DebugUart_Logf("[46] ST=%u\r\n", (unsigned int)state);
      break;
    case MISSION_STATE_S4_FINE32_TRACK:
    case MISSION_STATE_S4_FINE42_TRACK:
    case MISSION_STATE_S4_FINE42_TURN:
    case MISSION_STATE_S4_FINE42_REVERSE:
    case MISSION_STATE_S4_FINE42_FORWARD:
    case MISSION_STATE_S4_FINE42_COUNTER_TURN:
    case MISSION_STATE_S4_FINE42_FINISH_TURN_LEFT:
      (void)DebugUart_Logf("[FINE] s=%u side=%c\r\n", (unsigned int)state,
          mission_ctx.s4_fine_turn_left ? 'L' : 'R');
      break;
    case MISSION_STATE_VISION_AVOID_START_TURN:
    case MISSION_STATE_VISION_AVOID_START_WAIT_ACK:
      (void)DebugUart_Logf("[AE] ST=%u\r\n", (unsigned int)state);
      break;
    case MISSION_STATE_STOPPED:
      (void)DebugUart_Log("[ST] STOPPED\r\n");
      break;
    case MISSION_STATE_S6_RECHECK_WAIT_ACK:
    case MISSION_STATE_S6_RECHECK_WAIT_RESULT:
    case MISSION_STATE_S6_RECHECK_WIDE_SETTLE:
      (void)DebugUart_Logf("[ST] CHECK05 s=%u\r\n", (unsigned int)state);
      break;
    case MISSION_STATE_FAULT:
      (void)DebugUart_Logf("[ST] FAULT flags=0x%08lX\r\n",
                           (unsigned long)mission_ctx.mission_snapshot.fault_flags);
      break;
    case MISSION_STATE_BOOT:
    default:
      break;
  }

  /* Ordinary state boundaries decelerate; terminal/fault states bypass the
     slew limiter so an unsafe condition never waits for a smooth ramp. */
  if ((state == MISSION_STATE_FAULT) || (state == MISSION_STATE_STOPPED))
  {
    ChassisMotion_EmergencyStop();
    mission_ctx.commanded_left_rpm = 0;
    mission_ctx.commanded_right_rpm = 0;
    mission_ctx.mission_snapshot.left_target_rpm = 0;
    mission_ctx.mission_snapshot.right_target_rpm = 0;
  }
  else
  {
    MissionTask_StopWheels();
  }
  MissionTask_DebugAngles(now, true);
}

static void MissionTask_UpdateInputs(uint32_t now)
{
  IMU_Yaw attitude;

  MissionTask_UpdatePe13();
  MissionTask_UpdatePe14();
  MissionTask_UpdatePe15();

  mission_ctx.mission_snapshot.left_motor_online =
      MissionTask_MotorIsOnline(MISSION_LEFT_MOTOR_INDEX, now);
  mission_ctx.mission_snapshot.right_motor_online =
      MissionTask_MotorIsOnline(MISSION_RIGHT_MOTOR_INDEX, now);

  mission_ctx.mission_snapshot.imu_valid = IMU_GetYaw(&attitude) &&
      ((uint32_t)(now - attitude.update_tick) <= MISSION_IMU_TIMEOUT_MS);
  if (mission_ctx.mission_snapshot.imu_valid)
  {
    mission_ctx.mission_snapshot.yaw_cdeg = attitude.yaw_cdeg;
  }

  MissionTask_UpdateVisionInput(now);
}

void MissionTask_StartS1(uint32_t now)
{
  MissionTask_StopWheels();
  if (Actuator_SetBaffleAngle(ROBOT_BAFFLE_DOWN_DEG) != HAL_OK ||
      Actuator_SetClawClosed() != HAL_OK)
  {
    mission_ctx.mission_snapshot.fault_flags |= MISSION_FAULT_ACTUATOR;
    MissionTask_EnterState(MISSION_STATE_FAULT, now);
    return;
  }
  MissionTask_ResetSupplement();
  mission_ctx.s7_material_delivery_count = 0U;
  mission_ctx.s7_casualty_delivery_count = 0U;
  mission_ctx.s7_delivery_counted = false;
  mission_ctx.s7_material_only_reselect_pending = false;
  mission_ctx.s7_target_switch_enabled = false;
  mission_ctx.s4_initial_green02_used = false;
  mission_ctx.s4_initial_green02_active = false;
  mission_ctx.s7_red_recovery_used = false;
  mission_ctx.s51_active_object_id = 0U;
  mission_ctx.s51_e3_recovery_count = 0U;
  mission_ctx.s51_second_e3_pending = false;
  mission_ctx.s6_carried_object_id = 0U;
  mission_ctx.s5_load_class_confirmed = false;
  mission_ctx.s7_red_switch_sync_pending = false;
  mission_ctx.s7_red_coordinate_fence_active = false;
  mission_ctx.s3_target_select_command = MAIXCAM_COMMAND_SELECT_GREEN;
  mission_ctx.s4_post22_search_active = false;
  mission_ctx.s4_arrange_cycle_count = 0U;
  mission_ctx.s4_post22_failed_sweeps = 0U;
  int32_t yaw_delta;

  if (!mission_ctx.mission_snapshot.team_locked)
  {
    mission_ctx.mission_snapshot.team = pe13_stable == GPIO_PIN_SET
                                ? ROBOT_PE13_HIGH_TEAM
                                : ROBOT_PE13_LOW_TEAM;
    mission_ctx.mission_snapshot.team_locked = true;
  }

  if (!mission_ctx.mission_snapshot.start_zone_locked)
  {
    mission_ctx.mission_snapshot.start_pe14_high = pe14_stable == GPIO_PIN_SET;
    mission_ctx.mission_snapshot.start_pe15_high = pe15_stable == GPIO_PIN_SET;
    mission_ctx.mission_snapshot.start_zone = MissionTask_DecodeStartZone(pe14_stable,
                                                              pe15_stable);
    mission_ctx.mission_snapshot.start_zone_locked = true;
  }

  mission_ctx.mission_snapshot.team_selector_high = pe13_stable == GPIO_PIN_SET;
  mission_ctx.mission_snapshot.yaw_start_cdeg =
      MissionTask_WrapYaw(mission_ctx.mission_snapshot.yaw_cdeg);
  mission_ctx.mission_snapshot.safe_zone_yaw_target_cdeg = MissionTask_WrapYaw(
      mission_ctx.mission_snapshot.yaw_start_cdeg +
      MissionTask_GetSafeZoneYawOffset(mission_ctx.mission_snapshot.start_zone,
                                       mission_ctx.mission_snapshot.team));

  /* Start areas 1/4 turn left; areas 2/3 turn right. */
  yaw_delta = (int32_t)ROBOT_YAW_LEFT_SIGN * ROBOT_S1_TURN_ANGLE_CDEG;
  if ((mission_ctx.mission_snapshot.start_zone == 2U) ||
      (mission_ctx.mission_snapshot.start_zone == 3U))
  {
    yaw_delta = -yaw_delta;
  }
  mission_ctx.mission_snapshot.yaw_target_cdeg =
      MissionTask_WrapYaw(mission_ctx.mission_snapshot.yaw_start_cdeg + yaw_delta);
  mission_ctx.mission_snapshot.yaw_error_cdeg = MissionTask_YawError(
      mission_ctx.mission_snapshot.yaw_target_cdeg, mission_ctx.mission_snapshot.yaw_cdeg);

  s1_turn_stable_since = 0U;
  s1_next_debug_tick = now;
  MissionTask_SetS1Phase(MISSION_S1_TURN, now);
  MissionTask_EnterState(MISSION_STATE_S1_DEPART, now);
  (void)DebugUart_Logf(
      "[S1] START team=%s zone=%u sw=%u%u dir=%s zero=%ld goal=%ld safe=%ld\r\n",
      mission_ctx.mission_snapshot.team == ROBOT_TEAM_RED ? "RED" : "BLUE",
      (unsigned int)mission_ctx.mission_snapshot.start_zone,
      mission_ctx.mission_snapshot.start_pe14_high ? 1U : 0U,
      mission_ctx.mission_snapshot.start_pe15_high ? 1U : 0U,
      ((mission_ctx.mission_snapshot.start_zone == 2U) ||
       (mission_ctx.mission_snapshot.start_zone == 3U)) ? "RIGHT" : "LEFT",
      (long)mission_ctx.mission_snapshot.yaw_start_cdeg,
      (long)mission_ctx.mission_snapshot.yaw_target_cdeg,
      (long)mission_ctx.mission_snapshot.safe_zone_yaw_target_cdeg);
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
      "[S1] p=%u r=%ld e=%ld w=%d/%d act=%d/%d\r\n",
      (unsigned int)mission_ctx.mission_snapshot.s1_phase,
      (long)MissionTask_WrapYaw(mission_ctx.mission_snapshot.yaw_cdeg),
      (long)mission_ctx.mission_snapshot.yaw_error_cdeg,
      (int)mission_ctx.mission_snapshot.left_target_rpm,
      (int)mission_ctx.mission_snapshot.right_target_rpm,
      (int)left_actual_rpm, (int)right_actual_rpm);
}

static void MissionTask_StartS2CrossBump(uint32_t now)
{
  MissionTask_SetS1Phase(MISSION_S1_DONE, now);
  s2_drive_finished = false;
  s2_next_debug_tick = now;
  MissionTask_EnterState(MISSION_STATE_S2_CROSS_BUMP, now);
  (void)DebugUart_Logf(
      "[S2] FWD v=%d ms=%lu wait=%lu yaw_target=%ld\r\n",
                       ROBOT_S2_CROSS_BUMP_RPM,
                       (unsigned long)ROBOT_S2_CROSS_BUMP_DRIVE_MS,
                       (unsigned long)ROBOT_S2_CROSS_BUMP_SETTLE_MS,
                       (long)mission_ctx.mission_snapshot.yaw_target_cdeg);
}

static void MissionTask_StartS1ClawOpenWait(uint32_t now)
{
  if (DepartureFlow_OpenClaw(MissionTask_StopWheels, Actuator_SetClawAngle) != HAL_OK)
  {
    mission_ctx.mission_snapshot.fault_flags |= MISSION_FAULT_ACTUATOR;
    MissionTask_EnterState(MISSION_STATE_FAULT, now);
    return;
  }
  MissionTask_SetS1Phase(MISSION_S1_CLAW_OPEN_WAIT, now);
  (void)DebugUart_Logf("[S1] CLAW=%u WAIT%lu >BAFFLE UP\r\n",
      (unsigned int)ROBOT_CLAW_MATERIAL_OPEN_DEG, (unsigned long)ROBOT_S1_CLAW_OPEN_SETTLE_MS);
}

static void MissionTask_RunS1(uint32_t now)
{
  if (mission_ctx.mission_snapshot.state != MISSION_STATE_S1_DEPART) return;
  if (mission_ctx.mission_snapshot.s1_phase == MISSION_S1_CLAW_OPEN_WAIT)
  {
    const DepartureStepStatus status = DepartureFlow_LiftAfterClaw(now,
        mission_ctx.mission_snapshot.phase_entry_tick, MissionTask_StopWheels, Actuator_SetBaffleAngle);
    if (status == DEPARTURE_ACTUATOR_FAULT)
    {
      mission_ctx.mission_snapshot.fault_flags |= MISSION_FAULT_ACTUATOR;
      MissionTask_EnterState(MISSION_STATE_FAULT, now);
    }
    else if (status == DEPARTURE_READY) MissionTask_StartS2CrossBump(now);
    return;
  }
  mission_ctx.mission_snapshot.yaw_error_cdeg = MissionTask_YawError(
      mission_ctx.mission_snapshot.yaw_target_cdeg, mission_ctx.mission_snapshot.yaw_cdeg);
  if (mission_ctx.mission_snapshot.s1_phase == MISSION_S1_TURN ||
      mission_ctx.mission_snapshot.s1_phase == MISSION_S1_TURN_STABLE)
  {
    const bool was_stable = mission_ctx.mission_snapshot.s1_phase == MISSION_S1_TURN_STABLE;
    const FlowMotion io = { .stop = MissionTask_StopWheels, .wheels = MissionTask_SetWheelTargets };
    const DepartureTurnResult result = DepartureFlow_Turn(&io, MissionTask_HeadingTurnExpired,
        now, mission_ctx.mission_snapshot.yaw_target_cdeg, mission_ctx.mission_snapshot.yaw_error_cdeg,
        was_stable, &s1_turn_stable_since);
    if (result.status == DEPARTURE_WRONG_DIRECTION)
    {
      mission_ctx.mission_snapshot.fault_flags |= MISSION_FAULT_YAW_DIRECTION;
      MissionTask_EnterState(MISSION_STATE_FAULT, now);
      return;
    }
    if (result.newly_stable)
    {
      MissionTask_SetS1Phase(MISSION_S1_TURN_STABLE, now);
      (void)DebugUart_Log("[S1] TURN OK\r\n");
    }
    if (result.status == DEPARTURE_OPEN_CLAW)
    {
      MissionTask_StartS1ClawOpenWait(now);
      return;
    }
    if (result.status == DEPARTURE_TURNING && was_stable)
      MissionTask_SetS1Phase(MISSION_S1_TURN, now);
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
      "[S2] r=%ld goal=%ld e=%ld u=%d w=%d/%d act=%d/%d\r\n",
      (long)MissionTask_WrapYaw(mission_ctx.mission_snapshot.yaw_cdeg),
      (long)mission_ctx.mission_snapshot.yaw_target_cdeg,
      (long)mission_ctx.mission_snapshot.yaw_error_cdeg,
      (int)correction_rpm,
      (int)mission_ctx.mission_snapshot.left_target_rpm,
      (int)mission_ctx.mission_snapshot.right_target_rpm,
      (int)left_actual_rpm,
      (int)right_actual_rpm);
}

static int16_t MissionTask_CalculateS2YawCorrection(void)
{
  mission_ctx.mission_snapshot.yaw_error_cdeg = MissionTask_YawError(
      mission_ctx.mission_snapshot.yaw_target_cdeg, mission_ctx.mission_snapshot.yaw_cdeg);
  /* Positive correction means the chassis must turn left. */
  return HeadingControl_ProportionalRpm(mission_ctx.mission_snapshot.yaw_error_cdeg,
      ROBOT_S2_YAW_HOLD_TOLERANCE_CDEG, ROBOT_YAW_LEFT_SIGN,
      ROBOT_S2_YAW_HOLD_KP, ROBOT_S2_YAW_HOLD_MAX_CORRECTION_RPM);
}

static void MissionTask_RunS2CrossBump(uint32_t now)
{
  if (mission_ctx.mission_snapshot.state != MISSION_STATE_S2_CROSS_BUMP) return;
  const FlowMotion io = { .stop = MissionTask_StopWheels, .wheels = MissionTask_SetWheelTargets };
  const DepartureBumpResult result = DepartureFlow_CrossBump(&io, Actuator_SetBaffleAngle,
      Actuator_SetClawClosed, MissionTask_CalculateS2YawCorrection, now,
      mission_ctx.mission_snapshot.state_entry_tick, &s2_drive_finished, &s2_baffle_down_tick);
  if (result.status == DEPARTURE_DRIVING) MissionTask_DebugS2(now, result.correction);
  if (result.just_lowered)
    (void)DebugUart_Logf("[S2] BAFFLE DOWN WAIT%lu >CLAW CLOSE\r\n",
        (unsigned long)ROBOT_S2_CROSS_BUMP_SETTLE_MS);
  if (result.status == DEPARTURE_ACTUATOR_FAULT)
  {
    mission_ctx.mission_snapshot.fault_flags |= MISSION_FAULT_ACTUATOR;
    MissionTask_EnterState(MISSION_STATE_FAULT, now);
  }
  else if (result.status == DEPARTURE_READY)
  {
    (void)DebugUart_Log("[S2] DONE, ENTER S3\r\n");
    (void)MissionTask_StartS3Align(now, MAIXCAM_COMMAND_SELECT_GREEN);
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
                                    &AppRtos_MissionTaskAttr);
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
  *snapshot = mission_ctx.mission_snapshot;
  (void)osMutexRelease(mission_snapshot_mutex);
  return true;
}

static void MissionTask_DebugYaw(uint32_t now)
{
  int32_t raw_yaw;
  int32_t field_yaw;
  if ((int32_t)(now - yaw_next_debug_tick) < 0) return;
  yaw_next_debug_tick = now + ROBOT_YAW_DEBUG_PERIOD_MS;
  if (!mission_ctx.mission_snapshot.imu_valid)
  {
    (void)DebugUart_Logf("[YAW] s=%u NO DATA\r\n",
        (unsigned int)mission_ctx.mission_snapshot.state);
    return;
  }
  raw_yaw = MissionTask_WrapYaw(mission_ctx.mission_snapshot.yaw_cdeg);
  field_yaw = MissionTask_WrapYaw(raw_yaw - mission_ctx.mission_snapshot.yaw_start_cdeg);
  (void)DebugUart_Logf("[YAW] s=%u r=%ld.%02ld f=%ld.%02ld\r\n",
      (unsigned int)mission_ctx.mission_snapshot.state,
      (long)(raw_yaw / 100L), (long)(raw_yaw % 100L),
      (long)(field_yaw / 100L), (long)(field_yaw % 100L));
}

static void MissionTask_DebugAngles(uint32_t now, bool force)
{
  int32_t raw_current;
  int32_t raw_zero;
  int32_t field_current;
  int32_t raw_goal = mission_ctx.mission_snapshot.safe_zone_yaw_target_cdeg;
  int32_t field_goal;
  const char *role = "SAFE_REF";

  if (!force && ((int32_t)(now - angle_next_debug_tick) < 0))
  {
    return;
  }
  if (!force && ((mission_ctx.mission_snapshot.state == MISSION_STATE_STOPPED) ||
                 (mission_ctx.mission_snapshot.state == MISSION_STATE_FAULT)))
  {
    return;
  }
  angle_next_debug_tick = now + ROBOT_ANGLE_DEBUG_PERIOD_MS;

  if (mission_ctx.mission_snapshot.state == MISSION_STATE_WAIT_START)
  {
    if (!mission_ctx.mission_snapshot.imu_valid)
    {
      (void)DebugUart_Log("[YAW] WAIT NO DATA\r\n");
      return;
    }
    raw_current = MissionTask_WrapYaw(mission_ctx.mission_snapshot.yaw_cdeg);
    (void)DebugUart_Logf("[YAW] WAIT r=%ld.%02ld\r\n",
        (long)(raw_current / 100L), (long)(raw_current % 100L));
    return;
  }

  switch (mission_ctx.mission_snapshot.state)
  {
    case MISSION_STATE_VISION_AVOID_START_TURN:
    case MISSION_STATE_VISION_AVOID_START_WAIT_ACK:
      raw_goal = mission_ctx.ae_turn_target_cdeg;
      role = "AE_CCW180";
      break;
    case MISSION_STATE_S4_FINE42_TURN:
    case MISSION_STATE_S4_FINE42_COUNTER_TURN:
    case MISSION_STATE_S4_FINE42_FINISH_TURN_LEFT:
      raw_goal = mission_ctx.s4_fine_turn_yaw_cdeg;
      role = "42_SIDE";
      break;
    case MISSION_STATE_S4_FINE42_FORWARD:
      raw_goal = mission_ctx.s4_fine_forward_yaw_cdeg;
      role = "42_PUSH_REF";
      break;
    case MISSION_STATE_S4_FINE42_REVERSE:
      raw_goal = mission_ctx.mission_snapshot.yaw_cdeg;
      role = "42_BACK_SKIP";
      break;
    case MISSION_STATE_S1_DEPART:
    case MISSION_STATE_S2_CROSS_BUMP:
      raw_goal = mission_ctx.mission_snapshot.yaw_target_cdeg;
      role = "DEPART";
      break;
    case MISSION_STATE_S3_SEARCH_TURN_CW:
    case MISSION_STATE_S3_SEARCH_TURN_CCW:
      raw_goal = mission_ctx.s3_search_yaw_target_cdeg;
      role = mission_ctx.vision_search_forward_active ? "SEARCH_REF" : "SEARCH_TURN";
      break;
    case MISSION_STATE_S3_E3_TURN_AWAY:
      raw_goal = mission_ctx.s3_search_yaw_target_cdeg;
      role = "E3_AWAY";
      break;
    case MISSION_STATE_S6_BAR_REVERSE:
      raw_goal = mission_ctx.s6_bar_reverse_yaw_cdeg;
      role = "46_BACK";
      break;
    case MISSION_STATE_S6_BAR_TURN_SIDE:
    case MISSION_STATE_S6_BAR_FORWARD:
      raw_goal = mission_ctx.s6_bar_side_yaw_cdeg;
      role = "46_SIDE";
      break;
    case MISSION_STATE_S6_BAR_TURN_SAFE:
    case MISSION_STATE_S6_BAR_WAIT_ACK:
      role = "46_SAFE";
      break;
    case MISSION_STATE_SUPPLEMENT_TURN_LEFT:
      raw_goal = mission_ctx.supplement_left_yaw_cdeg;
      role = "SUPP_L45";
      break;
    case MISSION_STATE_SUPPLEMENT_TURN_RIGHT:
      raw_goal = mission_ctx.supplement_right_yaw_cdeg;
      role = "SUPP_R90";
      break;
    case MISSION_STATE_SUPPLEMENT_E3_SPIN_CCW:
      raw_goal = mission_ctx.supplement_e3_spin_previous_yaw_cdeg;
      role = mission_ctx.supplement_e3_spin_goal_cdeg == ROBOT_SUPPLEMENT_E3_FIRST_SPIN_CDEG
          ? "SUPP_CCW180" : "SUPP_CCW360";
      break;
    case MISSION_STATE_S4_ARRANGE_RECOVERY_TURN_LEFT:
      raw_goal = mission_ctx.s4_arrange_recovery_left_yaw_cdeg;
      role = "ARRANGE_LEFT";
      break;
    case MISSION_STATE_S4_E4_PUSH_RIGHT:
    case MISSION_STATE_S4_E4_REVERSE_RIGHT:
    case MISSION_STATE_S4_E4_PUSH_LEFT:
    case MISSION_STATE_S4_E4_REVERSE_LEFT:
      raw_goal = mission_ctx.s4_green_e4_half_turn_active
          ? mission_ctx.s4_e4_spin_origin_yaw_cdeg + (int32_t)ROBOT_YAW_LEFT_SIGN * ROBOT_S4_E4_GREEN_FIRST_TURN_CDEG
          : mission_ctx.s4_green_scatter_yaw_cdeg;
      role = mission_ctx.s4_green_e4_half_turn_active ? "E4_CCW180"
          : (mission_ctx.mission_snapshot.state == MISSION_STATE_S4_E4_REVERSE_RIGHT
              ? "E5_FWD_REF" : "E5_FIXED_TURN");
      break;
    case MISSION_STATE_S4_E4_TURN_LEFT:
      raw_goal = mission_ctx.s4_e4_left_yaw_cdeg;
      role = "E4_LEFT";
      break;
    case MISSION_STATE_S4_E4_TURN_AWAY:
    case MISSION_STATE_S4_E4_FORWARD:
      raw_goal = mission_ctx.s4_e4_away_yaw_cdeg;
      role = "E4_AWAY";
      break;
    case MISSION_STATE_S4_E4_TURN_RIGHT:
      raw_goal = mission_ctx.s4_e4_right_yaw_cdeg;
      role = "E4_RIGHT";
      break;
    case MISSION_STATE_S4_E4_GREEN_SPIN_360:
      raw_goal = mission_ctx.s4_e4_spin_origin_yaw_cdeg;
      role = "E4_360";
      break;
    case MISSION_STATE_S4_E4_RED_SPIN_360:
      raw_goal = mission_ctx.s4_red_e4_align_away_active ? mission_ctx.s4_e4_away_yaw_cdeg : mission_ctx.s3_search_yaw_target_cdeg;
      role = "E4_RED_SCAN";
      break;
    case MISSION_STATE_S4_E4_MATERIAL_SPIN_CW_360:
      raw_goal = mission_ctx.s4_e4_spin_origin_yaw_cdeg;
      role = "E4_MAT_CW360";
      break;
    case MISSION_STATE_S4_POST22_TURN_LEFT:
      raw_goal = mission_ctx.s4_post22_left_yaw_cdeg;
      role = "22E2_LEFT";
      break;
    case MISSION_STATE_S4_POST22_TURN_RIGHT:
      raw_goal = mission_ctx.s4_post22_right_yaw_cdeg;
      role = "22E2_RIGHT";
      break;
    case MISSION_STATE_S4_ARRANGE_RECOVERY_TURN_BACK:
      raw_goal = mission_ctx.s4_arrange_recovery_right_yaw_cdeg;
      role = "ARRANGE_BACK";
      break;
    case MISSION_STATE_S4_FINAL_RECOVERY_TURN_270:
      raw_goal = mission_ctx.s4_final_first_yaw_cdeg;
      role = "24_SIDE1";
      break;
    case MISSION_STATE_S4_FINAL_RECOVERY_TURN_90:
      raw_goal = mission_ctx.s4_final_second_yaw_cdeg;
      role = "24_SIDE2";
      break;
    case MISSION_STATE_S6_REPOSITION_TURN_SIDE:
    case MISSION_STATE_S6_REPOSITION_FORWARD:
      raw_goal = mission_ctx.s6_reposition_side_yaw_cdeg;
      role = "SIDE";
      break;
    case MISSION_STATE_S6_CORNER_TURN_AWAY:
    case MISSION_STATE_S6_CORNER_FORWARD_AWAY:
      raw_goal = mission_ctx.s6_corner_away_yaw_cdeg;
      role = "CORNER_AWAY";
      break;
    case MISSION_STATE_S6_CORNER_TURN_SIDE:
    case MISSION_STATE_S6_CORNER_FORWARD_SIDE:
      raw_goal = mission_ctx.s6_corner_side_yaw_cdeg;
      role = "CORNER_SIDE";
      break;
    case MISSION_STATE_S6_RECOVERY_START:
      raw_goal = mission_ctx.s6_recovery_first_forward_active ? mission_ctx.mission_snapshot.yaw_cdeg
                                                : mission_ctx.s6_recovery_base_yaw_cdeg;
      role = mission_ctx.s6_recovery_first_forward_active ? "RECOVER_F170"
          : (mission_ctx.s6_recovery_count < ROBOT_S6_RECOVERY_LONG_START_COUNT
                 ? "RECOVER_SAFE" : "RECOVER_FWD");
      break;
    case MISSION_STATE_S6_RECOVERY_TURN_LEFT:
      raw_goal = mission_ctx.s6_recovery_left_yaw_cdeg;
      role = "RECOVER_LEFT";
      break;
    case MISSION_STATE_S6_RECOVERY_TURN_BACK:
      raw_goal = mission_ctx.s6_recovery_right_yaw_cdeg;
      role = "RECOVER_BACK";
      break;
    case MISSION_STATE_S6_OBSTACLE_TURN_OUT:
    case MISSION_STATE_S6_OBSTACLE_FORWARD:
      raw_goal = mission_ctx.s6_obstacle_out_yaw_cdeg;
      role = mission_ctx.s6_obstacle_turn_right ? "36_RIGHT" : "36_LEFT";
      break;
    case MISSION_STATE_S6_OBSTACLE_TURN_BACK:
      raw_goal = mission_ctx.s6_obstacle_back_yaw_cdeg;
      role = mission_ctx.s6_obstacle_turn_right ? "36_BACK_LEFT" : "36_BACK_RIGHT";
      break;
    case MISSION_STATE_S6_EXIT_TURN_180:
      raw_goal = mission_ctx.s6_exit_yaw_target_cdeg;
      role = "EXIT180";
      break;
    case MISSION_STATE_S7_SILENT_TURN_LEFT:
      raw_goal = mission_ctx.s7_silent_left_yaw_cdeg;
      role = "07_LEFT45";
      break;
    case MISSION_STATE_S7_SILENT_TURN_BACK:
      raw_goal = mission_ctx.s7_silent_right_yaw_cdeg;
      role = "07_RIGHT90";
      break;
    case MISSION_STATE_S4_FINAL_TURN_SAFE:
    case MISSION_STATE_S5_ALIGN_FOR_ARRANGE:
    case MISSION_STATE_S6_SEARCH_SAFE_ZONE:
    case MISSION_STATE_S6_RECOVERY_RETURN_SAFE:
    case MISSION_STATE_S6_REPOSITION_TURN_SAFE:
    case MISSION_STATE_S6_CORNER_TURN_SAFE:
    case MISSION_STATE_S6_OBSTACLE_FACE_SAFE:
    case MISSION_STATE_S6_FINAL_ALIGN:
    case MISSION_STATE_S6_FINAL_VERIFY:
    case MISSION_STATE_S6_FINAL_REVERSE:
      role = "SAFE_HOLD";
      break;
    case MISSION_STATE_S6_BORDER_LOCATE:
    case MISSION_STATE_S6_BORDER_BACKOFF:
    case MISSION_STATE_S6_FINAL_PUSH:
      raw_goal = mission_ctx.s6_push_yaw_target_cdeg;
      role = mission_ctx.mission_snapshot.state == MISSION_STATE_S6_BORDER_LOCATE ? "BORDER_LOCATE" :
          (mission_ctx.mission_snapshot.state == MISSION_STATE_S6_BORDER_BACKOFF ? "BORDER_BACK" : "PUSH_IN");
      break;
    default:
      /* Vision PID states use safe direction as a reference, not a yaw goal. */
      break;
  }

  raw_current = MissionTask_WrapYaw(mission_ctx.mission_snapshot.yaw_cdeg);
  raw_zero = MissionTask_WrapYaw(mission_ctx.mission_snapshot.yaw_start_cdeg);
  field_current = MissionTask_WrapYaw(raw_current - raw_zero);
  raw_goal = MissionTask_WrapYaw(raw_goal);
  field_goal = MissionTask_WrapYaw(raw_goal - raw_zero);
  (void)DebugUart_Logf(
      "[ANGLE] s=%u r=%ld.%02ld z=%ld.%02ld f=%ld.%02ld zone=%u team=%u ok=%u\r\n",
      (unsigned int)mission_ctx.mission_snapshot.state,
      (long)(raw_current / 100L), (long)(raw_current % 100L),
      (long)(raw_zero / 100L), (long)(raw_zero % 100L),
      (long)(field_current / 100L), (long)(field_current % 100L),
      (unsigned int)mission_ctx.mission_snapshot.start_zone,
      (unsigned int)mission_ctx.mission_snapshot.team,
      mission_ctx.mission_snapshot.imu_valid ? 1U : 0U);
  (void)DebugUart_Logf(
      "[GOAL] %s r=%ld.%02ld f=%ld.%02ld e=%ld\r\n",
      role, (long)(raw_goal / 100L), (long)(raw_goal % 100L),
      (long)(field_goal / 100L), (long)(field_goal % 100L),
      (long)MissionTask_YawError(raw_goal, raw_current));
}

void StartMissionTask(void *argument)
{
  uint32_t events;
  uint32_t now;

  (void)argument;
  mission_ctx.mission_snapshot.state = MISSION_STATE_BOOT;
  mission_ctx.mission_snapshot.s1_phase = MISSION_S1_IDLE;
  mission_ctx.mission_snapshot.fault_flags = MISSION_FAULT_NONE;
  mission_ctx.mission_snapshot.state_entry_tick = osKernelGetTickCount();
  yaw_next_debug_tick = HAL_GetTick();
  mission_ctx.commanded_left_rpm = 0;
  mission_ctx.commanded_right_rpm = 0;
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
      /* Only time in16 PID counts; recovery/side/corner moves/36 pause it. */
      if (mission_ctx.mission_snapshot.state == MISSION_STATE_S6_TRACK_SAFE_ZONE)
      {
        mission_ctx.s6_track_elapsed_ms += (uint32_t)(now - mission_ctx.s6_track_accounting_tick);
      }
      mission_ctx.s6_track_accounting_tick = now;
      MissionTask_UpdateInputs(now);
      MissionTask_ProcessEvents(events, now);
      MissionTask_CheckRunningHealth(now);
      if (Actuator_ServiceDeferredClawClose(now) != HAL_OK)
      {
        mission_ctx.mission_snapshot.fault_flags |= MISSION_FAULT_ACTUATOR;
        MissionTask_EnterState(MISSION_STATE_FAULT, now);
      }
      (void)MissionTask_HandleLoadCheckRequest(now);
      MissionTask_HandleSupplementControl(now);
      MissionTask_DropUnexpectedS5E5();
      MissionTask_HandleStartZoneAvoidanceRequest(now);
      MissionTask_RunStartZoneAvoidance(now);
      /* AE replies may deliver fresh coordinates/events during UART TX. */
      now = HAL_GetTick();
      MissionTask_UpdateVisionInput(now);
      MissionTask_HandleRedPriorityRequest(now);
      MissionTask_DropDuplicateRecoveryEvent(now);
      MissionTask_HandleS6BarShiftRequest(now);
      MissionTask_HandleS6AlignRequest(now);
      MissionTask_RunS1(now);
      MissionTask_RunS2CrossBump(now);
      MissionTask_RunS3SearchTurn(now);
      MissionTask_RunS3Track(now);
      MissionTask_RunS3LowerFrame(now);
      MissionTask_HandleS4FineArrangeRequest(now);
      MissionTask_RunS4Arrange(now);
      /* First-capture TX34 can create a new supplement epoch after the
         cycle's original tick. Use fresh time for its settle/budget checks. */
      now = HAL_GetTick();
      MissionTask_RunS4FineArrange(now);
      MissionTask_RunS4Post22Recovery(now);
      MissionTask_RunS4E4Recovery(now);
      MissionTask_RunS6LoadRecheck(now);
      MissionTask_RunS5WaitSingleGreen(now);
      MissionTask_RunS5PrepareArrange(now);
      MissionTask_RunSupplement(now);
      MissionTask_RunS6BarShift(now);
      /*46 may enter tracking/final26/review05 after a blocking UART TX.
         Never evaluate those new epochs using the pre-TX cycle timestamp.
         Refresh coordinate age too; do not discard newly received frames. */
      now = HAL_GetTick();
      MissionTask_UpdateVisionInput(now);
      MissionTask_ServiceS6FoundBafflePulse(now);
      MissionTask_RunS6SearchSafeZone(now);
      MissionTask_RunS6SideReposition(now);
      MissionTask_RunS6CornerEvacuation(now);
      MissionTask_RunS6Recovery(now);
      MissionTask_RunS6TrackSafeZone(now);
      MissionTask_RunS6TrackTimeoutReverse(now);
      MissionTask_RunS6ObstacleAvoidance(now);
      MissionTask_RunS6FinalAlign(now);
      MissionTask_RunS6BorderLocate(now);
      MissionTask_RunS6FinalReverse(now);
      MissionTask_RunS6ExitTurn180(now);
      MissionTask_RunS7SilentSearch(now);
      MissionTask_RunS7Decision(now);
      MissionTask_DebugYaw(now);
      MissionTask_DebugAngles(now, false);
      MissionTask_DebugVisionAge(now);
      (void)osMutexRelease(mission_snapshot_mutex);
    }
  }
}
