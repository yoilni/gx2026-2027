#include "mission_safe_zone.h"
#include "mission_internal.h"
#include "mission_state.h"
#include "mission_safety.h"
#include "mission_vision.h"
#include "mission_capture.h"
#include "mission_load.h"
#include "mission_delivery.h"

#include "M2006.h"
#include "actuator_task.h"
#include "chassis_actions.h"
#include "chassis_motion.h"
#include "bar_shift_flow.h"
#include "recovery_flow.h"
#include "obstacle_flow.h"
#include "reposition_flow.h"
#include "debug_uart_task.h"
#include "heading_control.h"
#include "maixcam_task.h"
#include "robot_config.h"
#include "stm32f4xx_hal.h"
#include "vision_motion.h"
#include "flow_motion.h"

static HeadingControl_State s6_yaw_pid;
static MissionState s6_turn_timeout_state;
static uint32_t s6_turn_timeout_entry_tick;
static bool s6_turn_timeout_reported;
static uint32_t s6_found_baffle_pwm_tick;
static uint32_t s6_missing_since;
static uint32_t s6_search_start_tick;
static bool s6_search_timeout_reported;
static uint32_t s6_approach_start_tick;
/* Freeze travel at the decision angle, never at the subsequent90/270 heading. */
static uint32_t s6_reposition_forward_ms;
static uint32_t s6_x_align_last_sequence;
static uint32_t s6_x_align_first_tick;
static uint8_t s6_x_align_id;
static bool s6_x_align_has_sequence;
static bool s6_supply_y_started;
static int32_t s6_obstacle_decision_yaw_cdeg;
static uint8_t s6_reposition_count;
static bool s6_recovery_from_tracking;

static void MissionTask_StartS6ObstacleAvoidance(uint32_t now);

void MissionTask_ResetS6ControllerState(void)
{
  s6_yaw_pid = (HeadingControl_State){0};

  mission_ctx.mission_snapshot.vision_turn_rpm = 0;
  mission_ctx.mission_snapshot.vision_forward_rpm = 0;
  mission_ctx.mission_snapshot.safe_zone_yaw_command_cdeg =
      mission_ctx.mission_snapshot.safe_zone_yaw_target_cdeg;
  mission_ctx.mission_snapshot.safe_zone_yaw_error_cdeg = 0;
  mission_ctx.mission_snapshot.safe_zone_fixed_heading_active = false;
}

bool MissionTask_HeadingTurnExpired(uint32_t now,
                                             int32_t target_yaw_cdeg)
{
  uint32_t elapsed = (uint32_t)(now - mission_ctx.mission_snapshot.state_entry_tick);
  if (elapsed < ROBOT_HEADING_TURN_TIMEOUT_MS) return false;

  mission_ctx.mission_snapshot.safe_zone_yaw_command_cdeg = target_yaw_cdeg;
  mission_ctx.mission_snapshot.safe_zone_yaw_error_cdeg = MissionTask_YawError(
      target_yaw_cdeg, mission_ctx.mission_snapshot.yaw_cdeg);
  if (s6_turn_timeout_state != mission_ctx.mission_snapshot.state ||
      s6_turn_timeout_entry_tick != mission_ctx.mission_snapshot.state_entry_tick)
  {
    s6_turn_timeout_state = mission_ctx.mission_snapshot.state;
    s6_turn_timeout_entry_tick = mission_ctx.mission_snapshot.state_entry_tick;
    s6_turn_timeout_reported = false;
  }
  if (!s6_turn_timeout_reported)
  {
    s6_turn_timeout_reported = true;
    (void)DebugUart_Logf(
        "[TURN-TMO] s=%u ms=%lu r=%ld goal=%ld e=%ld NEXT\r\n",
        (unsigned int)mission_ctx.mission_snapshot.state, (unsigned long)elapsed,
        (long)MissionTask_WrapYaw(mission_ctx.mission_snapshot.yaw_cdeg),
        (long)MissionTask_WrapYaw(target_yaw_cdeg),
        (long)mission_ctx.mission_snapshot.safe_zone_yaw_error_cdeg);
  }
  MissionTask_StopWheels();
  mission_ctx.mission_snapshot.vision_turn_rpm = 0;
  mission_ctx.mission_snapshot.vision_forward_rpm = 0;
  s6_yaw_pid.integral = 0.0f;
  s6_yaw_pid.initialized = false;
  return true;
}

void MissionTask_StartS6FromLoad(uint32_t now)
{
  if (!mission_ctx.s5_recheck_resume_safe_tracking) mission_ctx.s6_corner_completed = false;
  if (!mission_ctx.s5_recheck_resume_safe_tracking) mission_ctx.s6_bar_shift_completed = false;
  mission_ctx.s6_bar_shift_ack_pending = false;
  mission_ctx.s6_bar_initial_align_pending = false;
  mission_ctx.supplement_capture_active = false;
  mission_ctx.supplement_budget_started = false;
  MaixCam_SetSupplementWindowEnabled(false);
  (void)DebugUart_Log(
      "[05] RX06 >SAFE\r\n");
  /* Disable PA2 PWM until safe-zone detection and hold PA3 closed. The
     paired pose setter cannot write or restart PA2 while suspended. */
  MissionTask_StopWheels();
  if ((Actuator_StopBafflePwm() != HAL_OK) ||
      (Actuator_SetFrameLowered() != HAL_OK) ||
      (Actuator_SetCameraSafeZoneView() != HAL_OK))
  {
    mission_ctx.mission_snapshot.fault_flags |= MISSION_FAULT_ACTUATOR;
    MissionTask_EnterState(MISSION_STATE_FAULT, now);
    return;
  }
  mission_ctx.s5_close_view_active = false;
  (void)DebugUart_Log("[06] PA2 PWM OFF\r\n");
  (void)DebugUart_Logf("[06] CAM=%u\r\n",
                       (unsigned int)ROBOT_CAMERA_SAFE_ZONE_ANGLE_DEG);
  (void)DebugUart_Logf("[CARGO] LOCK id=%u cmd=%02X kind=%s src=%s\r\n",
      (unsigned int)MissionTask_GetCarriedObjectId(),
      (unsigned int)mission_ctx.s3_target_select_command,
      MissionTask_S6IsSupplyTarget() ? "SUPPLY" :
          (MissionTask_S6IsCasualtyTarget() ? "CASUALTY" : "UNKNOWN"),
      mission_ctx.s5_load_class_confirmed ? "VISION05" : "TASK");

  MaixCam_ClearObject();
  mission_ctx.mission_snapshot.vision_target_valid = false;
  MissionTask_ResetVisionPid();
  MissionTask_ResetS6ControllerState();
  mission_ctx.s6_phase_stable_since = 0U;
  mission_ctx.s6_next_debug_tick = now;
  s6_missing_since = 0U;
  s6_search_start_tick = now;
  s6_search_timeout_reported = false;
  mission_ctx.s6_track_elapsed_ms = 0U;
  mission_ctx.s6_track_accounting_tick = now;
  mission_ctx.s6_align26_pending = false;
  mission_ctx.s6_pre_reposition_decision_pending = false;
  s6_supply_y_started = false;
  mission_ctx.s6_obstacle_side_required = false;
  mission_ctx.s6_obstacle_side_decision_pending = false;
  mission_ctx.s6_obstacle_side_decision_done = false;
  MissionTask_ResetS6XConfirmation();
  mission_ctx.s6_recovery_count = 0U;
  MissionTask_EnterState(MISSION_STATE_S6_SEARCH_SAFE_ZONE, now);
  (void)DebugUart_Logf(
      "[06] TURN goal=%ld e=%ld max=%d tmo=%lu\r\n",
      (long)mission_ctx.mission_snapshot.safe_zone_yaw_target_cdeg,
      (long)MissionTask_YawError(mission_ctx.mission_snapshot.safe_zone_yaw_target_cdeg,
                                 mission_ctx.mission_snapshot.yaw_cdeg),
      (int)ROBOT_S6_SEARCH_TURN_MAX_RPM,
      (unsigned long)ROBOT_S6_SEARCH_TIMEOUT_MS);
}

bool MissionTask_S6IsSupplyTarget(void)
{
  uint8_t object_id = MissionTask_GetCarriedObjectId();
  return object_id == 3U || object_id == 5U || object_id == 6U ||
      (object_id == 0U &&
       mission_ctx.s3_target_select_command == MAIXCAM_COMMAND_SELECT_BLACK_GREEN);
}

bool MissionTask_S6IsCasualtyTarget(void)
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
static const MissionSideRule s6_corner_rules[] = { ROBOT_S6_CORNER_RULE_ROWS };
static const MissionSideRule *MissionTask_GetS6SideRule(
    int32_t decision_yaw_cdeg, bool *has_calibrated_rule)
{
  uint32_t i;
  uint8_t kind;
  int32_t yaw_from_start = MissionTask_WrapYaw(
      decision_yaw_cdeg - mission_ctx.mission_snapshot.yaw_start_cdeg);

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
    if (rule->enabled && (rule->start_zone == mission_ctx.mission_snapshot.start_zone) &&
        (rule->team == mission_ctx.mission_snapshot.team) && (rule->target_kind == kind))
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

static bool MissionTask_GetS6CornerHeading(int32_t decision_yaw_cdeg,
                                            int32_t *side_yaw_cdeg)
{
  uint32_t i;
  int32_t yaw_from_start = MissionTask_WrapYaw(
      decision_yaw_cdeg - mission_ctx.mission_snapshot.yaw_start_cdeg);
  /* Corner geometry is shared by supplies and casualties. The common angle
     selector checks evacuation before ordinary side motion, including the
     overlapping part of the casualty side sector. */
  if (!MissionTask_S6IsSupplyTarget() && !MissionTask_S6IsCasualtyTarget())
    return false;
  for (i = 0U; i < sizeof(s6_corner_rules) / sizeof(s6_corner_rules[0]); ++i)
  {
    const MissionSideRule *rule = &s6_corner_rules[i];
    if (rule->enabled && rule->start_zone == mission_ctx.mission_snapshot.start_zone &&
        rule->team == mission_ctx.mission_snapshot.team)
    {
      int32_t span = MissionTask_WrapYaw(rule->sector_end_cdeg - rule->sector_start_cdeg);
      int32_t position = MissionTask_WrapYaw(yaw_from_start - rule->sector_start_cdeg);
      if (((position > 0L) || rule->include_start) &&
          ((position < span) || ((position == span) && rule->include_end)))
      {
        *side_yaw_cdeg = MissionTask_WrapYaw(
            mission_ctx.mission_snapshot.yaw_start_cdeg + rule->side_heading_cdeg);
        return true;
      }
    }
  }
  return false;
}

static bool MissionTask_S6NeedsSideReposition(int32_t decision_yaw_cdeg)
{
  bool has_calibrated_rule;
  const MissionSideRule *rule = MissionTask_GetS6SideRule(
      decision_yaw_cdeg, &has_calibrated_rule);
  int32_t yaw_error = MissionTask_YawError(
      mission_ctx.mission_snapshot.safe_zone_yaw_target_cdeg,
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

static uint32_t MissionTask_S6SideForwardDuration(int32_t decision_yaw_cdeg,
                                                   const MissionSideRule *rule)
{
  float distance_scale = ROBOT_S6_REPOSITION_DISTANCE_MIN_SCALE;
  if (rule != NULL)
  {
    int32_t start = 0L;
    int32_t end = MissionTask_WrapYaw(rule->sector_end_cdeg - rule->sector_start_cdeg);
    int32_t position = MissionTask_WrapYaw(decision_yaw_cdeg -
        mission_ctx.mission_snapshot.yaw_start_cdeg - rule->sector_start_cdeg);
    uint32_t i;
    /* Corner evacuation wins at overlapping edges. Interpolate only across
       the remaining ordinary side sector, e.g. casualty135..165, not90..165. */
    for (i = 0U; i < sizeof(s6_corner_rules) / sizeof(s6_corner_rules[0]); ++i)
    {
      const MissionSideRule *corner = &s6_corner_rules[i];
      if (corner->enabled && corner->start_zone == rule->start_zone &&
          corner->team == rule->team)
      {
        int32_t corner_start = MissionTask_YawError(corner->sector_start_cdeg,
                                                   rule->sector_start_cdeg);
        int32_t corner_end = corner_start + MissionTask_WrapYaw(
            corner->sector_end_cdeg - corner->sector_start_cdeg);
        if (corner_start <= start && corner_end > start && corner_end < end)
          start = corner_end;
        if (corner_end >= end && corner_start < end && corner_start > start)
          end = corner_start;
      }
    }
    if (end > start)
    {
      int32_t near_start_error = MissionTask_Abs32(MissionTask_YawError(
          mission_ctx.mission_snapshot.safe_zone_yaw_target_cdeg,
          MissionTask_WrapYaw(mission_ctx.mission_snapshot.yaw_start_cdeg + rule->sector_start_cdeg + start)));
      int32_t near_end_error = MissionTask_Abs32(MissionTask_YawError(
          mission_ctx.mission_snapshot.safe_zone_yaw_target_cdeg,
          MissionTask_WrapYaw(mission_ctx.mission_snapshot.yaw_start_cdeg + rule->sector_start_cdeg + end)));
      float far_fraction = near_start_error > near_end_error
          ? (float)(end - position) / (float)(end - start)
          : (float)(position - start) / (float)(end - start);
      far_fraction = MissionTask_ClampFloat(far_fraction, 0.0f, 1.0f);
      distance_scale += far_fraction * (ROBOT_S6_REPOSITION_DISTANCE_MAX_SCALE -
                                        ROBOT_S6_REPOSITION_DISTANCE_MIN_SCALE);
    }
  }
  return (uint32_t)((float)ROBOT_S6_REPOSITION_FORWARD_MS *
      (float)ROBOT_S6_REPOSITION_REFERENCE_RPM / (float)ROBOT_S6_REPOSITION_FORWARD_RPM *
      distance_scale + 0.5f);
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
  mission_ctx.mission_snapshot.vision_target_valid = false;
  MissionTask_ResetVisionPid();
  MissionTask_ResetS6ControllerState();
  mission_ctx.s6_phase_stable_since = 0U;
  MissionTask_ResetS6XConfirmation();
  s6_approach_start_tick = now;
  s6_reposition_forward_ms = MissionTask_S6SideForwardDuration(decision_yaw_cdeg, rule);
  mission_ctx.s6_reposition_side_yaw_cdeg = rule != NULL
      ? MissionTask_WrapYaw(mission_ctx.mission_snapshot.yaw_start_cdeg +
                           rule->side_heading_cdeg)
      : MissionTask_WrapYaw(
      mission_ctx.mission_snapshot.safe_zone_yaw_target_cdeg +
      (MissionTask_S6IsSupplyTarget() ? -1L : 1L) *
          ROBOT_S6_REPOSITION_SECTOR_CDEG);
  ++s6_reposition_count;
  (void)DebugUart_Logf("[SIDE] field=%ld v=%d ms=%lu dist=%lu%%\r\n",
      (long)MissionTask_WrapYaw(decision_yaw_cdeg - mission_ctx.mission_snapshot.yaw_start_cdeg),
      ROBOT_S6_REPOSITION_FORWARD_RPM, (unsigned long)s6_reposition_forward_ms,
      (unsigned long)((s6_reposition_forward_ms * (uint32_t)ROBOT_S6_REPOSITION_FORWARD_RPM * 100U +
          ROBOT_S6_REPOSITION_FORWARD_MS * ROBOT_S6_REPOSITION_REFERENCE_RPM / 2U) /
          (ROBOT_S6_REPOSITION_FORWARD_MS * ROBOT_S6_REPOSITION_REFERENCE_RPM)));
  mission_ctx.s6_next_debug_tick = now;
  MissionTask_EnterState(MISSION_STATE_S6_REPOSITION_TURN_SIDE, now);
  (void)DebugUart_Logf(
      "[SIDE] n=%u r=%ld safe=%ld side=%ld cmd=%02X id=%u kind=%s "
      "zone=%u team=%u rule=%s\r\n",
      (unsigned int)s6_reposition_count,
      (long)MissionTask_WrapYaw(mission_ctx.mission_snapshot.yaw_cdeg),
      (long)mission_ctx.mission_snapshot.safe_zone_yaw_target_cdeg,
      (long)mission_ctx.s6_reposition_side_yaw_cdeg,
      (unsigned int)mission_ctx.s3_target_select_command,
      (unsigned int)MissionTask_GetCarriedObjectId(),
      MissionTask_S6IsSupplyTarget() ? "SUPPLY" :
          (MissionTask_S6IsCasualtyTarget()
               ? "CASUALTY" : "LEGACY"),
      (unsigned int)mission_ctx.mission_snapshot.start_zone,
      (unsigned int)mission_ctx.mission_snapshot.team,
      rule != NULL ? "CFG" : "DEFAULT");
  current_relative_cdeg = MissionTask_WrapYaw(
      mission_ctx.mission_snapshot.yaw_cdeg - mission_ctx.mission_snapshot.yaw_start_cdeg);
  target_relative_cdeg = MissionTask_WrapYaw(
      mission_ctx.s6_reposition_side_yaw_cdeg - mission_ctx.mission_snapshot.yaw_start_cdeg);
  safe_relative_cdeg = MissionTask_WrapYaw(
      mission_ctx.mission_snapshot.safe_zone_yaw_target_cdeg - mission_ctx.mission_snapshot.yaw_start_cdeg);
  (void)DebugUart_Logf(
      "[SIDE] dec=%ld.%02ld cur=%ld.%02ld goal=%ld.%02ld safe=%ld.%02ld\r\n",
      (long)(MissionTask_WrapYaw(decision_yaw_cdeg -
          mission_ctx.mission_snapshot.yaw_start_cdeg) / 100L),
      (long)(MissionTask_WrapYaw(decision_yaw_cdeg -
          mission_ctx.mission_snapshot.yaw_start_cdeg) % 100L),
      (long)(current_relative_cdeg / 100L),
      (long)(current_relative_cdeg % 100L),
      (long)(target_relative_cdeg / 100L),
      (long)(target_relative_cdeg % 100L),
      (long)(safe_relative_cdeg / 100L),
      (long)(safe_relative_cdeg % 100L));
  (void)DebugUart_Logf(
      "[SIDE] r=%ld goal=%ld zero=%ld\r\n",
      (long)MissionTask_WrapYaw(mission_ctx.mission_snapshot.yaw_cdeg),
      (long)mission_ctx.s6_reposition_side_yaw_cdeg,
      (long)mission_ctx.mission_snapshot.yaw_start_cdeg);
}

static void MissionTask_StartS6CornerEvacuation(uint32_t now,
                                                int32_t decision_yaw_cdeg,
                                                int32_t side_yaw_cdeg)
{
  MissionTask_StopWheels();
  MaixCam_ClearObject();
  mission_ctx.mission_snapshot.vision_target_valid = false;
  MissionTask_ResetVisionPid();
  MissionTask_ResetS6ControllerState();
  MissionTask_ResetS6XConfirmation();
  mission_ctx.s6_corner_away_yaw_cdeg = MissionTask_WrapYaw(
      mission_ctx.mission_snapshot.safe_zone_yaw_target_cdeg + 18000L);
  mission_ctx.s6_corner_side_yaw_cdeg = side_yaw_cdeg;
  s6_approach_start_tick = now;
  mission_ctx.s6_phase_stable_since = 0U;
  mission_ctx.s6_next_debug_tick = now;
  MissionTask_EnterState(MISSION_STATE_S6_CORNER_TURN_AWAY, now);
  (void)DebugUart_Logf("[CORNER] START id=%u zone=%u team=%u field=%ld\r\n",
      (unsigned int)MissionTask_GetCarriedObjectId(),
      (unsigned int)mission_ctx.mission_snapshot.start_zone, (unsigned int)mission_ctx.mission_snapshot.team,
      (long)MissionTask_WrapYaw(decision_yaw_cdeg - mission_ctx.mission_snapshot.yaw_start_cdeg));
  (void)DebugUart_Logf("[CORNER] away=%ld side=%ld safe=%ld v=%d ms=%lu/%lu\r\n",
      (long)MissionTask_WrapYaw(mission_ctx.s6_corner_away_yaw_cdeg - mission_ctx.mission_snapshot.yaw_start_cdeg),
      (long)MissionTask_WrapYaw(mission_ctx.s6_corner_side_yaw_cdeg - mission_ctx.mission_snapshot.yaw_start_cdeg),
      (long)MissionTask_WrapYaw(mission_ctx.mission_snapshot.safe_zone_yaw_target_cdeg - mission_ctx.mission_snapshot.yaw_start_cdeg),
      ROBOT_S6_CORNER_FORWARD_RPM,
      (unsigned long)ROBOT_S6_CORNER_AWAY_FORWARD_MS,
      (unsigned long)ROBOT_S6_CORNER_SIDE_FORWARD_MS);
}

/* Both ordinary16 and the saved post36 decision use this selector. */
static bool MissionTask_TryS6AngleManeuver(uint32_t now, int32_t decision_yaw_cdeg)
{
  int32_t side_yaw_cdeg;
  if (mission_ctx.s6_align26_pending) return false;
  if (MissionTask_GetS6CornerHeading(decision_yaw_cdeg, &side_yaw_cdeg))
  {
    if (mission_ctx.s6_corner_completed)
    {
      (void)DebugUart_Log("[CORNER] DONE BEFORE; KEEP XY\r\n");
      return false;
    }
    MissionTask_StartS6CornerEvacuation(now, decision_yaw_cdeg, side_yaw_cdeg);
    return true;
  }
  if (MissionTask_S6NeedsSideReposition(decision_yaw_cdeg))
  {
    MissionTask_StartS6SideReposition(now, decision_yaw_cdeg);
    return true;
  }
  return false;
}

static void MissionTask_ResetS6PreDecisionTracking(uint32_t now)
{
  mission_ctx.s6_pre_reposition_track_elapsed_ms = 0U;
  mission_ctx.s6_pre_reposition_track_tick = now;
  mission_ctx.s6_pre_reposition_track_active = false;
}

static bool MissionTask_S6PreDecisionTrackingReady(void)
{
  return mission_ctx.s6_pre_reposition_track_elapsed_ms >= ROBOT_S6_PRE_REPOSITION_TRACK_MS;
}

static void MissionTask_UpdateS6PreDecisionTracking(uint32_t now)
{
  uint32_t interval_ms = (uint32_t)(now - mission_ctx.s6_pre_reposition_track_tick);
  bool valid_tracking = mission_ctx.mission_snapshot.vision_target_valid &&
      mission_ctx.mission_snapshot.target_age_ms <= MISSION_VISION_TARGET_TIMEOUT_MS;
  mission_ctx.s6_pre_reposition_track_tick = now;
  if (valid_tracking && mission_ctx.s6_pre_reposition_track_active &&
      interval_ms <= MISSION_VISION_TARGET_TIMEOUT_MS &&
      !MissionTask_S6PreDecisionTrackingReady())
  {
    uint32_t remaining_ms = ROBOT_S6_PRE_REPOSITION_TRACK_MS -
        mission_ctx.s6_pre_reposition_track_elapsed_ms;
    mission_ctx.s6_pre_reposition_track_elapsed_ms +=
        interval_ms < remaining_ms ? interval_ms : remaining_ms;
    if (MissionTask_S6PreDecisionTrackingReady())
      (void)DebugUart_Logf("[SIDE] TRACK=%lu READY\r\n",
          (unsigned long)mission_ctx.s6_pre_reposition_track_elapsed_ms);
  }
  /* Missing coordinates pause the interval; they do not erase prior tracking. */
  mission_ctx.s6_pre_reposition_track_active = valid_tracking;
}

void MissionTask_ResetS6XConfirmation(void)
{
  mission_ctx.s6_x_align_frames = 0U;
  s6_x_align_first_tick = 0U;
  mission_ctx.s6_side_x_align_frames = 0U;
  mission_ctx.s6_side_x_align_first_tick = 0U;
  mission_ctx.s6_x_align_last_tick = 0U;
  s6_x_align_has_sequence = false;
}

static bool MissionTask_S6SideXConfirmed(void)
{
  bool confirmed = mission_ctx.mission_snapshot.vision_target_valid &&
      mission_ctx.mission_snapshot.target_age_ms <= ROBOT_VISION_COORD_DECEL_START_MS &&
      s6_x_align_has_sequence &&
      mission_ctx.mission_snapshot.target_sequence == s6_x_align_last_sequence &&
      mission_ctx.mission_snapshot.target_id == s6_x_align_id &&
      MissionTask_Abs32(mission_ctx.mission_snapshot.vision_x_error_px) <= ROBOT_S6_SIDE_X_TOLERANCE_PX &&
      mission_ctx.s6_side_x_align_frames >= ROBOT_S6_X_ALIGN_MIN_FRAMES;
#if ROBOT_S6_X_ALIGN_STABLE_MS > 0U
  confirmed = confirmed && (uint32_t)(mission_ctx.s6_x_align_last_tick - mission_ctx.s6_side_x_align_first_tick) >= ROBOT_S6_X_ALIGN_STABLE_MS;
#endif
  return confirmed;
}

static bool MissionTask_S6XConfirmed(void)
{
  bool confirmed = mission_ctx.mission_snapshot.vision_target_valid &&
      mission_ctx.mission_snapshot.target_age_ms <= ROBOT_VISION_COORD_DECEL_START_MS &&
      s6_x_align_has_sequence &&
      mission_ctx.mission_snapshot.target_sequence == s6_x_align_last_sequence &&
      mission_ctx.mission_snapshot.target_id == s6_x_align_id &&
      MissionTask_Abs32(mission_ctx.mission_snapshot.vision_x_error_px) <= ROBOT_S6_APPROACH_STOP_X_PX &&
      mission_ctx.s6_x_align_frames >= ROBOT_S6_X_ALIGN_MIN_FRAMES;
#if ROBOT_S6_X_ALIGN_STABLE_MS > 0U
  confirmed = confirmed && (uint32_t)(mission_ctx.s6_x_align_last_tick - s6_x_align_first_tick) >= ROBOT_S6_X_ALIGN_STABLE_MS;
#endif
  return confirmed;
}

static bool MissionTask_UpdateS6XConfirmation(uint32_t now)
{
  uint32_t receipt_tick = now - mission_ctx.mission_snapshot.target_age_ms;
  bool consecutive;
  if (!mission_ctx.mission_snapshot.vision_target_valid)
  {
    MissionTask_ResetS6XConfirmation();
    mission_ctx.s6_pre_reposition_decision_pending = !mission_ctx.s6_obstacle_side_decision_done;
    return false;
  }
  /* An aging cached frame may drive deceleration, but cannot prove alignment. */
  if (mission_ctx.mission_snapshot.target_age_ms > ROBOT_VISION_COORD_DECEL_START_MS) return false;
  if (s6_x_align_has_sequence &&
      mission_ctx.mission_snapshot.target_sequence == s6_x_align_last_sequence)
    return MissionTask_S6SideXConfirmed();

  consecutive = s6_x_align_has_sequence &&
      (uint32_t)(mission_ctx.mission_snapshot.target_sequence - s6_x_align_last_sequence) == 1U &&
      mission_ctx.mission_snapshot.target_id == s6_x_align_id &&
      (uint32_t)(receipt_tick - mission_ctx.s6_x_align_last_tick) <= ROBOT_S6_X_ALIGN_MAX_GAP_MS;
  s6_x_align_last_sequence = mission_ctx.mission_snapshot.target_sequence;
  s6_x_align_id = mission_ctx.mission_snapshot.target_id;
  mission_ctx.s6_x_align_last_tick = receipt_tick;
  s6_x_align_has_sequence = true;
  if (MissionTask_Abs32(mission_ctx.mission_snapshot.vision_x_error_px) > ROBOT_S6_SIDE_X_TOLERANCE_PX)
    mission_ctx.s6_side_x_align_frames = 0U;
  else
  {
    if (!consecutive || mission_ctx.s6_side_x_align_frames == 0U)
    {
      mission_ctx.s6_side_x_align_frames = 0U;
      mission_ctx.s6_side_x_align_first_tick = receipt_tick;
    }
    if (mission_ctx.s6_side_x_align_frames < UINT8_MAX) ++mission_ctx.s6_side_x_align_frames;
  }
  if (MissionTask_Abs32(mission_ctx.mission_snapshot.vision_x_error_px) > ROBOT_S6_APPROACH_STOP_X_PX)
    mission_ctx.s6_x_align_frames = 0U;
  else
  {
    if (!consecutive || mission_ctx.s6_x_align_frames == 0U)
    {
      mission_ctx.s6_x_align_frames = 0U;
      s6_x_align_first_tick = receipt_tick;
    }
    if (mission_ctx.s6_x_align_frames < UINT8_MAX) ++mission_ctx.s6_x_align_frames;
  }
  if (mission_ctx.s6_pre_reposition_decision_pending || mission_ctx.s6_align26_pending)
    (void)DebugUart_Logf("[X] q=%lu x=%d side=%u ms=%lu xOK=%u final=%u fOK=%u\r\n",
        (unsigned long)s6_x_align_last_sequence, (int)mission_ctx.mission_snapshot.vision_x_error_px,
        (unsigned int)mission_ctx.s6_side_x_align_frames,
        (unsigned long)(mission_ctx.s6_side_x_align_frames ? mission_ctx.s6_x_align_last_tick - mission_ctx.s6_side_x_align_first_tick : 0U),
        MissionTask_S6SideXConfirmed() ? 1U : 0U,
        (unsigned int)mission_ctx.s6_x_align_frames,
        MissionTask_S6XConfirmed() ? 1U : 0U);
  return MissionTask_S6SideXConfirmed();
}

void MissionTask_StartS6VisionApproach(uint32_t now)
{
  MissionTask_StopWheels();
  mission_ctx.s6_recovery_found16_pending = false;
  /* Preserve the freshly recovered errors; only reset controller history. */
  MissionTask_ResetVisionControllerState();
  MissionTask_ResetS6ControllerState();
  MissionTask_ResetS6XConfirmation();
  MissionTask_ResetS6PreDecisionTracking(now);
  mission_ctx.s6_pre_reposition_decision_pending = !mission_ctx.s6_obstacle_side_decision_done;
  s6_approach_start_tick = now;
  s6_missing_since = mission_ctx.mission_snapshot.vision_target_valid ? 0U : now;
  /* Reacquisition does not restart the loss-recovery allowance. Keep the
     count throughout this delivery so a second loss uses long-forward6E. */
  mission_ctx.s6_next_debug_tick = now;
  MissionTask_EnterState(MISSION_STATE_S6_TRACK_SAFE_ZONE, now);
  (void)DebugUart_Logf(
      "[06] TRACK r=%ld safe=%ld side=%u y=%u rec=%u ok=%u wait=%lu\r\n",
      (long)MissionTask_WrapYaw(mission_ctx.mission_snapshot.yaw_cdeg),
      (long)mission_ctx.mission_snapshot.safe_zone_yaw_target_cdeg,
      (unsigned int)s6_reposition_count,
      s6_supply_y_started ? 1U : 0U,
      (unsigned int)mission_ctx.s6_recovery_count,
      mission_ctx.mission_snapshot.vision_target_valid ? 1U : 0U,
      (unsigned long)ROBOT_S6_PRE_REPOSITION_TRACK_MS);
}

static bool MissionTask_StartS6FoundBafflePulse(uint32_t now)
{
  if (mission_ctx.s6_found_baffle_pwm_active) return true;
  if (Actuator_SetBaffleAngle(ROBOT_BAFFLE_DOWN_DEG) != HAL_OK)
  {
    mission_ctx.mission_snapshot.fault_flags |= MISSION_FAULT_ACTUATOR;
    MissionTask_EnterState(MISSION_STATE_FAULT, now);
    return false;
  }
  s6_found_baffle_pwm_tick = now;
  mission_ctx.s6_found_baffle_pwm_active = true;
  return true;
}

void MissionTask_ServiceS6FoundBafflePulse(uint32_t now)
{
  if (mission_ctx.s6_found_baffle_pwm_active &&
      (uint32_t)(now - s6_found_baffle_pwm_tick) >= ROBOT_S6_FOUND_BAFFLE_PWM_MS &&
      !MissionTask_StopS6DeliveryBafflePwm())
  {
    MissionTask_EnterState(MISSION_STATE_FAULT, now);
  }
}

static void MissionTask_EnterS6TrackingFrom16(uint32_t now)
{
  (void)DebugUart_Log("[06] RX16, SAFE FOUND\r\n");
  MissionTask_StopWheels();
  if (!MissionTask_StartS6FoundBafflePulse(now)) return;
  /* Discard coordinates from the preceding search reference. MaixCam must
     publish a fresh safe-zone coordinate frame after event 16. */
  MaixCam_ClearObject();
  mission_ctx.mission_snapshot.vision_target_valid = false;
  mission_ctx.mission_snapshot.vision_x_error_px = 0;
  mission_ctx.mission_snapshot.vision_y_error_px = 0;
  MissionTask_ResetVisionPid();
  MissionTask_ResetS6ControllerState();
  s6_approach_start_tick = now;
  s6_reposition_count = 0U;
  s6_supply_y_started = false;
  mission_ctx.s6_next_debug_tick = now;
  MissionTask_StartS6VisionApproach(now);
  (void)DebugUart_Logf(
      "[06] SIDE wait=%lu v=%d/%d xlim=%d n=%u span=%lu\r\n",
      (unsigned long)ROBOT_S6_PRE_REPOSITION_TRACK_MS,
      ROBOT_S6_APPROACH_UNALIGNED_RPM, ROBOT_S6_APPROACH_RPM,
      ROBOT_S6_SIDE_X_TOLERANCE_PX,
      (unsigned int)ROBOT_S6_X_ALIGN_MIN_FRAMES,
      (unsigned long)ROBOT_S6_X_ALIGN_STABLE_MS);
}

static bool MissionTask_DecideS6Side(uint32_t now, const char *source)
{
  if (!MissionTask_S6PreDecisionTrackingReady() ||
      !mission_ctx.mission_snapshot.vision_target_valid ||
      mission_ctx.mission_snapshot.target_age_ms > MISSION_VISION_TARGET_TIMEOUT_MS) return false;
  mission_ctx.s6_pre_reposition_decision_pending = false;
  (void)DebugUart_Logf(
      "[DEC] src=%s cmd=%02X id=%u zone=%u team=%u ms=%lu x=%d\r\n",
      source, (unsigned int)mission_ctx.s3_target_select_command,
      (unsigned int)mission_ctx.mission_snapshot.target_id,
      (unsigned int)mission_ctx.mission_snapshot.start_zone,
      (unsigned int)mission_ctx.mission_snapshot.team,
      (unsigned long)mission_ctx.s6_pre_reposition_track_elapsed_ms,
      (int)mission_ctx.mission_snapshot.vision_x_error_px);
  (void)DebugUart_Logf(
      "[DEC] field=%ld safe=%ld p26=%u\r\n",
      (long)MissionTask_WrapYaw(mission_ctx.mission_snapshot.yaw_cdeg - mission_ctx.mission_snapshot.yaw_start_cdeg),
      (long)MissionTask_WrapYaw(mission_ctx.mission_snapshot.safe_zone_yaw_target_cdeg -
                               mission_ctx.mission_snapshot.yaw_start_cdeg),
      mission_ctx.s6_align26_pending ? 1U : 0U);
  return MissionTask_TryS6AngleManeuver(now, mission_ctx.mission_snapshot.yaw_cdeg);
}

static void MissionTask_StartS6FinalFrom26(uint32_t now)
{
  int32_t corner_side_yaw_cdeg;
  MissionTask_StopWheels();
  mission_ctx.s6_align26_pending = false;
  mission_ctx.s6_pre_reposition_decision_pending = false;
  mission_ctx.s6_obstacle_side_required = false;
  mission_ctx.s6_obstacle_side_decision_pending = false;
  mission_ctx.s6_obstacle_side_decision_done = false;
  /*26 checks CURRENT field yaw before committing to delivery. It may start
     corner evacuation without waiting for XY or the normal1s tracking gate,
     but must never re-enable ordinary side motion or repeat this load's
     completed corner. The consumed26 is not replayed after the05 review. */
  if (!mission_ctx.s6_corner_completed &&
      MissionTask_GetS6CornerHeading(mission_ctx.mission_snapshot.yaw_cdeg, &corner_side_yaw_cdeg))
  {
    (void)DebugUart_Log("[26] CORNER FIRST\r\n");
    MissionTask_StartS6CornerEvacuation(
        now, mission_ctx.mission_snapshot.yaw_cdeg, corner_side_yaw_cdeg);
    return;
  }
  mission_ctx.s6_push_yaw_target_cdeg = MissionTask_WrapYaw(mission_ctx.mission_snapshot.safe_zone_yaw_target_cdeg);
  MaixCam_ClearObject();
  mission_ctx.mission_snapshot.vision_target_valid = false;
  MissionTask_ResetVisionPid();
  MissionTask_ResetS6ControllerState();
  mission_ctx.s6_phase_stable_since = 0U;
  mission_ctx.s6_next_debug_tick = now;
  MissionTask_EnterState(MISSION_STATE_S6_FINAL_ALIGN, now);
  (void)DebugUart_Logf("[26] RX26 goal=%ld >BORDER/RELEASE/LIFT/BACK/180\r\n",
      (long)MissionTask_WrapYaw(mission_ctx.s6_push_yaw_target_cdeg - mission_ctx.mission_snapshot.yaw_start_cdeg));
}

void MissionTask_HandleS6AlignRequest(uint32_t now)
{
  MissionState state = mission_ctx.mission_snapshot.state;
  bool side_move = (state == MISSION_STATE_S6_REPOSITION_TURN_SIDE) ||
      (state == MISSION_STATE_S6_REPOSITION_FORWARD) ||
      (state == MISSION_STATE_S6_REPOSITION_TURN_SAFE) ||
      (state == MISSION_STATE_S6_REPOSITION_FACE_SAFE);
  bool recovery = state >= MISSION_STATE_S6_RECOVERY_START &&
      state <= MISSION_STATE_S6_RECOVERY_RETURN_SAFE;
  bool obstacle = state >= MISSION_STATE_S6_OBSTACLE_TURN_OUT &&
      state <= MISSION_STATE_S6_OBSTACLE_FACE_SAFE;
  bool corner = state >= MISSION_STATE_S6_CORNER_TURN_AWAY &&
      state <= MISSION_STATE_S6_CORNER_TURN_SAFE;
  uint16_t ignored26 = MaixCam_TakeIgnoredCornerAlignCount();
  if (ignored26 != 0U)
    (void)DebugUart_Logf("[CORNER] IGNORE26 n=%u NEED NEW26\r\n",
        (unsigned int)ignored26);

  if (MaixCam_TakeSafeZoneAlignRequest())
  {
    if (state == MISSION_STATE_S6_RECHECK_WIDE_SETTLE)
    {
      mission_ctx.s6_align26_pending = true;
      (void)DebugUart_Log("[CHECK05] HOLD26 WAIT WIDE\r\n");
      return;
    }
    if (corner)
    {
      /* Discard a latch already pending when evacuation started as well. */
      mission_ctx.s6_align26_pending = false;
      (void)DebugUart_Log("[CORNER] DROP OLD26 NEED NEW26\r\n");
      return;
    }
    if (side_move)
    {
      /* Finish the whole side route, including the turn back to safety.
         Repeated26 coalesces without stopping wheels or restarting a leg. */
      if (!mission_ctx.s6_align26_pending)
        (void)DebugUart_Logf("[SIDE] HOLD26 s=%u\r\n", (unsigned int)state);
      mission_ctx.s6_align26_pending = true;
      return;
    }
    if ((state == MISSION_STATE_S6_SEARCH_SAFE_ZONE) ||
        (state == MISSION_STATE_S6_TRACK_SAFE_ZONE) || obstacle ||
        (state == MISSION_STATE_S6_TRACK_TIMEOUT_REVERSE) || recovery)
    {
      if (!mission_ctx.s6_align26_pending)
        (void)DebugUart_Logf("[26] HOLD s=%u dec=%u\r\n",
            (unsigned int)state, mission_ctx.s6_pre_reposition_decision_pending ? 1U : 0U);
      mission_ctx.s6_align26_pending = true;
      /*26 overrides the approach/recovery gate, but the common entry
         first checks whether CURRENT yaw requires corner evacuation. */
      MissionTask_StartS6FinalFrom26(now);
    }
    else
    {
      (void)DebugUart_Logf("[26] IGNORE s=%u\r\n",
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
    mission_ctx.s6_pre_reposition_decision_pending = true;
  }

  /* Persistent loss must not wrap the counter back to the first sweep. */
  if (mission_ctx.s6_recovery_count < UINT8_MAX)
  {
    ++mission_ctx.s6_recovery_count;
  }
  use_long_forward =
      mission_ctx.s6_recovery_count >= ROBOT_S6_RECOVERY_LONG_START_COUNT;
  mission_ctx.s6_recovery_first_forward_active = !use_long_forward;
  /* First recovery is centered on the field's known safe-zone heading,
     even if the vehicle lost tracking while pointed elsewhere. */
  mission_ctx.s6_recovery_base_yaw_cdeg = MissionTask_WrapYaw(
      use_long_forward ? mission_ctx.mission_snapshot.yaw_cdeg
                       : mission_ctx.mission_snapshot.safe_zone_yaw_target_cdeg);
  mission_ctx.s6_recovery_left_yaw_cdeg = MissionTask_WrapYaw(
      mission_ctx.s6_recovery_base_yaw_cdeg +
      (int32_t)ROBOT_YAW_LEFT_SIGN *
          ROBOT_S6_RECOVERY_LEFT_ANGLE_CDEG);
  mission_ctx.s6_recovery_right_yaw_cdeg = MissionTask_WrapYaw(
      mission_ctx.s6_recovery_left_yaw_cdeg -
      (int32_t)ROBOT_YAW_LEFT_SIGN *
          ROBOT_S6_RECOVERY_RIGHT_ANGLE_CDEG);
  s6_recovery_from_tracking = from_tracking;
  mission_ctx.s6_recovery_found16_pending = false;
  s6_missing_since = 0U;
  mission_ctx.s6_phase_stable_since = 0U;
  MissionTask_ResetVisionPid();
  MissionTask_ResetS6ControllerState();
  mission_ctx.s6_next_debug_tick = now;
  MissionTask_EnterState(MISSION_STATE_S6_RECOVERY_START, now);
  (void)DebugUart_Logf(
      "[6E] START n=%u src=%s r=%ld L=%ld R=%ld mode=%s\r\n",
      (unsigned int)mission_ctx.s6_recovery_count,
      from_tracking ? "TRACK_LOST" : "WAIT16",
      (long)mission_ctx.s6_recovery_base_yaw_cdeg,
      (long)mission_ctx.s6_recovery_left_yaw_cdeg,
      (long)mission_ctx.s6_recovery_right_yaw_cdeg,
      use_long_forward ? "F120/1500" :
                         "F170/300>SAFE");
  if (use_long_forward)
    (void)DebugUart_Logf("[6E] LOCK FWD%lu; DEFER XY/16\r\n",
        (unsigned long)ROBOT_S6_RECOVERY_LONG_FORWARD_MS);
}

static void MissionTask_ReportS6SearchSoftTimeout(uint32_t now)
{
  if (!s6_search_timeout_reported &&
      (uint32_t)(now - s6_search_start_tick) >= ROBOT_S6_SEARCH_TIMEOUT_MS)
  {
    s6_search_timeout_reported = true;
    (void)DebugUart_Logf(
        "[6E-TMO] SEARCH SOFT s=%u ms=%lu r=%ld goal=%ld WAIT16\r\n",
        (unsigned int)mission_ctx.mission_snapshot.state,
        (unsigned long)(now - s6_search_start_tick),
        (long)MissionTask_WrapYaw(mission_ctx.mission_snapshot.yaw_cdeg),
        (long)mission_ctx.mission_snapshot.safe_zone_yaw_target_cdeg);
  }
}

void MissionTask_RunS6SearchSafeZone(uint32_t now)
{
  uint8_t event_code;
  int32_t absolute_yaw_error;
  int32_t heading_tolerance_cdeg;
  int16_t turn_rpm;
  bool turn_expired;

  if (mission_ctx.mission_snapshot.state != MISSION_STATE_S6_SEARCH_SAFE_ZONE)
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
    (void)DebugUart_Logf("[06] RX%02X, WAIT16\r\n",
                         (unsigned int)event_code);
  }

  MissionTask_ReportS6SearchSoftTimeout(now);

  turn_rpm = MissionTask_CalculateS6YawTurn(
      now, mission_ctx.mission_snapshot.safe_zone_yaw_target_cdeg);
  absolute_yaw_error = MissionTask_Abs32(
      mission_ctx.mission_snapshot.safe_zone_yaw_error_cdeg);
  turn_expired = MissionTask_HeadingTurnExpired(
      now, mission_ctx.mission_snapshot.safe_zone_yaw_target_cdeg);
  /* The wait timestamp also marks that the heading has already been reached.
     Use a wider release threshold to avoid stop/start chatter near4 degrees. */
  heading_tolerance_cdeg = s6_missing_since == 0U
      ? ROBOT_S6_TURN_TOLERANCE_CDEG
      : ROBOT_S6_SEARCH_HOLD_TOLERANCE_CDEG;
  mission_ctx.mission_snapshot.safe_zone_fixed_heading_active = true;
  mission_ctx.mission_snapshot.vision_forward_rpm = 0;
  if (turn_expired || absolute_yaw_error <= heading_tolerance_cdeg)
  {
    turn_rpm = 0;
    MissionTask_StopWheels();
#if ROBOT_S6_SEARCH_WAIT16_MS == 0U
    (void)DebugUart_Log("[06] TURN OK >REC NO WAIT\r\n");
    MissionTask_StartS6Recovery(now, false);
    return;
#else
    if (s6_missing_since == 0U)
    {
      s6_missing_since = now;
      (void)DebugUart_Logf(
          "[06] SAFE OK WAIT16=%lu goal=%ld\r\n",
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
  mission_ctx.mission_snapshot.vision_turn_rpm = turn_rpm;

  if ((int32_t)(now - mission_ctx.s6_next_debug_tick) >= 0)
  {
    mission_ctx.s6_next_debug_tick = now + ROBOT_S6_DEBUG_PERIOD_MS;
    (void)DebugUart_Logf(
        "[06-S] r=%ld goal=%ld e=%ld u=%d w=%d/%d\r\n",
        (long)MissionTask_WrapYaw(mission_ctx.mission_snapshot.yaw_cdeg),
        (long)mission_ctx.mission_snapshot.safe_zone_yaw_target_cdeg,
        (long)mission_ctx.mission_snapshot.safe_zone_yaw_error_cdeg,
        (int)turn_rpm,
        (int)mission_ctx.mission_snapshot.left_target_rpm,
        (int)mission_ctx.mission_snapshot.right_target_rpm);
  }
}

int16_t MissionTask_CalculateS6YawTurn(uint32_t now,
                                              int32_t yaw_command_cdeg)
{
  bool recovery_scan = mission_ctx.mission_snapshot.state == MISSION_STATE_S6_RECOVERY_TURN_LEFT ||
      mission_ctx.mission_snapshot.state == MISSION_STATE_S6_RECOVERY_TURN_BACK;
  bool recovery_safe_turn = mission_ctx.mission_snapshot.state == MISSION_STATE_S6_RECOVERY_START ||
      mission_ctx.mission_snapshot.state == MISSION_STATE_S6_RECOVERY_RETURN_SAFE;
  float turn_min_rpm = mission_ctx.mission_snapshot.state == MISSION_STATE_S6_EXIT_TURN_180
      ? ROBOT_S6_EXIT_TURN_MIN_RPM
      : (recovery_scan ? ROBOT_S6_RECOVERY_SCAN_MIN_RPM
          : (recovery_safe_turn ? ROBOT_S6_RECOVERY_SAFE_MIN_RPM : ROBOT_S6_YAW_MIN_TURN_RPM));
  float turn_max_rpm = mission_ctx.mission_snapshot.state == MISSION_STATE_S6_SEARCH_SAFE_ZONE
      ? (float)ROBOT_S6_SEARCH_TURN_MAX_RPM
      : (mission_ctx.mission_snapshot.state == MISSION_STATE_S6_EXIT_TURN_180
          ? (float)ROBOT_S6_EXIT_TURN_MAX_RPM
          : (recovery_scan ? ROBOT_S6_RECOVERY_SCAN_MAX_RPM
              : (recovery_safe_turn ? ROBOT_S6_RECOVERY_SAFE_MAX_RPM : ROBOT_S6_YAW_MAX_TURN_RPM)));

  /* State-specific limits leave other turns and straight yaw correction alone. */
  switch (mission_ctx.mission_snapshot.state)
  {
    case MISSION_STATE_S4_FINE42_COUNTER_TURN:
    case MISSION_STATE_S4_FINE42_FINISH_TURN_LEFT:
    case MISSION_STATE_S4_E4_PUSH_LEFT:
    case MISSION_STATE_S4_E4_REVERSE_LEFT:
      turn_min_rpm = ROBOT_S4_FINE42_COUNTER_TURN_MIN_RPM;
      turn_max_rpm = ROBOT_S4_FINE42_COUNTER_TURN_MAX_RPM;
      break;
    case MISSION_STATE_S4_FINE42_TURN:
    case MISSION_STATE_S4_E4_PUSH_RIGHT:
      turn_min_rpm = ROBOT_S4_FINE42_TURN_MIN_RPM;
      turn_max_rpm = ROBOT_S4_FINE42_TURN_MAX_RPM;
      break;
    case MISSION_STATE_S4_FINAL_TURN_SAFE:
    case MISSION_STATE_S5_ALIGN_FOR_ARRANGE:
    {
      int32_t error_cdeg = MissionTask_Abs32(MissionTask_YawError(
          yaw_command_cdeg, mission_ctx.mission_snapshot.yaw_cdeg));
      float fast_scale = MissionTask_ClampFloat(
          (float)(error_cdeg - ROBOT_S5_ARRANGE_TURN_SLOW_END_CDEG) /
          (float)(ROBOT_S5_ARRANGE_TURN_SLOW_BEGIN_CDEG -
                  ROBOT_S5_ARRANGE_TURN_SLOW_END_CDEG), 0.0f, 1.0f);
      bool capture_turn = mission_ctx.mission_snapshot.state == MISSION_STATE_S4_FINAL_TURN_SAFE;
      float fast_min_rpm = capture_turn ? ROBOT_S4_FINAL_TURN_SAFE_MIN_RPM
                                       : ROBOT_S5_ARRANGE_TURN_MIN_RPM;
      float fast_max_rpm = capture_turn ? ROBOT_S4_FINAL_TURN_SAFE_MAX_RPM
                                       : ROBOT_S5_ARRANGE_TURN_MAX_RPM;
      /* Blend into the common slow band without a speed step near the goal. */
      turn_min_rpm = ROBOT_S5_ARRANGE_TURN_SLOW_MIN_RPM + fast_scale *
          (fast_min_rpm - ROBOT_S5_ARRANGE_TURN_SLOW_MIN_RPM);
      turn_max_rpm = ROBOT_S5_ARRANGE_TURN_SLOW_MAX_RPM + fast_scale *
          (fast_max_rpm - ROBOT_S5_ARRANGE_TURN_SLOW_MAX_RPM);
      break;
    }
    case MISSION_STATE_S6_REPOSITION_TURN_SIDE:
    case MISSION_STATE_S6_REPOSITION_TURN_SAFE:
      turn_max_rpm = ROBOT_S6_REPOSITION_TURN_MAX_RPM;
      break;
    case MISSION_STATE_S6_CORNER_TURN_AWAY:
    case MISSION_STATE_S6_CORNER_TURN_SIDE:
    case MISSION_STATE_S6_CORNER_TURN_SAFE:
      turn_max_rpm = ROBOT_S6_CORNER_TURN_MAX_RPM;
      break;
    case MISSION_STATE_S6_OBSTACLE_TURN_OUT:
    case MISSION_STATE_S6_OBSTACLE_TURN_BACK:
    case MISSION_STATE_S6_OBSTACLE_FACE_SAFE:
      turn_max_rpm = ROBOT_S6_OBSTACLE_TURN_MAX_RPM;
      break;
    case MISSION_STATE_S6_BAR_TURN_SIDE:
    case MISSION_STATE_S6_BAR_TURN_SAFE:
    case MISSION_STATE_S6_BAR_REVERSE:
    case MISSION_STATE_S6_BAR_FORWARD:
      turn_min_rpm = ROBOT_S6_BAR_TURN_MIN_RPM;
      turn_max_rpm = ROBOT_S6_BAR_TURN_MAX_RPM;
      break;
    case MISSION_STATE_S4_E4_TURN_AWAY:
    case MISSION_STATE_S4_E4_TURN_LEFT:
    case MISSION_STATE_S4_E4_TURN_RIGHT:
      turn_max_rpm = mission_ctx.s7_target_switch_enabled &&
          ((mission_ctx.s3_target_select_command == MAIXCAM_COMMAND_SELECT_BLACK) ||
           (mission_ctx.s3_target_select_command == MAIXCAM_COMMAND_SELECT_BLACK_GREEN))
          ? ROBOT_S4_E4_MATERIAL_TURN_MAX_RPM : ROBOT_S4_E4_TURN_MAX_RPM;
      break;
    case MISSION_STATE_S4_E4_RED_SPIN_360:
      turn_max_rpm = ROBOT_S7_E3_SEARCH_TURN_FAST_RPM;
      break;
    case MISSION_STATE_S4_POST22_TURN_LEFT:
    case MISSION_STATE_S4_POST22_TURN_RIGHT:
    case MISSION_STATE_S4_ARRANGE_RECOVERY_TURN_LEFT:
    case MISSION_STATE_S4_ARRANGE_RECOVERY_TURN_BACK:
      turn_max_rpm = ROBOT_S4_E2_TURN_MAX_RPM;
      break;
    case MISSION_STATE_SUPPLEMENT_TURN_LEFT:
    case MISSION_STATE_SUPPLEMENT_TURN_RIGHT:
      turn_max_rpm = mission_ctx.supplement_recovery_event == MAIXCAM_EVENT_SEARCH_TARGET_LOST &&
          mission_ctx.supplement_reference_command == MAIXCAM_COMMAND_SEARCH_TARGET
          ? ROBOT_SUPPLEMENT_E3_TURN_MAX_RPM : ROBOT_SUPPLEMENT_TURN_MAX_RPM;
      break;
    default:
      break;
  }

  return MissionTask_CalculateS6YawPid(now, yaw_command_cdeg,
                                      turn_min_rpm, turn_max_rpm);
}

int16_t MissionTask_CalculateS6YawPid(uint32_t now,
                                             int32_t yaw_command_cdeg,
                                             float turn_min_rpm,
                                             float turn_max_rpm)
{
  HeadingControl_Config control = {
    .kp = ROBOT_S6_YAW_KP,
    .ki = ROBOT_S6_YAW_KI,
    .kd = ROBOT_S6_YAW_KD,
    .integral_limit = ROBOT_S6_YAW_INTEGRAL_LIMIT,
    .output_scale = 1.0f,
    .min_turn_rpm = turn_min_rpm,
    .max_turn_rpm = turn_max_rpm,
    .initial_period_ms = MISSION_TASK_PERIOD_TICKS,
    .yaw_left_sign = ROBOT_YAW_LEFT_SIGN,
  };

  mission_ctx.mission_snapshot.safe_zone_yaw_command_cdeg = yaw_command_cdeg;
  mission_ctx.mission_snapshot.safe_zone_yaw_error_cdeg = MissionTask_YawError(
      yaw_command_cdeg, mission_ctx.mission_snapshot.yaw_cdeg);

  if (mission_ctx.mission_snapshot.state == MISSION_STATE_S6_REPOSITION_TURN_SIDE ||
      mission_ctx.mission_snapshot.state == MISSION_STATE_S6_REPOSITION_TURN_SAFE)
  {
    /* Increase large-angle side-turn response without changing corner/46
       or moving yaw loops. Retain the old gain within10deg and blend it
       continuously to the faster gain at20deg, avoiding a speed step. */
    float fast_scale = MissionTask_ClampFloat(
        (float)(MissionTask_Abs32(mission_ctx.mission_snapshot.safe_zone_yaw_error_cdeg) -
                ROBOT_S6_REPOSITION_SLOW_END_CDEG) /
        (float)(ROBOT_S6_REPOSITION_SLOW_BEGIN_CDEG -
                ROBOT_S6_REPOSITION_SLOW_END_CDEG), 0.0f, 1.0f);
    control.kp = ROBOT_S6_YAW_KP + fast_scale *
        (ROBOT_S6_REPOSITION_YAW_KP - ROBOT_S6_YAW_KP);
    control.ki = ROBOT_S6_REPOSITION_YAW_KI;
    control.kd = ROBOT_S6_REPOSITION_YAW_KD;
  }
  else if (mission_ctx.mission_snapshot.state == MISSION_STATE_S4_FINE42_TURN ||
      mission_ctx.mission_snapshot.state == MISSION_STATE_S4_E4_PUSH_RIGHT)
  {
    /*02/42 initial7deg and E5 initial10deg turns use the smaller floor and their
       own proportional response, avoiding a fixed30rpm near the goal. */
    control.kp = ROBOT_S4_FINE42_TURN_KP;
    control.ki = ROBOT_S4_FINE42_TURN_KI;
    control.kd = ROBOT_S4_FINE42_TURN_KD;
  }
  if (mission_ctx.mission_snapshot.state >= MISSION_STATE_S6_OBSTACLE_TURN_OUT &&
      mission_ctx.mission_snapshot.state <= MISSION_STATE_S6_OBSTACLE_FACE_SAFE)
    control.output_scale = ROBOT_S6_OBSTACLE_TURN_GAIN_SCALE;

  return HeadingControl_Update(&s6_yaw_pid, &control, now,
                               mission_ctx.mission_snapshot.safe_zone_yaw_error_cdeg);
}

void MissionTask_DebugS6(uint32_t now)
{
  int16_t left_actual_rpm;
  int16_t right_actual_rpm;

  if ((int32_t)(now - mission_ctx.s6_next_debug_tick) < 0)
  {
    return;
  }
  mission_ctx.s6_next_debug_tick = now + ROBOT_S6_DEBUG_PERIOD_MS;

  left_actual_rpm = (int16_t)(
      ((float)moto_chassis[MISSION_LEFT_MOTOR_INDEX].speed_rpm / 36.0f) *
      ROBOT_LEFT_WHEEL_FORWARD_SIGN);
  right_actual_rpm = (int16_t)(
      ((float)moto_chassis[MISSION_RIGHT_MOTOR_INDEX].speed_rpm / 36.0f) *
      ROBOT_RIGHT_WHEEL_FORWARD_SIGN);

  (void)DebugUart_Logf(
      "[06] s=%u ok=%u rec=%u x=%d y=%d r=%ld g=%ld e=%ld u=%d v=%d w=%d/%d act=%d/%d\r\n",
      (unsigned int)mission_ctx.mission_snapshot.state,
      mission_ctx.mission_snapshot.vision_target_valid ? 1U : 0U,
      (unsigned int)mission_ctx.s6_recovery_count,
      (int)mission_ctx.mission_snapshot.vision_x_error_px,
      (int)mission_ctx.mission_snapshot.vision_y_error_px,
      (long)mission_ctx.mission_snapshot.yaw_cdeg,
      (long)mission_ctx.mission_snapshot.safe_zone_yaw_command_cdeg,
      (long)mission_ctx.mission_snapshot.safe_zone_yaw_error_cdeg,
      (int)mission_ctx.mission_snapshot.vision_turn_rpm,
      (int)mission_ctx.mission_snapshot.vision_forward_rpm,
      (int)mission_ctx.mission_snapshot.left_target_rpm,
      (int)mission_ctx.mission_snapshot.right_target_rpm,
      (int)left_actual_rpm,
      (int)right_actual_rpm);
}

bool MissionTask_RunS6TurnToHeading(uint32_t now,
                                           int32_t target_yaw_cdeg)
{
  int32_t absolute_yaw_error;
  int32_t stop_tolerance_cdeg = mission_ctx.mission_snapshot.state == MISSION_STATE_S6_EXIT_TURN_180
      ? ROBOT_S6_EXIT_TURN_TOLERANCE_CDEG : ROBOT_S6_TURN_TOLERANCE_CDEG;
  int16_t turn_rpm;
  bool exit_turn = mission_ctx.mission_snapshot.state == MISSION_STATE_S6_EXIT_TURN_180;
  bool recovery_turn = mission_ctx.mission_snapshot.state >= MISSION_STATE_S6_RECOVERY_START &&
      mission_ctx.mission_snapshot.state <= MISSION_STATE_S6_RECOVERY_RETURN_SAFE;
  bool obstacle_turn = mission_ctx.mission_snapshot.state >= MISSION_STATE_S6_OBSTACLE_TURN_OUT &&
      mission_ctx.mission_snapshot.state <= MISSION_STATE_S6_OBSTACLE_FACE_SAFE;
  bool bar_turn = mission_ctx.mission_snapshot.state >= MISSION_STATE_S6_BAR_REVERSE &&
      mission_ctx.mission_snapshot.state <= MISSION_STATE_S6_BAR_WAIT_ACK;
  bool fine_turn = mission_ctx.mission_snapshot.state == MISSION_STATE_S4_FINE42_TURN ||
      mission_ctx.mission_snapshot.state == MISSION_STATE_S4_FINE42_COUNTER_TURN ||
      mission_ctx.mission_snapshot.state == MISSION_STATE_S4_FINE42_FINISH_TURN_LEFT ||
      mission_ctx.mission_snapshot.state == MISSION_STATE_S4_E4_PUSH_RIGHT ||
      mission_ctx.mission_snapshot.state == MISSION_STATE_S4_E4_PUSH_LEFT ||
      mission_ctx.mission_snapshot.state == MISSION_STATE_S4_E4_REVERSE_LEFT;
  uint32_t stable_ms = (bar_turn || fine_turn) ? 0U : (obstacle_turn ? ROBOT_S6_OBSTACLE_TURN_STABLE_MS
      : (recovery_turn ? ROBOT_S6_RECOVERY_TURN_STABLE_MS : ROBOT_S6_TURN_STABLE_MS));
  if (MissionTask_HeadingTurnExpired(now, target_yaw_cdeg)) return true;

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
      mission_ctx.mission_snapshot.safe_zone_yaw_error_cdeg);

  const int32_t release_tolerance_cdeg =
      (mission_ctx.mission_snapshot.state == MISSION_STATE_S5_ALIGN_FOR_ARRANGE ||
       mission_ctx.mission_snapshot.state == MISSION_STATE_S4_FINAL_TURN_SAFE)
          ? ROBOT_S5_ARRANGE_TURN_RELEASE_CDEG : stop_tolerance_cdeg;
  const ChassisActionStep step = ChassisMotion_ResolveTurnStep(&s6_yaw_pid,
      &mission_ctx.s6_phase_stable_since, now, absolute_yaw_error, stop_tolerance_cdeg,
      release_tolerance_cdeg, stable_ms, turn_rpm);
  if (step.brake)
  {
    MissionTask_StopWheels();
    mission_ctx.mission_snapshot.vision_turn_rpm = 0;
    mission_ctx.mission_snapshot.vision_forward_rpm = 0;
    if (step.status == CHASSIS_ACTION_DONE) return true;
  }
  else
  {
    mission_ctx.mission_snapshot.vision_turn_rpm = turn_rpm;
    mission_ctx.mission_snapshot.vision_forward_rpm = 0;
    MissionTask_SetWheelTargets(step.wheels.left_rpm, step.wheels.right_rpm);
  }

  MissionTask_DebugS6(now);
  return false;
}

void MissionTask_CommandS6Straight(uint32_t now,
                                           int32_t target_yaw_cdeg,
                                           int16_t forward_rpm,
                                           int16_t wheel_max_rpm)
{
  int32_t absolute_yaw_error = MissionTask_Abs32(MissionTask_YawError(
      target_yaw_cdeg, mission_ctx.mission_snapshot.yaw_cdeg));
  int16_t turn_rpm;
  bool bar_drive = mission_ctx.mission_snapshot.state == MISSION_STATE_S6_BAR_REVERSE ||
      mission_ctx.mission_snapshot.state == MISSION_STATE_S6_BAR_FORWARD;
  const bool delivery_push = forward_rpm > 0 &&
      (mission_ctx.mission_snapshot.state == MISSION_STATE_S6_BORDER_LOCATE ||
       mission_ctx.mission_snapshot.state == MISSION_STATE_S6_FINAL_PUSH);
  if (bar_drive &&
      absolute_yaw_error <= ROBOT_S6_YAW_FORWARD_ENABLE_CDEG)
  {
    /* Small moving corrections stay proportional, without a turn-speed floor.
       Large heading errors still stop forward travel and use the normal turn. */
    turn_rpm = MissionTask_CalculateS6YawPid(
        now, target_yaw_cdeg, 0.0f,
        ROBOT_S6_BAR_YAW_CORRECTION_MAX_RPM);
  }
  else
  {
    turn_rpm = MissionTask_CalculateS6YawTurn(now, target_yaw_cdeg);
  }

  HeadingControl_LimitStraight(&s6_yaw_pid, absolute_yaw_error, ROBOT_S6_YAW_TOLERANCE_CDEG,
      delivery_push ? ROBOT_S6_PUSH_FORWARD_STOP_CDEG : ROBOT_S6_YAW_FORWARD_ENABLE_CDEG,
      delivery_push, &forward_rpm, &turn_rpm);

  mission_ctx.mission_snapshot.vision_turn_rpm = turn_rpm;
  mission_ctx.mission_snapshot.vision_forward_rpm = forward_rpm;
  MissionTask_SetTrackingTargets(forward_rpm, turn_rpm, wheel_max_rpm);
  MissionTask_DebugS6(now);
}

void MissionTask_RunS6SideReposition(uint32_t now)
{
  const MissionState state = mission_ctx.mission_snapshot.state;
  if (state != MISSION_STATE_S6_REPOSITION_TURN_SIDE &&
      state != MISSION_STATE_S6_REPOSITION_FORWARD &&
      state != MISSION_STATE_S6_REPOSITION_TURN_SAFE &&
      state != MISSION_STATE_S6_REPOSITION_FACE_SAFE) return;
  if ((uint32_t)(now - s6_approach_start_tick) >= ROBOT_S6_APPROACH_TIMEOUT_MS)
  {
    mission_ctx.mission_snapshot.fault_flags |= MISSION_FAULT_S6_TIMEOUT;
    MissionTask_EnterState(MISSION_STATE_FAULT, now);
    return;
  }
  if (state == MISSION_STATE_S6_REPOSITION_FACE_SAFE)
  {
    if (!mission_ctx.s6_align26_pending) MissionTask_StartS6VisionApproach(now);
    else
    {
      MissionTask_StopWheels();
      MissionTask_ResetS6ControllerState();
      mission_ctx.s6_phase_stable_since = 0U;
      MissionTask_EnterState(MISSION_STATE_S6_REPOSITION_TURN_SAFE, now);
    }
    return;
  }

  uint32_t forward_ms = s6_reposition_forward_ms;
  /* A held26 caps the original leg's total time, never restarts its clock. */
  if (mission_ctx.s6_align26_pending && forward_ms > ROBOT_S6_REPOSITION_ALIGN26_FORWARD_MS)
    forward_ms = ROBOT_S6_REPOSITION_ALIGN26_FORWARD_MS;
  const SideFlow_Phase phase = state == MISSION_STATE_S6_REPOSITION_TURN_SIDE ? SIDE_TURN
      : (state == MISSION_STATE_S6_REPOSITION_FORWARD ? SIDE_FORWARD : SIDE_FACE_SAFE);
  const FlowMotion io = { .stop = MissionTask_StopWheels, .wheels = MissionTask_SetWheelTargets,
    .turn = MissionTask_RunS6TurnToHeading, .straight = MissionTask_CommandS6Straight };
  const SideFlow_Phase next = RepositionFlow_StepSide(&io, phase, now,
      mission_ctx.mission_snapshot.state_entry_tick, forward_ms, mission_ctx.s6_reposition_side_yaw_cdeg,
      mission_ctx.mission_snapshot.safe_zone_yaw_target_cdeg);
  if (next == phase)
  {
    if (phase == SIDE_FORWARD)
    {
      mission_ctx.mission_snapshot.safe_zone_fixed_heading_active = false;
      mission_ctx.mission_snapshot.safe_zone_yaw_command_cdeg = mission_ctx.s6_reposition_side_yaw_cdeg;
      mission_ctx.mission_snapshot.safe_zone_yaw_error_cdeg = MissionTask_YawError(
          mission_ctx.s6_reposition_side_yaw_cdeg, mission_ctx.mission_snapshot.yaw_cdeg);
      mission_ctx.mission_snapshot.vision_forward_rpm = ROBOT_S6_REPOSITION_FORWARD_RPM;
      mission_ctx.mission_snapshot.vision_turn_rpm = 0;
      MissionTask_DebugS6(now);
    }
    return;
  }
  if (next == SIDE_FORWARD)
  {
    MissionTask_ResetS6ControllerState();
    mission_ctx.s6_next_debug_tick = now;
    MissionTask_EnterState(MISSION_STATE_S6_REPOSITION_FORWARD, now);
    (void)DebugUart_Logf("[SIDE] TURN OK fwd=%d/%lu\r\n",
        ROBOT_S6_REPOSITION_FORWARD_RPM, (unsigned long)forward_ms);
    (void)DebugUart_Logf("[SIDE] TURN OK cur=%ld goal=%ld\r\n",
        (long)MissionTask_WrapYaw(mission_ctx.mission_snapshot.yaw_cdeg - mission_ctx.mission_snapshot.yaw_start_cdeg),
        (long)MissionTask_WrapYaw(mission_ctx.s6_reposition_side_yaw_cdeg - mission_ctx.mission_snapshot.yaw_start_cdeg));
  }
  else if (next == SIDE_FACE_SAFE)
  {
    MaixCam_ClearObject();
    mission_ctx.mission_snapshot.vision_target_valid = false;
    MissionTask_ResetVisionPid();
    MissionTask_ResetS6ControllerState();
    mission_ctx.s6_phase_stable_since = 0U;
    mission_ctx.s6_next_debug_tick = now;
    MissionTask_EnterState(MISSION_STATE_S6_REPOSITION_TURN_SAFE, now);
    (void)DebugUart_Logf("[SIDE] FWD OK >SAFE r=%ld field=%ld\r\n",
        (long)mission_ctx.mission_snapshot.safe_zone_yaw_target_cdeg,
        (long)MissionTask_WrapYaw(mission_ctx.mission_snapshot.safe_zone_yaw_target_cdeg -
                                 mission_ctx.mission_snapshot.yaw_start_cdeg));
  }
  else if (mission_ctx.s6_align26_pending)
  {
    (void)DebugUart_Log("[SIDE] DONE >26\r\n");
    MissionTask_StartS6FinalFrom26(now);
  }
  else
  {
    MaixCam_ClearObject();
    mission_ctx.mission_snapshot.vision_target_valid = false;
    mission_ctx.s6_obstacle_side_decision_done = false;
    MissionTask_ResetS6PreDecisionTracking(now);
    MissionTask_StartS6VisionApproach(now);
    (void)DebugUart_Logf("[SIDE] SAFE OK TRACK=%lu\r\n",
        (unsigned long)ROBOT_S6_PRE_REPOSITION_TRACK_MS);
  }
}

void MissionTask_RunS6CornerEvacuation(uint32_t now)
{
  const MissionState state = mission_ctx.mission_snapshot.state;
  if (state < MISSION_STATE_S6_CORNER_TURN_AWAY ||
      state > MISSION_STATE_S6_CORNER_TURN_SAFE) return;
  if ((uint32_t)(now - s6_approach_start_tick) >= ROBOT_S6_APPROACH_TIMEOUT_MS)
  {
    mission_ctx.mission_snapshot.fault_flags |= MISSION_FAULT_S6_TIMEOUT;
    MissionTask_EnterState(MISSION_STATE_FAULT, now);
    return;
  }
  static const MissionState stage[] = {
    [CORNER_TURN_AWAY] = MISSION_STATE_S6_CORNER_TURN_AWAY,
    [CORNER_FORWARD_AWAY] = MISSION_STATE_S6_CORNER_FORWARD_AWAY,
    [CORNER_TURN_SIDE] = MISSION_STATE_S6_CORNER_TURN_SIDE,
    [CORNER_FORWARD_SIDE] = MISSION_STATE_S6_CORNER_FORWARD_SIDE,
    [CORNER_FACE_SAFE] = MISSION_STATE_S6_CORNER_TURN_SAFE,
  };
  CornerFlow_Phase phase = CORNER_TURN_AWAY;
  while (phase < CORNER_DONE && stage[phase] != state) ++phase;
  if (phase == CORNER_DONE) return;
  const FlowMotion io = { .stop = MissionTask_StopWheels, .wheels = MissionTask_SetWheelTargets,
    .turn = MissionTask_RunS6TurnToHeading, .straight = MissionTask_CommandS6Straight };
  const CornerFlow_Phase next = RepositionFlow_StepCorner(&io, phase, now,
      mission_ctx.mission_snapshot.state_entry_tick, mission_ctx.s6_corner_away_yaw_cdeg,
      mission_ctx.s6_corner_side_yaw_cdeg, mission_ctx.mission_snapshot.safe_zone_yaw_target_cdeg);
  if (next == phase) return;
  if (next == CORNER_DONE)
  {
    MissionTask_StartS6LoadRecheck(now);
    return;
  }
  MissionTask_ResetS6ControllerState();
  mission_ctx.s6_phase_stable_since = 0U;
  MissionTask_EnterState(stage[next], now);
  if (next == CORNER_FORWARD_AWAY || next == CORNER_FORWARD_SIDE)
    (void)DebugUart_Logf("[CORNER] %s TURN OK fwd=%d/%lu\r\n",
        next == CORNER_FORWARD_AWAY ? "AWAY" : "SIDE", ROBOT_S6_CORNER_FORWARD_RPM,
        (unsigned long)(next == CORNER_FORWARD_AWAY ? ROBOT_S6_CORNER_AWAY_FORWARD_MS
                                                    : ROBOT_S6_CORNER_SIDE_FORWARD_MS));
  else
    (void)DebugUart_Logf("[CORNER] %s FWD OK >%s\r\n",
        phase == CORNER_FORWARD_AWAY ? "AWAY" : "SIDE",
        phase == CORNER_FORWARD_AWAY ? "SIDE" : "SAFE");
}

static bool MissionTask_TryFinishS6Recovery(uint32_t now)
{
  uint8_t event_code;
  bool forward_active = mission_ctx.mission_snapshot.state == MISSION_STATE_S6_RECOVERY_START &&
      mission_ctx.s6_recovery_count >= ROBOT_S6_RECOVERY_LONG_START_COUNT &&
      (uint32_t)(now - mission_ctx.mission_snapshot.state_entry_tick) < ROBOT_S6_RECOVERY_LONG_FORWARD_MS;

  if (MaixCam_TakeEvent(&event_code))
  {
    if (event_code == MAIXCAM_EVENT_SAFE_ZONE_FOUND)
    {
      if (!forward_active)
      {
        mission_ctx.s6_recovery_found16_pending = false;
        MissionTask_EnterS6TrackingFrom16(now);
        return true;
      }
      if (!mission_ctx.s6_recovery_found16_pending)
      {
        /* Fence the old search reference now, retaining later new zone frames
           until the forward leg ends. Repeated16 must not clear fresh frames. */
        MaixCam_ClearObject();
        mission_ctx.mission_snapshot.vision_target_valid = false;
        (void)DebugUart_Log("[6E] HOLD16 UNTIL FWD OK\r\n");
      }
      mission_ctx.s6_recovery_found16_pending = true;
    }
    else if (s6_recovery_from_tracking &&
        (event_code == MAIXCAM_EVENT_SAFE_ZONE_OBSTACLE))
    {
      MissionTask_StartS6ObstacleAvoidance(now);
      return true;
    }
    else
      (void)DebugUart_Logf("[6E] RX%02X, WAIT16\r\n",
                           (unsigned int)event_code);
  }

  if (forward_active) return false;
  if (mission_ctx.s6_recovery_found16_pending)
  {
    mission_ctx.s6_recovery_found16_pending = false;
    (void)DebugUart_Log("[6E] FWD OK APPLY16+XY\r\n");
    if (mission_ctx.mission_snapshot.vision_target_valid)
    {
      if (!MissionTask_StartS6FoundBafflePulse(now)) return true;
      s6_reposition_count = 0U;
      s6_supply_y_started = false;
      MissionTask_StartS6VisionApproach(now);
    }
    else
      MissionTask_EnterS6TrackingFrom16(now);
    return true;
  }
  if (s6_recovery_from_tracking &&
      mission_ctx.mission_snapshot.vision_target_valid)
  {
    MissionTask_StartS6VisionApproach(now);
    (void)DebugUart_Logf(
        "[6E] FOUND x=%d y=%d >TRACK\r\n",
        (int)mission_ctx.mission_snapshot.vision_x_error_px,
        (int)mission_ctx.mission_snapshot.vision_y_error_px);
    return true;
  }
  return false;
}

void MissionTask_RunS6Recovery(uint32_t now)
{
  bool use_long_forward;

  if ((mission_ctx.mission_snapshot.state != MISSION_STATE_S6_RECOVERY_START) &&
      (mission_ctx.mission_snapshot.state != MISSION_STATE_S6_RECOVERY_TURN_LEFT) &&
      (mission_ctx.mission_snapshot.state != MISSION_STATE_S6_RECOVERY_TURN_BACK) &&
      (mission_ctx.mission_snapshot.state != MISSION_STATE_S6_RECOVERY_RETURN_SAFE))
  {
    return;
  }

  if (MissionTask_TryFinishS6Recovery(now))
  {
    return;
  }

  use_long_forward =
      mission_ctx.s6_recovery_count >= ROBOT_S6_RECOVERY_LONG_START_COUNT;

  if (!s6_recovery_from_tracking)
  {
    MissionTask_ReportS6SearchSoftTimeout(now);
  }

  ZoneRecovery_Phase phase;
  switch (mission_ctx.mission_snapshot.state)
  {
    case MISSION_STATE_S6_RECOVERY_START:
      phase = mission_ctx.s6_recovery_first_forward_active ? ZONE_RECOVERY_FIRST_DRIVE :
          (use_long_forward ? ZONE_RECOVERY_LONG_DRIVE : ZONE_RECOVERY_FACE);
      break;
    case MISSION_STATE_S6_RECOVERY_TURN_LEFT: phase = ZONE_RECOVERY_LEFT; break;
    case MISSION_STATE_S6_RECOVERY_TURN_BACK: phase = ZONE_RECOVERY_RIGHT; break;
    default: phase = ZONE_RECOVERY_HOME; break;
  }
  const FlowMotion io = { .stop = MissionTask_StopWheels, .wheels = MissionTask_SetWheelTargets,
    .turn = MissionTask_RunS6TurnToHeading };
  const ZoneRecovery_Phase next = RecoveryFlow_StepZone(&io, phase, now,
      mission_ctx.mission_snapshot.state_entry_tick, mission_ctx.s6_recovery_left_yaw_cdeg,
      mission_ctx.s6_recovery_right_yaw_cdeg, mission_ctx.mission_snapshot.safe_zone_yaw_target_cdeg);
  if (next == phase)
  {
    if (phase == ZONE_RECOVERY_FIRST_DRIVE || phase == ZONE_RECOVERY_LONG_DRIVE)
    {
      mission_ctx.mission_snapshot.safe_zone_fixed_heading_active = false;
      mission_ctx.mission_snapshot.vision_forward_rpm = phase == ZONE_RECOVERY_FIRST_DRIVE
          ? ROBOT_S6_RECOVERY_FIRST_FORWARD_RPM : ROBOT_S6_RECOVERY_LONG_FORWARD_RPM;
      mission_ctx.mission_snapshot.vision_turn_rpm = 0;
      MissionTask_DebugS6(now);
    }
    return;
  }
  MissionTask_ResetS6ControllerState();
  if (next != ZONE_RECOVERY_DONE) mission_ctx.s6_phase_stable_since = 0U;
  if (next == ZONE_RECOVERY_FACE)
  {
    mission_ctx.s6_recovery_first_forward_active = false;
    mission_ctx.mission_snapshot.state_entry_tick = now;
    (void)DebugUart_Log("[6E] F170/300 OK >SAFE\r\n");
    return;
  }
  mission_ctx.s6_next_debug_tick = now;
  switch (next)
  {
    case ZONE_RECOVERY_LEFT:
      if (phase == ZONE_RECOVERY_FACE) (void)DebugUart_Log("[6E] SAFE OK >L45\r\n");
      else (void)DebugUart_Logf("[6E] FWD=%d/%lu OK >L45\r\n",
          ROBOT_S6_RECOVERY_LONG_FORWARD_RPM, (unsigned long)ROBOT_S6_RECOVERY_LONG_FORWARD_MS);
      MissionTask_EnterState(MISSION_STATE_S6_RECOVERY_TURN_LEFT, now);
      (void)DebugUart_Logf("[6E] L45 goal=%ld max=%d min=%d wait=%lu\r\n",
          (long)mission_ctx.s6_recovery_left_yaw_cdeg, (int)ROBOT_S6_RECOVERY_SCAN_MAX_RPM,
          (int)ROBOT_S6_RECOVERY_SCAN_MIN_RPM, (unsigned long)ROBOT_S6_RECOVERY_TURN_STABLE_MS);
      break;
    case ZONE_RECOVERY_RIGHT:
      MissionTask_EnterState(MISSION_STATE_S6_RECOVERY_TURN_BACK, now);
      (void)DebugUart_Logf("[6E] L OK R90 goal=%ld max=%d min=%d\r\n",
          (long)mission_ctx.s6_recovery_right_yaw_cdeg, (int)ROBOT_S6_RECOVERY_SCAN_MAX_RPM,
          (int)ROBOT_S6_RECOVERY_SCAN_MIN_RPM);
      break;
    case ZONE_RECOVERY_HOME:
      MissionTask_EnterState(MISSION_STATE_S6_RECOVERY_RETURN_SAFE, now);
      (void)DebugUart_Logf("[6E] SWEEP OK SAFE=%ld\r\n", (long)mission_ctx.mission_snapshot.safe_zone_yaw_target_cdeg);
      break;
    case ZONE_RECOVERY_DONE:
      s6_missing_since = now;
      if (s6_recovery_from_tracking)
      {
        MissionTask_StartS6VisionApproach(now);
        (void)DebugUart_Logf("[6E] SAFE OK WAIT XY=%lu\r\n", (unsigned long)ROBOT_S6_RECOVERY_WAIT_MS);
      }
      else
      {
        MissionTask_EnterState(MISSION_STATE_S6_SEARCH_SAFE_ZONE, now);
        (void)DebugUart_Logf("[6E] SAFE OK WAIT16=%lu\r\n", (unsigned long)ROBOT_S6_SEARCH_WAIT16_MS);
      }
      break;
    default: break;
  }
}

static bool MissionTask_S6CanStartBarShift(void)
{
  MissionState state = mission_ctx.mission_snapshot.state;
  return (state >= MISSION_STATE_S6_SEARCH_SAFE_ZONE &&
          state <= MISSION_STATE_S6_TRACK_SAFE_ZONE) ||
      state == MISSION_STATE_S6_REPOSITION_TURN_SAFE ||
      (state >= MISSION_STATE_S6_OBSTACLE_TURN_OUT &&
       state <= MISSION_STATE_S6_TRACK_TIMEOUT_REVERSE);
}

void MissionTask_HandleS6BarShiftRequest(uint32_t now)
{
  MissionState state = mission_ctx.mission_snapshot.state;
  if (!MaixCam_TakeBarShiftRequest()) return;
  if (state >= MISSION_STATE_S6_BAR_REVERSE && state <= MISSION_STATE_S6_BAR_WAIT_ACK)
  {
    if (state == MISSION_STATE_S6_BAR_WAIT_ACK) mission_ctx.s6_bar_shift_ack_pending = true;
    (void)DebugUart_Log("[46] DUP KEEP\r\n");
    return;
  }
  bool final_delivery = (state >= MISSION_STATE_S6_FINAL_ALIGN &&
      state <= MISSION_STATE_S6_SAFE_ZONE_REACHED) ||
      state == MISSION_STATE_S6_BORDER_LOCATE || state == MISSION_STATE_S6_BORDER_BACKOFF;
  if (mission_ctx.s6_bar_shift_completed && (MissionTask_S6CanStartBarShift() || final_delivery))
  {
    /* A lost ACK must not drive the route twice or clear new coordinates26. */
    HAL_StatusTypeDef result = MaixCam_SendCommand(MAIXCAM_COMMAND_BAR_SHIFT_DONE_ACK);
    (void)DebugUart_Log(result == HAL_OK ? "[46] REACK\r\n" : "[46] REACK FAIL\r\n");
    return;
  }
  if (!MissionTask_S6CanStartBarShift())
  {
    (void)DebugUart_Logf("[46] IGNORE s=%u\r\n", (unsigned int)state);
    return;
  }
  uint8_t cargo = MissionTask_GetCarriedObjectId();
  bool casualty = cargo == 4U;
  if (!casualty && cargo != 3U && cargo != 5U && cargo != 6U)
  {
    MissionTask_StopWheels();
    (void)DebugUart_Logf("[46] NO CARGO id=%u\r\n", (unsigned int)cargo);
    return;
  }
  MissionTask_StopWheels();
  mission_ctx.s6_bar_reverse_yaw_cdeg = MissionTask_WrapYaw(mission_ctx.mission_snapshot.safe_zone_yaw_target_cdeg);
  mission_ctx.s6_bar_side_yaw_cdeg = MissionTask_WrapYaw(mission_ctx.s6_bar_reverse_yaw_cdeg +
      (casualty ? -1L : 1L) * (int32_t)ROBOT_YAW_LEFT_SIGN * ROBOT_S6_BAR_SIDE_ANGLE_CDEG);
  mission_ctx.s6_bar_shift_ack_pending = false;
  mission_ctx.s6_bar_initial_align_pending = true;
  mission_ctx.s6_align26_pending = false;
  mission_ctx.s6_obstacle_side_decision_pending = false;
  mission_ctx.s6_obstacle_side_decision_done = false;
  mission_ctx.s6_obstacle_side_required = false;
  mission_ctx.s6_recovery_found16_pending = false;
  s6_missing_since = 0U;
  MissionTask_ResetVisionPid();
  MissionTask_ResetS6ControllerState();
  MissionTask_ResetS6XConfirmation();
  mission_ctx.s6_phase_stable_since = 0U;
  /* Keep UART parsing,46 and05 alive; fence only maneuver motion inputs. */
  MaixCam_SetBarShiftActive(true);
  (void)MaixCam_FenceObjectReference();
  (void)MaixCam_DropEventIf(MAIXCAM_EVENT_SAFE_ZONE_FOUND);
  (void)MaixCam_DropEventIf(MAIXCAM_EVENT_SAFE_ZONE_OBSTACLE);
  mission_ctx.mission_snapshot.vision_target_valid = false;
  MissionTask_EnterState(MISSION_STATE_S6_BAR_TURN_SAFE, now);
  (void)DebugUart_Logf("[46] START id=%u SAFE %s90 B%d/%lu F%d/%lu\r\n",
      (unsigned int)cargo, casualty ? "R" : "L", ROBOT_S6_BAR_REVERSE_RPM,
      (unsigned long)ROBOT_S6_BAR_REVERSE_MS, ROBOT_S6_BAR_FORWARD_RPM,
      (unsigned long)ROBOT_S6_BAR_FORWARD_MS);
  (void)DebugUart_Logf("[46] back=%ld side=%ld safe=%ld\r\n",
      (long)MissionTask_WrapYaw(mission_ctx.s6_bar_reverse_yaw_cdeg - mission_ctx.mission_snapshot.yaw_start_cdeg),
      (long)MissionTask_WrapYaw(mission_ctx.s6_bar_side_yaw_cdeg - mission_ctx.mission_snapshot.yaw_start_cdeg),
      (long)MissionTask_WrapYaw(mission_ctx.mission_snapshot.safe_zone_yaw_target_cdeg - mission_ctx.mission_snapshot.yaw_start_cdeg));
}

void MissionTask_RunS6BarShift(uint32_t now)
{
  const MissionState state = mission_ctx.mission_snapshot.state;
  if (state < MISSION_STATE_S6_BAR_REVERSE || state > MISSION_STATE_S6_BAR_WAIT_ACK) return;
  if (MissionTask_HandleLoadCheckRequest(now)) return;
  if (state == MISSION_STATE_S6_BAR_WAIT_ACK)
  {
  MissionTask_StopWheels();
  if (!mission_ctx.s6_bar_shift_ack_pending) return;
  mission_ctx.s6_bar_shift_ack_pending = false;
  (void)MaixCam_FenceObjectReference();
  (void)MaixCam_DropEventIf(MAIXCAM_EVENT_SAFE_ZONE_FOUND);
  (void)MaixCam_DropEventIf(MAIXCAM_EVENT_SAFE_ZONE_OBSTACLE);
  mission_ctx.mission_snapshot.vision_target_valid = false;
  /* Open before TX: a camera response may arrive before HAL returns.
     No clearing after TX46, or a new coordinate/26 could be lost. */
  MaixCam_SetBarShiftActive(false);
  if (MaixCam_SendCommand(MAIXCAM_COMMAND_BAR_SHIFT_DONE_ACK) != HAL_OK)
  {
    MaixCam_SetBarShiftActive(true);
    (void)DebugUart_Log("[46] ACK FAIL WAIT46\r\n");
    return;
  }
  (void)DebugUart_Log("[46] DONE TX46 WAIT XY/26\r\n");
  /* UART TX can cross a tick. Use one post-ACK epoch for the handoff;
     the mission loop refreshes its own clock before running this state. */
  now = HAL_GetTick();
  if (MissionTask_HandleLoadCheckRequest(now)) return;
  MissionTask_StartS6VisionApproach(now);
  MissionTask_UpdateVisionInput(now);
  MissionTask_HandleS6AlignRequest(now);

    return;
  }
  static const MissionState stage[] = {
    [BAR_SHIFT_ALIGN] = MISSION_STATE_S6_BAR_TURN_SAFE,
    [BAR_SHIFT_BACK] = MISSION_STATE_S6_BAR_REVERSE,
    [BAR_SHIFT_TURN_SIDE] = MISSION_STATE_S6_BAR_TURN_SIDE,
    [BAR_SHIFT_FORWARD] = MISSION_STATE_S6_BAR_FORWARD,
    [BAR_SHIFT_FACE_SAFE] = MISSION_STATE_S6_BAR_TURN_SAFE,
  };
  BarShiftFlow_Phase phase;
  switch (state)
  {
    case MISSION_STATE_S6_BAR_REVERSE: phase = BAR_SHIFT_BACK; break;
    case MISSION_STATE_S6_BAR_TURN_SIDE: phase = BAR_SHIFT_TURN_SIDE; break;
    case MISSION_STATE_S6_BAR_FORWARD: phase = BAR_SHIFT_FORWARD; break;
    default: phase = mission_ctx.s6_bar_initial_align_pending ? BAR_SHIFT_ALIGN : BAR_SHIFT_FACE_SAFE; break;
  }
  const FlowMotion io = { .stop = MissionTask_StopWheels, .wheels = MissionTask_SetWheelTargets,
    .turn = MissionTask_RunS6TurnToHeading, .straight = MissionTask_CommandS6Straight };
  const BarShiftFlow_Phase next = BarShiftFlow_Step(&io, phase, now,
      mission_ctx.mission_snapshot.state_entry_tick, mission_ctx.s6_bar_reverse_yaw_cdeg, mission_ctx.s6_bar_side_yaw_cdeg,
      mission_ctx.mission_snapshot.safe_zone_yaw_target_cdeg);
  if (next == phase) return;
  if (next == BAR_SHIFT_DONE)
  {
    mission_ctx.s6_bar_shift_completed = true;
    mission_ctx.s6_bar_shift_ack_pending = true;
    MissionTask_EnterState(MISSION_STATE_S6_BAR_WAIT_ACK, now);
    return;
  }
  MissionTask_ResetS6ControllerState();
  if (phase != BAR_SHIFT_TURN_SIDE) mission_ctx.s6_phase_stable_since = 0U;
  if (phase == BAR_SHIFT_ALIGN) mission_ctx.s6_bar_initial_align_pending = false;
  MissionTask_EnterState(stage[next], now);
  if (phase == BAR_SHIFT_ALIGN) (void)DebugUart_Log("[46] SAFE >BACK\r\n");
}

static void MissionTask_StartS6ObstacleAvoidance(uint32_t now)
{
  /* Latch the confirmed cargo class for the entire36 maneuver; safe-zone
     coordinate IDs and the original search command must not select its side. */
  mission_ctx.s6_obstacle_turn_right = MissionTask_S6IsCasualtyTarget();
  /* Keep the pre-36 heading, but evaluate its side sector only after a new
     configured tracking interval on return, never while starting the move. */
  s6_obstacle_decision_yaw_cdeg = MissionTask_WrapYaw(
      mission_ctx.mission_snapshot.yaw_cdeg);
  mission_ctx.s6_obstacle_side_required = false;
  mission_ctx.s6_obstacle_side_decision_pending = true;
  mission_ctx.s6_obstacle_side_decision_done = false;
  (void)DebugUart_Logf(
      "[36] saved=%ld wait=%lu p26=%u\r\n",
      (long)s6_obstacle_decision_yaw_cdeg,
      (unsigned long)ROBOT_S6_PRE_REPOSITION_TRACK_MS,
      mission_ctx.s6_align26_pending ? 1U : 0U);
  MissionTask_StopWheels();
  mission_ctx.s6_obstacle_out_yaw_cdeg = ObstacleFlow_RelativeHeading(
      mission_ctx.mission_snapshot.yaw_cdeg, mission_ctx.s6_obstacle_turn_right, true);
  MaixCam_ClearObject();
  mission_ctx.mission_snapshot.vision_target_valid = false;
  MissionTask_ResetVisionPid();
  MissionTask_ResetS6ControllerState();
  mission_ctx.s6_phase_stable_since = 0U;
  mission_ctx.s6_next_debug_tick = now;
  MissionTask_EnterState(MISSION_STATE_S6_OBSTACLE_TURN_OUT, now);
  (void)DebugUart_Logf(
      "[36] id=%u %s45 g=%ld safe=%ld u=%d FWD=%d/%lu >%s45\r\n",
      (unsigned int)MissionTask_GetCarriedObjectId(),
      mission_ctx.s6_obstacle_turn_right ? "RIGHT" : "LEFT",
      (long)mission_ctx.s6_obstacle_out_yaw_cdeg,
      (long)mission_ctx.mission_snapshot.safe_zone_yaw_target_cdeg,
      (int)ROBOT_S6_OBSTACLE_TURN_MAX_RPM,
      ROBOT_S6_OBSTACLE_DRIVE_RPM,
      (unsigned long)ROBOT_S6_OBSTACLE_FORWARD_MS,
      mission_ctx.s6_obstacle_turn_right ? "LEFT" : "RIGHT");
}

void MissionTask_RunS6ObstacleAvoidance(uint32_t now)
{
  static const MissionState stage[] = {
    [OBSTACLE_TURN_OUT] = MISSION_STATE_S6_OBSTACLE_TURN_OUT,
    [OBSTACLE_FORWARD] = MISSION_STATE_S6_OBSTACLE_FORWARD,
    [OBSTACLE_TURN_BACK] = MISSION_STATE_S6_OBSTACLE_TURN_BACK,
    [OBSTACLE_FACE_SAFE] = MISSION_STATE_S6_OBSTACLE_FACE_SAFE,
  };
  ObstacleFlow_Phase phase = OBSTACLE_TURN_OUT;
  while (phase < OBSTACLE_DONE && stage[phase] != mission_ctx.mission_snapshot.state) ++phase;
  if (phase == OBSTACLE_DONE) return;
  const FlowMotion io = { .stop = MissionTask_StopWheels, .wheels = MissionTask_SetWheelTargets,
    .turn = MissionTask_RunS6TurnToHeading, .straight = MissionTask_CommandS6Straight };
  const ObstacleFlow_Phase next = ObstacleFlow_Step(&io, phase, now,
      mission_ctx.mission_snapshot.state_entry_tick, mission_ctx.s6_obstacle_out_yaw_cdeg,
      mission_ctx.s6_obstacle_back_yaw_cdeg, mission_ctx.mission_snapshot.safe_zone_yaw_target_cdeg);
  if (next == phase) return;
  if (next == OBSTACLE_DONE)
  {
    /* Preserve the original pre-ack fence and camera handoff. */
    MaixCam_ClearEvent();
    MaixCam_ClearObject();
    mission_ctx.mission_snapshot.vision_target_valid = false;
    if (MaixCam_SendCommand(MAIXCAM_COMMAND_OBSTACLE_DONE_ACK) != HAL_OK)
    {
      mission_ctx.mission_snapshot.fault_flags |= MISSION_FAULT_VISION_TX;
      MissionTask_EnterState(MISSION_STATE_FAULT, now);
      return;
    }
    MissionTask_StartS6VisionApproach(now);
    (void)DebugUart_Log("[36] DONE TX36, WAIT XY\r\n");
    return;
  }
  if (next == OBSTACLE_TURN_BACK)
    mission_ctx.s6_obstacle_back_yaw_cdeg = ObstacleFlow_RelativeHeading(
        mission_ctx.mission_snapshot.yaw_cdeg, mission_ctx.s6_obstacle_turn_right, false);
  MissionTask_ResetS6ControllerState();
  mission_ctx.s6_phase_stable_since = 0U;
  MissionTask_EnterState(stage[next], now);
  if (next == OBSTACLE_TURN_BACK)
    (void)DebugUart_Logf("[36] FWD OK, %s45 goal=%ld\r\n",
        mission_ctx.s6_obstacle_turn_right ? "LEFT" : "RIGHT", (long)mission_ctx.s6_obstacle_back_yaw_cdeg);
  else if (next == OBSTACLE_FACE_SAFE)
    (void)DebugUart_Logf("[36] TURN BACK OK >SAFE=%ld\r\n",
        (long)mission_ctx.mission_snapshot.safe_zone_yaw_target_cdeg);
}

void MissionTask_RunS6TrackTimeoutReverse(uint32_t now)
{
  if (mission_ctx.mission_snapshot.state != MISSION_STATE_S6_TRACK_TIMEOUT_REVERSE)
  {
    return;
  }
  if ((uint32_t)(now - mission_ctx.mission_snapshot.state_entry_tick) <
      ROBOT_S6_TRACK_REVERSE_MS)
  {
    MissionTask_SetWheelTargets(-ROBOT_S6_TRACK_REVERSE_RPM,
                                -ROBOT_S6_TRACK_REVERSE_RPM);
    return;
  }
  MissionTask_StopWheels();
  MaixCam_ClearObject();
  mission_ctx.mission_snapshot.vision_target_valid = false;
  mission_ctx.s6_track_elapsed_ms = 0U;
  mission_ctx.s6_track_accounting_tick = now;
  MissionTask_StartS6VisionApproach(now);
  (void)DebugUart_Log("[06-8S] BACK OK, RESUME 16 TRACK\r\n");
}

void MissionTask_RunS6TrackSafeZone(uint32_t now)
{
  uint8_t event_code;
  bool side_x_ready;
  int32_t absolute_x_error;
  int32_t absolute_y_error;
  int16_t turn_rpm;
  int16_t forward_rpm;

  if (mission_ctx.mission_snapshot.state != MISSION_STATE_S6_TRACK_SAFE_ZONE)
  {
    return;
  }

  /* A caller sampled before this state was entered must wait one cycle,
     not interpret unsigned underflow as a15s timeout. Keep pending events. */
  if ((int32_t)(now - s6_approach_start_tick) < 0)
  {
    MissionTask_StopWheels();
    return;
  }

  if (mission_ctx.s6_align26_pending)
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

  if (mission_ctx.s6_track_elapsed_ms >= ROBOT_S6_TRACK_REVERSE_INTERVAL_MS)
  {
    MissionTask_StopWheels();
    MissionTask_ResetVisionControllerState();
    MissionTask_ResetS6ControllerState();
    MaixCam_ClearObject();
    mission_ctx.mission_snapshot.vision_target_valid = false;
    MissionTask_EnterState(MISSION_STATE_S6_TRACK_TIMEOUT_REVERSE, now);
    (void)DebugUart_Logf(
        "[06-8S] track=%lu back=%d/%lu\r\n",
        (unsigned long)mission_ctx.s6_track_elapsed_ms,
        ROBOT_S6_TRACK_REVERSE_RPM,
        (unsigned long)ROBOT_S6_TRACK_REVERSE_MS);
    return;
  }

  if ((uint32_t)(now - s6_approach_start_tick) >=
      ROBOT_S6_APPROACH_TIMEOUT_MS)
  {
    mission_ctx.mission_snapshot.fault_flags |= MISSION_FAULT_S6_TIMEOUT;
    MissionTask_EnterState(MISSION_STATE_FAULT, now);
    return;
  }

  if (!mission_ctx.mission_snapshot.vision_target_valid)
  {
    MissionTask_StopWheels();
    (void)MissionTask_UpdateS6XConfirmation(now);
    if (s6_missing_since == 0U)
    {
      s6_missing_since = now;
      (void)DebugUart_Logf(
          "[06] LOST wait=%lu >6E\r\n",
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

  side_x_ready = MissionTask_UpdateS6XConfirmation(now);
  /* Side decisions depend on tracking time and yaw only, never pixel error
     or whether Y-axis approach has already started. */
  if (mission_ctx.s6_obstacle_side_decision_pending &&
      MissionTask_S6PreDecisionTrackingReady())
  {
    /* Honor the saved pre-36 heading after the resumed tracking interval. */
    mission_ctx.s6_obstacle_side_required = false;
    mission_ctx.s6_obstacle_side_decision_pending = false;
    mission_ctx.s6_obstacle_side_decision_done = true;
    mission_ctx.s6_pre_reposition_decision_pending = false;
    (void)DebugUart_Logf(
        "[36] track=%lu decision=%ld p26=%u\r\n",
        (unsigned long)ROBOT_S6_PRE_REPOSITION_TRACK_MS,
        (long)MissionTask_WrapYaw(s6_obstacle_decision_yaw_cdeg - mission_ctx.mission_snapshot.yaw_start_cdeg),
        mission_ctx.s6_align26_pending ? 1U : 0U);
    if (MissionTask_TryS6AngleManeuver(now, s6_obstacle_decision_yaw_cdeg)) return;
  }
  if (!mission_ctx.s6_obstacle_side_decision_done &&
      mission_ctx.s6_pre_reposition_decision_pending &&
      MissionTask_DecideS6Side(now, "TRACK_INTERVAL")) return;

  if (side_x_ready && MissionTask_S6IsSupplyTarget() && !s6_supply_y_started)
  {
    s6_supply_y_started = true;
    (void)DebugUart_Logf(
        "[06] X OK lim=%d v=%d\r\n",
        ROBOT_S6_SIDE_X_TOLERANCE_PX, ROBOT_S6_APPROACH_RPM);
  }

  absolute_x_error = MissionTask_Abs32(
      mission_ctx.mission_snapshot.vision_x_error_px);
  absolute_y_error = MissionTask_Abs32(
      mission_ctx.mission_snapshot.vision_y_error_px);

  if (absolute_x_error <= ROBOT_S6_APPROACH_STOP_X_PX)
  {
    turn_rpm = 0;
    VisionMotion_ResetAxis(&mission_ctx.vision_motion.x);
  }
  else
  {
    turn_rpm = MissionTask_CalculateVisionTurn(
        now, mission_ctx.mission_snapshot.vision_x_error_px);
    if (turn_rpm > ROBOT_S6_APPROACH_TURN_MAX_RPM)
    {
      turn_rpm = ROBOT_S6_APPROACH_TURN_MAX_RPM;
    }
    else if (turn_rpm < -ROBOT_S6_APPROACH_TURN_MAX_RPM)
    {
      turn_rpm = -ROBOT_S6_APPROACH_TURN_MAX_RPM;
    }
  }

  /* Keep Y tracking while turning toward X, with the alignment-dependent limit. */
  if (absolute_y_error <= ROBOT_S6_APPROACH_STOP_Y_PX)
  {
    forward_rpm = 0;
    VisionMotion_ResetAxis(&mission_ctx.vision_motion.y);
  }
  else
  {
    forward_rpm = MissionTask_CalculateVisionForward(
        now, mission_ctx.mission_snapshot.vision_y_error_px);
    forward_rpm = VisionMotion_LimitSafeZoneForward(forward_rpm, absolute_y_error, side_x_ready);
  }

  mission_ctx.mission_snapshot.safe_zone_fixed_heading_active = false;
  mission_ctx.mission_snapshot.vision_turn_rpm = turn_rpm;
  mission_ctx.mission_snapshot.vision_forward_rpm = forward_rpm;
  MissionTask_SetVisionTrackingTargets(forward_rpm, turn_rpm,
      forward_rpm < 0 ? ROBOT_S6_APPROACH_MAX_REVERSE_RPM
                      : ROBOT_S6_APPROACH_WHEEL_MAX_RPM);
  MissionTask_DebugS6(now);
}
