#ifndef MISSION_INTERNAL_H
#define MISSION_INTERNAL_H

#include "mission_task.h"
#include "delivery_flow.h"
#include "recovery_flow.h"
#include "maixcam_task.h"
#include "robot_config.h"
#include "vision_motion.h"

#define MISSION_TASK_PERIOD_TICKS       20U
#define MISSION_MOTOR_TIMEOUT_MS        100U
#define MISSION_IMU_TIMEOUT_MS          150U
#define MISSION_VISION_TARGET_TIMEOUT_MS MAIXCAM_DATA_TIMEOUT_MS
#if ROBOT_VISION_COORD_DECEL_START_MS >= MAIXCAM_DATA_TIMEOUT_MS
#error "Coordinate fade must start before the coordinate timeout"
#endif
#if ROBOT_VISION_OBJECT_NEAR_END_PX <= ROBOT_VISION_Y_TOLERANCE_PX || \
    ROBOT_VISION_OBJECT_FAR_BEGIN_PX <= ROBOT_VISION_OBJECT_NEAR_END_PX
#error "Object approach distances must increase from deadband to near to far"
#endif
#if ROBOT_VISION_03_FAR_FULL_SPEED_PX <= ROBOT_VISION_OBJECT_FAR_BEGIN_PX
#error "03 full-speed Y distance must exceed its far-band start"
#endif
#if ROBOT_VISION_OBJECT_REVERSE_FRAMES < 2U || ROBOT_VISION_OBJECT_REVERSE_FRAMES > 255U
#error "Reversal confirmation needs between2 and255 fresh frames"
#endif
#if ROBOT_S4_POST22_MAX_SWEEPS < 1U || ROBOT_S4_POST22_MAX_SWEEPS > 255U
#error "Post22 sweep budget must fit a nonzero8-bit count"
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

typedef enum
{
  S7_RED_E3_FORWARD = RECOVERY_RED_FORWARD,
  S7_RED_E3_TURN_LEFT = RECOVERY_RED_LEFT,
  S7_RED_E3_HOLD_LEFT = RECOVERY_RED_HOLD_LEFT,
  S7_RED_E3_TURN_RIGHT = RECOVERY_RED_RIGHT,
  S7_RED_E3_HOLD_RIGHT = RECOVERY_RED_HOLD_RIGHT,
  S7_RED_E3_TURN_HOME = RECOVERY_RED_HOME
} S7RedE3Phase;

/* State shared by more than one Mission source file. The mission task is the
   only writer; other tasks read snapshots through MissionTask_GetSnapshot(). */
typedef struct
{
  MissionSnapshot mission_snapshot;
  /* Target changes are allowed only after this mission has entered 07. */
  bool s7_target_switch_enabled;
  /* Mission-wide51 delivery count, incremented only after the final push. */
  uint16_t s7_material_delivery_count;
  /* Confirmed single-casualty deliveries, independent of the search command. */
  uint16_t s7_casualty_delivery_count;
  bool s7_delivery_counted;
  /* Deduplicate rejected visual11 requests until fresh material coordinates. */
  bool s7_material_only_reselect_pending;
  /* A recovery is consumed when it starts, even if coordinates interrupt it. */
  bool s7_red_recovery_used;
  /* Retain 51's actual selected object through a confirmed loss. ID4 frames
     arriving with RX11 must not overwrite the last black/green identity. */
  uint8_t s51_active_object_id;
  /* Ordinary51/03 accepted E3 requests only. Duplicate packets, silent searches
     and supplement recoveries do not consume this count; reacquisition keeps it. */
  uint16_t s51_e3_recovery_count;
  bool s51_second_e3_pending;
  /* Freeze the carried class for 06; safe-zone coordinate IDs are not cargo. */
  uint8_t s6_carried_object_id;
  bool s5_load_class_confirmed;
  bool s7_red_switch_sync_pending;
  bool s7_red_coordinate_fence_active;
  uint32_t s7_last_message_tick;
  uint32_t s7_last_coordinate_sequence;
  int32_t s7_silent_left_yaw_cdeg;
  int32_t s7_silent_right_yaw_cdeg;
  bool s3_entry_pose_ready;
  uint32_t s3_search_turn_stable_since;
  int32_t s3_search_origin_yaw_cdeg;
  int32_t s3_search_yaw_target_cdeg;
  uint8_t s3_target_select_command;
  bool s3_search_sweep_completed;
  bool s3_search_ccw_midpoint_done;
  bool vision_search_forward_active;
  int16_t vision_search_forward_rpm;
  uint32_t vision_search_forward_ms;
  bool vision_search_e3_mode;
  S7RedE3Phase s7_red_e3_phase;
  bool vision_search_green_e3_mode;
  MissionState vision_search_resume_state;
  uint32_t s3_target_lost_since;
  uint32_t s3_track_stable_since;
  bool s3_wait_event04_logged;
  uint32_t s3_next_debug_tick;
  VisionMotion_State vision_motion;
  uint32_t s4_next_debug_tick;
  /* Alternate successful42 acknowledgments;32/E4/reacquisition do not
     advance the direction. New target03/31 sessions start left. */
  bool s4_fine_next_left;
  bool s4_fine_turn_left;
  bool s4_fine_waiting_decision;
  bool s4_direct_capture_wait24;
  /*42 shares its normal fixed motion with02, not its completion handshake.
     Initial-green first02 has its own forward/full-circle override below.
     Both entries track ROI200ms;02 replies12;12 only replies22. */
  uint8_t s4_fixed_motion_ack;
  int32_t s4_fine_turn_yaw_cdeg;
  int32_t s4_fine_forward_yaw_cdeg;
  /* Independent of the shared E4 allowance:32/42/E4 cannot consume first02. */
  bool s4_initial_green02_used;
  bool s4_initial_green02_active;
  MissionState ae_resume_state;
  int32_t ae_turn_target_cdeg;
  bool ae_reply_pending;
  bool ae_ack_completed;
  bool s4_final_mode_already_active;
  bool s4_final_event34_pending;
  bool s4_capture_claw_open;
  uint32_t s4_capture_claw_open_tick;
  bool s4_e4_recovery_used;
  /* Red E4 shares the E3 sweep after facing away from the mapped safe zone. */
  bool s4_red_e4_align_away_active;
  /* Saturates at two; coordinate-only resumes must preserve this task budget. */
  uint8_t s4_material_e4_recovery_count;
  bool s4_e4_protocol_active;
  uint8_t s4_green_e4_push_cycles;
  bool s4_green_e4_spin_exhausted;
  bool s4_green_e4_half_turn_active;
  int32_t s4_e4_spin_origin_yaw_cdeg;
  int32_t s4_e4_left_yaw_cdeg;
  int32_t s4_e4_away_yaw_cdeg;
  int32_t s4_e4_right_yaw_cdeg;
  int32_t s4_post22_left_yaw_cdeg;
  int32_t s4_post22_right_yaw_cdeg;
  bool s4_post22_search_active;
  uint8_t s4_post22_failed_sweeps;
  uint16_t s4_arrange_cycle_count;
  uint32_t s4_center_target_lost_since;
  bool s4_center_search_sweep_completed;
  uint32_t s4_final_missing_since;
  uint32_t s4_final_turn_stable_since;
  int32_t s4_final_first_yaw_cdeg;
  int32_t s4_final_second_yaw_cdeg;
  uint8_t s4_final_recovery_count;
  int32_t s4_arrange_recovery_left_yaw_cdeg;
  int32_t s4_arrange_recovery_right_yaw_cdeg;
  bool s5_close_view_active;
  bool s5_arrange_backing;
  bool s5_casualty_timeout_check;
  bool s5_casualty_adjust_active;
  bool s5_casualty_adjust_done;
  bool s5_recheck_resume_supplement;
  bool s5_recheck_resume_safe_tracking;
  bool s5_empty_recovery_active;
  /* Legacy E4 four-leg state IDs now mean L10/F450/R45/L20, no reverse. */
  int32_t s4_green_scatter_yaw_cdeg;
  /* Separate from normal 03/E3/E4/EE retry counters. Reset only at a new
     mission/delivery cycle, never on a recovery or another05 handshake. */
  bool supplement_budget_started;
  bool supplement_budget_exhausted;
  bool supplement_capture_active;
  bool supplement_first_supply_ack_sent;
  uint8_t supplement_capture_count;
  uint32_t supplement_first_request_tick;
  uint32_t supplement_coordinate_floor;
  uint32_t supplement_missing_since;
  uint8_t supplement_reference_command;
  uint8_t supplement_recovery_event;
  /* Count actual accepted03 E3 requests, not duplicates or silent recovery. */
  uint8_t supplement_e3_recovery_count;
  bool supplement_e3_first_hold_active;
  int32_t supplement_e3_spin_previous_yaw_cdeg;
  /* Freeze each recovery's first180/100 or later360/30 profile at entry. */
  int32_t supplement_e3_spin_goal_cdeg;
  int32_t supplement_left_yaw_cdeg;
  int32_t supplement_right_yaw_cdeg;
  uint32_t s6_phase_stable_since;
  uint32_t s6_next_debug_tick;
  int32_t s6_recovery_base_yaw_cdeg;
  int32_t s6_recovery_left_yaw_cdeg;
  int32_t s6_recovery_right_yaw_cdeg;
  bool s6_recovery_first_forward_active;
  int32_t s6_push_yaw_target_cdeg;
  DeliveryFlow_State delivery_flow;
  bool s6_found_baffle_pwm_active;
  int32_t s6_exit_yaw_target_cdeg;
  int32_t s6_obstacle_out_yaw_cdeg;
  int32_t s6_obstacle_back_yaw_cdeg;
  bool s6_obstacle_turn_right;
  uint32_t s6_track_elapsed_ms;
  uint32_t s6_track_accounting_tick;
  int32_t s6_reposition_side_yaw_cdeg;
  int32_t s6_corner_away_yaw_cdeg;
  int32_t s6_corner_side_yaw_cdeg;
  /* Retained through recheck/6E/36; a new carried-load06 clears this flag. */
  bool s6_corner_completed;
  /*46 completes once per carried-load06 session; retries only request an ACK. */
  bool s6_bar_shift_completed;
  bool s6_bar_shift_ack_pending;
  /* TURN_SAFE is used before reversing and again after side travel. */
  bool s6_bar_initial_align_pending;
  int32_t s6_bar_reverse_yaw_cdeg;
  int32_t s6_bar_side_yaw_cdeg;
  uint32_t s6_x_align_last_tick;
  uint8_t s6_x_align_frames;
  uint32_t s6_side_x_align_first_tick;
  uint8_t s6_side_x_align_frames;
  bool s6_pre_reposition_decision_pending;
  uint32_t s6_pre_reposition_track_elapsed_ms;
  uint32_t s6_pre_reposition_track_tick;
  bool s6_pre_reposition_track_active;
  bool s6_align26_pending;
  bool s6_obstacle_side_required;
  bool s6_obstacle_side_decision_pending;
  bool s6_obstacle_side_decision_done;
  uint8_t s6_recovery_count;
  bool s6_recovery_found16_pending;
  int16_t commanded_left_rpm;
  int16_t commanded_right_rpm;
} MissionContext;

extern MissionContext mission_ctx;

#endif
