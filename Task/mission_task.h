#ifndef MISSION_TASK_H
#define MISSION_TASK_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
  MISSION_STATE_BOOT = 0,
  MISSION_STATE_WAIT_START,
  MISSION_STATE_S1_DEPART,
  MISSION_STATE_S2_CROSS_BUMP,
  MISSION_STATE_S3_ALIGN_GREEN,
  MISSION_STATE_S3_SEARCH_TURN_CW,
  MISSION_STATE_S3_SEARCH_TURN_CCW,
  MISSION_STATE_S3_TRACK_GREEN,
  MISSION_STATE_S3_LOWER_FRAME,
  MISSION_STATE_S4_TRACK_CENTER,
  MISSION_STATE_S4_CENTER_FOLLOW_THROUGH,
  MISSION_STATE_S4_WAIT_ARRANGE_READY,
  MISSION_STATE_S4_TRACK_RIGHT_BLOCK,
  MISSION_STATE_S4_REVERSE_RIGHT_BLOCK,
  MISSION_STATE_S4_WAIT_LEFT_TARGET,
  MISSION_STATE_S4_TRACK_LEFT_BLOCK,
  MISSION_STATE_S4_REVERSE_LEFT_BLOCK,
  MISSION_STATE_S4_ARRANGE_RECOVERY_REVERSE,
  MISSION_STATE_S4_ARRANGE_RECOVERY_TURN_LEFT,
  MISSION_STATE_S4_ARRANGE_RECOVERY_TURN_BACK,
  MISSION_STATE_S4_RAISE_FRAME,
  MISSION_STATE_S4_TRACK_FINAL_BLOCK,
  MISSION_STATE_S4_FINAL_RECOVERY_REVERSE,
  MISSION_STATE_S4_FINAL_RECOVERY_TURN_270,
  MISSION_STATE_S4_FINAL_RECOVERY_TURN_90,
  MISSION_STATE_S4_FINAL_CENTER_OBJECT,
  MISSION_STATE_S4_FINAL_LOWER_FRAME,
  MISSION_STATE_S5_WAIT_SINGLE_GREEN,
  MISSION_STATE_S5_ALIGN_FOR_ARRANGE,
  MISSION_STATE_S5_RAISE_FOR_ARRANGE,
  MISSION_STATE_S5_REVERSE_FOR_ARRANGE,
  MISSION_STATE_S5_LOWER_FOR_ARRANGE,
  MISSION_STATE_S6_SEARCH_SAFE_ZONE,
  MISSION_STATE_S6_REPOSITION_TURN_SIDE,
  MISSION_STATE_S6_REPOSITION_FORWARD,
  MISSION_STATE_S6_REPOSITION_FACE_SAFE,
  MISSION_STATE_S6_RECOVERY_FORWARD,
  MISSION_STATE_S6_RECOVERY_TURN_LEFT,
  MISSION_STATE_S6_RECOVERY_TURN_BACK,
  MISSION_STATE_S6_RECOVERY_RETURN_SAFE,
  MISSION_STATE_S6_TRACK_SAFE_ZONE,
  MISSION_STATE_S6_FINAL_ALIGN,
  MISSION_STATE_S6_RAISE_FRAME,
  MISSION_STATE_S6_PRE_PUSH_REVERSE,
  MISSION_STATE_S6_FINAL_PUSH,
  MISSION_STATE_S6_FINAL_REVERSE,
  MISSION_STATE_S6_EXIT_TURN_180,
  MISSION_STATE_S6_SAFE_ZONE_REACHED,
  MISSION_STATE_STOPPED,
  MISSION_STATE_FAULT
} MissionState;

typedef enum
{
  MISSION_S1_IDLE = 0,
  MISSION_S1_TURN,
  MISSION_S1_TURN_STABLE,
  MISSION_S1_DONE
} MissionS1Phase;

typedef enum
{
  MISSION_FAULT_NONE        = 0U,
  MISSION_FAULT_LEFT_MOTOR  = (1UL << 0),
  MISSION_FAULT_RIGHT_MOTOR = (1UL << 1),
  MISSION_FAULT_IMU         = (1UL << 2),
  MISSION_FAULT_EMERGENCY   = (1UL << 3),
  MISSION_FAULT_S1_TIMEOUT  = (1UL << 4),
  MISSION_FAULT_YAW_DIRECTION = (1UL << 5),
  MISSION_FAULT_VISION_TX   = (1UL << 6),
  MISSION_FAULT_S3_TIMEOUT  = (1UL << 7),
  MISSION_FAULT_ACTUATOR    = (1UL << 8),
  MISSION_FAULT_S6_TIMEOUT  = (1UL << 9),
  MISSION_FAULT_S4_TIMEOUT  = (1UL << 10)
} MissionFault;

typedef struct
{
  MissionState state;
  MissionS1Phase s1_phase;
  uint32_t fault_flags;
  uint32_t state_entry_tick;
  uint32_t phase_entry_tick;
  bool left_motor_online;
  bool right_motor_online;
  bool imu_valid;
  bool vision_target_valid;
  bool team_selector_high;
  bool start_pe14_high;
  bool start_pe15_high;
  bool team_locked;
  bool start_zone_locked;
  bool safe_zone_fixed_heading_active;
  uint8_t team;
  uint8_t start_zone;
  int32_t yaw_cdeg;
  int32_t yaw_start_cdeg;
  int32_t yaw_target_cdeg;
  int32_t safe_zone_yaw_target_cdeg;
  int32_t safe_zone_yaw_command_cdeg;
  int32_t safe_zone_yaw_error_cdeg;
  int32_t yaw_error_cdeg;
  int16_t left_target_rpm;
  int16_t right_target_rpm;
  uint8_t target_id;
  uint32_t target_sequence;
  int16_t vision_x_error_px;
  int16_t vision_y_error_px;
  int16_t vision_turn_rpm;
  int16_t vision_forward_rpm;
} MissionSnapshot;

/* Called once from MX_FREERTOS_Init() after osKernelInitialize(). */
bool MissionTask_Create(void);

/* Task-context control requests. Emergency stop is latched until reset. */
void MissionTask_RequestStart(void);
void MissionTask_RequestStop(void);
void MissionTask_RequestReset(void);
void MissionTask_RequestEmergencyStop(void);

/* Returns a coherent copy of the application state. Do not call from an ISR. */
bool MissionTask_GetSnapshot(MissionSnapshot *snapshot);

/* FreeRTOS entry point for the vehicle mission state machine. */
void StartMissionTask(void *argument);

#endif /* MISSION_TASK_H */
