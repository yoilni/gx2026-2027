#ifndef MISSION_SAFE_ZONE_H
#define MISSION_SAFE_ZONE_H

#include "mission_task.h"

/* Private Mission interface. 06/16 search and tracking, side/corner decisions and 36/46/6E handling. */
void MissionTask_ResetS6ControllerState(void);
bool MissionTask_HeadingTurnExpired(uint32_t now,
                                             int32_t target_yaw_cdeg);
void MissionTask_StartS6FromLoad(uint32_t now);
bool MissionTask_S6IsSupplyTarget(void);
bool MissionTask_S6IsCasualtyTarget(void);
void MissionTask_ResetS6XConfirmation(void);
void MissionTask_StartS6VisionApproach(uint32_t now);
void MissionTask_ServiceS6FoundBafflePulse(uint32_t now);
void MissionTask_HandleS6AlignRequest(uint32_t now);
void MissionTask_RunS6SearchSafeZone(uint32_t now);
int16_t MissionTask_CalculateS6YawTurn(uint32_t now,
                                              int32_t yaw_command_cdeg);
int16_t MissionTask_CalculateS6YawPid(uint32_t now,
                                             int32_t yaw_command_cdeg,
                                             float turn_min_rpm,
                                             float turn_max_rpm);
void MissionTask_DebugS6(uint32_t now);
bool MissionTask_RunS6TurnToHeading(uint32_t now,
                                           int32_t target_yaw_cdeg);
void MissionTask_CommandS6Straight(uint32_t now,
                                           int32_t target_yaw_cdeg,
                                           int16_t forward_rpm,
                                           int16_t wheel_max_rpm);
void MissionTask_RunS6SideReposition(uint32_t now);
void MissionTask_RunS6CornerEvacuation(uint32_t now);
void MissionTask_RunS6Recovery(uint32_t now);
void MissionTask_HandleS6BarShiftRequest(uint32_t now);
void MissionTask_RunS6BarShift(uint32_t now);
void MissionTask_RunS6ObstacleAvoidance(uint32_t now);
void MissionTask_RunS6TrackTimeoutReverse(uint32_t now);
void MissionTask_RunS6TrackSafeZone(uint32_t now);

#endif
