#ifndef MISSION_VISION_H
#define MISSION_VISION_H

#include "mission_task.h"

/* Private Mission interface. Coordinate admission, 03 tracking, E3 recovery and AE avoidance. */
void MissionTask_ResetVisionControllerState(void);
void MissionTask_ResetVisionPid(void);
uint8_t MissionTask_ActiveRecoveryLossEvent(void);
void MissionTask_DropDuplicateRecoveryEvent(uint32_t now);
bool MissionTask_RedCoordinateIsFresh(void);
void MissionTask_UpdateVisionInput(uint32_t now);
bool MissionTask_BeginS3Vision(uint32_t now);
bool MissionTask_StartS3Align(uint32_t now, uint8_t select_command);
int16_t MissionTask_CalculateVisionTurn(uint32_t now, int16_t error_px);
bool MissionTask_HandleS3TargetLost(uint32_t now);
void MissionTask_StartVisionSearchRecovery(
    uint32_t now, MissionState resume_state, int32_t origin_yaw_cdeg,
    bool initial_green_e3);
bool MissionTask_RunRedSearchSweep(uint32_t now);
void MissionTask_RunS3SearchTurn(uint32_t now);
int16_t MissionTask_CalculateVisionForward(uint32_t now, int16_t error_px);
void MissionTask_SetTrackingTargets(int16_t forward_rpm,
                                           int16_t turn_rpm,
                                           int16_t wheel_max_rpm);
void MissionTask_SetVisionTrackingTargets(int16_t forward_rpm,
                                                int16_t turn_rpm,
                                                int16_t wheel_max_rpm);
void MissionTask_RunS3Track(uint32_t now);
void MissionTask_RunS3LowerFrame(uint32_t now);
bool MissionTask_CanAvoidStartZone(void);
void MissionTask_HandleStartZoneAvoidanceRequest(uint32_t now);
void MissionTask_RunStartZoneAvoidance(uint32_t now);
void MissionTask_DebugVisionAge(uint32_t now);

#endif
