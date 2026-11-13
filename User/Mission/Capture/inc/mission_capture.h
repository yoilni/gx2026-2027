#ifndef MISSION_CAPTURE_H
#define MISSION_CAPTURE_H

#include "mission_task.h"

/* Private Mission interface. 24 acquisition and 34/54 capture entry, finish and timeout policy. */
uint8_t MissionTask_GetSelectedObjectId(void);
uint8_t MissionTask_GetCarriedObjectId(void);
void MissionTask_SkipToS4FinalTrack(uint32_t now);
bool MissionTask_PrepareS4CapturePose(uint32_t now);
bool MissionTask_HoldS4ClawOpening(uint32_t now);
void MissionTask_FinishS4Capture(uint32_t now);
void MissionTask_CaptureS4From34(uint32_t now);
void MissionTask_StartS4FinalRecovery(uint32_t now);
bool MissionTask_TryResumeS4FinalRecovery(uint32_t now);
bool MissionTask_RunS4ForcedLeftTurn(uint32_t now,
                                             int32_t target_yaw_cdeg);
void MissionTask_RetryS3AfterS4(uint32_t now, bool from_arrange02);
void MissionTask_HandleS4FinalTrackTimeout(uint32_t now);

#endif
