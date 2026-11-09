#ifndef MISSION_LOAD_H
#define MISSION_LOAD_H

#include "mission_task.h"

/* Private Mission interface. 05 load check, 45 adjustment, arrangement preparation and post-06 recheck. */
void MissionTask_DropUnexpectedS5E5(void);
bool MissionTask_CanPauseForLoadCheck(void);
bool MissionTask_CanLatchCasualtyRegrip(MissionState state);
void MissionTask_EnterS5(uint32_t now);
void MissionTask_RunS5WaitSingleGreen(uint32_t now);
void MissionTask_StartS6LoadRecheck(uint32_t now);
bool MissionTask_LoadCheckPausesMotion(void);
bool MissionTask_HandleLoadCheckRequest(uint32_t now);
void MissionTask_WaitFinalLoadCheck(uint32_t now, const char *reason);
void MissionTask_RunS6LoadRecheck(uint32_t now);
void MissionTask_RunS5PrepareArrange(uint32_t now);

#endif
