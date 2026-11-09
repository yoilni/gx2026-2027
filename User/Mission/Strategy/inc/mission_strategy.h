#ifndef MISSION_STRATEGY_H
#define MISSION_STRATEGY_H

#include "mission_task.h"

/* Private Mission interface. 07 trip/target selection and priority switches between casualty and supply. */
bool MissionTask_S7MaterialOnly(void);
void MissionTask_RunS7RedE3Recovery(uint32_t now);
void MissionTask_CompleteS7RedRecovery(uint32_t now);
void MissionTask_StartS7Decision(uint32_t now);
void MissionTask_SwitchS7ToBlackGreen(uint32_t now);
void MissionTask_HandleRedPriorityRequest(uint32_t now);
void MissionTask_ResumeRedTracking(uint32_t now);
void MissionTask_RunS7Decision(uint32_t now);
void MissionTask_RunS7SilentSearch(uint32_t now);

#endif
