#ifndef MISSION_SUPPLEMENT_H
#define MISSION_SUPPLEMENT_H

#include "mission_task.h"

/* Private Mission interface. Supplemental capture budget, handshakes and search recovery. */
void MissionTask_EndSupplementRun(void);
void MissionTask_ResetSupplement(void);
bool MissionTask_HandleSupplementFinish(uint32_t now);
void MissionTask_StartSupplement(uint32_t now, uint32_t request_tick,
                                        bool confirmed_by_load_check);
void MissionTask_HandleSupplementControl(uint32_t now);
void MissionTask_RunSupplement(uint32_t now);

#endif
