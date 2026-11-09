#ifndef MISSION_DELIVERY_H
#define MISSION_DELIVERY_H

#include "mission_task.h"

/* Private Mission interface. 26 delivery phase adapter, baffle release and return turn into 07. */
void MissionTask_ResetS6BorderProgress(uint32_t now);
bool MissionTask_StopS6DeliveryBafflePwm(void);
void MissionTask_RunS6FinalAlign(uint32_t now);
void MissionTask_RunS6BorderLocate(uint32_t now);
void MissionTask_RunS6FinalReverse(uint32_t now);
void MissionTask_RunS6ExitTurn180(uint32_t now);

#endif
