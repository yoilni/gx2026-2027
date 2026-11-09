#ifndef MISSION_STATE_H
#define MISSION_STATE_H

#include "mission_task.h"

/* Private Mission interface. Task lifecycle, departure, central state transitions and dispatch. */
int32_t MissionTask_WrapYaw(int32_t angle_cdeg);
int32_t MissionTask_YawError(int32_t target_cdeg,
                                    int32_t current_cdeg);
int32_t MissionTask_Abs32(int32_t value);
float MissionTask_ClampFloat(float value, float minimum, float maximum);
void MissionTask_SetS1Phase(MissionS1Phase phase, uint32_t now);
void MissionTask_EnterState(MissionState state, uint32_t now);
void MissionTask_StartS1(uint32_t now);

#endif
