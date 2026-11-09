#ifndef MISSION_SAFETY_H
#define MISSION_SAFETY_H

#include "mission_task.h"

/* Private Mission interface. Start/stop/emergency requests and running motor/IMU health checks. */
void MissionTask_SetWheelTargets(int16_t left_rpm,
                                        int16_t right_rpm);
void MissionTask_StopWheels(void);
bool MissionTask_MotorIsOnline(uint8_t motor_index, uint32_t now);
void MissionTask_ProcessEvents(uint32_t events, uint32_t now);
void MissionTask_CheckRunningHealth(uint32_t now);

#endif
