#ifndef MISSION_ARRANGE_H
#define MISSION_ARRANGE_H

#include "mission_task.h"

/* Private Mission interface. 04/14 and ROI alignment, push/arrange commands and E4/22 recovery. */
bool MissionTask_ReturnToS4TrackCenter(uint32_t now);
void MissionTask_StartS4TrackCenterFrom04(uint32_t now);
bool MissionTask_CommandS4VisionTracking(uint32_t now,
                                                 bool *xy_in_tolerance);
void MissionTask_AckS4Center(uint32_t now);
void MissionTask_StartS4GreenScatter(uint32_t now);
void MissionTask_RunS4E4Recovery(uint32_t now);
void MissionTask_RecordInitialGreenPush(uint8_t command);
void MissionTask_HandleS4FineArrangeRequest(uint32_t now);
void MissionTask_RunS4FineArrange(uint32_t now);
void MissionTask_RunS4Post22Recovery(uint32_t now);
void MissionTask_RunS4Arrange(uint32_t now);

#endif
