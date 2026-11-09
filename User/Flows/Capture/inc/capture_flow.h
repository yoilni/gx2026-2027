#ifndef CAPTURE_FLOW_H
#define CAPTURE_FLOW_H
#include "flow_motion.h"
#include "stm32f4xx_hal.h"

typedef enum { CAPTURE_RUNNING, CAPTURE_READY, CAPTURE_ACTUATOR_FAULT } CaptureFlow_Result;
typedef enum { CAPTURE_TRACK_ACTIVE, CAPTURE_TRACK_TIMEOUT, CAPTURE_TRACK_LOST } CaptureTrackStatus;

HAL_StatusTypeDef CaptureFlow_Open(HAL_StatusTypeDef (*camera_view)(void),
    HAL_StatusTypeDef (*claw)(uint16_t), uint16_t angle);
CaptureFlow_Result CaptureFlow_Close(void (*stop)(void), uint8_t (*deferred)(void),
    HAL_StatusTypeDef (*close_claw)(void));
CaptureFlow_Result CaptureFlow_EndRegrip(uint32_t now, uint32_t started_at,
    void (*stop)(void), HAL_StatusTypeDef (*close_claw)(void));
/* Process an already accepted capture frame. Handshake events are handled
   before this call by the application, so a valid close can win at timeout. */
CaptureTrackStatus CaptureFlow_Track(uint32_t now, uint32_t started_at,
    bool target_valid, uint32_t *missing_since, bool (*track)(uint32_t, bool *));
bool CaptureFlow_BackoffDone(uint32_t now, uint32_t started_at);

#endif
