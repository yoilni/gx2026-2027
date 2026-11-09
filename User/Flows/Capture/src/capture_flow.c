#include "capture_flow.h"
#include "robot_config.h"
#include <stddef.h>

HAL_StatusTypeDef CaptureFlow_Open(HAL_StatusTypeDef (*camera_view)(void),
    HAL_StatusTypeDef (*claw)(uint16_t), uint16_t angle)
{
  if (camera_view() != HAL_OK) return HAL_ERROR;
  return claw(angle);
}

CaptureFlow_Result CaptureFlow_Close(void (*stop)(void), uint8_t (*deferred)(void),
    HAL_StatusTypeDef (*close_claw)(void))
{
  stop();
  if (deferred() != 0U) return CAPTURE_RUNNING;
  return close_claw() == HAL_OK ? CAPTURE_READY : CAPTURE_ACTUATOR_FAULT;
}

CaptureFlow_Result CaptureFlow_EndRegrip(uint32_t now, uint32_t started_at,
    void (*stop)(void), HAL_StatusTypeDef (*close_claw)(void))
{
  if ((uint32_t)(now - started_at) < ROBOT_S4_CASUALTY_REGRIP_MS) return CAPTURE_RUNNING;
  stop();
  return close_claw() == HAL_OK ? CAPTURE_READY : CAPTURE_ACTUATOR_FAULT;
}

CaptureTrackStatus CaptureFlow_Track(uint32_t now, uint32_t started_at,
    bool target_valid, uint32_t *missing_since, bool (*track)(uint32_t, bool *))
{
  if ((uint32_t)(now - started_at) >= ROBOT_S4_FINAL_TRACK_RETRY_MS) return CAPTURE_TRACK_TIMEOUT;
  if (target_valid) *missing_since = 0U;
  else if (*missing_since == 0U) *missing_since = now;
  else if ((uint32_t)(now - *missing_since) >= ROBOT_S4_FINAL_MISSING_TIMEOUT_MS)
    return CAPTURE_TRACK_LOST;
  (void)track(now, NULL);
  return CAPTURE_TRACK_ACTIVE;
}

bool CaptureFlow_BackoffDone(uint32_t now, uint32_t started_at)
{
  return (uint32_t)(now - started_at) >= ROBOT_S4_FINAL_TIMEOUT_BACKOFF_MS;
}
