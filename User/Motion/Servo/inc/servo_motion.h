#ifndef SERVO_MOTION_H
#define SERVO_MOTION_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

/* Single-output actions: no paired poses, waiting, or deferred commands. */
HAL_StatusTypeDef ServoMotion_SetBaffleAngle(uint16_t angle_deg);
HAL_StatusTypeDef ServoMotion_StopBafflePwm(void);
HAL_StatusTypeDef ServoMotion_SetClawAngle(uint16_t angle_deg);
HAL_StatusTypeDef ServoMotion_SetCameraAngle(uint16_t angle_deg);

/* Shared calibration for the legacy startup and paired-pose adapter, which
   must preload multiple compares before enabling the outputs. */
uint16_t ServoMotion_CameraPulseUs(uint16_t angle_deg);
uint16_t ServoMotion_FramePulseUs(uint16_t angle_deg);

#endif
