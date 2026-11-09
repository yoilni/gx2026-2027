#ifndef ACTUATOR_TASK_H
#define ACTUATOR_TASK_H

#include "stm32f4xx_hal.h"

/* Initializes the production servo outputs used before the RTOS starts. */
HAL_StatusTypeDef Actuator_Init(void);

/* Arrange-release pose: PA2 up (unless suspended), PA3 open for the cargo. */
HAL_StatusTypeDef Actuator_SetBaffleUpClawOpen(uint16_t claw_angle_deg);

/* Search/retain pose: PA2 down (unless suspended), PA3 closed. */
HAL_StatusTypeDef Actuator_SetFrameLowered(void);

/* Legacy mirrored-frame bench test only. */
HAL_StatusTypeDef Actuator_SetFramePartiallyRaised(uint16_t lift_angle_deg);

/* Updates only PA3/TIM2 CH4. Requires Actuator_Init to have enabled CH4 PWM;
   leaves PA2 and PA5 alone and keeps the PWM running. */
HAL_StatusTypeDef Actuator_SetClawAngle(uint16_t angle_deg);
HAL_StatusTypeDef Actuator_SetClawClosed(void);
/* Allow PA2 to lower while PA3 stays open. Normal close requests are queued
   until the guard expires; a newer open pose cancels the queued close. */
void Actuator_DeferClawClose(uint32_t now);
uint8_t Actuator_ClawCloseDeferred(void);
HAL_StatusTypeDef Actuator_ServiceDeferredClawClose(uint32_t now);
void Actuator_CancelDeferredClawClose(void);

/* PA2/TIM2 CH3 only. Suspension persists through paired pose commands;
   explicitly setting a baffle angle resumes PWM. */
HAL_StatusTypeDef Actuator_SetBaffleAngle(uint16_t angle_deg);
HAL_StatusTypeDef Actuator_StopBafflePwm(void);

/* Restores PA5/TIM2 CH1 to the 140-degree startup/04/material capture view. */
HAL_StatusTypeDef Actuator_SetCameraWideView(void);

/* PA5/TIM2 CH1 view for object tracking in 03, including supplement 03. */
HAL_StatusTypeDef Actuator_SetCameraObjectTrackView(void);

/* Uses the dedicated PA5/TIM2 CH1 view for safe-zone search and tracking. */
HAL_StatusTypeDef Actuator_SetCameraSafeZoneView(void);

/* Moves PA5/TIM2 CH1 to the common wide capture view in24. */
HAL_StatusTypeDef Actuator_SetCameraFinalTrackView(void);

/* Moves PA5/TIM2 CH1 to the calibrated load-check close view and holds it. */
HAL_StatusTypeDef Actuator_SetCameraNearView(void);

#endif /* ACTUATOR_TASK_H */
