#ifndef ACTUATOR_TASK_H
#define ACTUATOR_TASK_H

#include "stm32f4xx_hal.h"

/* Initializes the production servo outputs used before the RTOS starts. */
HAL_StatusTypeDef Actuator_Init(void);

/* Starts/updates PA2 CH3 and PA3 CH4, then continuously holds the collection
   frame at its calibrated raised position. Safe to call more than once. */
HAL_StatusTypeDef Actuator_SetFrameRaised(void);

/* Lowers the two mirrored collection-frame servos to their calibrated down
   endpoints and continuously holds them there. */
HAL_StatusTypeDef Actuator_SetFrameLowered(void);

/* Raises each mirrored servo from its calibrated down endpoint by the given
   command-angle amount, clamped at its normal raised endpoint. */
HAL_StatusTypeDef Actuator_SetFramePartiallyRaised(uint16_t lift_angle_deg);

/* Restores PA5/TIM2 CH1 to the calibrated 80-degree search view. */
HAL_StatusTypeDef Actuator_SetCameraWideView(void);

/* Moves PA5/TIM2 CH1 to the calibrated 45-degree close view and holds it. */
HAL_StatusTypeDef Actuator_SetCameraNearView(void);

#endif /* ACTUATOR_TASK_H */
