#ifndef DEPARTURE_FLOW_H
#define DEPARTURE_FLOW_H
#include "flow_motion.h"
#include "stm32f4xx_hal.h"

typedef enum { DEPARTURE_TURNING, DEPARTURE_STABLE, DEPARTURE_OPEN_CLAW,
               DEPARTURE_WRONG_DIRECTION } DepartureTurnStatus;
typedef struct { DepartureTurnStatus status; bool newly_stable; } DepartureTurnResult;
DepartureTurnResult DepartureFlow_Turn(const FlowMotion *io,
    bool (*expired)(uint32_t now, int32_t target), uint32_t now, int32_t target,
    int32_t yaw_error, bool was_stable, uint32_t *stable_since);

HAL_StatusTypeDef DepartureFlow_OpenClaw(void (*stop)(void), HAL_StatusTypeDef (*claw)(uint16_t));
typedef enum { DEPARTURE_WAITING, DEPARTURE_DRIVING, DEPARTURE_READY,
               DEPARTURE_ACTUATOR_FAULT } DepartureStepStatus;
DepartureStepStatus DepartureFlow_LiftAfterClaw(uint32_t now, uint32_t started_at,
    void (*stop)(void), HAL_StatusTypeDef (*baffle)(uint16_t));
typedef struct { DepartureStepStatus status; bool just_lowered; int16_t correction; } DepartureBumpResult;
DepartureBumpResult DepartureFlow_CrossBump(const FlowMotion *io,
    HAL_StatusTypeDef (*baffle)(uint16_t), HAL_StatusTypeDef (*close_claw)(void),
    int16_t (*yaw_correction)(void), uint32_t now, uint32_t started_at,
    bool *drive_finished, uint32_t *baffle_down_tick);
#endif
