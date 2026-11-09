#ifndef DELIVERY_FLOW_H
#define DELIVERY_FLOW_H

#include "flow_motion.h"
#include "stm32f4xx_hal.h"

typedef struct
{
  uint32_t forward_ms;
  uint32_t command_tick;
  bool advancing;
  bool baffle_held;
} DeliveryFlow_State;

typedef enum
{
  DELIVERY_ALIGN, DELIVERY_LOCATE, DELIVERY_BACKOFF, DELIVERY_PUSH, DELIVERY_REVERSE
} DeliveryFlow_Phase;

typedef enum
{
  DELIVERY_RUNNING, DELIVERY_ALIGNED, DELIVERY_LOCATED, DELIVERY_BACKED_OFF,
  DELIVERY_PLACED, DELIVERY_FORCED_RETREAT, DELIVERY_RETREATED,
  DELIVERY_ACTUATOR_FAULT, DELIVERY_DRIVE_TIMEOUT
} DeliveryFlow_Result;

typedef struct
{
  FlowMotion motion;
  HAL_StatusTypeDef (*baffle)(uint16_t angle);
  HAL_StatusTypeDef (*claw)(uint16_t angle);
  HAL_StatusTypeDef (*close_claw)(void);
} DeliveryFlow_Io;

void DeliveryFlow_ResetProgress(DeliveryFlow_State *flow, uint32_t now);
/* `also_active` is a separate incoming baffle pulse that the caller owns. */
bool DeliveryFlow_ReleasePwm(DeliveryFlow_State *flow, bool also_active,
    HAL_StatusTypeDef (*stop_pwm)(void));
DeliveryFlow_Result DeliveryFlow_Step(DeliveryFlow_State *flow, const DeliveryFlow_Io *io,
    DeliveryFlow_Phase phase, uint32_t now, uint32_t started_at,
    int32_t yaw_target, int32_t yaw_now, uint16_t release_angle,
    const int16_t *effective_forward_rpm);

#endif
