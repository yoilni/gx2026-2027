#ifndef __M2006_Speed_H
#define __M2006_Speed_H

#include <stdint.h>
#include "M2006_PID.h"
#include "M2006.h"

#define run_current_limit 3000
#define stop_current_limit 3000
#define M2006_FEEDBACK_TIMEOUT_MS 100U
#define M2006_MAX_CURRENT_LIMIT 10000

/* Chassis mapping: motor 1/4 are left, motor 2/3 are right. */
#define M2006_LEFT_FRONT_INDEX  0U
#define M2006_RIGHT_FRONT_INDEX 1U
#define M2006_RIGHT_REAR_INDEX  2U
#define M2006_LEFT_REAR_INDEX   3U
// 速度环控制结构体
typedef struct {
    float target_speed[4];       // 目标速度(RPM)
    float current_speed[4];   // 当前速度数组(RPM)
    uint32_t last_update;     // 最后更新时间
} SpeedLoopController;

// 新增函数声明
void SpeedLoop_Init(void);
void SpeedLoop_SetMotorTarget(uint8_t motor_id, float speed, int16_t c_limit);
void SpeedLoop_SetTarget(float speed_left, float speed_right, int16_t current_limit);
void brake(void);
void SpeedLoop_Update(CAN_HandleTypeDef *hcan);
void MotorPID_Init(uint8_t Motor_Number);

#endif 
