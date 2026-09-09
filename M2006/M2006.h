#ifndef M2006_H
#define M2006_H

#include "main.h"
#include <stdint.h>

typedef enum {
    CAN_2006Moto_ALL_ID = 0x200,
    CAN_2006Moto1_ID = 0x201,
    CAN_2006Moto2_ID = 0x202,
    CAN_2006Moto3_ID = 0x203,
    CAN_2006Moto4_ID = 0x204,
} CAN_Message_ID;

typedef struct {
    volatile int16_t speed_rpm;
    volatile int16_t real_current;
    volatile int16_t given_current;
    volatile uint8_t hall;
    volatile uint16_t angle;
    volatile uint16_t last_angle;
    volatile uint16_t offset_angle;
    volatile int32_t round_cnt;
    volatile int32_t total_angle;
    volatile uint32_t msg_cnt;
    volatile uint32_t last_rx_tick;
} moto_measure_t;

extern moto_measure_t moto_chassis[4];
HAL_StatusTypeDef M2006_Init(CAN_HandleTypeDef *hcan);
void get_moto_measure(moto_measure_t *ptr, const uint8_t data[8]);
void get_moto_offset(moto_measure_t *ptr, const uint8_t data[8]);
HAL_StatusTypeDef set_moto_current(CAN_HandleTypeDef *hcan, int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4);

#endif
