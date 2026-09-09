#include "M2006.h"

moto_measure_t moto_chassis[4] = {0};

static int16_t be16_to_i16(const uint8_t *data)
{
    return (int16_t)(((uint16_t)data[0] << 8) | data[1]);
}

void get_moto_measure(moto_measure_t *ptr, const uint8_t data[8])
{
    uint16_t new_angle = (uint16_t)(((uint16_t)data[0] << 8) | data[1]);

    if (ptr->msg_cnt == 0U) {
        ptr->angle = new_angle;
        ptr->last_angle = new_angle;
        ptr->offset_angle = new_angle;
        ptr->round_cnt = 0;
    } else {
        ptr->last_angle = ptr->angle;
        ptr->angle = new_angle;
        if ((int32_t)ptr->angle - ptr->last_angle > 4096) ptr->round_cnt--;
        else if ((int32_t)ptr->angle - ptr->last_angle < -4096) ptr->round_cnt++;
    }

    ptr->speed_rpm = be16_to_i16(&data[2]);
    ptr->real_current = be16_to_i16(&data[4]);
    ptr->given_current = ptr->real_current;
    ptr->hall = data[6];
    ptr->total_angle = ptr->round_cnt * 8192 + ptr->angle - ptr->offset_angle;
    ptr->last_rx_tick = HAL_GetTick();
    ptr->msg_cnt++;
}

void get_moto_offset(moto_measure_t *ptr, const uint8_t data[8])
{
    ptr->angle = (uint16_t)(((uint16_t)data[0] << 8) | data[1]);
    ptr->offset_angle = ptr->angle;
}

HAL_StatusTypeDef M2006_Init(CAN_HandleTypeDef *hcan)
{
    CAN_FilterTypeDef filter = {0};
    if (hcan == NULL) return HAL_ERROR;

    filter.FilterBank = 0;
    filter.FilterMode = CAN_FILTERMODE_IDLIST;
    filter.FilterScale = CAN_FILTERSCALE_16BIT;
    filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    filter.FilterActivation = ENABLE;
    filter.SlaveStartFilterBank = 14;
    filter.FilterIdHigh = CAN_2006Moto1_ID << 5U;
    filter.FilterIdLow = CAN_2006Moto2_ID << 5U;
    filter.FilterMaskIdHigh = CAN_2006Moto3_ID << 5U;
    filter.FilterMaskIdLow = CAN_2006Moto4_ID << 5U;
    if (HAL_CAN_ConfigFilter(hcan, &filter) != HAL_OK) return HAL_ERROR;
    if (HAL_CAN_Start(hcan) != HAL_OK) return HAL_ERROR;
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
    return HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
}

HAL_StatusTypeDef set_moto_current(CAN_HandleTypeDef *hcan, int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4)
{
    CAN_TxHeaderTypeDef header = { .StdId = CAN_2006Moto_ALL_ID, .IDE = CAN_ID_STD,
        .RTR = CAN_RTR_DATA, .DLC = 8, .TransmitGlobalTime = DISABLE };
    uint8_t data[8] = { (uint8_t)(iq1 >> 8), (uint8_t)iq1, (uint8_t)(iq2 >> 8), (uint8_t)iq2,
        (uint8_t)(iq3 >> 8), (uint8_t)iq3, (uint8_t)(iq4 >> 8), (uint8_t)iq4 };
    uint32_t mailbox;
    if (hcan == NULL) return HAL_ERROR;
    if (HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0U) return HAL_BUSY;
    return HAL_CAN_AddTxMessage(hcan, &header, data, &mailbox);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef header;
    uint8_t data[8];
    if (hcan->Instance != CAN1 || HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &header, data) != HAL_OK ||
        header.DLC != 8U ||
        header.IDE != CAN_ID_STD || header.RTR != CAN_RTR_DATA) return;
    switch (header.StdId) {
    case CAN_2006Moto1_ID: get_moto_measure(&moto_chassis[0], data); break;
    case CAN_2006Moto2_ID: get_moto_measure(&moto_chassis[1], data); break;
    case CAN_2006Moto3_ID: get_moto_measure(&moto_chassis[2], data); break;
    case CAN_2006Moto4_ID: get_moto_measure(&moto_chassis[3], data); break;
    default: break;
    }
}
