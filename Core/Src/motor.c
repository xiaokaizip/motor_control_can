//
// Created by 58286 on 2022/7/19.
//
#include "motor.h"
#include "usart.h"
#include "printf.h"
#include "oled.h"

uint8_t can_rx_data[8];
moto_measure_t moto_measure;

static CAN_TxHeaderTypeDef can_header;
static uint8_t can_tx_data[8];
extern CAN_HandleTypeDef hcan;

int16_t get_moto_measure(moto_measure_t *ptr) {
    ptr->last_angle = ptr->angle;
    ptr->angle = (uint16_t) (can_rx_data[0] << 8 | can_rx_data[1]);
    ptr->real_current = (int16_t) (can_rx_data[2] << 8 | can_rx_data[3]);
    ptr->speed_rpm = ptr->real_current;    //这里是因为两种电调对应位不一样的信息
    ptr->given_current = (int16_t) (can_rx_data[4] << 8 | can_rx_data[5]) / -5;
    ptr->hall = can_rx_data[6];
    if (ptr->angle - ptr->last_angle > 4096)
        ptr->round_cnt--;
    else if (ptr->angle - ptr->last_angle < -4096)
        ptr->round_cnt++;
    ptr->total_angle = ptr->round_cnt * 8192 + ptr->angle - ptr->offset_angle;

    return ptr->speed_rpm;

}

void set_moto_current(int16_t current) {
    can_header.IDE = CAN_ID_STD;
    can_header.RTR = CAN_RTR_DATA;
    can_header.StdId = 0x200;
    can_header.DLC = 8;

    can_tx_data[0] = current >> 8;
    can_tx_data[1] = current & 0xFF;
    uint32_t tx_mailbox;

    HAL_CAN_AddTxMessage(&hcan, &can_header, can_tx_data, &tx_mailbox);

}