//
// Created by 58286 on 2022/7/19.
//

#ifndef MOTOR_TEST_CORE_INC_MOTOR_H_
#define MOTOR_TEST_CORE_INC_MOTOR_H_

#include "main.h"

#define CAN_ID 0x201

#define RC (1/(2*3*3.1415926f*10))
#define SAMPLE_FREQ 1000

#define C1 (1/(1+RC*SAMPLE_FREQ))
#define C2 (1-C1)

#define FILTER_BUF_LEN 2

typedef struct {
    int16_t speed_rpm;
    int16_t last_speed_rmp;
    int16_t real_current;
    int16_t given_current;
    uint8_t hall;
    uint16_t angle;                //abs angle range:[0,8191]
    uint16_t last_angle;    //abs angle range:[0,8191]
    uint16_t offset_angle;
    int32_t round_cnt;
    int32_t total_angle;
    uint8_t buf_idx;
    uint16_t angle_buf[FILTER_BUF_LEN];
    uint16_t fited_angle;
    uint32_t msg_cnt;
} moto_measure_t;

extern uint8_t can_rx_data[8];
extern moto_measure_t moto_measure;

int16_t get_moto_measure(moto_measure_t *ptr);

void set_moto_current(int16_t current);

#endif // MOTOR_TEST_CORE_INC_MOTOR_H_
