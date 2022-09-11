//
// Created by 86136 on 2022/9/10.
//

#include "key.h"
#include "main.h"

int flag_key = 0;
int break_flag = 0;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == GPIO_PIN_1) {
        flag_key = 1;
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_0);
    } else if (GPIO_Pin == GPIO_PIN_2) {
        flag_key = 2;
    } else if (GPIO_Pin == GPIO_PIN_10) {
        flag_key = 3;
        break_flag = 1;
    } else if (GPIO_Pin == GPIO_PIN_11) {
        flag_key = 4;
    }
}


int KEY_Open() {
    int key = flag_key;
    flag_key = 0;

    return key;
}

int count = 0;

void KEY_Select(motor_oled_ *motor_par) {
    int flag = KEY_Open();
    if (flag == 4) {
        motor_par->count = (motor_par->count + 1) % 3;
    }
    if (motor_par->count == 0) {
        if (flag == 2)
            motor_par->motor_type = (motor_par->motor_type + 1 + 2) % 2;
        else if (flag == 1)
            motor_par->motor_type = (motor_par->motor_type - 1 + 2) % 2;

    }
    if (motor_par->count == 1) {
        if (flag == 2)
            motor_par->id = (motor_par->id + 1 + 2) % 2;
        else if (flag == 1)
            motor_par->id = (motor_par->id - 1 + 2) % 2;

    }
    if (motor_par->count == 2) {
        if (flag == 1)
            motor_par->motor_speed -= 1;
        else if (flag == 2)
            motor_par->motor_speed += 1;
    }

}


