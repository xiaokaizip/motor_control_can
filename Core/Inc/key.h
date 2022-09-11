//
// Created by 86136 on 2022/9/10.
//

#ifndef MOTOR_CONTROL_CODE_KEY_H
#define MOTOR_CONTROL_CODE_KEY_H

extern int break_flag;
typedef struct motor_oled {
    int count;
    int motor_type;
    int id;
    float motor_speed;

} motor_oled_;

int KEY_Open();

void KEY_Select(motor_oled_ *motor);


#endif //MOTOR_CONTROL_CODE_KEY_H
