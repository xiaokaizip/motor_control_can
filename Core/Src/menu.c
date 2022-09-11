#include "oled.h"
#include "key.h"

enum motor_type {
    M2006 = 0, M3508 = 1,
};


int motor_name[2] = {2006, 3508};
int id = 0;
float speed = 123.45f;

int type_cursor_flag = 0;
int id_cursor_flag = 0;
int speed_cursor_flag = 0;


void OLED_Menu(struct motor_oled *par) {

    KEY_Select(par);
    switch (par->count) {
        case 0:
            type_cursor_flag = 1;
            id_cursor_flag = 0;
            speed_cursor_flag = 0;
            break;
        case 1:
            type_cursor_flag = 0;
            id_cursor_flag = 1;
            speed_cursor_flag = 0;
            break;
        case 2:
            type_cursor_flag = 0;
            id_cursor_flag = 0;
            speed_cursor_flag = 1;
            break;
        default:
            break;
    }


    OLED_ShowNum(8 * 2, 0, motor_name[par->motor_type], 4, 16, type_cursor_flag);
    OLED_ShowString(8 * 8, 0, "id:", 16, 0);
    OLED_ShowNum(8 * 11, 0, par->id, 1, 16, id_cursor_flag);
    OLED_ShowString(0, 2, "speed:", 16, 0);
    OLED_Showdecimal(8 * 7, 2, par->motor_speed, 4, 3, 16, speed_cursor_flag);
    OLED_ShowNum(8 * 11, 5, par->count, 1, 16, id_cursor_flag);


}