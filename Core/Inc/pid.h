

#ifndef MOTOR_TEST_CORE_INC_PID_H_
#define MOTOR_TEST_CORE_INC_PID_H_

#include "main.h"

enum {
    LLAST = 0,        //上上次
    LAST = 1,         //上次
    NOW = 2,          //此刻

    POSITION_PID,     //位置式pid
    DELTA_PID,        //增量式pid
};
typedef struct __pid_t_ {
    float p;  //比例
    float i;  //积分
    float d;  //微分

    float set[3]; //目标值,包含NOW， LAST， LLAST上上次
    float get[3]; //测量值
    float err[3]; //误差

    float pout; //p(比例)输出
    float iout; //i(积分)输出
    float dout; //d(微分)输出

    float pos_out;      //本次位置式输出
    float last_pos_out; //上次输出
    float delta_u;      //本次增量值
    float delta_out;    //本次增量式输出 = last_delta_out + delta_u
    float last_delta_out;

    float max_err;        //最大误差
    float deadband; //err < deadband return
    uint32_t pid_mode;        //增量式pid或位置式pid
    uint32_t MaxOutput;     //输出限幅
    uint32_t IntegralLimit; //积分限幅

    void (*f_param_init)(struct __pid_t_ *pid, //PID参数初始化
                         uint32_t pid_mode,
                         uint32_t maxOutput,
                         uint32_t integralLimit,
                         float p,
                         float i,
                         float d);

    void (*f_pid_reset)(struct __pid_t_ *pid, float p, float i, float d); //pid三个参数修改

} pid_t_;

void PID_struct_init(
        pid_t_ *pid,
        uint32_t mode,
        uint32_t maxout,
        uint32_t intergral_limit,

        float kp,
        float ki,
        float kd);

float pid_calc(pid_t_ *pid, float fdb, float ref);

extern pid_t_ pid_rol;
extern pid_t_ pid_pit;
extern pid_t_ pid_yaw;
extern pid_t_ pid_pit_omg;
extern pid_t_ pid_yaw_omg;
extern pid_t_ pid_spd[4];
extern pid_t_ pid_yaw_alfa;
extern pid_t_ pid_chassis_angle;
extern pid_t_ pid_poke;
extern pid_t_ pid_poke_omg;
extern pid_t_ pid_imu_tmp;  //imu_temperature
extern pid_t_ pid_cali_bby; //big buff yaw
extern pid_t_ pid_cali_bbp;
extern pid_t_ pid_omg;
extern pid_t_ pid_pos;

#endif // MOTOR_TEST_CORE_INC_PID_H_
