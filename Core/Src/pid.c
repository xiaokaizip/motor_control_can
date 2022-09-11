
#include "pid.h"

#define ABS(x)        ((x>0)? (x): (-x))      //相当于计算x的结绝对值

//限制
void abs_limit(float *a, float ABS_MAX) {
    if (*a > ABS_MAX)
        *a = ABS_MAX;
    if (*a < -ABS_MAX)
        *a = -ABS_MAX;
}

/*参数初始化--------------------------------------------------------------*/
static void pid_param_init(
        pid_t_ *pid,
        uint32_t mode,
        uint32_t maxout,
        uint32_t intergral_limit,
        float kp,
        float ki,
        float kd) {
    pid->IntegralLimit = intergral_limit; //积分限幅
    pid->MaxOutput = maxout;              //输出限幅
    pid->pid_mode = mode;                 //pid模式选择
    pid->p = kp;
    pid->i = ki;
    pid->d = kd;
}

/*中途更改参数设定(调试)------------------------------------------------------------*/
static void pid_reset(pid_t_ *pid, float kp, float ki, float kd) {
    pid->p = kp;
    pid->i = ki;
    pid->d = kd;
}

/**
    *@bref. calculate delta PID and position PID
    *@param[in] set： target
    *@param[in] real	measure
    */
float pid_calc(pid_t_ *pid, float get, float set) {
    pid->get[NOW] = get;          //获取当前的转速
    pid->set[NOW] = set;          //获取设置的转速
    pid->err[NOW] = set - get;    //获取当前的误差（set - measure）
    //如果当前的误差大于最大误差，则直接退出本次pid控制
    if (pid->max_err != 0 && ABS(pid->err[NOW]) > pid->max_err)
        return 0;
    if (pid->deadband != 0 && ABS(pid->err[NOW]) < pid->deadband)
        return 0;

    if (pid->pid_mode == POSITION_PID) //位置式p
    {
        pid->pout = pid->p * pid->err[NOW];                         //比例输出
        pid->iout += pid->i * pid->err[NOW];                        //积分输出（注意+=）
        pid->dout = pid->d * (pid->err[NOW] - pid->err[LAST]);     //微分输出（本次的误差-上次误差）

        abs_limit(&(pid->iout), pid->IntegralLimit);                //积分限幅，将积分输出最大限制在IntegralLimit
        pid->pos_out = pid->pout + pid->iout + pid->dout;           //最终的pid输出
        abs_limit(&(pid->pos_out), pid->MaxOutput);                 //输出限幅，接最终的输出结果限制在MaxOutput
        pid->last_pos_out = pid->pos_out;    //update last time      //输出结果运算结束，将本次的结果当成上次的输出结果
    } else if (pid->pid_mode == DELTA_PID)//增量式P
    {
        pid->pout = pid->p * (pid->err[NOW] - pid->err[LAST]);                      //比例输出
        pid->iout = pid->i * pid->err[NOW];                                         //积分输出
        pid->dout = pid->d * (pid->err[NOW] - 2 * pid->err[LAST] + pid->err[LLAST]);  //微分输出

        abs_limit(&(pid->iout), pid->IntegralLimit);                                //积分限幅
        pid->delta_u = pid->pout + pid->iout + pid->dout;                           //当前通过pid计算出来的结果
        pid->delta_out = pid->last_delta_out + pid->delta_u;                        //上次的输出结果加上本次计算出来的结果 组成增量式pid
        abs_limit(&(pid->delta_out), pid->MaxOutput);                               //输出限幅
        pid->last_delta_out = pid->delta_out;    //update last time                  //更新结果
    }
    //运算完成（更新）
    pid->err[LLAST] = pid->err[LAST];             //上次的误差变为上上次
    pid->err[LAST] = pid->err[NOW];               //当前的误差变为上次
    pid->get[LLAST] = pid->get[LAST];             //上次的结果变为上上次
    pid->get[LAST] = pid->get[NOW];               //当前的结果变为上次
    pid->set[LLAST] = pid->set[LAST];             //上次设定的目标变为上上次
    pid->set[LAST] = pid->set[NOW];               //当前设定的目标变为上次
    return pid->pid_mode == POSITION_PID ? pid->pos_out : pid->delta_out;
//
}

/*pid总体初始化-----------------------------------------------------------------*/
void PID_struct_init(
        pid_t_ *pid,
        uint32_t mode,
        uint32_t maxout,
        uint32_t intergral_limit,
        float kp,
        float ki,
        float kd) {
    /*init function pointer*/
    pid->f_param_init = pid_param_init;
    pid->f_pid_reset = pid_reset;
//	pid->f_cal_pid = pid_calc;
//	pid->f_cal_sp_pid = pid_sp_calc;	//addition

    /*init pid param */
    pid->f_param_init(pid, mode, maxout, intergral_limit, kp, ki, kd);
}
