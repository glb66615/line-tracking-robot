#include "pid.h"

/* 初始化 PID
 * 清零所有状态量，设置 Kp/Ki/Kd 和输出限幅
 */
void pid_init(PID_t *pid, float kp, float ki, float kd, float out_limit)
{
    pid->kp              = kp;
    pid->ki              = ki;
    pid->kd              = kd;
    pid->out_limit       = out_limit;
    pid->last_error      = 0.0f;
    pid->last_last_error = 0.0f;
    pid->out             = 0.0f;
}

/* 增量式 PID 核心计算
 *
 * 数学推导:
 *   位置式: out(k) = Kp*e(k) + Ki*Σe + Kd*(e(k)-e(k-1))
 *   增量式: Δout = out(k) - out(k-1)
 *         = Kp*(e(k)-e(k-1)) + Ki*e(k) + Kd*(e(k)-2*e(k-1)+e(k-2))
 *
 * 优势: 输出是增量而非绝对量，切换模式/启动时无冲击，适合电机 PWM 控制
 */
float pid_inc(PID_t *pid, float error)
{
    // 计算增量 Δout
    float delta = pid->kp * (error - pid->last_error)              // P: e(k) - e(k-1)
                + pid->ki * error                                  // I: e(k)
                + pid->kd * (error - 2.0f * pid->last_error + pid->last_last_error); // D: e(k) - 2e(k-1) + e(k-2)

    // 累加到输出
    pid->out += delta;

    // 输出限幅（防止 PWM 超范围）
    if(pid->out >  pid->out_limit)  pid->out =  pid->out_limit;
    if(pid->out < -pid->out_limit)  pid->out = -pid->out_limit;

    // 更新历史误差（为下一次计算做准备）
    pid->last_last_error = pid->last_error;   // e(k-2) ← e(k-1)
    pid->last_error      = error;              // e(k-1) ← e(k)

    return pid->out;
}
