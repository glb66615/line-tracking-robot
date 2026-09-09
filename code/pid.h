#ifndef PID_H
#define PID_H
#include "zf_common_headfile.h"

/* 增量式 PID
 * 公式: Δout = Kp*(e - e1) + Ki*e + Kd*(e - 2*e1 + e2)
 *       out += Δout
 * 循迹小车一般纯 PD (Ki=0)，避免积分饱和
 *
 * 适用场景: 电机速度环、舵机转向环（需要连续输出的系统）
 * 不适用:   位置式 PID（如舵机角度直接给定）
 */
typedef struct {
    float kp;               // 比例系数
    float ki;               // 积分系数（循迹一般设 0）
    float kd;               // 微分系数
    float out_limit;        // 输出限幅 ±out_limit（如 PWM 最大占空比）
    float last_error;       // e(k-1) 上一次误差
    float last_last_error;  // e(k-2) 上上次误差
    float out;              // 当前累积输出（控制量）
} PID_t;

/* 初始化 PID，清零状态量 */
void  pid_init(PID_t *pid, float kp, float ki, float kd, float out_limit);

/* 增量式 PID 计算
 * 参数: error = target - measure（误差）
 * 返回: 当前累积输出 out（已限幅）
 * 调用频率: 控制周期（如 5ms 一次）
 */
float pid_inc(PID_t *pid, float error);

#endif
