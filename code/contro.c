#include "contro.h"
#include "car.h"
#include "pid.h"
#include "track.h"
#include "param.h"

// 转向 PID 实例（循迹一般纯 PD，Ki=0）
static PID_t steer_pid;

void control_init(void)
{
    pid_init(&steer_pid, STEER_KP, 0.0f, STEER_KD, STEER_OUT_MAX);
}

void control_update(int16 base_speed)
{
    // 1. 读循迹误差（track 模块已算好）
    float error = (float)track_offset;

    // 2. 转向 PID 计算 → 差速量 steer
    float steer = pid_inc(&steer_pid, error);

    // 3. 差速合成: 左轮减 steer，右轮加 steer
    int16 duty_l = (int16)base_speed - (int16)steer;
    int16 duty_r = (int16)base_speed + (int16)steer;

    // 4. 输出到硬件（car 模块内部处理方向、死区、限幅、PWM）
    car_set_motor(duty_l, duty_r);
}
