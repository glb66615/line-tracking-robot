#ifndef _CAR_H_
#define _CAR_H_

#include "zf_common_headfile.h"

void car_init(void);
uint8 car_is_ready(void);                  // 自检是否通过
// 差速小车：left/right 范围 -MOTOR_DUTY_MAX ~ +MOTOR_DUTY_MAX
void car_set_motor(int16 left, int16 right);
void car_stop(void);

// 编码器测速：在 5ms 控制周期中断里调一次
void  car_update_speed(void);
int16 car_get_speed_l(void);
int16 car_get_speed_r(void);
int16 car_get_raw_l(void);                 // 原始脉冲数/5ms（底层调试）
int16 car_get_raw_r(void);

// 目标速度（car_set_motor 设置的 PWM 值，供 VOFA 显示）
int16 car_get_target_l(void);
int16 car_get_target_r(void);

// 编码器累积脉冲（从上电开始，前进为正后退为负）
int32 car_get_accum_l(void);
int32 car_get_accum_r(void);

#endif
