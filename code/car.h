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

// 目标速度（car_set_motor 设置的 PWM 值，供 VOFA 显示）
int16 car_get_target_l(void);
int16 car_get_target_r(void);

#endif
