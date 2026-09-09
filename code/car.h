#ifndef _CAR_H_
#define _CAR_H_

#include "zf_common_headfile.h"

void car_init(void);
// 差速小车：left/right 范围 -MOTOR_DUTY_MAX ~ +MOTOR_DUTY_MAX
void car_set_motor(int16 left, int16 right);
void car_stop(void);

// 编码器测速：在 5ms 控制周期中断里调一次
void  car_update_speed(void);
int16 car_get_speed_l(void);
int16 car_get_speed_r(void);

#endif
