#ifndef _CONTRO_H_
#define _CONTRO_H_

#include "zf_common_headfile.h"

/* 控制模块：转向 PID + 差速合成
 * 依赖: pid.h (PID算法) car.h (电机/编码器) track.h (循迹误差)
 * 不直接操作 PWM/方向脚/编码器寄存器，全部通过 car 模块访问
 */
void control_init(void);

/* 控制周期调一次（5ms）
 * 内部: 读 track_offset → steer PID → 差速合成 → car_set_motor
 * 参数: base_speed = 基础速度（正占空比，由赛道状态决定）
 */
void control_update(int16 base_speed);

#endif
