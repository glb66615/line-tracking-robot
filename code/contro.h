#ifndef _CONTRO_H_
#define _CONTRO_H_

#include "zf_common_headfile.h"

/* 控制模块：串级控制（转向PD外环 + 速度PI内环）
 * 依赖: pid.h (PID算法) car.h (电机/编码器) track.h (循迹误差)
 * 不直接操作 PWM/方向脚/编码器寄存器，全部通过 car 模块访问
 *
 * 数据流:
 *   track_offset → 转向PD → Δ(差速量) →
 *   base±Δ → v_L/v_R_target → 速度PI_L/R → PWM_L/R → car_set_motor
 */

// 初始化（转向PD + 左右速度PI）
void control_init(void);

// 5ms 中断里调一次（内部使用串口设定的目标速度和PID参数）
void control_update(void);

// 控制环使能（'c' 命令切换 手动测试/自动循迹）
void  control_enable(void);
void  control_disable(void);
uint8 control_is_enabled(void);

// ==================== 串口在线调参接口 ====================
// 目标速度（pulse/5ms）
void  control_set_base_target(int16 v);
int16 control_get_base_target(void);

// 转向 PD（外环）
void control_set_steer_kp(float kp);
void control_set_steer_kd(float kd);

// 速度 PI（内环）—— 左右独立设置
void control_set_speed_kp_l(float kp);
void control_set_speed_ki_l(float ki);
void control_set_speed_kp_r(float kp);
void control_set_speed_ki_r(float ki);
float control_get_speed_kp_l(void);
float control_get_speed_ki_l(void);
float control_get_speed_kp_r(void);
float control_get_speed_ki_r(void);
float control_get_steer_kp(void);
float control_get_steer_kd(void);

// ==================== VOFA 监测接口 ====================
int16 control_get_v_target_l(void);    // 左轮目标速度
int16 control_get_v_target_r(void);    // 右轮目标速度
int16 control_get_v_actual_l(void);    // 左轮实际速度（编码器）
int16 control_get_v_actual_r(void);    // 右轮实际速度
int16 control_get_pwm_l(void);         // 速度PI输出PWM（左）
int16 control_get_pwm_r(void);         // 速度PI输出PWM（右）
int16 control_get_delta(void);         // 转向PD输出（差速量）

#endif
