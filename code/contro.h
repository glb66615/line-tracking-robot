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
// 统一目标速度（循迹模式：左右轮 = v ± 转向差速）
void  control_set_base_target(int16 v);
int16 control_get_base_target(void);

// 独立左右轮目标（手动调试模式：直接指定，不经过转向PD）
void  control_set_target_l(int16 v);
void  control_set_target_r(int16 v);
int16 control_get_target_l(void);
int16 control_get_target_r(void);

// 转向 PD（外环）
void control_set_steer_kp(float kp);
void control_set_steer_kd(float kd);

// 速度 PI（内环）—— 左右独立设置
void control_set_speed_kp_l(float kp);
void control_set_speed_ki_l(float ki);
void control_set_speed_kff_l(float kff);   // 前馈系数
void control_set_speed_kp_r(float kp);
void control_set_speed_ki_r(float ki);
void control_set_speed_kff_r(float kff);
float control_get_speed_kp_l(void);
float control_get_speed_ki_l(void);
float control_get_speed_kff_l(void);
float control_get_speed_kp_r(void);
float control_get_speed_ki_r(void);
float control_get_speed_kff_r(void);
float control_get_steer_kp(void);
float control_get_steer_kd(void);

// ==================== VOFA/屏幕 监测接口 ====================
int16 control_get_v_target_l(void);    // 左轮目标速度
int16 control_get_v_target_r(void);    // 右轮目标速度
int16 control_get_delta(void);         // 转向PD输出（差速量）

// ==================== 定角转向模式（陀螺仪 yaw 反馈） ====================
// 串级结构不变：目标yaw - 当前yaw（环形差±180）→ 定角PD → 差速 → 速度PI
// base=0 时左右轮反转，原地转向；base>0 时定角直线行驶
void  control_set_yaw_target(float yaw);   // 'ya90'：设目标角(0~360°)并切入定角模式
void  control_hold_yaw(void);              // 'yr'：锁定当前朝向为目标（直线保持）
void  control_set_track_mode(void);        // 切回灰度循迹模式（'x' 停车时调用）
uint8 control_get_mode(void);              // 0=灰度循迹  1=定角转向
float control_get_yaw_target(void);        // 当前目标角（VOFA显示）
float control_get_yaw_error(void);         // 角度误差 ±180°（VOFA显示）
void  control_set_angle_kp(float kp);      // 定角PD调参（'ap'/'ad' 命令）
void  control_set_angle_kd(float kd);
float control_get_angle_kp(void);
float control_get_angle_kd(void);

#endif
