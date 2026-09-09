#ifndef _PARAM_H_
#define _PARAM_H_

#include "zf_common_headfile.h"

// ==================== 调试串口（已由库默认配置，勿动） ====================
// UART0  P14_0(TX) / P14_1(RX)  115200  用于 printf 调试输出

// ==================== 灰度传感器串口 ====================
// 传感器自带MCU，通过串口发送8路数据
#define GRAY_UART_NUM           UART_1
#define GRAY_UART_BAUD          115200
#define GRAY_UART_TX            UART1_TX_P33_12
#define GRAY_UART_RX            UART1_RX_P33_13
#define GRAY_CH_NUM             8               // 8路灰度

// 黑底白线：压白线=高，不压=低
// 加权系数 左→右，正值偏左，负值偏右
#define GRAY_WEIGHT_LEFT        {4, 3, 2, 1}    // 左4路
#define GRAY_WEIGHT_RIGHT       {-1, -2, -3, -4} // 右4路

// 归一化阈值（0~100，高于此值判定为压白线）
#define GRAY_TH_WHITE           50

// ==================== 电机 PWM（差速车） ====================
// 逐飞推荐8路PWM中取前2路
#define MOTOR_LEFT_PWM          ATOM0_CH0_P21_2
#define MOTOR_RIGHT_PWM         ATOM0_CH1_P21_3
#define MOTOR_FREQ              10000           // 10kHz
#define MOTOR_DUTY_MAX          PWM_DUTY_MAX    // 10000

// 电机方向脚（GPIO输出，0正转1反转）
#define MOTOR_LEFT_DIR          P21_4
#define MOTOR_RIGHT_DIR         P21_5

// 电机死区补偿（PWM占空比单位，低于此值电机不动）
#define MOTOR_DEADZONE          300

// ==================== 舵机（如用舵机车则取消注释） ====================
// #define SERVO_PWM             ATOM1_CH1_P33_9
// #define SERVO_FREQ            50              // 50Hz 周期20ms
// #define SERVO_LEFT            250             // 左极限占空比
// #define SERVO_CENTER          375             // 中位
// #define SERVO_RIGHT           500             // 右极限占空比

// ==================== 编码器（TC264 硬件约束：Port 20 只有 P20_0 和 P20_3 支持 GPT12） ====================
// 右轮：TIM4 + P02_8(A) + P00_9(B)
#define ENCODER_RIGHT           TIM4_ENCODER
#define ENCODER_RIGHT_CH1       TIM4_ENCODER_CH1_P02_8
#define ENCODER_RIGHT_CH2       TIM4_ENCODER_CH2_P00_9

// 左轮：TIM6 + P20_3(A) + P20_0(B)
//      ↑ 原理图标 B=P20.2 是笔误！TC264 硬件上 P20.2 不连 GPT12，只有 P20.0 能做 TIM6 CH2
//      ↑ 请确认 B 相物理插头插在 P20.0 排针上
#define ENCODER_LEFT            TIM6_ENCODER
#define ENCODER_LEFT_CH1        TIM6_ENCODER_CH1_P20_3
#define ENCODER_LEFT_CH2        TIM6_ENCODER_CH2_P20_0

// ==================== 转向 PD 控制参数 ====================
#define STEER_KP                8               // 比例系数
#define STEER_KD                3               // 微分系数
#define STEER_OUT_MAX           5000            // 转向输出限幅

// ==================== 速度档位（占空比百分比 0~100） ====================
#define SPEED_STRAIGHT          80              // 直道
#define SPEED_CURVE             60              // 普通弯道
#define SPEED_ISLAND            40              // 环岛
#define SPEED_ZEBRA             20              // 斑马线前减速
#define SPEED_STOP              0               // 停车

// ==================== 赛道元素参数 ====================
// 丢线确认帧数（连续N帧单侧丢线才触发环岛判定）
#define LOSS_CONFIRM_FRAMES     5
// 斑马线判定：连续N帧全黑才确认
#define ZEBRA_BLACK_FRAMES      15
// 环岛完成后屏蔽帧数（防止出环口误判为下一个入环口）
#define ISLAND_SHIELD_FRAMES    50

// ==================== 系统控制周期 ====================
#define CONTROL_PERIOD_MS       5               // 5ms 控制周期（200Hz）

#endif
