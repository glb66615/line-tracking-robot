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

// ==================== 电机 PWM（DRV8701E 双电机，P11 排针组） ====================
// 实测标定（2026-09-10）：物理接线左右交叉，已按实测对调
// P11_11 = 左电机 DIR    P11_12 = 左电机 PWM (ATOM2_CH7)
// P11_9  = 右电机 DIR    P11_10 = 右电机 PWM (ATOM3_CH5)
#define MOTOR_LEFT_PWM          ATOM2_CH7_P11_12
#define MOTOR_RIGHT_PWM         ATOM3_CH5_P11_10
#define MOTOR_FREQ              17000           // 17kHz（官方示例值）
#define MOTOR_DUTY_MAX          PWM_DUTY_MAX    // 10000

// 电机方向脚（DIR=HIGH 正转，DIR=LOW 反转）
#define MOTOR_LEFT_DIR          P11_11
#define MOTOR_RIGHT_DIR         P11_9

// 电机死区补偿（PWM占空比单位，低于此值电机不动）
#define MOTOR_DEADZONE          300

// ==================== 舵机（如用舵机车则取消注释） ====================
// #define SERVO_PWM             ATOM1_CH1_P33_9
// #define SERVO_FREQ            50              // 50Hz 周期20ms
// #define SERVO_LEFT            250             // 左极限占空比
// #define SERVO_CENTER          375             // 中位
// #define SERVO_RIGHT           500             // 右极限占空比

// ==================== 编码器（实测标定：物理接线左右交叉，已对调） ====================
// 左轮：TIM4 + P02_8(A) + P00_9(B)
//      注意：该编码器前进方向读数为负，car_update_speed() 里已取反使"前进为正"
#define ENCODER_LEFT            TIM4_ENCODER
#define ENCODER_LEFT_CH1        TIM4_ENCODER_CH1_P02_8
#define ENCODER_LEFT_CH2        TIM4_ENCODER_CH2_P00_9

// 右轮：TIM6 + P20_3(A) + P20_0(B)
// TC264 硬件：Port 20 只有 P20_0/P20_3 支持 GPT12，P20_2 不连编码器
#define ENCODER_RIGHT           TIM6_ENCODER
#define ENCODER_RIGHT_CH1       TIM6_ENCODER_CH1_P20_3
#define ENCODER_RIGHT_CH2       TIM6_ENCODER_CH2_P20_0

// ==================== 转向 PD 控制参数（外环） ====================
#define STEER_KP                8               // 比例系数
#define STEER_KD                3               // 微分系数
#define STEER_OUT_MAX           5000            // 转向输出限幅（差速量最大值）

// ==================== 速度 PI 控制参数（内环） ====================
// 串级控制：外环PD算差速量 → 内环PI算PWM
// 速度环输入: 目标速度(pulse/5ms)  输出: PWM占空比
#define SPEED_KP_L             1.5f             // 左轮速度PI比例
#define SPEED_KI_L             0.3f             // 左轮速度PI积分
#define SPEED_KP_R             1.5f             // 右轮速度PI比例
#define SPEED_KI_R             0.3f             // 右轮速度PI积分
#define SPEED_OUT_MAX          1000             // 速度环输出限幅（调试期=10% PWM，安全；调好后再放开到5000）

// ==================== 基准速度（pulse/5ms，串口 'v:' 命令可在线修改） ====================
// base_target 是"全速"基准；各赛道元素的限速由 track_speed_limit(0~100) 叠加：
//   实际目标速度 = base_target × track_speed_limit / 100
// 例：base_target=50、弯道限速85% → 弯道目标速度 42
#define SPEED_STRAIGHT          50              // 基准速度（调试用小速度，实测后加大）

// ==================== 赛道元素参数 ====================
// 丢线确认帧数（连续N帧单侧丢线才触发环岛判定）
#define LOSS_CONFIRM_FRAMES     5
// 环岛完成后屏蔽帧数（防止出环口误判为下一个入环口）
#define ISLAND_SHIELD_FRAMES    50

// ==================== 斑马线参数（黑白交替计数） ====================
// 交替计数阈值：每压过1条黑纹产生2次状态切换（有线→全丢、全丢→有线）。
// 阈值=4 表示压上第3条黑纹时确认（已完整经历2条黑纹），
// 十字路口只有进出2次切换，阈值≥4 天然区分；建议 4~6
#define ZEBRA_TOGGLE_NUM        4
// 相邻两次黑白切换的最大间隔帧数（150ms）。
// 斑马线条纹交替间隔短（约半条纹宽度对应时间）；
// 间隔超过该值说明不是连续条纹（如十字路口后、冲出赛道回线），自动重新计数，
// 可同时防止"切换间隔过长误伤斑马线"和"残留计数导致后续误判"
#define ZEBRA_MAX_GAP_FRAMES    30
// 进入斑马线状态后的最短停留帧数（防刚判定就抖动恢复）
#define ZEBRA_MIN_HOLD_FRAMES   20
// 斑马线状态最长停留帧数（超时兜底强制恢复，防止停在黑纹上无法恢复；400帧=2s）
#define ZEBRA_TIMEOUT_FRAMES    400

// ==================== 赛道限速百分比（track_speed_limit，0~100） ====================
// 实际目标速度 = 串口设定的 base_target × 百分比 / 100
#define SPEED_PCT_FULL          100     // 直道全速
#define SPEED_PCT_CURVE         85      // 普通弯道/十字直走
#define SPEED_PCT_ISLAND        65      // 环岛
#define SPEED_PCT_ZEBRA         30      // 斑马线（0=完全停车，>0=低速通过）

// ==================== 系统控制周期 ====================
#define CONTROL_PERIOD_MS       5               // 5ms 控制周期（200Hz）

#endif
