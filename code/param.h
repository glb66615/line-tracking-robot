#ifndef _PARAM_H_
#define _PARAM_H_

#include "zf_common_headfile.h"

// ==================== 调试输出（有线 debug 串口已停用） ====================
// printf 已重定向到无线串口 UART2（见 printf_redirect.h），UART0 不再初始化

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
// 实测2026-09-12架空：PWM约1200轮子才启动
#define MOTOR_DEADZONE          1200

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
// 默认全部 0：上电不动，以 VOFA+ 滑块实际调节值为准
#define STEER_KP                0               // 比例系数
#define STEER_KD                0               // 微分系数
#define STEER_OUT_MAX           100             // 转向差速限幅（速度百分比，单侧最多±100%）

// ==================== 定角转向 PD（外环，陀螺仪 yaw 反馈） ====================
// 输入目标角度，陀螺仪实时修正：误差 = 目标yaw - 当前yaw（环形差处理 359→0 跳变，归 ±180）
// 输出转向差速%（正=左轮加速），复用"base±差速"合成，base=0 可原地旋转
// 默认全 0：上电不动，VOFA 滑块 ap/ad 实际调节
#define ANGLE_KP                0               // 定角 Kp（°误差→差速%，起始 0.5 左右）
#define ANGLE_KD                0               // 定角 Kd（阻尼抑制振荡，起始 0.3 左右）
#define ANGLE_OUT_MAX           100             // 差速限幅（速度百分比）
#define ANGLE_SIGN              1               // 方向系数：车朝目标反方向转就改成 -1（IMU 安装朝向决定）

// ==================== 速度 PI 控制参数（内环） ====================
// 全系统统一百分比 0~100：
//   目标速度/实际速度 = 满转速的百分比（0~100）
//   PID输出 = PWM百分比（0~100），硬件层再 ×PWM_PCT_SCALE 换成绝对占空比
//   Kp≈1 表示"100%速度误差给100%PWM"，调参直观
#define SPEED_KP_L             0.0f             // 左轮速度PI比例（VOFA滑块设，起始0.5）
#define SPEED_KI_L             0.0f             // 左轮速度PI积分（VOFA滑块设，起始0.02）
#define SPEED_KP_R             0.0f             // 右轮速度PI比例
#define SPEED_KI_R             0.0f             // 右轮速度PI积分
#define SPEED_OUT_MAX          100              // 速度环输出限幅（100% PWM）
#define PWM_PCT_SCALE          80               // 百分比→绝对PWM：100%=8000 → ×80（PWM上限80%）

// ==================== 速度档位（百分比 0~100） ====================
#define SPEED_STRAIGHT          0               // 默认0（上电安全静止，调参用滑块给速度）
#define SPEED_CURVE             40              // 普通弯道
#define SPEED_ISLAND            30              // 环岛
#define SPEED_ZEBRA             15              // 斑马线前减速
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

// ==================== 速度量程归一化 ====================
// 编码器原始脉冲(pulse/5ms) → 转速百分比 0~100：
//   速度% = raw × 100 / SPEED_RAW_FULLSCALE
// SPEED_RAW_FULLSCALE = PWM给满(5000)匀速时的raw值，由实测标定（硬件决定，无法推算）
#define SPEED_FULL_SCALE        100             // 速度满量程（百分比）
#define SPEED_RAW_FULLSCALE     3033            // ★标定值：PWM=8000匀速时raw脉冲/5ms（悬空实测2965~3100）

// ==================== 负压风扇（无刷 ESC，50Hz PWM 伺服协议） ====================
// P21_2 → ATOM0_CH0_P21_2，TC264 ATOM 模块支持 50Hz
// 无刷 ESC 油门协议：50Hz 周期 20ms，高电平 1.0~2.0ms
//   1.0ms = 停机（占空比 5% → duty=500）
//   1.5ms = 中位 50%（占空比 7.5% → duty=750）
//   2.0ms = 满油门（占空比 10% → duty=1000）
// VOFA 命令 fn%f 取值 0~100，映射到 duty = FAN_DUTY_MIN + (value/100)*(FAN_DUTY_MAX-FAN_DUTY_MIN)
#define FAN_PWM                 ATOM0_CH0_P21_2
#define FAN_FREQ                50              // 50Hz 伺服协议
#define FAN_DUTY_MAX            1000            // 2.0ms 满油门
#define FAN_DUTY_MIN            500             // 1.0ms 停机（ESC 识别为零油门）
#define FAN_DUTY_STOP           500             // 停车时 duty

// ==================== 风扇按键（两个独立按键：开/停） ====================
// P20_6 = 风扇开按键  KEY2（按下 → 低电平有效）
// P20_7 = 风扇停按键  KEY1（按下 → 低电平有效）
// 按键消抖 20ms，短按生效
#define FAN_KEY_OPEN            P20_7           // 风扇开（实际丝印 KEY1）
#define FAN_KEY_CLOSE           P20_6           // 风扇停（实际丝印 KEY2）
#define FAN_ON_DUTY_PCT         70              // 按键开风扇时的固定油门百分比
#define FAN_KEY_DEBOUNCE_MS     20              // 消抖时间

// ==================== 无线串口（UART2，P10_5 TX / P10_6 RX） ====================
// 用于 VOFA+ 无线调试：上行接收命令，下行发送 JustFloat 数据
#define WIRELESS_UART           UART_2
#define WIRELESS_UART_BAUD      115200
#define WIRELESS_UART_TX        UART2_TX_P10_5
#define WIRELESS_UART_RX        UART2_RX_P10_6

#endif
