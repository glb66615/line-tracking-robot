#ifndef _IMU660_H_
#define _IMU660_H_

#include "zf_common_headfile.h"

// ==================== IMU660RC 官方驱动适配层 ====================
// 使用 libraries/zf_device/zf_device_imu660rc.c（硬件 SPI0 + 片上四元数解算）
// 接线与官方例程一致（宏定义见 zf_device_imu660rc.h）：
//   模块 SCL/SPC -> P20_11     模块 SDA/SDI -> P20_14
//   模块 SA0/SDO -> P20_12     模块 CS      -> P20_13
//   模块 INT2    -> P15_4（120Hz 四元数就绪中断，isr.c 的 exti_ch0_ch4_isr 响应）

// ==================== 数据结构体 ====================
typedef struct
{
    float gyro_x;   // °/s
    float gyro_y;
    float gyro_z;
    float accel_x;  // g（静止时 az ≈ 1.0）
    float accel_y;
    float accel_z;
    float temp;     // 官方驱动不读温度，恒为 0
    float yaw;      // 偏航角（°，0~360，片上四元数解算，无累计漂移）
    float pitch;    // 俯仰角（°）
    float roll;     // 横滚角（°）
} imu_data_t;

// ==================== 函数接口 ====================
uint8 imu_init(void);                    // 初始化 + 自检（内部重试，返回 1=成功）
uint8 imu_is_ok(void);                   // 初始化结果（屏幕显示用）
void  imu_read_all(imu_data_t *data);    // 从驱动全局数据换算填充（数据在 INT2 中断里采集）
float imu_get_yaw(void);                 // 快速读当前 yaw（0~360°，控制环 5ms 中断里用）

#endif
