#include "imu660.h"
#include "zf_device_imu660rc.h"

// ==================== IMU660RC 官方驱动适配层 ====================
// 写法与 E4_06_imu660rc_demo 例程一致：
//   imu660rc_init()   硬件 SPI0 初始化（10MHz），配置量程 ±8G / ±2000DPS，
//                     四元数 120Hz 模式并使能 INT2 外部中断（P15_4）
//   INT2 中断里调 imu660rc_callback() → 采集四元数 + 加速度 + 陀螺仪原始值
//   主循环 imu660rc_get_quarternion 不再需要：callback 内部已完成解算，
//   直接读驱动全局变量 imu660rc_yaw/pitch/roll、imu660rc_gyro_x/y/z、imu660rc_acc_x/y/z

static uint8 imu_init_ok = 0;

uint8 imu_init(void)
{
    uint8 retry;
    for(retry = 0; retry < 10; retry++)
    {
        if(0 == imu660rc_init(IMU660RC_QUARTERNION_120HZ))   // 返回 0 = 成功
        {
            imu_init_ok = 1;
            break;
        }
        system_delay_ms(50);
    }

    // 初始化状态由屏幕显示（ips_show_status），串口不再发上电文本；仅故障时发一条提示方便查线
    if(!imu_init_ok)
        printf("[imu] init FAIL, check wiring (SPC=P20_11 SDI=P20_14 SDO=P20_12 CS=P20_13 INT2=P15_4)\r\n");
    return imu_init_ok;
}

uint8 imu_is_ok(void) { return imu_init_ok; }

float imu_get_yaw(void) { return imu660rc_yaw; }    // 片上 SFLP 已解算，直接读全局量

void imu_read_all(imu_data_t *data)
{
    // 四元数模式下 acc/gyro 已由 INT2 中断（imu660rc_callback）采集好，这里只做量纲换算
    data->gyro_x  = imu660rc_gyro_transition(imu660rc_gyro_x);   // °/s（±2000DPS 档）
    data->gyro_y  = imu660rc_gyro_transition(imu660rc_gyro_y);
    data->gyro_z  = imu660rc_gyro_transition(imu660rc_gyro_z);
    data->accel_x = imu660rc_acc_transition(imu660rc_acc_x);     // g（±8G 档）
    data->accel_y = imu660rc_acc_transition(imu660rc_acc_y);
    data->accel_z = imu660rc_acc_transition(imu660rc_acc_z);
    data->temp    = 0.0f;                                        // 官方驱动未读温度
    data->yaw     = imu660rc_yaw;                                // 0~360°，片上 SFLP 解算
    data->pitch   = imu660rc_pitch;
    data->roll    = imu660rc_roll;
}
