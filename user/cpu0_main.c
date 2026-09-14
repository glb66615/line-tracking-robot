/*********************************************************************************************************************
* TC264 Opensourec Library 即（TC264 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
*
* 本文件是 TC264 开源库的一部分
*
* TC264 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）来修改并再发布
* （参见 GNU General Public License）.
*
* TC264 开源库分发的目的是希望它有用，但没有任何的保证；
* 甚至没有适合特定目的的隐含的保证。详见 GNU GPL 文件.
*
* 您应该与本开源库一起收到一份 GPL 许可证的副本。
* 如果没有，查看 <https://www.gnu.org/licenses/>
*
* 注意：
* 本开源库使用 GPL3.0 开源许可证授权，因此在使用本开源库之前
* 请先阅读英文原文 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件
* 并确保 libraries 文件夹下 所有文件中的 LICENSE 文件
* 的许可范围使用正确 否则请修改后再使用，逐飞科技保留最终解释权
*
* 文件名称          cpu0_main
* 公司名称          成都逐飞科技有限公司
* 版本信息          请查看 libraries/doc 文件夹下的 version 文件 版本说明
* 适用平台          ADS v1.10.2
* 平台              TC264D
* 在线购买          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者                备注
* 2022-09-15       pudding            first version
********************************************************************************************************************/
#include "zf_common_headfile.h"
#include "param.h"
#include "car.h"
#include "fan.h"
#include "contro.h"
#include "track.h"
#include "ips.h"
#include "vofa.h"
#include "imu660.h"
#pragma section all "cpu0_dsram"

int core0_main(void)
{
    clock_init();
    wireless_init();                                // 无线串口 UART2 必须最先初始化（printf 已重定向到 UART2）

    // 外设初始化顺序：无线串口 → 传感器 → 电机/PWM → 屏幕 → PID/控制
    ips_init();                                     // 屏幕初始化
    car_init();                                     // 电机 + 编码器（内部自检）
    fan_init();                                     // 负压风扇 PWM（P21_2，50Hz 伺服协议）+ 按键
    imu_init();                                     // IMU660RC 官方驱动（硬件SPI0：SPC=P20_11 SDI=P20_14 SDO=P20_12 CS=P20_13，INT2=P15_4 四元数120Hz，内部重试）
    ips_show_status();                              // 屏幕显示自检结果
    track_init();                                   // 循迹状态机
    control_init();                                 // 转向 PID

    // 5ms 定时中断：始终跑编码器采样；control_enable() 后才跑闭环控制
    pit_ms_init(CCU60_CH0, CONTROL_PERIOD_MS);

    cpu_wait_event_ready();

    uint8 screen_div = 0;
    imu_data_t imu;
    while (TRUE)
    {
        wireless_task();                             // 无线 UART2 命令解析（唯一命令入口）
        fan_update();                                // 风扇软启动渐变推进（每帧一步）
        fan_key_update();                            // 风扇按键检测（开 P20_7 / 停 P20_6）
        imu_read_all(&imu);                          // 读 IMU 数据（每帧一次）

        // === VOFA JustFloat 实时数据 ===
        // 帧率由 5ms PIT 分频控制：精确 10ms/帧（100Hz），不受主循环耗时波动影响
        // 上电默认不发，收到 ms/ma/md/me 指令后才开始发数据
        if(vofa_send_due() && vofa_is_active())
        {
            float vofa_ch[8];
            if(vofa_get_mode() == 0)   // MODE_SPEED
            {
                vofa_ch[0] = (float)control_get_v_target_l();
                vofa_ch[1] = (float)car_get_speed_l();
                vofa_ch[2] = (float)control_get_v_target_r();
                vofa_ch[3] = (float)car_get_speed_r();
                vofa_ch[4] = (float)car_get_target_l() / PWM_PCT_SCALE;
                vofa_ch[5] = (float)car_get_target_r() / PWM_PCT_SCALE;
                vofa_ch[6] = (float)car_get_raw_l();
                vofa_ch[7] = (float)car_get_raw_r();
            }
            else if(vofa_get_mode() == 1)  // MODE_IMU
            {
                vofa_ch[0] = imu.gyro_x;
                vofa_ch[1] = imu.gyro_y;
                vofa_ch[2] = imu.gyro_z;
                vofa_ch[3] = imu.accel_x;
                vofa_ch[4] = imu.accel_y;
                vofa_ch[5] = imu.accel_z;
                vofa_ch[6] = imu.temp;
                vofa_ch[7] = 0.0f;
            }
            else if(vofa_get_mode() == 2)  // MODE_DIR：定角转向环调试
            {
                vofa_ch[0] = control_get_yaw_target();   // I0: 目标yaw（°）
                vofa_ch[1] = imu.yaw;                    // I1: 当前yaw（°）
                vofa_ch[2] = control_get_yaw_error();    // I2: 角度误差（±180°）
                vofa_ch[3] = imu.gyro_z;                 // I3: 陀螺仪z轴角速度（°/s）
                vofa_ch[4] = (float)control_get_delta(); // I4: 转向差速输出（±100）
                vofa_ch[5] = (float)control_get_base_target(); // I5: 基础速度%
                vofa_ch[6] = (float)car_get_speed_l();   // I6: 左轮实际速度%
                vofa_ch[7] = (float)car_get_speed_r();   // I7: 右轮实际速度%
            }
            else  // MODE_ENCODER (3)
            {
                vofa_ch[0] = (float)car_get_accum_l();    // I0: 左轮累积脉冲
                vofa_ch[1] = (float)car_get_accum_r();    // I1: 右轮累积脉冲
                vofa_ch[2] = (float)car_get_raw_l();       // I2: 左轮瞬时脉冲/5ms
                vofa_ch[3] = (float)car_get_raw_r();       // I3: 右轮瞬时脉冲/5ms
                vofa_ch[4] = (float)car_get_speed_l();     // I4: 左轮速度%
                vofa_ch[5] = (float)car_get_speed_r();     // I5: 右轮速度%
                vofa_ch[6] = 0.0f;                          // I6: 保留
                vofa_ch[7] = 0.0f;                          // I7: 保留
            }
            vofa_send(vofa_ch, 8);
        }

        if(++screen_div >= 20) { screen_div = 0; ips_show_speed(); }  // 每200ms刷一次屏幕
        system_delay_ms(10);                        // 10ms → 100Hz
    }
}

#pragma section all restore
// **************************** 用户代码 ****************************

// *************************** 下载调试说明 ***************************
// 下载程序时，请务必注意以下几点：
// 注意1：下载不下来？
//      请检查仿真器的驱动是否安装正确，仿真器的 COM 口是否能正确识别 或者 USB-TTL 模块的 COM 口
//      请使用逐飞 英飞凌TriCore 专用下载器，确认下载器是否被识别，确认是否已经连接，具体请查看下载器的原理图或者说明书
//      使用 USB-TTL 模块下载程序，请检查接线是否正确，模块的 TX 是否接到板子的 RX，模块 RX 是否接到板子的 TX
// 注意2：下载之后没有反应？
//      请检查下载的工程配置是否正确，可查看 zf_common_debug.h 文件 DEBUG_UART_BAUDRATE 宏定义为 debug uart 使用的波特率
// 注意3：显示的数值不准或者乱码？
//      请确认信号源是否正常，信号是否被干扰
//      使用示波器实测信号的电压大小， 然后 voltage/3V3*256 即可
