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
#include "contro.h"
#include "track.h"
#include "ips.h"
#include "vofa.h"
#pragma section all "cpu0_dsram"

int core0_main(void)
{
    clock_init();
    debug_init();

    // 外设初始化顺序：传感器 → 电机/PWM → 屏幕 → PID/控制
    ips_init();                                     // 屏幕初始化
    car_init();                                     // 电机 + 编码器（内部自检）
    ips_show_status();                              // 屏幕显示自检结果
    track_init();                                   // 循迹状态机
    control_init();                                 // 转向 PID

    // 5ms 定时中断：始终跑编码器采样；control_enable() 后才跑闭环控制
    pit_ms_init(CCU60_CH0, CONTROL_PERIOD_MS);

    printf("\r\n");
    printf("======================================\r\n");
    printf("  TC264 Grayscale Car - Speed PI Tuning\r\n");
    printf("======================================\r\n");
    printf("  cmds (one line + Enter):\r\n");
    printf("  lp:1.5 li:0.3   L wheel PI\r\n");
    printf("  rp:1.5 ri:0.3   R wheel PI\r\n");
    printf("  kp:8 kd:3       steer PD\r\n");
    printf("  v:30            target speed\r\n");
    printf("--------------------------------------\r\n");
    printf("  c = AUTO    x = STOP    p = params\r\n");
    printf("======================================\r\n");
    printf("  Waiting command...\r\n");
    cpu_wait_event_ready();

    while (TRUE)
    {
        vofa_task();                                 // 串口命令解析（整行）

        // === VOFA JustFloat 实时数据（8通道，按调参用途分组） ===
        // 速度组 I0~I3（量程 0~60）：目标 vs 实际，调速度PI看这组
        // PWM组  I4~I5（量程 ±1000）：速度PI输出，看是否顶限幅
        // 转向组 I6~I7（灰度通了才有数据）
        {
            float vofa_ch[8];
            vofa_ch[0] = (float)control_get_v_target_l();  // I0: 左轮目标速度
            vofa_ch[1] = (float)car_get_speed_l();         // I1: 左轮实际速度
            vofa_ch[2] = (float)control_get_v_target_r();  // I2: 右轮目标速度
            vofa_ch[3] = (float)car_get_speed_r();         // I3: 右轮实际速度
            vofa_ch[4] = (float)car_get_target_l();        // I4: 左PWM（速度PI输出）
            vofa_ch[5] = (float)car_get_target_r();        // I5: 右PWM
            vofa_ch[6] = (float)control_get_delta();       // I6: 差速量Δ（转向PD输出）
            vofa_ch[7] = (float)track_offset;              // I7: 循迹误差（转向PD输入）
            vofa_send(vofa_ch, 8);
        }

        ips_show_speed();                           // 刷新屏幕
        system_delay_ms(50);                        // 50ms → 20Hz
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
