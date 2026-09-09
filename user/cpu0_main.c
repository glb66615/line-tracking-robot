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
#pragma section all "cpu0_dsram"

int core0_main(void)
{
    clock_init();
    debug_init();

    // 外设初始化顺序：传感器 → 电机/PWM → 屏幕 → PID/控制
    ips_init();                                     // 屏幕初始化
    car_init();                                     // 电机 + 编码器
    track_init();                                   // 循迹状态机
    control_init();                                 // 转向 PID

    // 控制周期定时器（中断里调 car_update_speed → track_update → control_update）
    pit_ms_init(CCU60_CH0, CONTROL_PERIOD_MS);

    printf("\r\n=== init ok ===\r\n");
    cpu_wait_event_ready();

    while (TRUE)
    {
        ips_show_speed();                           // 50ms 刷新屏幕（不跟控制周期同频）
        system_delay_ms(50);
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
