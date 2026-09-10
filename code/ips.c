#include "ips.h"
#include "car.h"
#include "contro.h"
#include "track.h"

/* 启动画面：IPS + 电机/编码器自检状态 */
void ips_init(void)
{
    ips200_init(IPS200_TYPE_SPI);
    ips200_clear();

    ips200_show_string(0, 0,  "System Init...");
    ips200_show_string(0, 20, "IPS:    OK");           // IPS200 初始化本身 OK 才到这行
}

/* 显示电机/编码器自检结果（car_init 之后调一次） */
void ips_show_status(void)
{
    ips200_show_string(0, 40, "Motor:  ");
    ips200_show_string(48, 40, car_is_ready() ? "OK" : "FAIL");
}

/* 运行画面：每 50ms 刷新一次（固定位置覆盖写，不闪屏）
 *
 *   y=60   Mode: MANUAL / AUTO
 *   y=80   L: 实际速度      tgt:目标速度
 *   y=100  R: 实际速度      tgt:目标速度
 *   y=120  PWM L: xxxx      R: xxxx
 *   y=140  offset: xxx      d: xxx
 */
void ips_show_speed(void)
{
    // === 模式 ===
    ips200_show_string(0, 60, "Mode: ");
    ips200_show_string(40, 60, control_is_enabled() ? "AUTO  " : "MANUAL");

    // === 左轮：实际速度 / 目标速度 ===
    ips200_show_string(0, 80, "L v:");
    ips200_show_int(32, 80, car_get_speed_l(), 4);
    ips200_show_string(80, 80, "tgt:");
    ips200_show_int(112, 80, control_get_v_target_l(), 4);

    // === 右轮：实际速度 / 目标速度 ===
    ips200_show_string(0, 100, "R v:");
    ips200_show_int(32, 100, car_get_speed_r(), 4);
    ips200_show_string(80, 100, "tgt:");
    ips200_show_int(112, 100, control_get_v_target_r(), 4);

    // === PWM 输出 ===
    ips200_show_string(0, 120, "PWM L:");
    ips200_show_int(48, 120, car_get_target_l(), 5);
    ips200_show_string(120, 120, "R:");
    ips200_show_int(140, 120, car_get_target_r(), 5);

    // === 转向环 ===
    ips200_show_string(0, 140, "off:");
    ips200_show_int(32, 140, track_offset, 4);
    ips200_show_string(80, 140, "d:");
    ips200_show_int(100, 140, control_get_delta(), 5);

    // === 串口设定速度 + PI 参数（调参用） ===
    ips200_show_string(0, 165, "set:");
    ips200_show_int(32, 165, control_get_base_target(), 4);

    ips200_show_string(0, 190, "L Kp:");
    ips200_show_float(40, 190, control_get_speed_kp_l(), 4, 2);
    ips200_show_string(100, 190, "Ki:");
    ips200_show_float(132, 190, control_get_speed_ki_l(), 4, 2);

    ips200_show_string(0, 215, "R Kp:");
    ips200_show_float(40, 215, control_get_speed_kp_r(), 4, 2);
    ips200_show_string(100, 215, "Ki:");
    ips200_show_float(132, 215, control_get_speed_ki_r(), 4, 2);
}
