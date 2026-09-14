#include "ips.h"
#include "car.h"
#include "contro.h"
#include "param.h"
#include "imu660.h"
#include "vofa.h"

/* 启动画面：IPS + 电机/编码器自检状态 */
void ips_init(void)
{
    ips200_init(IPS200_TYPE_SPI);
    ips200_clear();

    ips200_show_string(0, 0,  "System Init...");
    ips200_show_string(0, 20, "IPS:    OK");           // IPS200 初始化本身 OK 才到这行
}

/* 显示自检结果（所有初始化完成后调一次）：初始化信息改为屏幕显示，串口上电不再发文本 */
void ips_show_status(void)
{
    ips200_show_string(0, 0,  "System Ready    ");          // 覆盖 "System Init..."
    ips200_show_string(0, 20, "Motor: ");
    ips200_show_string(48, 20, car_is_ready() ? "OK  " : "FAIL");
    ips200_show_string(104, 20, "IMU: ");
    ips200_show_string(136, 20, imu_is_ok() ? "OK  " : "FAIL");
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
    // === VOFA 无线发送状态（实时，一眼区分"车没在发"还是"无线链路断了"） ===
    static const char *vofa_name[4] = {"SPD ", "IMU ", "DIR ", "ENC "};
    ips200_show_string(0, 40, "VOFA: ");
    ips200_show_string(40, 40, vofa_name[vofa_get_mode() & 3]);
    ips200_show_string(80, 40, vofa_is_active() ? "100Hz TX" : "STOP    ");

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

    // === PWM 输出（百分比 0~100） ===
    ips200_show_string(0, 120, "PWM L:");
    ips200_show_int(48, 120, car_get_target_l() / PWM_PCT_SCALE, 4);
    ips200_show_string(120, 120, "R:");
    ips200_show_int(140, 120, car_get_target_r() / PWM_PCT_SCALE, 4);

    // === 编码器原始脉冲/5ms（底层调试，未归一化） ===
    ips200_show_string(0, 140, "rawL:");
    ips200_show_int(40, 140, car_get_raw_l(), 5);
    ips200_show_string(110, 140, "rawR:");
    ips200_show_int(150, 140, car_get_raw_r(), 5);

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
