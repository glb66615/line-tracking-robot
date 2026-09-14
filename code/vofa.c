#include "vofa.h"
#include "param.h"
#include "car.h"
#include "fan.h"
#include "contro.h"
#include "imu660.h"
#include "stdio.h"
#include "string.h"
#include "zf_common_function.h"
#include "zf_driver_uart.h"

// printf 已由 printf_redirect.h 重定向到无线 UART2；上电不再发送任何初始化文本（状态看屏幕）

// JustFloat 帧尾
static const uint8 vofa_tail[4] = {0x00, 0x00, 0x80, 0x7f};

// 显示模式：0=速度环，1=姿态IMU，2=方向环，3=编码器累积
#define MODE_SPEED   0
#define MODE_IMU     1
#define MODE_DIR     2
#define MODE_ENCODER 3
static uint8 vofa_mode = MODE_SPEED;

// 数据发送使能：上电自动开流。断电重启后无需重发命令，数据直接恢复——
// 否则每次重启 vofa_active 清零，上次发的 ma/ms 全丢，看起来就是"串口重连后死了"
static uint8 vofa_active = 1;

uint8 vofa_get_mode(void) { return vofa_mode; }
uint8 vofa_is_active(void) { return vofa_active; }

void vofa_send(float *ch, uint8 n)
{
    // 无线 UART2（VOFA+ 无线模块）
    uart_write_buffer(WIRELESS_UART, (const uint8 *)ch, n * sizeof(float));
    uart_write_buffer(WIRELESS_UART, vofa_tail, 4);
}

// ==================== 串口命令解析（VOFA+ 滑块格式） ====================
//
// 统一格式: key%f\r\n  → VOFA 命令控件自动替换 %f 为滑块值
//
//   lp%f   → lp0.5      左轮速度PI Kp
//   li%f   → li0.02     左轮速度PI Ki
//   lf%f   → lf0.5      左轮速度环前馈系数Kff（PWM%=Kff×目标速度%）
//   rp%f   → rp0.5      右轮速度PI Kp
//   ri%f   → ri0.02     右轮速度PI Ki
//   rf%f   → rf0.5      右轮速度环前馈系数Kff
//   kp%f   → kp8        转向PD Kp（循迹）
//   kd%f   → kd3        转向PD Kd（循迹）
//   ap%f   → ap2        定角PD Kp（陀螺仪，°误差→差速%）
//   ad%f   → ad0.5      定角PD Kd
//   ya%f   → ya90       定角转向：目标yaw 0~360°
//   yr     → yr         定角转向：锁定当前朝向（直线保持）
//   v%f    → v50        目标速度百分比(0~100)
//   vl/vr  → vl50       独立左右轮目标速度百分比
//   ml/mr  → ml1900     开环PWM绝对值(±5000，不经PID，标定/验证硬件用)
//
// 手动输入也直接敲 lp0.5，不带冒号
// 速度命令自动启动闭环；PID命令只改参数不动车
//   x = 停车

#define CMD_BUF_LEN     48
static char cmd_buf[CMD_BUF_LEN];
static uint8 cmd_idx = 0;

// 开环PWM缓存（ml/mr 命令用，不经PID，直接驱动电机验证硬件）
static int16 open_pwm_l = 0;
static int16 open_pwm_r = 0;

// 手写浮点数解析（不依赖 sscanf %f，支持负号和小数）
static float parse_float(const char *s)
{
    float val = 0.0f, digit_w = 0.1f;
    uint8 neg = 0, in_frac = 0;

    while(*s == ' ' || *s == '\t') s++;
    if(*s == '-') { neg = 1; s++; }
    else if(*s == '+') s++;

    while(1)
    {
        if(*s >= '0' && *s <= '9')
        {
            if(!in_frac) { val = val * 10.0f + (float)(*s - '0'); }
            else         { val += (float)(*s - '0') * digit_w; digit_w *= 0.1f; }
            s++;
        }
        else if(*s == '.' && !in_frac) { in_frac = 1; s++; }
        else break;
    }
    return neg ? -val : val;
}

// 在命令串里找 "key" 后面直接跟数值（VOFA 滑块格式 "kp1.5"）
// key 后必须紧跟数字/负号/小数点，防止 "kp" 匹配到 "kpvalue"
static int get_val(const char *s, const char *key, float *val)
{
    const char *p = strstr(s, key);
    if(p)
    {
        const char *q = p + strlen(key);
        if(*q == '-' || *q == '+' || *q == '.' || (*q >= '0' && *q <= '9'))
        {
            *val = parse_float(q);
            return 1;
        }
    }
    return 0;
}

// 切换显示模式
static void vofa_set_mode(uint8 m)
{
    vofa_mode = m;
    vofa_active = 1;
}

// ==================== 帧率控制：5ms PIT 分频 → 精确 10ms/帧（100Hz） ====================
static volatile uint8 send_due = 0;
static uint8 tick_div = 0;

void vofa_pit_tick(void)        // PIT 5ms 中断里调
{
    if(++tick_div >= 2) { tick_div = 0; send_due = 1; }
}

uint8 vofa_send_due(void)       // 主循环查询：到点清标志并返回1
{
    if(send_due) { send_due = 0; return 1; }
    return 0;
}

static void cmd_exec(char *s)
{
    float f;
    uint8 speed_hit = 0;   // 只有速度命令才自动启动闭环

    // === 单字符命令 ===
    if(s[0] == 'x' && s[1] == '\0')
    {
        control_disable();
        car_stop();
        fan_stop();                   // 停车同时关风扇（安全）
        control_set_target_l(0);    // 清掉独立轮目标，防止下次启动残留
        control_set_target_r(0);
        control_set_track_mode();   // 回到灰度循迹默认模式
        open_pwm_l = 0;             // 清掉开环PWM
        open_pwm_r = 0;
        return;
    }

    // === PID 参数命令：只改参数，不启动闭环（调参时车不会自己跑） ===
    if(get_val(s, "lp", &f)) { control_set_speed_kp_l(f); }
    if(get_val(s, "li", &f)) { control_set_speed_ki_l(f); }
    if(get_val(s, "lf", &f)) { control_set_speed_kff_l(f); }   // 左轮前馈系数
    if(get_val(s, "rp", &f)) { control_set_speed_kp_r(f); }
    if(get_val(s, "ri", &f)) { control_set_speed_ki_r(f); }
    if(get_val(s, "rf", &f)) { control_set_speed_kff_r(f); }   // 右轮前馈系数
    if(get_val(s, "kp", &f)) { control_set_steer_kp(f);  }
    if(get_val(s, "kd", &f)) { control_set_steer_kd(f);  }
    if(get_val(s, "ap", &f)) { control_set_angle_kp(f);  }
    if(get_val(s, "ad", &f)) { control_set_angle_kd(f);  }

    // === 速度命令：设目标 + 自动启动闭环 ===
    if(get_val(s, "v",  &f)) { control_set_base_target((int16)f); speed_hit = 1; }
    if(get_val(s, "vl", &f)) { control_set_target_l((int16)f);    speed_hit = 1; }
    if(get_val(s, "vr", &f)) { control_set_target_r((int16)f);    speed_hit = 1; }

    // === 定角转向：陀螺仪 yaw 闭环（IMU 未初始化时忽略，防止乱转） ===
    if(imu_is_ok())
    {
        if(get_val(s, "ya", &f))   { control_set_yaw_target(f); speed_hit = 1; }
        if(s[0] == 'y' && s[1] == 'r') { control_hold_yaw();    speed_hit = 1; }
    }

    // === 开环PWM：PID全0时用，直接驱动电机（验证电机转向/编码器读数） ===
    if(get_val(s, "ml", &f)) { open_pwm_l = (int16)f; control_disable(); car_set_motor(open_pwm_l, open_pwm_r); }
    if(get_val(s, "mr", &f)) { open_pwm_r = (int16)f; control_disable(); car_set_motor(open_pwm_l, open_pwm_r); }

    // === 负压风扇：fn%f 取值 0~100（油门百分比），ESC 零油门=停机 ===
    if(get_val(s, "fn", &f))
    {
        if(f <= 0.0f) fan_stop();
        else          fan_set((uint8)(f > 100.0f ? 100 : f));
    }

    // === 显示模式切换（收到后才开始发 VOFA 数据） ===
    if(s[0] == 'm' && s[1] == 's')   vofa_set_mode(MODE_SPEED);
    if(s[0] == 'm' && s[1] == 'a')   vofa_set_mode(MODE_IMU);
    if(s[0] == 'm' && s[1] == 'd')   vofa_set_mode(MODE_DIR);
    if(s[0] == 'm' && s[1] == 'e')   vofa_set_mode(MODE_ENCODER);

    if(speed_hit && !control_is_enabled()) control_enable();
}

// 往命令缓冲追加一个字符，遇到换行符就执行一帧
// 返回1表示这帧已执行
static uint8 cmd_feed(char c)
{
    if(c == '\r' || c == '\n')
    {
        if(cmd_idx > 0)
        {
            cmd_buf[cmd_idx] = '\0';
            cmd_exec(cmd_buf);
            cmd_idx = 0;
            return 1;
        }
        return 0;
    }

    if(cmd_idx < CMD_BUF_LEN - 1)
    {
        // 统一转小写（命令大小写不敏感）
        if(c >= 'A' && c <= 'Z') c = c + 32;
        cmd_buf[cmd_idx++] = c;
    }
    else
    {
        cmd_idx = 0;  // 缓冲满，丢弃重来
    }
    return 0;
}

// ==================== 无线串口（UART2） ====================
// RX 中断只做轻量入队，命令解析/执行在主循环 wireless_task 里做
// （printf 等重活放 ISR 会打爆中断栈导致 MCU 复位）

#define WIRELESS_RX_BUF_LEN  64
static volatile uint8 w_rx_buf[WIRELESS_RX_BUF_LEN];
static volatile uint8 w_rx_head = 0;    // ISR 写
static volatile uint8 w_rx_tail = 0;    // 主循环读

void wireless_init(void)
{
    uart_init(WIRELESS_UART, WIRELESS_UART_BAUD, WIRELESS_UART_TX, WIRELESS_UART_RX);
    uart_rx_interrupt(WIRELESS_UART, 1);            // uart_init 末尾默认关闭 RX 中断，必须显式打开，否则收不到命令
}

// UART2 RX 中断调用：轻量入队，满则丢弃最新字节
void wireless_rx_push(uint8 ch)
{
    uint8 next = (uint8)((w_rx_head + 1) % WIRELESS_RX_BUF_LEN);
    if(next != w_rx_tail)
    {
        w_rx_buf[w_rx_head] = ch;
        w_rx_head = next;
    }
}

// 主循环调：把中断队列里的字节喂给命令解析器（安全执行 printf 等重活）
void wireless_task(void)
{
    while(w_rx_tail != w_rx_head)
    {
        uint8 ch = w_rx_buf[w_rx_tail];
        w_rx_tail = (uint8)((w_rx_tail + 1) % WIRELESS_RX_BUF_LEN);
        cmd_feed((char)ch);
    }
}
