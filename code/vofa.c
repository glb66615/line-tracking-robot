#include "vofa.h"
#include "param.h"
#include "car.h"
#include "contro.h"
#include "stdio.h"
#include "string.h"
#include "zf_common_debug.h"
#include "zf_driver_uart.h"

// JustFloat 帧尾
static const uint8 vofa_tail[4] = {0x00, 0x00, 0x80, 0x7f};

void vofa_send(float *ch, uint8 n)
{
    uart_write_buffer(UART_0, (const uint8 *)ch, n * sizeof(float));
    uart_write_buffer(UART_0, vofa_tail, 4);
}

// ==================== 行缓冲串口命令解析 ====================
// 收到 '\r' 或 '\n' 视为一行结束，解析整串命令
//
// 命令表（VOFA发送框输入整串，发送时带换行 \n）:
//   键值对（可任意组合，空格分隔，只更新出现的项）:
//     lp:1.5 li:0.3     左轮速度 PI: Kp Ki
//     rp:1.5 ri:0.3     右轮速度 PI: Kp Ki
//     kp:8  kd:3        转向 PD:    Kp Kd
//     v:30              目标速度 (pulse/5ms)
//   示例: lp:1.5 li:0.3 v:30
//
//   单字符:
//     c    启动 AUTO（速度+转向闭环）
//     x    停车回 MANUAL
//     p    打印当前全部参数
#define CMD_BUF_LEN     48
static char cmd_buf[CMD_BUF_LEN];
static uint8 cmd_idx = 0;

// 从命令串里找 "key:" 后面的浮点数，找到返回1
static int get_val(const char *s, const char *key, float *val)
{
    const char *p = strstr(s, key);
    if(p && sscanf(p + strlen(key), "%f", val) == 1)
        return 1;
    return 0;
}

static void cmd_exec(char *s)
{
    // 单字符命令
    if(s[0] == 'c' && s[1] == '\0')
    {
        control_enable();
        printf("[MODE] AUTO\r\n");
        return;
    }
    if(s[0] == 'x' && s[1] == '\0')
    {
        control_disable();
        car_stop();
        printf("[MODE] MANUAL / STOP\r\n");
        return;
    }
    if(s[0] == 'p' && s[1] == '\0')
    {
        printf("===== PARAMS =====\r\n");
        printf("mode  : %s\r\n", control_is_enabled() ? "AUTO" : "MANUAL");
        printf("target: %d pulse/5ms\r\n", control_get_base_target());
        printf("L-PI  : Kp=%.2f Ki=%.2f\r\n", control_get_speed_kp_l(), control_get_speed_ki_l());
        printf("R-PI  : Kp=%.2f Ki=%.2f\r\n", control_get_speed_kp_r(), control_get_speed_ki_r());
        printf("STEER : Kp=%.2f Kd=%.2f\r\n", control_get_steer_kp(), control_get_steer_kd());
        printf("==================\r\n");
        return;
    }

    // 键值对命令（任意组合）
    float f;
    uint8 hit = 0;
    if(get_val(s, "lp:", &f)) { control_set_speed_kp_l(f); hit = 1; }
    if(get_val(s, "li:", &f)) { control_set_speed_ki_l(f); hit = 1; }
    if(get_val(s, "rp:", &f)) { control_set_speed_kp_r(f); hit = 1; }
    if(get_val(s, "ri:", &f)) { control_set_speed_ki_r(f); hit = 1; }
    if(get_val(s, "kp:", &f)) { control_set_steer_kp(f);  hit = 1; }
    if(get_val(s, "kd:", &f)) { control_set_steer_kd(f);  hit = 1; }
    if(get_val(s, "v:",  &f)) { control_set_base_target((int16)f); hit = 1; }

    if(hit)
    {
        printf("[OK] L-PI Kp=%.2f Ki=%.2f | R-PI Kp=%.2f Ki=%.2f | STEER Kp=%.2f Kd=%.2f | v=%d\r\n",
               control_get_speed_kp_l(), control_get_speed_ki_l(),
               control_get_speed_kp_r(), control_get_speed_ki_r(),
               control_get_steer_kp(), control_get_steer_kd(),
               control_get_base_target());
    }
    else
    {
        printf("[ERR] unknown cmd: %s\r\n", s);
    }
}

/* 在 while(1) 里循环调，非阻塞，逐字节收满一行后解析 */
void vofa_task(void)
{
    uint8 ch;
    while(debug_read_ring_buffer(&ch, 1) == 1)
    {
        if(ch == '\r' || ch == '\n')
        {
            if(cmd_idx > 0)
            {
                cmd_buf[cmd_idx] = '\0';
                cmd_exec(cmd_buf);
                cmd_idx = 0;
            }
        }
        else if(cmd_idx < CMD_BUF_LEN - 1)
        {
            cmd_buf[cmd_idx++] = (char)ch;
        }
        else  // 缓冲满，丢弃重来
        {
            cmd_idx = 0;
        }
    }
}
