#include "vofa.h"
#include "param.h"
#include "car.h"
#include "contro.h"
#include "stdio.h"
#include "string.h"
#include "zf_common_debug.h"
#include "zf_common_function.h"
#include "zf_driver_uart.h"

// JustFloat 帧尾
static const uint8 vofa_tail[4] = {0x00, 0x00, 0x80, 0x7f};

void vofa_send(float *ch, uint8 n)
{
    uart_write_buffer(UART_0, (const uint8 *)ch, n * sizeof(float));
    uart_write_buffer(UART_0, vofa_tail, 4);
}

// ==================== 串口命令解析（VOFA+ 滑块格式） ====================
//
// 统一格式: key%f\r\n  → VOFA 命令控件自动替换 %f 为滑块值
//
//   lp%f   → lp1.5      左轮速度PI Kp
//   li%f   → li0.3      左轮速度PI Ki
//   rp%f   → rp1.5      右轮速度PI Kp
//   ri%f   → ri0.3      右轮速度PI Ki
//   kp%f   → kp8        转向PD Kp
//   kd%f   → kd3        转向PD Kd
//   v%f    → v30        目标速度(pulse/5ms)
//
// 手动输入也直接敲 lp1.5，不带冒号
// 发完自动启动闭环
//   x = 停车    p = 打印当前全部参数

#define CMD_BUF_LEN     48
static char cmd_buf[CMD_BUF_LEN];
static uint8 cmd_idx = 0;

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

static void cmd_exec(char *s)
{
    float f;
    uint8 hit = 0;

    // === 单字符命令 ===
    if(s[0] == 'x' && s[1] == '\0')
    {
        control_disable();
        car_stop();
        return;
    }

    // === VOFA 滑块格式 ===
    if(get_val(s, "lp", &f)) { control_set_speed_kp_l(f); hit = 1; }
    if(get_val(s, "li", &f)) { control_set_speed_ki_l(f); hit = 1; }
    if(get_val(s, "rp", &f)) { control_set_speed_kp_r(f); hit = 1; }
    if(get_val(s, "ri", &f)) { control_set_speed_ki_r(f); hit = 1; }
    if(get_val(s, "kp", &f)) { control_set_steer_kp(f);  hit = 1; }
    if(get_val(s, "kd", &f)) { control_set_steer_kd(f);  hit = 1; }
    if(get_val(s, "v",  &f)) { control_set_base_target((int16)f); hit = 1; }
    if(get_val(s, "vl", &f)) { control_set_target_l((int16)f); hit = 1; }
    if(get_val(s, "vr", &f)) { control_set_target_r((int16)f); hit = 1; }

    if(hit && !control_is_enabled()) control_enable();
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

/* 在 while(1) 里循环调，非阻塞
 * 不依赖发送方带换行符：收完一段后短暂等待确认无后续字节，就当作一帧执行
 */
void vofa_task(void)
{
    uint8 ch;
    uint8 got = 0;

    // 先把缓冲区已有的字节全部读走
    while(debug_read_ring_buffer(&ch, 1) == 1)
    {
        if(cmd_feed((char)ch)) return;
        got = 1;
    }

    // 收到过内容但没遇到换行符 → 轮询等待尾部字节（发送方可能不带 \n）
    if(got && cmd_idx > 0)
    {
        uint8 retry;
        for(retry = 0; retry < 5; retry++)
        {
            func_soft_delay(20000);      // 短暂空转等字节
            while(debug_read_ring_buffer(&ch, 1) == 1)
            {
                if(cmd_feed((char)ch)) return;
            }
        }
        // 等了几轮都没有新字节，认定一帧结束，强制执行
        cmd_buf[cmd_idx] = '\0';
        cmd_exec(cmd_buf);
        cmd_idx = 0;
    }
}
