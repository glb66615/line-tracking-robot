#include "fan.h"
#include "param.h"
#include "zf_common_debug.h"

// ==================== 负压风扇（无刷 ESC，50Hz PWM 伺服协议） ====================
static uint16 fan_duty_cur = FAN_DUTY_STOP;   // 当前实际 duty
static uint16 fan_duty_target = FAN_DUTY_STOP; // 目标 duty

void fan_init(void)
{
    pwm_init(FAN_PWM, FAN_FREQ, FAN_DUTY_STOP);   // 上电即 1.0ms 零油门
    fan_duty_cur = FAN_DUTY_STOP;
    fan_duty_target = FAN_DUTY_STOP;
    // 按键初始化：输入上拉，按下接地→低电平
    gpio_init(FAN_KEY_OPEN,  GPI, GPIO_HIGH, GPI_PULL_UP);
    gpio_init(FAN_KEY_CLOSE, GPI, GPIO_HIGH, GPI_PULL_UP);
    printf("\r\n[fan] init ok (50Hz, P21_2, keys P20_6/P20_7)\r\n");
}

// 设风扇目标油门百分比（0~100），实际 PWM 由 fan_update 渐变到目标
void fan_set(uint8 duty_pct)
{
    if(duty_pct > 100) duty_pct = 100;
    // 0% → FAN_DUTY_MIN(500)，100% → FAN_DUTY_MAX(1000)
    fan_duty_target = FAN_DUTY_MIN + (uint16)duty_pct * (FAN_DUTY_MAX - FAN_DUTY_MIN) / 100;
}

// 每帧调一次（主循环），把当前 duty 渐变到目标，避免瞬间大电流冲击
// 每步 ±8 duty ≈ 0.16ms，500→1000 约 62 步 ≈ 310ms（主循环 10ms）
void fan_update(void)
{
    if(fan_duty_cur < fan_duty_target)
    {
        fan_duty_cur += 8;
        if(fan_duty_cur > fan_duty_target) fan_duty_cur = fan_duty_target;
    }
    else if(fan_duty_cur > fan_duty_target)
    {
        fan_duty_cur -= 8;
        if(fan_duty_cur < fan_duty_target) fan_duty_cur = fan_duty_target;
    }
    pwm_set_duty(FAN_PWM, fan_duty_cur);
}

void fan_stop(void)
{
    fan_set(0);   // 走渐变停机，避免急刹电流冲击
}

// ==================== 按键检测（两个独立按键：开/停） ====================
// 在主循环里调，每 10ms 调一次，消抖 20ms
static uint16 key_open_cnt = 0;     // 开键按下持续计数（10ms/步）
static uint16 key_close_cnt = 0;    // 停键按下持续计数

void fan_key_update(void)
{
    uint8 open  = gpio_get_level(FAN_KEY_OPEN);    // 1=没按，0=按下
    uint8 close = gpio_get_level(FAN_KEY_CLOSE);

    // 开键：按下且持续 >= 20ms → 开风扇
    if(open == 0)
    {
        if(key_open_cnt < 65535) key_open_cnt++;
        if(key_open_cnt >= FAN_KEY_DEBOUNCE_MS / 10)
        {
            fan_set(FAN_ON_DUTY_PCT);
            key_open_cnt = 0;   // 清零，松手后才能再次触发
        }
    }
    else
    {
        key_open_cnt = 0;
    }

    // 停键：按下且持续 >= 20ms → 停风扇
    if(close == 0)
    {
        if(key_close_cnt < 65535) key_close_cnt++;
        if(key_close_cnt >= FAN_KEY_DEBOUNCE_MS / 10)
        {
            fan_stop();
            key_close_cnt = 0;
        }
    }
    else
    {
        key_close_cnt = 0;
    }
}
