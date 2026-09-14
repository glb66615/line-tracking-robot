#include "car.h"
#include "param.h"
#include "zf_common_debug.h"   // debug_send_buffer

// 编码器速度缓存（5ms采样一次）
static int16 car_speed_l = 0;      // 归一化后（转速百分比 0~100，给PID/显示）
static int16 car_speed_r = 0;
static int16 car_raw_l = 0;        // 原始脉冲数/5ms（底层调试用，未缩放）
static int16 car_raw_r = 0;

// 编码器累积脉冲（从上电开始累加，用于测距/里程）
static int32 car_accum_l = 0;
static int32 car_accum_r = 0;

// 编码器一阶低通滤波（平滑脉冲读数，alpha越大越平滑但滞后越大）
static int32 raw_l_filtered = 0;
static int32 raw_r_filtered = 0;
#define ENC_FILTER_ALPHA    4   // 0~16，4≈0.25（25%新值+75%旧值）

// 目标速度缓存（car_set_motor 设置的 PWM 绝对值，带符号；显示时÷PWM_PCT_SCALE）
static int16 car_target_l = 0;
static int16 car_target_r = 0;

// 初始化状态：0=未初始化 1=成功
static uint8 car_ready = 0;

static void car_gpio_init(void)
{
    gpio_init(MOTOR_LEFT_DIR,  GPO, GPIO_HIGH, GPO_PUSH_PULL);
    gpio_init(MOTOR_RIGHT_DIR, GPO, GPIO_HIGH, GPO_PUSH_PULL);
}

void car_init(void)
{
    pwm_init(MOTOR_LEFT_PWM,  MOTOR_FREQ, 0);
    pwm_init(MOTOR_RIGHT_PWM, MOTOR_FREQ, 0);
    car_gpio_init();
    // 编码器初始化（单通道计数+方向脚模式，无硬件4倍频）
    encoder_dir_init(ENCODER_LEFT,  ENCODER_LEFT_CH1,  ENCODER_LEFT_CH2);
    encoder_dir_init(ENCODER_RIGHT, ENCODER_RIGHT_CH1, ENCODER_RIGHT_CH2);

    // 上电确保电机停止（DIR 默认 HIGH，但 PWM=0 所以不会转）
    car_stop();
    encoder_clear_count(ENCODER_LEFT);
    encoder_clear_count(ENCODER_RIGHT);
    car_ready = 1;
    printf("\r\n[car] init ok\r\n");
}

uint8 car_is_ready(void) { return car_ready; }

void car_set_motor(int16 left, int16 right)
{
    // 保存目标值（带符号，VOFA 显示用）
    car_target_l = left;
    car_target_r = right;

    // DIR=HIGH 正转，DIR=LOW 反转（DRV8701E 驱动板标准逻辑）
    if(left  >= 0) { gpio_set_level(MOTOR_LEFT_DIR,  GPIO_HIGH); }
    else           { gpio_set_level(MOTOR_LEFT_DIR,  GPIO_LOW);  left  = -left;  }
    if(right >= 0) { gpio_set_level(MOTOR_RIGHT_DIR, GPIO_HIGH); }
    else           { gpio_set_level(MOTOR_RIGHT_DIR, GPIO_LOW);  right = -right; }

    // 无死区补偿：PWM连续输出，避免小信号时0↔死区跳变造成极限环振荡
    // 电机自然死区(PWM<1200不动)由前馈Kff托住正常速度，低速不响应可接受

    // 限幅（防超PWM满量程）
    if(left  > MOTOR_DUTY_MAX)  left  = MOTOR_DUTY_MAX;
    if(right > MOTOR_DUTY_MAX)  right = MOTOR_DUTY_MAX;

    pwm_set_duty(MOTOR_LEFT_PWM,  left);
    pwm_set_duty(MOTOR_RIGHT_PWM, right);
}

void car_stop(void)
{
    car_target_l = 0;
    car_target_r = 0;
    pwm_set_duty(MOTOR_LEFT_PWM,  0);
    pwm_set_duty(MOTOR_RIGHT_PWM, 0);
}

// 在 5ms 控制周期中断里调一次，更新速度缓存
void car_update_speed(void)
{
    // 左轮编码器(TIM4)实测前进读数为负，取反统一为"前进为正，后退为负"
    int32 raw_l = -encoder_get_count(ENCODER_LEFT);
    int32 raw_r =  encoder_get_count(ENCODER_RIGHT);
    car_raw_l = (int16)raw_l;   // 原始值存给VOFA底层观察
    car_raw_r = (int16)raw_r;

    // 累积脉冲（前进为正，后退为负）
    car_accum_l += raw_l;
    car_accum_r += raw_r;

    // 一阶低通滤波：filtered += alpha * (raw - filtered) / 16
    // 平滑脉冲跳变，让速度读数更稳定
    raw_l_filtered += (raw_l - raw_l_filtered) * ENC_FILTER_ALPHA / 16;
    raw_r_filtered += (raw_r - raw_r_filtered) * ENC_FILTER_ALPHA / 16;

    // 归一化到 0~SPEED_FULL_SCALE（百分比 0~100），用int32中间值防异常脉冲溢出
    int32 s_l = raw_l_filtered * SPEED_FULL_SCALE / SPEED_RAW_FULLSCALE;
    int32 s_r = raw_r_filtered * SPEED_FULL_SCALE / SPEED_RAW_FULLSCALE;
    // 限幅：允许±150%的瞬时过冲，但防止异常脉冲把int16冲爆
    if(s_l >  150) s_l =  150;
    if(s_l < -150) s_l = -150;
    if(s_r >  150) s_r =  150;
    if(s_r < -150) s_r = -150;

    car_speed_l = (int16)s_l;
    car_speed_r = (int16)s_r;
    encoder_clear_count(ENCODER_LEFT);
    encoder_clear_count(ENCODER_RIGHT);
}

int16 car_get_speed_l(void) { return car_speed_l; }
int16 car_get_speed_r(void) { return car_speed_r; }
int16 car_get_raw_l(void)   { return car_raw_l; }
int16 car_get_raw_r(void)   { return car_raw_r; }
int16 car_get_target_l(void) { return car_target_l; }
int16 car_get_target_r(void) { return car_target_r; }
int32 car_get_accum_l(void)  { return car_accum_l; }
int32 car_get_accum_r(void)  { return car_accum_r; }
