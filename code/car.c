#include "car.h"
#include "param.h"
#include "zf_common_debug.h"   // debug_send_buffer

// 编码器速度缓存（5ms采样一次，单位：每5ms脉冲数）
static int16 car_speed_l = 0;
static int16 car_speed_r = 0;

// 目标速度缓存（car_set_motor 设置的 PWM 值，带符号）
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

    // 死区
    if(left  < MOTOR_DEADZONE)  left  = 0;
    if(right < MOTOR_DEADZONE)  right = 0;

    // 限幅
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
    car_speed_l = -encoder_get_count(ENCODER_LEFT);
    car_speed_r =  encoder_get_count(ENCODER_RIGHT);
    encoder_clear_count(ENCODER_LEFT);
    encoder_clear_count(ENCODER_RIGHT);
}

int16 car_get_speed_l(void) { return car_speed_l; }
int16 car_get_speed_r(void) { return car_speed_r; }
int16 car_get_target_l(void) { return car_target_l; }
int16 car_get_target_r(void) { return car_target_r; }
