#include "car.h"
#include "param.h"

// 编码器速度缓存（5ms采样一次，单位：每5ms脉冲数）
static int16 car_speed_l = 0;
static int16 car_speed_r = 0;

static void car_gpio_init(void)
{
    gpio_init(MOTOR_LEFT_DIR,  GPO, GPIO_LOW, GPO_PUSH_PULL);
    gpio_init(MOTOR_RIGHT_DIR, GPO, GPIO_LOW, GPO_PUSH_PULL);
}

void car_init(void)
{
    pwm_init(MOTOR_LEFT_PWM,  MOTOR_FREQ, 0);
    pwm_init(MOTOR_RIGHT_PWM, MOTOR_FREQ, 0);
    car_gpio_init();
    // 编码器初始化（单通道计数+方向脚模式）
    encoder_dir_init(ENCODER_LEFT,  ENCODER_LEFT_CH1,  ENCODER_LEFT_CH2);
    encoder_dir_init(ENCODER_RIGHT, ENCODER_RIGHT_CH1, ENCODER_RIGHT_CH2);
}

void car_set_motor(int16 left, int16 right)
{
    // 方向脚：正转=0 反转=1
    if(left  >= 0) { gpio_set_level(MOTOR_LEFT_DIR,  0); }
    else           { gpio_set_level(MOTOR_LEFT_DIR,  1); left  = -left;  }
    if(right >= 0) { gpio_set_level(MOTOR_RIGHT_DIR, 0); }
    else           { gpio_set_level(MOTOR_RIGHT_DIR, 1); right = -right; }

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
    pwm_set_duty(MOTOR_LEFT_PWM,  0);
    pwm_set_duty(MOTOR_RIGHT_PWM, 0);
}

// 在 5ms 控制周期中断里调一次，更新速度缓存
void car_update_speed(void)
{
    car_speed_l = encoder_get_count(ENCODER_LEFT);
    car_speed_r = encoder_get_count(ENCODER_RIGHT);
    encoder_clear_count(ENCODER_LEFT);
    encoder_clear_count(ENCODER_RIGHT);
}

int16 car_get_speed_l(void) { return car_speed_l; }
int16 car_get_speed_r(void) { return car_speed_r; }
