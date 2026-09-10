#include "contro.h"
#include "car.h"
#include "pid.h"
#include "track.h"
#include "param.h"

// ==================== PID 实例 ====================
static PID_t steer_pid;        // 外环：转向 PD（循迹位置）
static PID_t speed_pid_l;      // 内环：左轮速度 PI
static PID_t speed_pid_r;      // 内环：右轮速度 PI

// ==================== 状态缓存（VOFA/屏幕显示用） ====================
static int16 base_target = SPEED_STRAIGHT;  // 串口设定的目标速度
static int16 v_target_l = 0;
static int16 v_target_r = 0;
static int16 v_actual_l = 0;
static int16 v_actual_r = 0;
static int16 pwm_out_l   = 0;
static int16 pwm_out_r   = 0;
static int16 delta_out   = 0;

// 控制环使能标志：0=手动 1=自动
static uint8 control_run = 0;

void control_init(void)
{
    pid_init(&steer_pid, STEER_KP, 0.0f, STEER_KD, STEER_OUT_MAX);
    pid_init(&speed_pid_l, SPEED_KP_L, SPEED_KI_L, 0.0f, SPEED_OUT_MAX);
    pid_init(&speed_pid_r, SPEED_KP_R, SPEED_KI_R, 0.0f, SPEED_OUT_MAX);
    base_target = SPEED_STRAIGHT;
}

void control_enable(void)
{
    // 清零PID历史状态，防止积分冲击
    pid_init(&steer_pid, steer_pid.kp, 0.0f, steer_pid.kd, STEER_OUT_MAX);
    pid_init(&speed_pid_l, speed_pid_l.kp, speed_pid_l.ki, 0.0f, SPEED_OUT_MAX);
    pid_init(&speed_pid_r, speed_pid_r.kp, speed_pid_r.ki, 0.0f, SPEED_OUT_MAX);
    control_run = 1;
}

void control_disable(void) { control_run = 0; }
uint8 control_is_enabled(void) { return control_run; }

void control_update(void)
{
    // ============ 外环：转向 PD ============
    float error = (float)track_offset;
    float steer = pid_inc(&steer_pid, error);
    delta_out = (int16)steer;

    // ============ 差速合成 ============
    v_target_l = base_target + (int16)steer;
    v_target_r = base_target - (int16)steer;

    // ============ 内环：速度 PI（左右独立） ============
    v_actual_l = car_get_speed_l();
    v_actual_r = car_get_speed_r();

    float e_l = (float)v_target_l - (float)v_actual_l;
    float e_r = (float)v_target_r - (float)v_actual_r;

    pwm_out_l = (int16)pid_inc(&speed_pid_l, e_l);
    pwm_out_r = (int16)pid_inc(&speed_pid_r, e_r);

    car_set_motor(pwm_out_l, pwm_out_r);
}

// ==================== 串口在线调参接口 ====================
void  control_set_base_target(int16 v) { base_target = v; }
int16 control_get_base_target(void)   { return base_target; }

void control_set_steer_kp(float kp) { steer_pid.kp = kp; }
void control_set_steer_kd(float kd) { steer_pid.kd = kd; }
float control_get_steer_kp(void) { return steer_pid.kp; }
float control_get_steer_kd(void) { return steer_pid.kd; }

void  control_set_speed_kp_l(float kp) { speed_pid_l.kp = kp; }
void  control_set_speed_ki_l(float ki) { speed_pid_l.ki = ki; }
void  control_set_speed_kp_r(float kp) { speed_pid_r.kp = kp; }
void  control_set_speed_ki_r(float ki) { speed_pid_r.ki = ki; }
float control_get_speed_kp_l(void) { return speed_pid_l.kp; }
float control_get_speed_ki_l(void) { return speed_pid_l.ki; }
float control_get_speed_kp_r(void) { return speed_pid_r.kp; }
float control_get_speed_ki_r(void) { return speed_pid_r.ki; }

// ==================== VOFA 监测接口 ====================
int16 control_get_v_target_l(void) { return v_target_l; }
int16 control_get_v_target_r(void) { return v_target_r; }
int16 control_get_v_actual_l(void) { return v_actual_l; }
int16 control_get_v_actual_r(void) { return v_actual_r; }
int16 control_get_pwm_l(void)       { return pwm_out_l; }
int16 control_get_pwm_r(void)       { return pwm_out_r; }
int16 control_get_delta(void)       { return delta_out; }
