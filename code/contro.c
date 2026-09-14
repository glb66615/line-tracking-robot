#include "contro.h"
#include "car.h"
#include "pid.h"
#include "track.h"
#include "imu660.h"
#include "param.h"

// ==================== PID 实例 ====================
static PID_t steer_pid;        // 外环：转向 PD（循迹位置）
static PID_t angle_pid;        // 外环：定角转向 PD（陀螺仪 yaw 反馈）
static PID_t speed_pid_l;      // 内环：左轮速度 PI
static PID_t speed_pid_r;      // 内环：右轮速度 PI

// ==================== 状态缓存（VOFA/屏幕显示用） ====================
static int16 base_target = SPEED_STRAIGHT;  // 统一目标（循迹模式用）
static int16 v_target_l_direct = 0;         // 独立左轮目标（手动模式用）
static int16 v_target_r_direct = 0;         // 独立右轮目标
static uint8 direct_mode = 0;               // 0=循迹(base+Δ)  1=独立目标(vl,vr)
static float yaw_target = 0.0f;             // 定角模式目标偏航角（0~360°）
static float yaw_error = 0.0f;              // 当前角度误差（环形差±180°，VOFA显示用）
static uint8 ctrl_mode = 0;                 // 0=灰度循迹  1=定角转向（陀螺仪）
static int16 v_target_l = 0;
static int16 v_target_r = 0;
static int16 v_actual_l = 0;
static int16 v_actual_r = 0;
static int16 pwm_out_l   = 0;
static int16 pwm_out_r   = 0;
static int16 delta_out   = 0;

// 控制环使能标志：0=手动 1=自动
static uint8 control_run = 0;

// 速度环前馈系数：PWM% = Kff × 目标速度%
// P=I=0时输出纯前馈，可用来标定"目标速度→基础PWM"的映射
static float speed_kff_l = 0.0f;
static float speed_kff_r = 0.0f;

void control_init(void)
{
    pid_init(&steer_pid, STEER_KP, 0.0f, STEER_KD, STEER_OUT_MAX);
    pid_init(&angle_pid, ANGLE_KP, 0.0f, ANGLE_KD, ANGLE_OUT_MAX);
    pid_init(&speed_pid_l, SPEED_KP_L, SPEED_KI_L, 0.0f, SPEED_OUT_MAX);
    pid_init(&speed_pid_r, SPEED_KP_R, SPEED_KI_R, 0.0f, SPEED_OUT_MAX);
    base_target = SPEED_STRAIGHT;
}

void control_enable(void)
{
    // 清零PID历史状态，防止积分冲击
    pid_init(&steer_pid, steer_pid.kp, 0.0f, steer_pid.kd, STEER_OUT_MAX);
    pid_init(&angle_pid, angle_pid.kp, 0.0f, angle_pid.kd, ANGLE_OUT_MAX);
    pid_init(&speed_pid_l, speed_pid_l.kp, speed_pid_l.ki, 0.0f, SPEED_OUT_MAX);
    pid_init(&speed_pid_r, speed_pid_r.kp, speed_pid_r.ki, 0.0f, SPEED_OUT_MAX);
    control_run = 1;
}

void control_disable(void)
{
    control_run = 0;
    // 清掉全部显示缓存，防止停车后屏幕/VOFA残留旧目标、旧PWM
    v_target_l = v_target_r = 0;
    v_actual_l = v_actual_r = 0;
    pwm_out_l = pwm_out_r = 0;
    delta_out = 0;
    car_stop();        // 强制PWM=0，双保险
}
uint8 control_is_enabled(void) { return control_run; }

void control_update(void)
{
    // ============ 外环：转向 PD ============
    float steer;
    if(ctrl_mode)
    {
        // 定角模式：陀螺仪 yaw 反馈（0~360°）
        // 环形差：359°→0° 走 1° 而不是倒转 359°，误差归到 ±180
        float err = yaw_target - imu_get_yaw();
        if(err > 180.0f)  err -= 360.0f;
        if(err < -180.0f) err += 360.0f;
        yaw_error = err;
        steer = ANGLE_SIGN * pid_inc(&angle_pid, err);
    }
    else
    {
        // 循迹模式：灰度偏移反馈
        yaw_error = 0.0f;
        steer = pid_inc(&steer_pid, (float)track_offset);
    }
    delta_out = (int16)steer;

    // ============ 差速合成 ============
    if(direct_mode)
    {
        // 手动模式：用独立左右轮目标
        v_target_l = v_target_l_direct;
        v_target_r = v_target_r_direct;
    }
    else
    {
        // 循迹模式：base_target + 转向PD的差速
        v_target_l = base_target + (int16)steer;
        v_target_r = base_target - (int16)steer;
    }

    // ============ 内环：速度 PI（左右独立） ============
    v_actual_l = car_get_speed_l();
    v_actual_r = car_get_speed_r();

    float e_l = (float)v_target_l - (float)v_actual_l;
    float e_r = (float)v_target_r - (float)v_actual_r;

    // 前馈：基础PWM = Kff × 目标速度，P=I=0时纯前馈用于标定
    float ff_l = speed_kff_l * (float)v_target_l;
    float ff_r = speed_kff_r * (float)v_target_r;

    pwm_out_l = (int16)(ff_l + pid_inc(&speed_pid_l, e_l));   // 百分比 -100~100
    pwm_out_r = (int16)(ff_r + pid_inc(&speed_pid_r, e_r));

    // 百分比 → 绝对PWM占空比（100% = 5000），交给硬件层
    car_set_motor(pwm_out_l * PWM_PCT_SCALE, pwm_out_r * PWM_PCT_SCALE);
}

// ==================== 串口在线调参接口 ====================
// 统一目标（循迹模式用，左右轮自动加差速）
void  control_set_base_target(int16 v) { base_target = v; direct_mode = 0; }
int16 control_get_base_target(void)   { return base_target; }

// 独立左右轮目标（手动调试用，不经过转向PD）
void  control_set_target_l(int16 v) { v_target_l_direct = v; direct_mode = 1; }
void  control_set_target_r(int16 v) { v_target_r_direct = v; direct_mode = 1; }
int16 control_get_target_l(void)    { return v_target_l_direct; }
int16 control_get_target_r(void)    { return v_target_r_direct; }

void control_set_steer_kp(float kp) { steer_pid.kp = kp; }
void control_set_steer_kd(float kd) { steer_pid.kd = kd; }
float control_get_steer_kp(void) { return steer_pid.kp; }
float control_get_steer_kd(void) { return steer_pid.kd; }

void  control_set_speed_kp_l(float kp) { speed_pid_l.kp = kp; }
void  control_set_speed_ki_l(float ki) { speed_pid_l.ki = ki; }
void  control_set_speed_kff_l(float kff) { speed_kff_l = kff; }
void  control_set_speed_kp_r(float kp) { speed_pid_r.kp = kp; }
void  control_set_speed_ki_r(float ki) { speed_pid_r.ki = ki; }
void  control_set_speed_kff_r(float kff) { speed_kff_r = kff; }
float control_get_speed_kp_l(void) { return speed_pid_l.kp; }
float control_get_speed_ki_l(void) { return speed_pid_l.ki; }
float control_get_speed_kff_l(void) { return speed_kff_l; }
float control_get_speed_kp_r(void) { return speed_pid_r.kp; }
float control_get_speed_ki_r(void) { return speed_pid_r.ki; }
float control_get_speed_kff_r(void) { return speed_kff_r; }

// ==================== VOFA/屏幕 监测接口 ====================
int16 control_get_v_target_l(void) { return v_target_l; }
int16 control_get_v_target_r(void) { return v_target_r; }
int16 control_get_delta(void)       { return delta_out; }

// ==================== 定角转向（陀螺仪 yaw 闭环） ====================
void control_set_yaw_target(float yaw)
{
    // 归一化 0~360
    while(yaw >= 360.0f) yaw -= 360.0f;
    while(yaw < 0.0f)    yaw += 360.0f;
    yaw_target = yaw;
    ctrl_mode = 1;
    direct_mode = 0;
}

void control_hold_yaw(void) { control_set_yaw_target(imu_get_yaw()); }  // 锁定当前朝向

void control_set_track_mode(void) { ctrl_mode = 0; yaw_error = 0.0f; }

uint8 control_get_mode(void)       { return ctrl_mode; }
float control_get_yaw_target(void) { return yaw_target; }
float control_get_yaw_error(void)  { return yaw_error; }

void  control_set_angle_kp(float kp) { angle_pid.kp = kp; }
void  control_set_angle_kd(float kd) { angle_pid.kd = kd; }
float control_get_angle_kp(void)     { return angle_pid.kp; }
float control_get_angle_kd(void)     { return angle_pid.kd; }
