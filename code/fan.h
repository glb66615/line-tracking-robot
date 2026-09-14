#ifndef _FAN_H_
#define _FAN_H_

#include "zf_common_headfile.h"

// ==================== 负压风扇（无刷 ESC，50Hz PWM 伺服协议） ====================
void fan_init(void);                                // 初始化风扇 PWM + 按键 GPIO
void fan_set(uint8 duty_pct);                       // 设置风扇油门目标：0~100（实际 PWM 渐变到目标）
void fan_update(void);                              // 每帧调一次（主循环），渐变推进
void fan_stop(void);                                // 风扇停机（渐变到零油门）
void fan_key_update(void);                          // 按键检测（主循环调，20ms消抖）

#endif
