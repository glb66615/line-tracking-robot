#ifndef _IPS_H_
#define _IPS_H_

#include "zf_common_headfile.h"

void ips_init(void);                    // 屏幕初始化 + 启动画面
void ips_show_status(void);             // 显示 car 自检结果（car_init 之后调）
void ips_show_speed(void);              // 刷新左右轮速度

#endif
