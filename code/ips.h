#ifndef _IPS_H_
#define _IPS_H_

#include "zf_common_headfile.h"

void ips_init(void);
void ips_show_speed(void);      // 显示左右轮速度（内部调用 car_get_speed）

#endif
