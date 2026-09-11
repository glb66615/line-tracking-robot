#ifndef VOFA_H
#define VOFA_H
#include "zf_common_headfile.h"

/* VOFA+ JustFloat 协议输出
 * 发送口: debug UART (UART0 P14_0 TX, 115200)
 * 帧格式: float数组[N] 小端原始字节 + 帧尾 {0x00,0x00,0x80,0x7f}
 */
void vofa_send(float *ch, uint8 n);

/* 串口命令任务：while(1) 里循环调，非阻塞
 * VOFA+ 滑块格式 key%f\r\n，任意组合、大小写不敏感、带不带换行都行，发完自动启动:
 *
 *   lp1.5 li0.3 rp1.5 ri0.3 kp8 kd3 v30
 *
 *   lp/li  左轮速度 PI 的 Kp / Ki
 *   rp/ri  右轮速度 PI 的 Kp / Ki
 *   kp/kd  转向 PD 的 Kp / Kd
 *   v      目标速度 (pulse/5ms)
 *
 *   x = 停车    p = 打印参数
 */
void vofa_task(void);

#endif
