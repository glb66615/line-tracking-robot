#ifndef VOFA_H
#define VOFA_H
#include "zf_common_headfile.h"

/* VOFA+ JustFloat 协议输出
 * 发送口: debug UART (UART0 P14_0 TX, 115200)
 * 帧格式: float数组[N] 小端原始字节 + 帧尾 {0x00,0x00,0x80,0x7f}
 */
void vofa_send(float *ch, uint8 n);

/* 串口命令任务：while(1) 里循环调，非阻塞
 * 整行命令（发送时带换行 \n），键值对可任意组合:
 *
 *   lp:1.5 li:0.3     左轮速度 PI: Kp Ki
 *   rp:1.5 ri:0.3     右轮速度 PI: Kp Ki
 *   kp:8  kd:3        转向 PD:    Kp Kd
 *   v:30              目标速度 (pulse/5ms)
 *   示例: lp:1.5 li:0.3 v:30
 *
 *   c    启动 AUTO    x = 停车    p = 打印参数
 */
void vofa_task(void);

#endif
