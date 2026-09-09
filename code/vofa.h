#ifndef VOFA_H
#define VOFA_H
#include "zf_common_headfile.h"

/* VOFA+ JustFloat 协议输出
 * 协议格式: float数组[N] + 帧尾 {0x00, 0x00, 0x80, 0x7f}
 * 小端浮点，TC264 TriCore 本身小端，直接 memcpy 即可
 * 发送口: debug UART (UART0 P14_0 TX, 115200)
 *
 * 解耦: 本模块只负责「给啥发啥」，不知道数据来自 track/car/pid 谁
 *       调用方自行组好 float 数组再传进来
 */

// 帧尾（JustFloat 协议固定）
#define VOFA_TAIL_0    0x00
#define VOFA_TAIL_1    0x00
#define VOFA_TAIL_2    0x80
#define VOFA_TAIL_3    0x7f

/* 发送 JustFloat 数据帧
 * 参数: ch   = float 数组指针（小端，按通道顺序）
 *       n    = 通道数
 * 帧格式: ch[0] ch[1] ... ch[n-1] + {0,0,0x80,0x7f}
 *
 * VOFA+ 那边选 JustFloat 协议，CH_COUNT 设成 n 即可
 */
void vofa_send(float *ch, uint8 n);

#endif
