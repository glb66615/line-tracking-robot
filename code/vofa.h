#ifndef VOFA_H
#define VOFA_H
#include "zf_common_headfile.h"

/* VOFA+ JustFloat 协议输出
 * 发送口: 无线 UART2 (P10_5 TX, 115200)
 * 帧格式: float数组[N] 小端原始字节 + 帧尾 {0x00,0x00,0x80,0x7f}
 */
void vofa_send(float *ch, uint8 n);

/* 无线串口初始化（UART2 P10_5/P10_6，VOFA+ 无线调试用）
 * 必须最先调用：printf 已重定向到 UART2，初始化前不能有任何 printf */
void wireless_init(void);

/* 无线串口命令任务：while(1) 里循环调，非阻塞，唯一命令入口 */
void wireless_task(void);

/* VOFA 显示模式：0=速度环 1=姿态IMU 2=方向环 */
uint8 vofa_get_mode(void);

/* 数据发送使能：收到 ms/ma/md 后才为1，上电默认0（不发数据） */
uint8 vofa_is_active(void);

/* PIT 5ms 中断调用：分频出精确 10ms 发送节拍（100Hz 帧率） */
void vofa_pit_tick(void);

/* 主循环查询发送节拍：到点返回1并自动清标志 */
uint8 vofa_send_due(void);

/* UART2 RX 中断调用：只做轻量入队（命令执行在主循环 wireless_task） */
void wireless_rx_push(uint8 ch);

#endif
