#ifndef _GRAY_SENSOR_H_
#define _GRAY_SENSOR_H_

#include "zf_common_headfile.h"
#include "param.h"

// 8路灰度原始值（0~100 归一化）
extern uint8 gray_raw[GRAY_CH_NUM];
// 8位数字位图，bit0=最左 bit7=最右，1=压白线
extern uint8 gray_pattern;
// 加权偏差，范围约 -1000 ~ +1000，负偏右正偏左
extern int16 gray_offset;
// 左/右丢线连续帧数
extern uint8 gray_loss_left, gray_loss_right;

void gray_init(void);
void gray_update(void);

#endif
