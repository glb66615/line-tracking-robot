#ifndef _TRACK_H_
#define _TRACK_H_

#include "zf_common_headfile.h"

typedef enum {
    TRACK_NORMAL,      // 正常直道/弯道（含十字直走判断）
    ISLAND_NONE,       // 十字路口：全丢线直走
    ISLAND_FOUND,      // 发现环岛：单侧丢线确认，保持上次偏移
    ISLAND_ENTER,      // 刚进入环岛
    ISLAND_IN,         // 环岛内行驶（补线）
    ISLAND_EXIT,       // 出环岛（屏蔽防误判）
    ZEBRA_STOP,        // 斑马线：黑白交替计数确认，停车/减速通过后自动恢复
} track_state_enum;

extern track_state_enum track_state;
extern int16 track_offset;      // 最终误差（环岛时含补线）
extern uint8 track_speed_limit; // 限速百分比

void track_init(void);
void track_update(void);

#endif
