#ifndef _TRACK_H_
#define _TRACK_H_

#include "zf_common_headfile.h"

typedef enum {
    TRACK_NORMAL,
    ISLAND_NONE,
    ISLAND_FOUND,
    ISLAND_ENTER,
    ISLAND_IN,
    ISLAND_EXIT,
    ZEBRA_STOP,
} track_state_enum;

extern track_state_enum track_state;
extern int16 track_offset;      // 最终误差（环岛时含补线）
extern uint8 track_speed_limit; // 限速百分比

void track_init(void);
void track_update(void);

#endif
