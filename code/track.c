#include "track.h"
#include "gray_sensor.h"
#include "param.h"

track_state_enum track_state    = TRACK_NORMAL;
int16 track_offset              = 0;
uint8 track_speed_limit         = SPEED_STRAIGHT;

void track_init(void)
{
    track_state    = TRACK_NORMAL;
    track_offset   = 0;
    track_speed_limit = SPEED_STRAIGHT;
}

void track_update(void)
{
    // TODO: 状态机逻辑
    // TRACK_NORMAL:  gray_offset 原样传递，按弯道路径限速
    // ISLAND_*:      单侧丢线≥N帧触发，环岛内用虚拟补线误差
    // ZEBRA_STOP:    连续全黑≥N帧触发，停车
    track_offset = gray_offset;
}
