#include "track.h"
#include "gray_sensor.h"
#include "param.h"

// 移植自 grayscale(1) 旧版状态机，已适配新版枚举（2026-09-10）
// 状态映射：
//   旧 TRACK_NORMAL  → 新版 TRACK_NORMAL（正常直道/弯道，含十字直走判断）
//   旧 TRACK_CROSS   → 新版 ISLAND_NONE（全丢线十字路口直走，占"非环岛"位）
//   旧 无            → 新版 ISLAND_FOUND（单侧丢线确认，发现环岛）
//   旧 TRACK_ISLAND_ENTER → 新版 ISLAND_ENTER（入环，保持上次偏移）
//   旧 TRACK_ISLAND_RUN   → 新版 ISLAND_IN（环内补线）
//   旧 TRACK_ISLAND_EXIT  → 新版 ISLAND_EXIT（出环屏蔽）
//   旧 无（旧版未实现）  → 新版 ZEBRA_STOP（黑白交替计数识别斑马线，减速/停车后自动恢复）

track_state_enum track_state    = TRACK_NORMAL;
int16 track_offset              = 0;
uint8 track_speed_limit         = SPEED_PCT_FULL;

// 状态机内部变量
static uint8 state_cnt = 0;       // 当前状态持续了多少帧
static int16 last_offset = 0;     // 上一次正常状态的偏移量（环岛内补线用）

// 斑马线黑白交替计数（仅 NORMAL 状态维护）
// 原理：斑马线是黑白条纹交替，压线时读数在"全丢↔有线"之间反复切换；
//       十字路口是连续一段全丢（只有进出2次切换），靠切换次数即可区分。
static uint8 zebra_prev_state = 0;   // 上一帧是否全丢（0=有线 1=全丢）
static uint8 zebra_toggle     = 0;   // 黑白切换累计次数（每压1条黑纹 +2）
static uint16 zebra_gap       = 0;   // 距上次切换的帧数（间隔过长则重新计数，防残留/防误伤）

void track_init(void)
{
    track_state    = TRACK_NORMAL;
    track_offset   = 0;
    track_speed_limit = SPEED_PCT_FULL;
    state_cnt = 0;
    last_offset = 0;
    zebra_prev_state = 0;
    zebra_toggle     = 0;
    zebra_gap        = 0;
}

void track_update(void)
{
    // ========== 先算几个通用标志 ==========
    // 左右两边有没有丢线
    uint8 left_lost  = (gray_pattern & 0x0F) == 0;   // 左边4路全黑
    uint8 right_lost = (gray_pattern & 0xF0) == 0;   // 右边4路全黑
    uint8 both_lost  = left_lost && right_lost;      // 两边都丢线

    // ========== 状态机切换 ==========
    switch(track_state)
    {
        // ========== 1. 正常直道/弯道 ==========
        case TRACK_NORMAL:
        {
            // 正常用传感器给的偏移量；全丢线时先直走（十字路口等状态接管）
            if(!both_lost)
            {
                track_offset = gray_offset;
                last_offset = gray_offset;
            }
            else
            {
                track_offset = 0;
            }

            // 根据偏移大小自动限速
            int16 abs_off = gray_offset > 0 ? gray_offset : -gray_offset;
            if(abs_off > 500)
                track_speed_limit = SPEED_PCT_CURVE;    // 大弯道减速
            else
                track_speed_limit = SPEED_PCT_FULL;     // 直道全速

            // --- 斑马线黑白交替计数 ---
            // 每次状态切换：若与上次切换间隔过长（非连续条纹，如十字/冲出赛道回线）
            // 则重新计数，防残留误判；斑马线条纹交替间隔短，不受影响
            if(zebra_gap < 0xFFFF) zebra_gap++;
            if(both_lost)
            {
                if(zebra_prev_state == 0)       // 有线 → 全丢：一次切换（压上黑纹）
                {
                    if(zebra_toggle > 0 && zebra_gap > ZEBRA_MAX_GAP_FRAMES)
                        zebra_toggle = 0;       // 间隔过长 → 重新计数
                    zebra_toggle++;
                    zebra_prev_state = 1;
                    zebra_gap = 0;
                }
            }
            else
            {
                if(zebra_prev_state == 1)       // 全丢 → 有线：一次切换（压到白纹）
                {
                    if(zebra_toggle > 0 && zebra_gap > ZEBRA_MAX_GAP_FRAMES)
                        zebra_toggle = 0;       // 间隔过长 → 重新计数
                    zebra_toggle++;
                    zebra_prev_state = 0;
                    zebra_gap = 0;
                }
            }

            // --- 状态切换判断 ---
            // 1. 斑马线：当前正压黑纹且交替次数达标
            if(both_lost && zebra_toggle >= ZEBRA_TOGGLE_NUM)
            {
                track_state = ZEBRA_STOP;
                state_cnt = 0;
                zebra_toggle = 0;
                zebra_prev_state = 1;   // 保持"当前全丢"，恢复由 ZEBRA_STOP 接管
                zebra_gap = 0;
            }
            // 2. 十字路口：全丢持续确认帧数，且尚未出现黑白交替（排除斑马线）
            else if(both_lost && gray_loss_left >= LOSS_CONFIRM_FRAMES
                    && gray_loss_right >= LOSS_CONFIRM_FRAMES
                    && zebra_toggle <= 1)
            {
                track_state = ISLAND_NONE;
                state_cnt = 0;
                zebra_toggle = 0;
                zebra_prev_state = 1;
                zebra_gap = 0;
            }
            // 3. 只有左边丢线 → 发现环岛（右边还有线）
            else if(left_lost && !right_lost && gray_loss_left >= LOSS_CONFIRM_FRAMES)
            {
                track_state = ISLAND_FOUND;
                state_cnt = 0;
                zebra_toggle = 0;
                zebra_prev_state = 1;
                zebra_gap = 0;
            }
            // 4. 只有右边丢线 → 发现环岛（左边还有线）
            else if(right_lost && !left_lost && gray_loss_right >= LOSS_CONFIRM_FRAMES)
            {
                track_state = ISLAND_FOUND;
                state_cnt = 0;
                zebra_toggle = 0;
                zebra_prev_state = 1;
                zebra_gap = 0;
            }
            break;
        }

        // ========== 2. 十字路口（占 ISLAND_NONE 位） ==========
        case ISLAND_NONE:
        {
            // 十字路口直走：保持原来的方向，偏移量给0
            track_offset = 0;
            track_speed_limit = SPEED_PCT_CURVE;
            state_cnt++;

            // 走几帧之后，重新看到线了 → 回到正常
            if(!both_lost && gray_pattern != 0 && state_cnt > 10)
            {
                track_state = TRACK_NORMAL;
                state_cnt = 0;
            }
            break;
        }

        // ========== 3. 发现环岛入口 ==========
        case ISLAND_FOUND:
        {
            track_speed_limit = SPEED_PCT_ISLAND;
            state_cnt++;

            // 保持上一次的偏移方向，慢慢转到环道里
            track_offset = last_offset;

            // 几帧之后进入环岛
            if(state_cnt > 8)
            {
                track_state = ISLAND_ENTER;
                state_cnt = 0;
            }
            break;
        }

        // ========== 4. 刚进入环岛 ==========
        case ISLAND_ENTER:
        {
            track_speed_limit = SPEED_PCT_ISLAND;
            state_cnt++;

            // 继续保持上一次的偏移方向，等传感器真正压到环内线
            track_offset = last_offset;

            // 几帧之后进入环岛内正常行驶
            if(state_cnt > 15)
            {
                track_state = ISLAND_IN;
                state_cnt = 0;
            }
            break;
        }

        // ========== 5. 环岛内行驶 ==========
        case ISLAND_IN:
        {
            track_speed_limit = SPEED_PCT_ISLAND;

            // 环内补线：用传感器实际读到的偏移
            // 如果又出现单边丢线，保持上一次的offset不要变
            if(!both_lost && gray_pattern != 0)
            {
                track_offset = gray_offset;
                last_offset = gray_offset;
            }
            else
            {
                track_offset = last_offset;  // 丢线了就保持上次的方向
            }
            state_cnt++;

            // 连续几十帧都能看到线了 → 出环岛
            if(!both_lost && gray_pattern != 0 && state_cnt > 30)
            {
                track_state = ISLAND_EXIT;
                state_cnt = 0;
            }
            break;
        }

        // ========== 6. 出环岛 ==========
        case ISLAND_EXIT:
        {
            track_speed_limit = SPEED_PCT_CURVE;
            track_offset = gray_offset;
            state_cnt++;

            // 屏蔽几十帧，防止刚出环口又误判成入环口
            if(state_cnt > ISLAND_SHIELD_FRAMES)
            {
                track_state = TRACK_NORMAL;
                state_cnt = 0;
            }
            break;
        }

        // ========== 7. 斑马线：减速/停车 + 自动恢复 ==========
        case ZEBRA_STOP:
        {
            // SPEED_PCT_ZEBRA=0 完全停车；>0 低速通过
            track_speed_limit = SPEED_PCT_ZEBRA;
            track_offset = 0;               // 斑马线直行
            state_cnt++;

            // 恢复条件1：最短保持时间后，重新稳定看到线（左右丢线计数已清零）
            if(state_cnt > ZEBRA_MIN_HOLD_FRAMES
               && gray_loss_left == 0 && gray_loss_right == 0)
            {
                track_state = TRACK_NORMAL;
                state_cnt = 0;
                zebra_toggle = 0;
                zebra_prev_state = 0;   // 当前视为有线，重新开始交替积累
                zebra_gap = 0;
            }
            // 恢复条件2：超时兜底，防止停在斑马线黑纹上方永远无法恢复
            else if(state_cnt > ZEBRA_TIMEOUT_FRAMES)
            {
                track_state = TRACK_NORMAL;
                state_cnt = 0;
                zebra_toggle = 0;
                zebra_prev_state = 0;
                zebra_gap = 0;
            }
            break;
        }
    }
}
