#include "vofa.h"
#include "zf_common_debug.h"    // debug_send_buffer

// JustFloat 帧尾（4字节，IEEE754 单精度 1.0f 的小端表示）
static const uint8 vofa_tail[4] = {VOFA_TAIL_0, VOFA_TAIL_1, VOFA_TAIL_2, VOFA_TAIL_3};

void vofa_send(float *ch, uint8 n)
{
    debug_send_buffer((const uint8 *)ch, n * sizeof(float));
    debug_send_buffer(vofa_tail, 4);
}
