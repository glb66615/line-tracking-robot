#include "gray_sensor.h"
#include "param.h"

uint8 gray_raw[GRAY_CH_NUM]   = {0};
uint8 gray_pattern            = 0;
int16 gray_offset             = 0;
uint8 gray_loss_left          = 0;
uint8 gray_loss_right         = 0;

void gray_init(void)
{
    uart_init(GRAY_UART_NUM, GRAY_UART_BAUD, GRAY_UART_TX, GRAY_UART_RX);
    // TODO: 串口协议未知，待补充接收解析逻辑
}

void gray_update(void)
{
    // TODO: 从串口接收一帧8路数据 → 填入 gray_raw[]
    //       每路归一化到 0~100，高于 GRAY_TH_WHITE 置 1
    //       按 {4,3,2,1,-1,-2,-3,-4} 加权求和得 gray_offset
    //       统计单侧连续丢线帧数 gray_loss_left/right
}
