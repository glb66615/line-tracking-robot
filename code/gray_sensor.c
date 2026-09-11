#include "gray_sensor.h"
#include "param.h"
#include "zf_common_debug.h"

// 移植自 grayscale(1) 旧版：完整串口帧解析（2026-09-10）
uint8 gray_raw[GRAY_CH_NUM]   = {0};    // 归一化后的0~100
uint16 gray_adc[GRAY_CH_NUM]  = {0};    // 原始模拟量0~4095
uint8 gray_pattern            = 0;      // 8位数字量
int16 gray_offset             = 0;      // 传感器直接给的偏移量
uint8 gray_loss_left          = 0;
uint8 gray_loss_right         = 0;

// 串口接收状态机
static uint8 rx_buf[24];       // 接收缓冲区
static uint8 rx_cnt = 0;       // 当前收到第几个字节

void gray_init(void)
{
    uart_init(GRAY_UART_NUM, GRAY_UART_BAUD, GRAY_UART_TX, GRAY_UART_RX);
    rx_cnt = 0;
    printf("\r\n[gray] init ok, baud=%d\r\n", GRAY_UART_BAUD);
}

void gray_update(void)
{
    uint8 ch;

    // 不断读串口字节，拼帧
    while(uart_read_byte(GRAY_UART_NUM, &ch) == 1)
    {
        // 找帧头：第一个字节是0x7E，第二个也是0x7E
        if(rx_cnt == 0)
        {
            if(ch == 0x7E) rx_buf[rx_cnt++] = ch;
        }
        else if(rx_cnt == 1)
        {
            if(ch == 0x7E) rx_buf[rx_cnt++] = ch;
            else rx_cnt = 0;  // 不对，重新找
        }
        else
        {
            rx_buf[rx_cnt++] = ch;

            // 收满24个字节了
            if(rx_cnt >= 24)
            {
                // 检查帧尾
                if(rx_buf[22] == 0xE7 && rx_buf[23] == 0xE7)
                {
                    // ===== 解析数据 =====
                    // 1. 偏移量：第3、4字节，有符号16位小端
                    int16 offset = (int16)(rx_buf[3] | (rx_buf[4] << 8));
                    gray_offset = offset;

                    // 2. 数字量：第5字节
                    gray_pattern = rx_buf[5];

                    // 3. 8路模拟量：第6~21字节，每路2字节小端
                    for(int i=0; i<8; i++)
                    {
                        gray_adc[i] = rx_buf[6 + i*2] | (rx_buf[6 + i*2 + 1] << 8);
                        // 归一化到0~100：4095 → 100
                        gray_raw[i] = (uint8)(gray_adc[i] * 100 / 4095);
                    }

                    // 4. 统计左右丢线帧数
                    // 左边4路（bit0~bit3）全没压到线
                    if((gray_pattern & 0x0F) == 0)
                    {
                        if(gray_loss_left < 255) gray_loss_left++;
                    }
                    else
                    {
                        gray_loss_left = 0;
                    }

                    // 右边4路（bit4~bit7）全没压到线
                    if((gray_pattern & 0xF0) == 0)
                    {
                        if(gray_loss_right < 255) gray_loss_right++;
                    }
                    else
                    {
                        gray_loss_right = 0;
                    }
                }

                rx_cnt = 0;  // 不管对不对，重新开始收下一帧
            }
        }
    }
}
