#include "gray_sensor.h"
#include "param.h"
#include <stdbool.h>

uint16 gray_raw[GRAY_CH_NUM]   = {0};
uint8 gray_pattern            = 0;
int16 gray_offset             = 0;
uint8 gray_loss_left          = 0;
uint8 gray_loss_right         = 0;

//串口收发缓冲区
static uint8 rx_buffer[24];//收发24字节
static uint8 rx_index = 0;//收发字节数
static uint8 frame_ready = 0;

void gray_uart_callback(void)//串口接收
{
    rx_buffer[rx_index++] = uart_read_byte(GRAY_UART_NUM);
    if(rx_index >= 24)
    {
       rx_index = 0;
       frame_ready = 1;

    }

}

void gray_init(void)
{
    uart_init(GRAY_UART_NUM, GRAY_UART_BAUD, GRAY_UART_TX, GRAY_UART_RX);
    //串口中断回调函数
    set_wireless_type(GRAY_UART_NUM,gray_uart_callback );
    // TODO: 串口协议未知，待补充接收解析逻辑
}

void gray_update(void)
{
    uint8 format; //放模拟量，数字量，偏移量

    bool is_offset,is_digital,is_analog;//判断是否为三个值

    uint8 idx;//位置索引

    if(!frame_ready)return;
    frame_ready = 0;


    if (rx_buffer[0] != 0x7E || rx_buffer[1] != 0x7E) return;
    //判断函数
    format     = rx_buffer[2] & 0x0F;
    is_offset  = format & 0x1;
    is_digital = format & 0x2;
    is_analog  = format & 0x4;
    //解析模拟量
    idx = 3;
    if(is_offset)
   {
      gray_offset = *(int16*)(&rx_buffer[idx]);
      idx += 2;
   }
    //解析数字量
    if (is_digital)
   {
      gray_pattern = rx_buffer[idx];
      idx += 1;   // 跳过1字节，指向模拟量
   }
    //解析模拟量
    if (is_analog)
   {
        gray_raw[0] = *(uint16*)(&rx_buffer[idx + 0]);
        gray_raw[1] = *(uint16*)(&rx_buffer[idx + 2]);
        gray_raw[2] = *(uint16*)(&rx_buffer[idx + 4]);
        gray_raw[3] = *(uint16*)(&rx_buffer[idx + 6]);
        gray_raw[4] = *(uint16*)(&rx_buffer[idx + 8]);
        gray_raw[5] = *(uint16*)(&rx_buffer[idx + 10]);
        gray_raw[6] = *(uint16*)(&rx_buffer[idx + 12]);
        gray_raw[7] = *(uint16*)(&rx_buffer[idx + 14]);
   }
    //丢线处理
    if ((gray_pattern & 0x0F) == 0) {
           if (gray_loss_left < 255) gray_loss_left++;
       } else {
           gray_loss_left = 0;
       }
    if ((gray_pattern & 0xF0) == 0) {
            if (gray_loss_right < 255) gray_loss_right++;
        } else {
            gray_loss_right = 0;
        }

    rx_index = 0;//重置
}

