/*********************************************************************************************************************
* TC264 Opensourec Library ����TC264 ��Դ�⣩��һ�����ڹٷ� SDK �ӿڵĵ�������Դ��
* Copyright (c) 2022 SEEKFREE ��ɿƼ�
*
* ���ļ��� TC264 ��Դ���һ����
*
* TC264 ��Դ�� ���������
* �����Ը���������������ᷢ���� GPL��GNU General Public License���� GNUͨ�ù�������֤��������
* �� GPL �ĵ�3�棨�� GPL3.0������ѡ��ģ��κκ����İ汾�����·�����/���޸���
*
* ����Դ��ķ�����ϣ�����ܷ������ã�����δ�������κεı�֤
* ����û�������������Ի��ʺ��ض���;�ı�֤
* ����ϸ����μ� GPL
*
* ��Ӧ�����յ�����Դ���ͬʱ�յ�һ�� GPL �ĸ���
* ���û�У������<https://www.gnu.org/licenses/>
*
* ����ע����
* ����Դ��ʹ�� GPL3.0 ��Դ����֤Э�� ������������Ϊ���İ汾
* ��������Ӣ�İ��� libraries/doc �ļ����µ� GPL3_permission_statement.txt �ļ���
* ����֤������ libraries �ļ����� �����ļ����µ� LICENSE �ļ�
* ��ӭ��λʹ�ò����������� ���޸�����ʱ���뱣����ɿƼ��İ�Ȩ����������������
*
* �ļ�����          isr
* ��˾����          �ɶ���ɿƼ����޹�˾
* �汾��Ϣ          �鿴 libraries/doc �ļ����� version �ļ� �汾˵��
* ��������          ADS v1.10.2
* ����ƽ̨          TC264D
* ��������          https://seekfree.taobao.com/
*
* �޸ļ�¼
* ����              ����                ��ע
* 2022-09-15       pudding            first version
********************************************************************************************************************/

#include "isr_config.h"
#include "isr.h"
#include "param.h"
#include "car.h"
#include "contro.h"
#include "track.h"
#include "vofa.h"

// ����TCϵ��Ĭ���ǲ�֧���ж�Ƕ�׵ģ�ϣ��֧���ж�Ƕ����Ҫ���ж���ʹ�� interrupt_global_enable(0); �������ж�Ƕ��
// �򵥵�˵ʵ���Ͻ����жϺ�TCϵ�е�Ӳ���Զ������� interrupt_global_disable(); ���ܾ���Ӧ�κε��жϣ������Ҫ�����Լ��ֶ����� interrupt_global_enable(0); �������жϵ���Ӧ��



// **************************** PIT定时器中断 ****************************
volatile uint32 g_pit_5ms_cnt = 0;                  // 5ms 节拍计数（IMU 中断限速用，imu 模块可读）

IFX_INTERRUPT(cc60_pit_ch0_isr, 0, CCU6_0_CH0_ISR_PRIORITY)
{
    interrupt_global_enable(0);                     // 开总中断
    pit_clear_flag(CCU60_CH0);
    g_pit_5ms_cnt++;                                // 5ms 节拍计数
    car_update_speed();                             // 1. 编码器测速采样（始终运行）
    vofa_pit_tick();                                // 2. VOFA 帧率节拍：5ms×2 = 10ms/帧
    if(control_is_enabled())                        // 3. 自动模式才跑闭环
    {
        track_update();                             //    循迹误差计算
        control_update();                           //    外环PD+内环PI（目标速度由串口设定）
    }
}



// **************************** 外部中断函数 ****************************
// IMU660RC INT2 (P15_4, ERU通道0)：120Hz 四元数就绪中断，写法与官方例程一致
static uint32 imu_last_cb_cnt = 0;                  // 上次 callback 时的 5ms 计数

IFX_INTERRUPT(exti_ch0_ch4_isr, 0, EXTI_CH0_CH4_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开中断嵌套

    if(exti_flag_get(ERU_CH0_REQ0_P15_4))
    {
        exti_flag_clear(ERU_CH0_REQ0_P15_4);
        // 限速闸门：INT2 线接触不良/受电机干扰时会高频抖动 → 中断风暴饿死主循环，
        // 表现为串口假死、命令无响应。同一 5ms 窗口最多执行一次 callback。
        // 芯片 120Hz 周期 8.3ms > 5ms，正常数据不受影响；风暴被限到最多 200 次/秒（约1% CPU）。
        if(g_pit_5ms_cnt != imu_last_cb_cnt)
        {
            imu_last_cb_cnt = g_pit_5ms_cnt;
            imu660rc_callback();                     // 中断内采集四元数+加速度+陀螺仪
        }
    }
}

IFX_INTERRUPT(cc60_pit_ch1_isr, 0, CCU6_0_CH1_ISR_PRIORITY)
{
    interrupt_global_enable(0);                     // �����ж�Ƕ��
    pit_clear_flag(CCU60_CH1);


}

IFX_INTERRUPT(cc61_pit_ch0_isr, 0, CCU6_1_CH0_ISR_PRIORITY)
{
    interrupt_global_enable(0);                     // �����ж�Ƕ��
    pit_clear_flag(CCU61_CH0);




}

IFX_INTERRUPT(cc61_pit_ch1_isr, 0, CCU6_1_CH1_ISR_PRIORITY)
{
    interrupt_global_enable(0);                     // �����ж�Ƕ��
    pit_clear_flag(CCU61_CH1);




}
// **************************** PIT�жϺ��� ****************************

//
//// **************************** �ⲿ�жϺ��� ****************************
//IFX_INTERRUPT(exti_ch0_ch4_isr, 0, EXTI_CH0_CH4_INT_PRIO)
//{
//    interrupt_global_enable(0);                     // �����ж�Ƕ��
//    if(exti_flag_get(ERU_CH4_REQ8_P33_7))           // ͨ��4�ж�
//    {
//        exti_flag_clear(ERU_CH4_REQ8_P33_7);
//        camera_vsync_handler_1();                   // ����ͷ1 �����ɼ�ͳһ�ص�����
//    }
//
//    if(exti_flag_get(ERU_CH0_REQ0_P15_4))           // ͨ��0�ж�
//    {
//        exti_flag_clear(ERU_CH0_REQ0_P15_4);
//        // ע��: ���ⲿ�ж�Ϊ����ͷ���ж� ����ʹ������ͷ �벻Ҫ��������������
//        // ע��: ���ⲿ�ж�Ϊ����ͷ���ж� ����ʹ������ͷ �벻Ҫ��������������
//        // ע��: ���ⲿ�ж�Ϊ����ͷ���ж� ����ʹ������ͷ �벻Ҫ��������������
//
//
//
//
//    }
//
//}
//
//// ��������ͷpclk����Ĭ��ռ���� 1ͨ�������ڴ���DMA��������ﲻ�ٶ����жϺ���
//IFX_INTERRUPT(exti_ch1_ch5_isr, 0, EXTI_CH1_CH5_INT_PRIO)
//{
//    interrupt_global_enable(0);                     // �����ж�Ƕ��
//
//    if(exti_flag_get(ERU_CH1_REQ10_P14_3))          // ͨ��1�ж�
//    {
//        exti_flag_clear(ERU_CH1_REQ10_P14_3);
//        // ע��: ���ⲿ�ж�Ϊ����ͷPCLK ����ʹ������ͷ �벻Ҫ��������������
//        // ע��: ���ⲿ�ж�Ϊ����ͷPCLK ����ʹ������ͷ �벻Ҫ��������������
//        // ע��: ���ⲿ�ж�Ϊ����ͷPCLK ����ʹ������ͷ �벻Ҫ��������������
//
//
//
//
//    }
//
//    if(exti_flag_get(ERU_CH5_REQ1_P15_8))           // ͨ��5�ж�
//    {
//        exti_flag_clear(ERU_CH5_REQ1_P15_8);
//        // ע��: ���ⲿ�ж�Ϊ����ͷPCLK ����ʹ������ͷ �벻Ҫ��������������
//        // ע��: ���ⲿ�ж�Ϊ����ͷPCLK ����ʹ������ͷ �벻Ҫ��������������
//        // ע��: ���ⲿ�ж�Ϊ����ͷPCLK ����ʹ������ͷ �벻Ҫ��������������
//
//
//
//
//    }
//}
//
//IFX_INTERRUPT(exti_ch2_ch6_isr, 0, EXTI_CH2_CH6_INT_PRIO)
//{
//    interrupt_global_enable(0);                     // �����ж�Ƕ��
//    if(exti_flag_get(ERU_CH2_REQ7_P00_4))           // ͨ��2�ж�
//    {
//        exti_flag_clear(ERU_CH2_REQ7_P00_4);
//        // ע��: ���ⲿ�ж�Ϊ����ͷPCLK ����ʹ������ͷ �벻Ҫ��������������
//        // ע��: ���ⲿ�ж�Ϊ����ͷPCLK ����ʹ������ͷ �벻Ҫ��������������
//        // ע��: ���ⲿ�ж�Ϊ����ͷPCLK ����ʹ������ͷ �벻Ҫ��������������
//
//
//
//    }
//    if(exti_flag_get(ERU_CH6_REQ9_P20_0))           // ͨ��6�ж�
//    {
//        exti_flag_clear(ERU_CH6_REQ9_P20_0);
//        // ע��: ���ⲿ�ж�Ϊ����ͷPCLK ����ʹ������ͷ �벻Ҫ��������������
//        // ע��: ���ⲿ�ж�Ϊ����ͷPCLK ����ʹ������ͷ �벻Ҫ��������������
//        // ע��: ���ⲿ�ж�Ϊ����ͷPCLK ����ʹ������ͷ �벻Ҫ��������������
//
//
//
//
//    }
//}
//
//IFX_INTERRUPT(exti_ch3_ch7_isr, 0, EXTI_CH3_CH7_INT_PRIO)
//{
//    interrupt_global_enable(0);                     // �����ж�Ƕ��
//
//    if(exti_flag_get(ERU_CH3_REQ3_P10_3))           // ͨ��3�ж�
//    {
//        exti_flag_clear(ERU_CH3_REQ3_P10_3);
//        camera_vsync_handler_2();                   // ����ͷ2�����ɼ�ͳһ�ص�����
//    }
//
//    if(exti_flag_get(ERU_CH7_REQ16_P15_1))          // ͨ��7�ж�
//    {
//        exti_flag_clear(ERU_CH7_REQ16_P15_1);
//        // ע��: ���ⲿ�ж�Ϊ����ͷ���ж� ����ʹ������ͷ �벻Ҫ��������������
//        // ע��: ���ⲿ�ж�Ϊ����ͷ���ж� ����ʹ������ͷ �벻Ҫ��������������
//        // ע��: ���ⲿ�ж�Ϊ����ͷ���ж� ����ʹ������ͷ �벻Ҫ��������������
//
//
//
//    }
//}
//// **************************** �ⲿ�жϺ��� ****************************


// **************************** DMA�жϺ��� ****************************
IFX_INTERRUPT(dma_ch6_isr, 0, DMA_INT_PRIO_1)
{
    interrupt_global_enable(0);                     // �����ж�Ƕ��
    camera_dma_handler_1();                         // ����ͷ1 �ɼ����ͳһ�ص�����
}

IFX_INTERRUPT(dma_ch7_isr, 0, DMA_INT_PRIO_2)
{
    interrupt_global_enable(0);                     // �����ж�Ƕ��
    camera_dma_handler_2();                         // ����ͷ2 �ɼ����ͳһ�ص�����
}
// **************************** DMA�жϺ��� ****************************


// **************************** �����жϺ��� ****************************
// ����0Ĭ����Ϊ���Դ���
IFX_INTERRUPT(uart0_tx_isr, 0, UART0_TX_INT_PRIO)
{
    interrupt_global_enable(0);                     // �����ж�Ƕ��



}
IFX_INTERRUPT(uart0_rx_isr, 0, UART0_RX_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开总中断

#if DEBUG_UART_USE_INTERRUPT                        // 启用 debug UART 中断
        debug_interrupr_handler();                  // 把 debug UART 收到的数据塞进调试环形缓冲区
#endif                                              // 更换 DEBUG_UART_INDEX 后记得把对应的串口中断改过来
}


// ����1Ĭ�����ӵ�����ͷ���ô���
IFX_INTERRUPT(uart1_tx_isr, 0, UART1_TX_INT_PRIO)
{
    interrupt_global_enable(0);                     // �����ж�Ƕ��




}
IFX_INTERRUPT(uart1_rx_isr, 0, UART1_RX_INT_PRIO)
{
    interrupt_global_enable(0);                     // �����ж�Ƕ��
    camera_uart_handler_1();                        // ����ͷ��������ͳһ�ص�����
}

// ����2Ĭ�����ӵ�����ת����ģ��
IFX_INTERRUPT(uart2_tx_isr, 0, UART2_TX_INT_PRIO)
{
    interrupt_global_enable(0);                     // �����ж�Ƕ��



}

IFX_INTERRUPT(uart2_rx_isr, 0, UART2_RX_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开总中断
    uint8 ch;
    while(uart_query_byte(UART_2, &ch))
    {
        wireless_rx_push(ch);                       // 只入队，主循环解析（printf等重活不能放ISR）
    }
}
// ����3Ĭ�����ӵ�GPS��λģ��
IFX_INTERRUPT(uart3_tx_isr, 0, UART3_TX_INT_PRIO)
{
    interrupt_global_enable(0);                     // �����ж�Ƕ��



}

IFX_INTERRUPT(uart3_rx_isr, 0, UART3_RX_INT_PRIO)
{
    interrupt_global_enable(0);                     // �����ж�Ƕ��

    gnss_uart_callback();                           // GNSS���ڻص�����



}

// ����ͨѶ�����ж�
IFX_INTERRUPT(uart0_er_isr, 0, UART0_ER_INT_PRIO)
{
    interrupt_global_enable(0);                     // �����ж�Ƕ��
    IfxAsclin_Asc_isrError(&uart0_handle);
}
IFX_INTERRUPT(uart1_er_isr, 0, UART1_ER_INT_PRIO)
{
    interrupt_global_enable(0);                     // �����ж�Ƕ��
    IfxAsclin_Asc_isrError(&uart1_handle);
}
IFX_INTERRUPT(uart2_er_isr, 0, UART2_ER_INT_PRIO)
{
    interrupt_global_enable(0);                     // �����ж�Ƕ��
    IfxAsclin_Asc_isrError(&uart2_handle);
}
IFX_INTERRUPT(uart3_er_isr, 0, UART3_ER_INT_PRIO)
{
    interrupt_global_enable(0);                     // �����ж�Ƕ��
    IfxAsclin_Asc_isrError(&uart3_handle);
}
