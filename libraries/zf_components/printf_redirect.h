/*********************************************************************************************************************
* TC264 Opensourec Library ����TC264 ��Դ�⣩��һ�����ڹٷ� SDK �ӿڵĵ�������Դ��
* Copyright (c) 2022 SEEKFREE ��ɿƼ�?
*
* ���ļ��� TC264 ��Դ���һ����?
*
* TC264 ��Դ�� ���������?
* �����Ը���������������ᷢ����?GPL��GNU General Public License���� GNUͨ�ù�������֤��������
* �� GPL �ĵ�3�棨�� GPL3.0������ѡ��ģ��κκ����İ汾�����·�����?���޸���
*
* ����Դ��ķ�����ϣ�����ܷ������ã�����δ�������κεı��?
* ����û�������������Ի��ʺ��ض���;�ı�֤
* ����ϸ����μ�?GPL
*
* ��Ӧ�����յ�����Դ���ͬʱ�յ�һ��?GPL �ĸ���
* ���û�У������<https://www.gnu.org/licenses/>
*
* ����ע����
* ����Դ��ʹ�� GPL3.0 ��Դ����֤Э�� ������������Ϊ���İ汾
* ��������Ӣ�İ��� libraries/doc �ļ����µ� GPL3_permission_statement.txt �ļ���
* ����֤������ libraries �ļ����� �����ļ����µ� LICENSE �ļ�
* ��ӭ��λʹ�ò����������� ���޸�����ʱ���뱣����ɿƼ��İ�Ȩ����������������?
*
* �ļ�����          printf_redirect
* ��˾����          �ɶ���ɿƼ����޹��?
* �汾��Ϣ          �鿴 libraries/doc �ļ����� version �ļ� �汾˵��
* ��������          ADS v1.10.2
* ����ƽ̨          TC264D
* ��������          https://seekfree.taobao.com/
*
* �޸ļ�¼
* ����              ����                ��ע
* 2025-04-19       pudding            first version
********************************************************************************************************************/
#ifndef _PRINTF_REDIRECT_H_
#define _PRINTF_REDIRECT_H_

#include "zf_common_debug.h"
#include "zf_driver_uart.h"
#include "zf_driver_spi.h"
#include "zf_device_ble6a20.h"
#include "zf_device_bluetooth_ch9141.h"
#include "zf_device_wifi_uart.h"
#include "zf_device_wireless_uart.h"
#include "zf_device_wifi_spi.h"


#define  PRINTF_OUTPUT_TYPE     0                       // printf ��ӡ�������?0 ʹ�ô���   1 ʹ��SPI


#if PRINTF_OUTPUT_TYPE == 0

// ��ɿƼ���Դ��?�������ģ��?Ĭ�ϴ��ں궨�����£�
// DEBUG_UART_INDEX                                     // debug ���Դ���
// BLE6A20_INDEX                                        // ble6a20 ��������ģ��
// BLUETOOTH_CH9141_INDEX                               // ch9141 ����ģ��
// WIFI_UART_INDEX                                      // wifi ����ģ��
// WIRELESS_UART_INDEX                                  // ���ߴ���ģ��

#define  PRINTF_USE_UART        (UART_2)                // printf redirect to wireless UART2 (P10_5/P10_6, VOFA+)


// ���?PRINTF_OUTPUT_TYPE Ϊ 1���·�������Ϊ��Ч״̬ printf ����ͨ�� SPI ģ�����?
#else

// ====================================  ע��  ====================================
// ע�⣺printfʹ��wifi-spiģ������ַ���������?\n"��Ϊ��������Ľ�β�����򽫵����޷���ӡ�����ڴ����
// ע�⣺printfʹ��wifi-spiģ������ַ���������?\n"��Ϊ��������Ľ�β�����򽫵����޷���ӡ�����ڴ����
// ע�⣺printfʹ��wifi-spiģ������ַ���������?\n"��Ϊ��������Ľ�β�����򽫵����޷���ӡ�����ڴ����
// ====================================  ע��  ====================================
#define  PRINTF_USE_SPI         (WIFI_SPI_INDEX)        // Ĭ�� printf ʹ�� WIFI_SPI_INDEX ģ�����?��ģʽ��ʱ��֧�� WIFI-SPI)

// ��ɿƼ���Դ��?SPI���ģ��?Ĭ�ϴ��ں궨�����£�
// WIFI_SPI_INDEX               // WIFI-SPI ģ��

#endif


#endif
