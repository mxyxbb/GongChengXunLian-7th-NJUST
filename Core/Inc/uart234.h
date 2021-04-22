#ifndef _UART234_H
#define _UART234_H

#include "stm32f4xx_hal.h"

#define Ov3Mode_QrCode 0
#define Ov3Mode_ColorBlock 1

extern uint8_t Color[6]; //������յ�����ɫ������
extern uint8_t Ov3Mode;				//Ҫ���е�ʶ��ģʽ,Ĭ�϶�ά��
extern uint32_t Catch1Time[6]; //��е�۶���ʱ��-ԭ����

extern void led_shan(void);



void Uart2_servoCtr(uint8_t CMD);
void Uart3_readQRcode(void);
void Uart3_readColor(void);
void HAL_UART_RxCpltCallback_color(void);
void waitForQrcode(void);
void waitForColor(void);
#endif
