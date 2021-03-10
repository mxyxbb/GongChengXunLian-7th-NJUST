/*
 * uart.c
 * UART�ӿ�
 * ����: 2019.9.7
 * ����: 
 */

#include <stdio.h>
#include "stm32f4xx_hal.h"
#include "uart.h"
#include "usart.h"

//UART �����ݻ�����
__IO uint8_t uartBuf[128];
__IO int head = 0;
__IO int tail  = 0;

void Uart_Flush(void)
{
	head = tail = 0;
}

int16_t Uart_Read(void)
{
	if(head!=tail){
		uint8_t Data = uartBuf[head];
		head =  (head+1)%128;
		return Data;
	}else{
		return -1;
	}
}


#define USE_UART5_//USART2_
/*---------------
ʹ��USE_USART1_ �궨��
����USART1���˿�ӳ��(TX)PA9/(RX)PA10
USART1��Ϊ�������
------------------*/
#ifdef USE_USART1_

uint8_t data_one_byte[1];	//���ڽ��ջ�������һ���ַ���
/**
 * @brief ���ڽ��մ�����
 */
void SCS_Uarthandle()
{

		uartBuf[tail] = data_one_byte[0];
		tail = (tail+1)%128;
		// �����һ�ν��պ󣬴����жϻᱻ�رգ���Ҫ�ٴδ�
		HAL_UART_Receive_IT(&huart1, data_one_byte, 1);
	
}

void Uart_Send(uint8_t *buf , uint8_t len)
{
	uint8_t i=0;
	for(i=0; i<len; i++){
		HAL_UART_Transmit(&huart1, (uint8_t*)&buf[i], 1, 0xFFFF);
	}
}

#endif

/*---------------
ʹ��USE_USART2_�궨��
����USART2���˿�ӳ��(TX)PA2/(RX)PA3
USART2��Ϊ�������
------------------*/
#ifdef USE_USART2_

uint8_t data_one_byte[1];	//���ڽ��ջ�������һ���ַ���
/**
 * @brief ���ڽ��մ�����
 */
void SCS_Uarthandle()
{

		uartBuf[tail] = data_one_byte[0];
		tail = (tail+1)%128;
		// �����һ�ν��պ󣬴����жϻᱻ�رգ���Ҫ�ٴδ�
		HAL_UART_Receive_IT(&huart2, data_one_byte, 1);
	
}

void Uart_Send(uint8_t *buf , uint8_t len)
{
	uint8_t i=0;
	for(i=0; i<len; i++){
		HAL_UART_Transmit(&huart2, (uint8_t*)&buf[i], 1, 0xFFFF);
	}
}

#endif
/*---------------
ʹ��USE_UART5_�궨��
����UART2���˿�ӳ��(TX)P/(RX)P
UART5��Ϊ�������
------------------*/
#ifdef USE_UART5_

uint8_t data_one_byte[1];	//���ڽ��ջ�������һ���ַ���
/**
 * @brief ���ڽ��մ�����
 */
void SCS_Uarthandle()
{

		uartBuf[tail] = data_one_byte[0];
		tail = (tail+1)%128;
		// �����һ�ν��պ󣬴����жϻᱻ�رգ���Ҫ�ٴδ�
		HAL_UART_Receive_IT(&huart5, data_one_byte, 1);
	
}

void Uart_Send(uint8_t *buf , uint8_t len)
{
	uint8_t i=0;
	for(i=0; i<len; i++){
		HAL_UART_Transmit(&huart5, (uint8_t*)&buf[i], 1, 0xFFFF);
	}
}

#endif
