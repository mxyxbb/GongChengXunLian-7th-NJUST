/*
 * uart.c
 * UART接口
 * 日期: 2019.9.7
 * 作者: 
 */

#include <stdio.h>
#include "stm32f4xx_hal.h"
#include "uart.h"
#include "usart.h"

//UART 读数据缓冲区
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
使用USE_USART1_ 宏定义
配置USART1，端口映射(TX)PA9/(RX)PA10
USART1作为舵机串口
------------------*/
#ifdef USE_USART1_

uint8_t data_one_byte[1];	//串口接收缓冲区（一个字符）
/**
 * @brief 串口接收处理函数
 */
void SCS_Uarthandle()
{

		uartBuf[tail] = data_one_byte[0];
		tail = (tail+1)%128;
		// 在完成一次接收后，串口中断会被关闭，需要再次打开
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
使用USE_USART2_宏定义
配置USART2，端口映射(TX)PA2/(RX)PA3
USART2作为舵机串口
------------------*/
#ifdef USE_USART2_

uint8_t data_one_byte[1];	//串口接收缓冲区（一个字符）
/**
 * @brief 串口接收处理函数
 */
void SCS_Uarthandle()
{

		uartBuf[tail] = data_one_byte[0];
		tail = (tail+1)%128;
		// 在完成一次接收后，串口中断会被关闭，需要再次打开
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
使用USE_UART5_宏定义
配置UART2，端口映射(TX)P/(RX)P
UART5作为舵机串口
------------------*/
#ifdef USE_UART5_

uint8_t data_one_byte[1];	//串口接收缓冲区（一个字符）
/**
 * @brief 串口接收处理函数
 */
void SCS_Uarthandle()
{

		uartBuf[tail] = data_one_byte[0];
		tail = (tail+1)%128;
		// 在完成一次接收后，串口中断会被关闭，需要再次打开
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
