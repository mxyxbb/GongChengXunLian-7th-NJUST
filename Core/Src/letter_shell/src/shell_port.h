#ifndef _SHELL_PORT_H_
#define _SHELL_PORT_H_

#include "shell.h"
#include <stm32f4xx_hal.h>

//串口接收缓冲区
extern uint8_t recv_buf;
/* 将shell定义为外部变量，在串口中断回调函数中还要使用 */
extern Shell shell;

/* 声明自己编写的初始化函数 */
void User_Shell_Init(void);

#endif /* _SHELL_PORT_H_ */
