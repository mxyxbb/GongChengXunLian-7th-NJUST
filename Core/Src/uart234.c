#include "uart234.h"
#include "user_usart.h"
#include "usart.h"
#include <stdio.h>
#include "i2c.h"
#include "MAX7219/max7219_matrix.h"
#include "MAX7219/max7219.h"
#include <string.h>
#include "SCS_servo/SCS_servo.h"
#include "letter_shell/src/shell_port.h"
#include "mymath.h"

uint8_t uart3ok=0;
SHELL_EXPORT_VAR(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_VAR_CHAR), u3ok, &uart3ok, uart3ok);
uint8_t readcolor_mode=1;//1--单次发、0---重复发
SHELL_EXPORT_VAR(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_VAR_CHAR), rc1111, &readcolor_mode, readcolor_mode);


uint8_t Ov3Mode = Ov3Mode_QrCode;
uint8_t Max7219_String[]="123--213";
uint8_t CMD_Serial[] = "CMD_Serial";
uint8_t CMD_qrcode[] = "qrcode";
uint8_t CMD_Material[] = "CMD_Material\r\n";
uint8_t CMD_color[] = "color";
uint8_t colororder[]={0,0,0,0,0,0};//用于LED点阵的显示，按自左到右的顺序显示在屏幕上

extern int32_t y_speed;
extern int32_t x_speed;	
	
/*
----------------新版动作组定义--------------
PS:车上的篮子颜色固定，Ⅰ、Ⅱ、Ⅲ号代表蓝、红、绿篮子，同地面从左到右的颜色顺序
控制机械臂执行特定动作 (堵塞式)

DOG		CMD			action
			---原料区---
0			1				抓上左，且上左是蓝色（红+1，绿+2）
3			4				抓上中，且上中是蓝色
6			7				抓上右，且上右是蓝色
9			10 			抓下左，且下左是蓝色
12		13			抓下中，且下中是蓝色
15		16			抓下右，且下右是蓝色
			---粗加工---
18		19			放Ⅰ号篮子物料（抓+1）
20		21			放Ⅱ号篮子物料
22		23			放Ⅲ号篮子物料
			---精加工---
24		25			放Ⅰ号篮子物料（上）
25		26      放Ⅱ号篮子物料
26		27      放Ⅲ号篮子物料
27		28      放Ⅰ号篮子物料（下）
28		29      放Ⅱ号篮子物料 
29		30      放Ⅲ号篮子物料
!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!（上）对应码垛下层
!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!（下）对应码垛上层
30            放一号篮子物料于台阶上层位置1
31
*/
void Uart2_servoCtr(uint8_t CMD){
	DoGroup(CMD-1);
	led_shan();
}

uint8_t RxBuff[1];      //进入中断接收数据的数组
uint8_t DataBuff[50]; //保存接收到的数据的数组，接收结束清空
uint8_t Color[6]; //保存接收到的颜色的数组

int RxLine=0;           //接收到的数据长度

void Uart3_readQRcode()
{
	printf("reading qrcode\n\r");
	Ov3Mode=Ov3Mode_QrCode;
	HAL_UART_Receive_IT(&huart3, (uint8_t *)RxBuff, 1); //打开串口中断接收
	HAL_UART_Transmit(&huart3, CMD_qrcode, 6, 0xffff);
	while(uart3ok==0)
	{
		__ExecuteOnce(user_main_printf("%s",CMD_qrcode));
	}
	x_speed=0;
	colororder[0]=Max7219_String[0]-'0';
	colororder[1]=Max7219_String[1]-'0';
	colororder[2]=Max7219_String[2]-'0';
	colororder[3]=Max7219_String[5]-'0';
	colororder[4]=Max7219_String[6]-'0';
	colororder[5]=Max7219_String[7]-'0';
	MAX7219_mywrite(colororder);
	MAX7219_MatrixUpdate();
	uart3ok=0;
	printf("qrcode:%s\n\r",Max7219_String);
	led_shan();
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC), u3rqr, Uart3_readQRcode, Uart3_readQRcode());

char led_str[6]="RGBRGB";

void Uart3_readColor()
{
	user_main_printf("%s",CMD_color);
	Ov3Mode=Ov3Mode_ColorBlock;
	HAL_UART_Receive_IT(&huart3, (uint8_t *)RxBuff, 1); //打开串口中断接收
	while(uart3ok==0){
		if(readcolor_mode)
			__ExecuteOnce(HAL_UART_Transmit(&huart3, CMD_color, 5, 0xffff));
		else
			HAL_UART_Transmit(&huart3, CMD_color, 5, 0xffff);
		HAL_Delay(40);
	}
	sprintf(led_str,"%c%c%c%c%c%c",Color[0],Color[1],Color[2],Color[3],Color[4],Color[5]);
	user_main_printf("color result:%s",led_str);
	uart3ok=0;
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC), u3rco, Uart3_readColor, Uart3_readColor());

void Uart2_sendColor()
{
	user_main_printf("color result:%s",led_str);
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC), u2sco, Uart2_sendColor, Uart2_sendColor());

void HAL_UART_RxCpltCallback_color()//在串口3的接收回调函数中调用此函数，处理openMV发回的颜色识别数据
{
	static char setVal,RxLine_Copy,i;
		
		if(RxLine==0)//初始化
		{
			RxLine_Copy=0;//初始化
			uart3ok=0;//初始化，默认状态接收未完成
			switch(Ov3Mode)
			{
				case Ov3Mode_QrCode:
					setVal = 11;//"123+321"\r\n
				break;
				case Ov3Mode_ColorBlock:
					setVal = 10;//"RGBBRG"\r\n
				break;
			}
		}
	
		
    RxLine++;                      //每接收到一个数据，进入回调数据长度加1
    DataBuff[RxLine-1]=RxBuff[0];  //把每次接收到的数据保存到缓存数组
    
    if((DataBuff[RxLine-1] == 0x0A)&&(DataBuff[RxLine-2] == 0x0D))            //接收结束标志位，这个数据可以自定义，根据实际需求，这里只做示例使用，不一定是0xff
    {
        //printf("Uart3-RXLen=%d\r\n",RxLine); 
        for(int i=0;i<RxLine;i++)
					//printf("UART DataBuff[%d] = 0x%x\r\n",i,DataBuff[i]);                               

				RxLine_Copy=RxLine;
				RxLine=0;  //清空接收长度
				
				
				
				if(RxLine_Copy<setVal)
				{	
					//HAL_UART_Receive_IT(&huart3, (uint8_t *)RxBuff, 1); //接受长度不够，要继续接收，继续打开一次串口中断接收，否则只会接收一个数据就停止接收
				}
				else
				{
					uart3ok=1;
					RxLine_Copy=0;
					switch(Ov3Mode)
					{
						case Ov3Mode_QrCode:
							Max7219_String[0]=DataBuff[1];
							Max7219_String[1]=DataBuff[2];
							Max7219_String[2]=DataBuff[3];
							Max7219_String[5]=DataBuff[5];
							Max7219_String[6]=DataBuff[6];
							Max7219_String[7]=DataBuff[7];
						break;
						case Ov3Mode_ColorBlock:
							for(i=0;i<6;i++)
							{
								Color[i]=DataBuff[i+1];
							}
						break;
					}
					memset(DataBuff,0,sizeof(DataBuff));  //清空缓存数组
					RxBuff[0]=0;
				}
				
		}
		

		if(RxLine_Copy<setVal&&!uart3ok)
		{	
			HAL_UART_Receive_IT(&huart3, (uint8_t *)RxBuff, 1); //接受长度不够，要继续接收，继续打开一次串口中断接收，否则只会接收一个数据就停止接收
		}
		
		
}
