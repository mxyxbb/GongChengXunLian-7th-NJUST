#include "uart234.h"
#include "user_usart.h"
#include "usart.h"
#include <stdio.h>
#include "i2c.h"
#include "MAX7219/max7219_matrix.h"
#include "MAX7219/max7219.h"
#include <string.h>
#include "SCS_servo/SCS_servo.h"

uint8_t uart3ok=0;

uint8_t Ov3Mode = Ov3Mode_QrCode;
uint8_t Max7219_String[]="123--213";
uint8_t CMD_Serial[] = "CMD_Serial";
uint8_t CMD_Material[] = "CMD_Material\r\n";
uint8_t colororder[]={0,0,0,0,0,0};//用于LED点阵的显示，按自左到右的顺序显示在屏幕上

extern int32_t y_speed;
extern int32_t x_speed;


//uint32_t Catch1Time[6]={5000,5500,5500,7800,6000,6500};

//uint8_t uart2_ServoInit[]={0x55,0x55,0x05,0x06,0x01,0x01,0x00};//动作组1，初始化
//uint8_t uart2_PreCatch[]={0x55,0x55,0x05,0x06,0x02,0x01,0x00};//动作组2，预抓取
///*yuan liao qu,from up to down,left to right*/ 
//uint8_t uart2_Catch1[]={0x55,0x55,0x05,0x06,0x03,0x01,0x00};//动作组3，抓上1, 5s
//uint8_t uart2_Catch2[]={0x55,0x55,0x05,0x06,0x04,0x01,0x00};//动作组4，抓上2, 5s
//uint8_t uart2_Catch3[]={0x55,0x55,0x05,0x06,0x05,0x01,0x00};//动作组5，抓上3, 5s
//uint8_t uart2_Catch4[]={0x55,0x55,0x05,0x06,0x06,0x01,0x00};//动作组6，抓下1, 6s
//uint8_t uart2_Catch5[]={0x55,0x55,0x05,0x06,0x07,0x01,0x00};//动作组7，抓下2, 5.3s
//uint8_t uart2_Catch6[]={0x55,0x55,0x05,0x06,0x08,0x01,0x00};//动作组8，抓下3, 5s
///*cu jia gong qu fang,from left to right*/
//uint8_t uart2_Catch7[]={0x55,0x55,0x05,0x06,0x0c,0x01,0x00};//动作组12
//uint8_t uart2_Catch8[]={0x55,0x55,0x05,0x06,0x0a,0x01,0x00};//动作组10
//uint8_t uart2_Catch9[]={0x55,0x55,0x05,0x06,0x0b,0x01,0x00};//动作组11
///*cu jia gong qu qu,from left to right*/
//uint8_t uart2_Catch10[]={0x55,0x55,0x05,0x06,0x10,0x01,0x00};//动作组16
//uint8_t uart2_Catch11[]={0x55,0x55,0x05,0x06,0x0e,0x01,0x00};//动作组14
//uint8_t uart2_Catch12[]={0x55,0x55,0x05,0x06,0x0f,0x01,0x00};//动作组15
///*cheng pin qu,from up to down,left to right*/
//uint8_t uart2_Catch13[]={0x55,0x55,0x05,0x06,0x14,0x01,0x00};//动作组20
//uint8_t uart2_Catch14[]={0x55,0x55,0x05,0x06,0x12,0x01,0x00};//动作组18
//uint8_t uart2_Catch15[]={0x55,0x55,0x05,0x06,0x13,0x01,0x00};//动作组19
//uint8_t uart2_Catch16[]={0x55,0x55,0x05,0x06,0x17,0x01,0x00};//动作组23
//uint8_t uart2_Catch17[]={0x55,0x55,0x05,0x06,0x15,0x01,0x00};//动作组21
//uint8_t uart2_Catch18[]={0x55,0x55,0x05,0x06,0x16,0x01,0x00};//动作组22

//uint32_t delayTime[]={0,0,3300,3500,3500,4600,5300,5000,0,3500,5500,5500,0,4000,4500,4500,0,3000,2500,2500,3000,4000,4000};
//uint32_t delayTimePlus=2000;
	
	
/*
控制机械臂执行特定动作 (堵塞式)

CMD			action
1				初始位置
2				抬起位置，准备抓物料(原料区)
---原料区---
3				抓上左
4				抓上中
5				抓上右
6				抓下左
7				抓下中
8				抓下右
---粗加工---
9				抓上左
10			抓上中
11			抓上右
12			抓下左
13			抓下中
14			抓下右
---精加工---
15			顺序同上
16
17
18
19
20
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
	Ov3Mode=Ov3Mode_QrCode;
	HAL_UART_Receive_IT(&huart3, (uint8_t *)RxBuff, 1); //打开串口中断接收
	while(uart3ok==0){
		HAL_UART_Transmit(&huart3, CMD_Serial, 10, 0xffff);
		x_speed=5;
		HAL_Delay(20);
		x_speed=-5;
		HAL_Delay(20);
		x_speed=0;
	}
	colororder[0]=Max7219_String[0];
	colororder[1]=Max7219_String[1];
	colororder[2]=Max7219_String[2];
	colororder[3]=Max7219_String[5];
	colororder[4]=Max7219_String[6];
	colororder[5]=Max7219_String[7];
	MAX7219_mywrite(colororder);
	MAX7219_MatrixUpdate();
	uart3ok=0;
	led_shan();
}


char led_str[6]="000000";
void Uart3_readColor()
{
	Ov3Mode=Ov3Mode_ColorBlock;
	HAL_UART_Receive_IT(&huart3, (uint8_t *)RxBuff, 1); //打开串口中断接收
	while(uart3ok==0){
		HAL_UART_Transmit(&huart3, CMD_Material, 12, 0xffff);
		user_main_printf("%s",CMD_Material);
		x_speed=5;
		HAL_Delay(20);
		x_speed=-5;
		HAL_Delay(20);
		x_speed=0;
	}
	sprintf(led_str,"%c%c%c/%c%c%c",Color[0],Color[1],Color[2],Color[3],Color[4],Color[5]);
	user_main_printf("color result: %s",led_str);
	uart3ok=0;
}

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
        printf("Uart3-RXLen=%d\r\n",RxLine); 
        for(int i=0;i<RxLine;i++)
					printf("UART DataBuff[%d] = 0x%x\r\n",i,DataBuff[i]);                               

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
								Color[i]=DataBuff[6-i];
							}
						break;
					}
					memset(DataBuff,0,sizeof(DataBuff));  //清空缓存数组
					RxBuff[0]=0;
				}
				
		}
		

		if(RxLine_Copy<setVal)
		{	
			HAL_UART_Receive_IT(&huart3, (uint8_t *)RxBuff, 1); //接受长度不够，要继续接收，继续打开一次串口中断接收，否则只会接收一个数据就停止接收
		}
		
		
}
