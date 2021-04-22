#include "gongxun.h"
#include "MotorControl.h"
#include "uart234.h"
#include "ControlTask.h"
#include "lineFollowSensor.h"
#include "PID.h"
#include "user_usart.h"
#include <stdio.h>
#include "main.h"
#include "letter_shell/src/shell_port.h"
#include "SCS_servo/SCS_servo.h"

extern int32_t x_position;
extern int32_t y_position;

Meterial meterial[6];
uint8_t queue[6];
uint8_t readQ=0;//0--关闭识别Qrcode，1--开启识别Qrcode
uint8_t readC=0;//0--关闭识别颜色，1--开启识别颜色

void GoFrontForQR(void);
void GoFrontForMaterial(void);

/*
start point 0,
erweima 1,
raw material area 2,
rough machining area 3,
Semi-finished products area 4,
return point 5,
*/
int Color2Addend(char ascllColor)//用法：将(meterial[queue[i+3*index]].itsColor%3)替换成Color2Addend(meterial[queue[i+3*index]].itsColor)
	{
		int idx;
		int Addend=0;
		char PlacemarkOrder[]="312";//改这里，从左到右代表地标的颜色，如312，代表蓝红绿
		for(idx=0;idx<3;idx++)
		{
			if(PlacemarkOrder[idx]==ascllColor)
			{
				Addend=idx;
				break;
			}
		}
		return Addend;
	}

void OnTheWay(unsigned int vectorFrom,unsigned int vectorTo)
{
	unsigned int vector = vectorFrom*10+vectorTo;
	switch(vector)
	{
		case 01:
			CarMovingTo=FRONT;
			a_speed=0;
			x_speed=10;
			y_speed=-8;
			HAL_Delay(3000);
			a_speed=0;
			x_speed=0;
			y_speed=0;
		
//			OneGrid(FRONT,-15);
//			OneGrid_sp(LEFT,FRONT,0);
//			OneGrid_sp(LEFT,FRONT,0);
//			OneGrid(FRONT,0);
////			GoPosSP(99);//机械臂.读二维码动作
//			OneGrid(FRONT,-15);
			Grid_Lock();
			HAL_Delay(1500);
			Grid_UnLock();
			if(readQ)
				Uart3_readQRcode();
			OneGrid(FRONT,0);
			waitForQrcode();
			GoPosSP(87);//回中
			Grid_UnLock();
			break;
		case 12:
			//			if(readC)
			Uart3_readColor();
			OneGrid(FRONT,13);
			OneGrid(FRONT,13);
//			GoPosSP(0);//机械臂.读颜色动作
			OneGrid(FRONT,20);
			waitForColor();
			OneGrid(BACK,-20);
			Grid_Lock();
		//此处机械臂可能需要预动作
			break;
		case 23:
			ni(10);
			HAL_Delay(800);
			Grid_UnLock();
			OneGrid(FRONT,0);
			OneGrid(FRONT,-15);
			Grid_Lock();
			HAL_Delay(800);
			break;
		case 32:
			Grid_UnLock();
			OneGrid(BACK,0);
			OneGrid(BACK,-15);
			Grid_Lock();
			HAL_Delay(800);
			Grid_UnLock();
			ni(-10);
			Grid_Lock();
			HAL_Delay(800);
			break;
		case 34:
			Grid_UnLock();
			OneGrid(FRONT,0);
			OneGrid(FRONT,-15);
			Grid_Lock();
			HAL_Delay(800);
			Grid_UnLock();
			ni(10);
			Grid_Lock();
			HAL_Delay(800);
			Grid_UnLock();
			OneGrid(FRONT,0);
			OneGrid(FRONT,-15);
			Grid_Lock();
			HAL_Delay(800);
			break;
		case 43:
			Grid_UnLock();
			OneGrid(BACK,0);
			OneGrid(BACK,-15);
			Grid_Lock();
			HAL_Delay(800);
			Grid_UnLock();
			ni(-10);
			Grid_Lock();
			HAL_Delay(800);
			Grid_UnLock();
			OneGrid(BACK,0);
			OneGrid(BACK,-15);
			Grid_Lock();
			HAL_Delay(800);
			break;
		case 42:
			GoPosSP(87);
			Grid_UnLock();
			OneGrid(BACK,0);
			OneGrid(BACK,-15);
			Grid_Lock();
			HAL_Delay(800);
			Grid_UnLock();
			ni(-10);
			Grid_Lock();
			HAL_Delay(800);
			Grid_UnLock();
			for(int i=0;i<3;i++)
				{
					OneGrid(BACK,6);
				}
			OneGrid(BACK,-15);
			Grid_Lock();
			HAL_Delay(500);
			Grid_UnLock();
			ni(-10);
			Grid_Lock();
			break;
		case 45:
			Grid_UnLock();
			GoPosSP(87);
		//冲回家
			AngleAndPositionTIM=0;//关闭循迹
			a_speed=0;
			x_speed=30;
			y_speed=18;
			HAL_Delay(2900);
			a_speed=0;
			x_speed=0;
			y_speed=0;
//			OneGrid(FRONT,0);
//			OneGrid(FRONT,0);
//			OneGrid(FRONT,-15);
//			OneGrid(RIGHT,0);
//			Grid_Lock();
//			HAL_Delay(1000);
//			Grid_UnLock();
//			AngleAndPositionTIM=0;
//			a_speed=0;
//			x_speed=5;
//			y_speed=5;
//			HAL_Delay(1800);
//			a_speed=0;
//			x_speed=0;
//			y_speed=0;
			
			break;
	}
	return;
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC), otw, OnTheWay, OnTheWay(from,to));

void ManufacturingProcesses()
{
	OnTheWay(0,1);
	OnTheWay(1,2);
//	//临时调试
//	Color[0]='R';
//	Color[1]='G';
//	Color[2]='B';
//	Color[3]='R';
//	Color[4]='G';
//	Color[5]='B';
//	Max7219_String[0]='2';
//	Max7219_String[1]='1';
//	Max7219_String[2]='3';
//	Max7219_String[3]='-';
//	Max7219_String[4]='-';
//	Max7219_String[5]='3';
//	Max7219_String[6]='2';
//	Max7219_String[7]='1';
	
	unsigned int index=0;
	
	/*data preparation*/
	for(;index<6;index++)
		{	
			if(Color[index]=='R')
				meterial[index].itsColor='1';//其ascll码值49
			if(Color[index]=='G')
				meterial[index].itsColor='2';
			if(Color[index]=='B')
				meterial[index].itsColor='3';
		}
	for(index=0;index<3;index++)
		{	
			if(meterial[index].itsColor==Max7219_String[index])
				meterial[index].itsOrder=index;
			else if(meterial[index].itsColor==Max7219_String[(index+1)%3])
				meterial[index].itsOrder=(index+1)%3;
			else if(meterial[index].itsColor==Max7219_String[(index+2)%3])
				meterial[index].itsOrder=(index+2)%3;
		}
	for(index=0;index<3;index++)
		{
			if(meterial[3+index].itsColor==Max7219_String[5+index])
				meterial[3+index].itsOrder=3+index;
			else if(meterial[3+index].itsColor==Max7219_String[5+(index+1)%3])
				meterial[3+index].itsOrder=3+(index+1)%3;
			else if(meterial[3+index].itsColor==Max7219_String[5+(index+2)%3])
				meterial[3+index].itsOrder=3+(index+2)%3;
		}
	for(index=0;index<6;index++)
		{
			queue[meterial[index].itsOrder]=index;
		}
		
		
//	while(SW2==UNPRESSED);//examing!
	for(uint8_t i=0;i<6;i++)
	{
		printf("queue[%d]=%d\t",i,queue[i]);
	}
	printf("\n\r");
	/*performing*/
	for(index=0;index<2;index++)
		{
			for(unsigned int i=0;i<3;i++)//一次装好3个，从原料区
				{ 
					Uart2_servoCtr(1+3*queue[i+3*index]+Color2Addend(meterial[queue[i+3*index]].itsColor));//上蓝（1,4,7），红+1，绿+2；下蓝（10,13,16），红+1，绿+2。
					led_shan();
				}
			OnTheWay(2,3);		
			uint8_t once__=0;//4月20日：去掉static
			for(unsigned int i=0;i<3;i++)//一次放好3个，在粗加工区
				{ 
					if(once__)
						GoPos(87);
					else
						once__=1;
					Uart2_servoCtr(19+2*(Color2Addend(meterial[queue[i+3*index]].itsColor)));//放蓝19，红21，绿23。
					led_shan();
				}
			for(unsigned int i=0;i<3;i++)//一次装好3个，从粗加工区
				{ 
					GoPos(87);
					Uart2_servoCtr(20+2*(Color2Addend(meterial[queue[i+3*index]].itsColor)));//装蓝20，红22，绿24。
					led_shan();
				}
			OnTheWay(3,4);
			for(unsigned int i=0;i<3;i++)//一次放好3个，在精加工区
				{ 
					Uart2_servoCtr(25+index*3+(Color2Addend(meterial[queue[i+3*index]].itsColor)));//放蓝25，红26，绿27。
					led_shan();
				}
			if(index==1)
				{
					OnTheWay(4,5);//over
					GoPos(87);
					/*收起机械臂();*/
				}
			else 
				  OnTheWay(4,2);
		}
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC), gxgo, ManufacturingProcesses, ManufacturingProcesses());

void GoFrontForQR()
{
	CarMovingTo=FRONT;
	x_speed=5;
	HAL_Delay(2500);
	x_speed=0;
	Uart3_readQRcode();
	OneGrid(0,-25);
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC), rqr, GoFrontForQR, GoFrontForQR());

void GoFrontForMaterial()
{
	CarMovingTo=FRONT;
	Grid_Lock();
	Uart3_readColor();
	Grid_UnLock();
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC), rm, GoFrontForMaterial, GoFrontForMaterial());

void resetCar()
{
	CarMovingTo=RIGHT;
	AngleAndPositionTIM=1;
	Grid_UnLock();
	y_speed=-10;
	while(HAL_GPIO_ReadPin(SJ2_GPIO_Port,SJ2_Pin)==1);
	y_speed=10;
	HAL_Delay(200);
	
	CarMovingTo=BACK;
	AngleAndPositionTIM=1;
	Grid_UnLock();
	x_speed=-10;
	while(HAL_GPIO_ReadPin(SJ1_GPIO_Port,SJ1_Pin)==1);
	x_speed=10;
	HAL_Delay(200);
	
	x_position=0;
	y_position=0;
}
