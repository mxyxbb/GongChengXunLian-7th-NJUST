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
void OnTheWay(unsigned int vectorFrom,unsigned int vectorTo)
{
	unsigned int vector = vectorFrom*10+vectorTo;
	switch(vector)
	{
		case 01:
			OneGrid(FRONT,-15);
			OneGrid_sp(LEFT,FRONT,0);
			OneGrid_sp(LEFT,FRONT,0);
			OneGrid(FRONT,0);
			OneGrid(FRONT,-15);
			Grid_Lock();
			HAL_Delay(500);
			HAL_Delay(500);
			GoPos(99);
			Uart3_readQRcode();
			GoPos(87);
			Grid_UnLock();
			break;
		case 12:
			OneGrid(FRONT,0);
			OneGrid(FRONT,0);
			OneGrid(FRONT,-15);
			GoPos(0);
			Uart3_readColor();
			Grid_Lock();
		//�˴���е�ۿ�����ҪԤ����
			break;
		case 23:
			ni(10);
			HAL_Delay(1000);
			Grid_UnLock();
			OneGrid(FRONT,0);
			OneGrid(FRONT,-15);
			Grid_Lock();
			HAL_Delay(1000);
			break;
		case 32:
			Grid_UnLock();
			OneGrid(BACK,0);
			OneGrid(BACK,-15);
			Grid_Lock();
			HAL_Delay(1000);
			Grid_UnLock();
			ni(-10);
			Grid_Lock();
			HAL_Delay(1000);
			break;
		case 34:
			Grid_UnLock();
			OneGrid(FRONT,0);
			OneGrid(FRONT,-15);
			Grid_Lock();
			HAL_Delay(1000);
			Grid_UnLock();
			ni(10);
			Grid_Lock();
			HAL_Delay(1000);
			Grid_UnLock();
			OneGrid(FRONT,0);
			OneGrid(FRONT,-15);
			Grid_Lock();
			HAL_Delay(1000);
			break;
		case 43:
			Grid_UnLock();
			OneGrid(BACK,0);
			OneGrid(BACK,-15);
			Grid_Lock();
			HAL_Delay(1000);
			Grid_UnLock();
			ni(-10);
			Grid_Lock();
			HAL_Delay(1000);
			Grid_UnLock();
			OneGrid(BACK,0);
			OneGrid(BACK,-15);
			Grid_Lock();
			HAL_Delay(1000);
			break;
		case 42:
			Grid_UnLock();
			OneGrid(BACK,0);
			OneGrid(BACK,-15);
			Grid_Lock();
			HAL_Delay(1000);
			Grid_UnLock();
			ni(-10);
			Grid_Lock();
			HAL_Delay(1000);
			Grid_UnLock();
			for(int i=0;i<3;i++)
				{
					OneGrid(BACK,0);
				}
			OneGrid(BACK,-15);
			Grid_Lock();
			HAL_Delay(1000);
			Grid_UnLock();
			ni(-10);
			Grid_Lock();
			break;
		case 45:
			Grid_UnLock();
			GoPos(87);
			OneGrid(FRONT,0);
			OneGrid(FRONT,0);
			OneGrid(FRONT,-15);
			OneGrid(RIGHT,0);
			Grid_Lock();
			HAL_Delay(1500);
			Grid_UnLock();
			AngleAndPositionTIM=0;
			a_speed=0;
			x_speed=5;
			y_speed=5;
			HAL_Delay(2000);
			a_speed=0;
			x_speed=0;
			y_speed=0;
			/*have some problems*/
			break;
		
	}
	return;
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC), otw, OnTheWay, OnTheWay(from,to));

void ManufacturingProcesses()
{
	OnTheWay(0,1);
	OnTheWay(1,2);
//	//��ʱ����
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
				meterial[index].itsColor='1';//��ascll��ֵ49
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
			for(unsigned int i=0;i<3;i++)//һ��װ��3������ԭ����
				{ 
					Uart2_servoCtr(1+3*queue[i+3*index]+meterial[queue[i+3*index]].itsColor%3);//������1,4,7������+1����+2��������10,13,16������+1����+2��
					led_shan();
				}
			OnTheWay(2,3);
			for(unsigned int i=0;i<3;i++)//һ�ηź�3�����ڴּӹ���
				{ 
					static uint8_t once__=0;
					if(once__)
						GoPos(87);
					else
						once__=1;
					Uart2_servoCtr(19+2*(meterial[queue[i+3*index]].itsColor%3));//����19����21����23��
					led_shan();
				}
			for(unsigned int i=0;i<3;i++)//һ��װ��3�����Ӵּӹ���
				{ 
					GoPos(87);
					Uart2_servoCtr(20+2*(meterial[queue[i+3*index]].itsColor%3));//װ��20����22����24��
					led_shan();
				}
			OnTheWay(3,4);
			for(unsigned int i=0;i<3;i++)//һ�ηź�3�����ھ��ӹ���
				{ 
					Uart2_servoCtr(25+index*3+(meterial[queue[i+3*index]].itsColor%3));//����25����26����27��
					led_shan();
				}
			if(index==1)
				{
					OnTheWay(4,5);//over
					GoPos(87);
					/*�����е��();*/
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
