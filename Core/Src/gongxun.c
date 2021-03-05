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

Meterial meterial[6];
uint8_t queue[6];


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
			OneGrid(FRONT,2);
			Uart3_readQRcode();
			break;
		case 12:
			OneGrid_sp(LEFT,FRONT,0);
			OneGrid(FRONT,0);
			OneGrid(FRONT,0);
			OneGrid(FRONT,10);
			Uart3_readColor();
			Uart2_servoCtr(2);
			OneGrid(FRONT,-15);
			Grid_Lock();
			HAL_Delay(2000);
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
			ni(-10);
			Grid_Lock();
			HAL_Delay(1000);
			break;
		case 34:
			Grid_UnLock();
			OneGrid(FRONT,0);
			OneGrid(FRONT,-15);
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
			ni(-10);
			Grid_Lock();
			HAL_Delay(1000);
			Grid_UnLock();
			for(int i=0;i<3;i++)
				{
					OneGrid(BACK,0);
				}
			OneGrid(BACK,-15);
			ni(-10);
			Grid_Lock();
			break;
		case 45:
			Grid_UnLock();
			OneGrid(FRONT,0);
			OneGrid(FRONT,0);
			OneGrid(FRONT,-15);
			OneGrid(RIGHT,0);
			AngleAndPositionTIM=0;
			a_speed=0;
			x_speed=5;
			y_speed=15;
			HAL_Delay(3000);
			a_speed=0;
			x_speed=0;
			y_speed=0;
			/*have some problems*/
			break;
		
	}
	return;
}

void ManufacturingProcesses()
{
	OnTheWay(0,1);
	OnTheWay(1,2);
	unsigned int index=0;
	
	/*data preparation*/
	for(;index<6;index++)
		{	
			if(Color[index]=='R')
				meterial[index].itsColor='1';
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
		
	/*performing*/
	for(index=0;index<2;index++)
		{
			for(unsigned int i=0;i<3;i++)
				{ 
					Uart2_servoCtr(3+queue[i+3*index]);//aiming  ascllÂë¼õ48
					led_shan();
					OnTheWay(2,3);
					if(meterial[queue[i+3*index]].itsColor=='3')
						{
							led_shan();
							Uart2_servoCtr(9);//aiming
						}
					else if(meterial[queue[i+3*index]].itsColor=='1')
						{
							led_shan();
							Uart2_servoCtr(10);//aiming
						}
					else if(meterial[queue[i+3*index]].itsColor=='2')
						{
							led_shan();
							Uart2_servoCtr(11);//aiming
						}
					if(i!=2)
						{
							OnTheWay(3,2);
						}
				}
//			HAL_Delay(3000);
			for(unsigned int i=0;i<3;i++)
				{
					if(meterial[queue[i+3*index]].itsColor=='3'){
						led_shan();
						Uart2_servoCtr(12);//aiming
					}
					else if(meterial[queue[i+3*index]].itsColor=='1'){
						led_shan();
						Uart2_servoCtr(13);//aiming
					}
					else if(meterial[queue[i+3*index]].itsColor=='2'){
						led_shan();
						Uart2_servoCtr(14);//aiming
					}
					OnTheWay(3,4);
					if(meterial[queue[i+3*index]].itsColor=='3'){

						Uart2_servoCtr(15+3*index);//aiming
						led_shan();
					}
					else if(meterial[queue[i+3*index]].itsColor=='1'){

						Uart2_servoCtr(16+3*index);//aiming
						led_shan();
					}
					else if(meterial[queue[i+3*index]].itsColor=='2'){

						Uart2_servoCtr(17+3*index);//aiming
						led_shan();
					}if(i!=2)
						{
							OnTheWay(4,3);
						}
				}
			if(index==1)
				{
					OnTheWay(4,5);//over
					Uart2_servoCtr(1);
				}
			else 
				OnTheWay(4,2);
		}
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC), gxgo, ManufacturingProcesses, ManufacturingProcesses());