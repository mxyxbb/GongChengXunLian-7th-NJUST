#include "MotorControl.h"
#include "ControlTask.h"
#include "lineFollowSensor.h"
#include "PID.h"
#include "user_usart.h"
#include <stdio.h>
#include "main.h"
#include "letter_shell/src/shell_port.h"
#include "Buzzer/buzzerDriver.h"
#include <string.h> //使用到了memcpy


#define SENSORNUM 4
#define DIRECTION 4
#define TOTAL_SENSORNUM 16	//SENSORNUM*DIRECTION
#define BLACK 1	//lineColor
#define WHITE 0	//bgColor



//#define Angle_Kp -5
//#define Angle_Ki 0
//#define Angle_Kd -1

//#define Position_Kp 5
//#define Position_Ki 0
//#define Position_Kd 1


#define xAngle_Kp 3//6
#define xAngle_Ki 0
#define xAngle_Kd 1

#define xPosition_Kp 3 //6
#define xPosition_Ki 0
#define xPosition_Kd 1

#define yAngle_Kp 2//-2//-6//-15
#define yAngle_Ki 0//0
#define yAngle_Kd 1

#define yPosition_Kp -2//-2//-1
#define yPosition_Ki -0
#define yPosition_Kd -1//-1






int32_t LineSensor[SENSORNUM];
int32_t ErrorRank[SENSORNUM]={-3,-1,1,3};
int32_t temp;
int32_t xunhuan;
int32_t Error[DIRECTION]={0}; 
int32_t HBPosition=0;
int32_t LRPosition=0;
int32_t xAngle=0;
int32_t yAngle=0;
//int32_t Angle=0;
int32_t Sensor_JG_Buffer[30];//JG == ji guang
int32_t Sensor_Check_Buffer[30];//JG == ji guang

int32_t SegOFFSET_JG[DIRECTION]={0};
//int32_t Angle_PIDControlVal=0;
int32_t xAngle_PIDControlVal=0;

//int32_t AngleSet = 0;
//int32_t PositionSet = 0;
int32_t xAngleSet = 0;
int32_t xPositionSet = 0;
int32_t yAngleSet = 0;
int32_t yPositionSet = 0;

int32_t CarMovingTo = RIGHT;//FRONT;
int32_t CarMovingTo_last = RIGHT;

//用于自动校正灰度传感器，会被存储在flash中
uint8_t index_gray[16];

extern int32_t y_speed;
extern int32_t x_speed;
extern int32_t a_speed;
extern int32_t obs[5][8];
extern int32_t START_MODE;//取左下角起始为0，右下角起始为1
extern int32_t x_position;
extern int32_t y_position;
extern int32_t BEGIN_X;
extern int32_t BEGIN_Y;
extern int32_t END_X;
extern int32_t END_Y;


void LineFollowInit()
{
	for(xunhuan=0;xunhuan<DIRECTION;xunhuan++)
		SegOFFSET_JG[xunhuan] = xunhuan*SENSORNUM;
	
	User_PID_Init(&xAnglePIDParameter4);
	xAnglePIDParameter4.Kp = xAngle_Kp;
	xAnglePIDParameter4.Ki = xAngle_Ki;
	xAnglePIDParameter4.Kd = xAngle_Kd;
	User_PID_Init(&xPositionPIDParameter5);
	xPositionPIDParameter5.Kp = xPosition_Kp;
	xPositionPIDParameter5.Ki = xPosition_Ki;
	xPositionPIDParameter5.Kd = xPosition_Kd;
	User_PID_Init(&yAnglePIDParameter6);
	yAnglePIDParameter6.Kp = yAngle_Kp;
	yAnglePIDParameter6.Ki = yAngle_Ki;
	yAnglePIDParameter6.Kd = yAngle_Kd;
	User_PID_Init(&yPositionPIDParameter7);
	yPositionPIDParameter7.Kp = yPosition_Kp;
	yPositionPIDParameter7.Ki = yPosition_Ki;
	yPositionPIDParameter7.Kd = yPosition_Kd;
}
void LockSetPID()
{
	xAnglePIDParameter4.Kp = 2;
	xAnglePIDParameter4.Ki = xAngle_Ki;
	xAnglePIDParameter4.Kd = xAngle_Kd;

	xPositionPIDParameter5.Kp = 1;
	xPositionPIDParameter5.Ki = xPosition_Ki;
	xPositionPIDParameter5.Kd = xPosition_Kd;

	yAnglePIDParameter6.Kp = 1;
	yAnglePIDParameter6.Ki = yAngle_Ki;
	yAnglePIDParameter6.Kd = yAngle_Kd;

	yPositionPIDParameter7.Kp = -1;
	yPositionPIDParameter7.Ki = yPosition_Ki;
	yPositionPIDParameter7.Kd = yPosition_Kd;
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC), lset, LockSetPID, LockSetPID());

void LockResetPID()
{
	xAnglePIDParameter4.Kp = xAngle_Kp;
	xAnglePIDParameter4.Ki = xAngle_Ki;
	xAnglePIDParameter4.Kd = xAngle_Kd;

	xPositionPIDParameter5.Kp = xPosition_Kp;
	xPositionPIDParameter5.Ki = xPosition_Ki;
	xPositionPIDParameter5.Kd = xPosition_Kd;

	yAnglePIDParameter6.Kp = yAngle_Kp;
	yAnglePIDParameter6.Ki = yAngle_Ki;
	yAnglePIDParameter6.Kd = yAngle_Kd;

	yPositionPIDParameter7.Kp = yPosition_Kp;
	yPositionPIDParameter7.Ki = yPosition_Ki;
	yPositionPIDParameter7.Kd = yPosition_Kd;
}


void xAngleControl()
{
	Error[FRONT] = DirectionError_Calc(FRONT);
	Error[BACK] = DirectionError_Calc(BACK);
//	HBPosition = 
	xAngle = Error[FRONT] +/*-*/ Error[BACK];//HBPosition;	//approximately equ
	a_speed = User_PID_Calc_angle(&xAnglePIDParameter4,xAngle,xAngleSet);
	
}

void xPositionControl()
{
	Error[FRONT] = DirectionError_Calc(FRONT);
	Error[BACK] = DirectionError_Calc(BACK);
	HBPosition=Error[FRONT]-Error[BACK];//add
	y_speed = User_PID_Calc_angle(&xPositionPIDParameter5,HBPosition,xPositionSet);
}

void yAngleControl()
{
	Error[LEFT] = DirectionError_Calc(LEFT);
	Error[RIGHT] = DirectionError_Calc(RIGHT);
//	LRPosition = Error[LEFT] +/*-*/ Error[RIGHT];
	yAngle = Error[LEFT] +/*-*/ Error[RIGHT];// LRPosition;	//approximately equ
	a_speed = User_PID_Calc_angle(&yAnglePIDParameter6,yAngle,yAngleSet);
	
}


void yPositionControl()
{
	Error[LEFT] = DirectionError_Calc(LEFT);
	Error[RIGHT] = DirectionError_Calc(RIGHT);
	LRPosition = Error[LEFT] - Error[RIGHT];
	x_speed = User_PID_Calc_angle(&yPositionPIDParameter7,Error[RIGHT],xPositionSet);
//	x_speed = yPosUser_PID_Calc_angle(&yPositionPIDParameter7,Error[RIGHT],xPositionSet);
}

void ySingleControl()
{
	Error[RIGHT] = DirectionError_Calc(RIGHT);
	x_speed = User_PID_Calc_angle(&yPositionPIDParameter7,Error[RIGHT],xPositionSet);
//	x_speed = yPosUser_PID_Calc_angle(&yPositionPIDParameter7,Error[RIGHT],xPositionSet);
}


void AnglePosControl(unsigned int dir)
{
		switch (dir)
		{
			case FRONT:
			case BACK:
				xPositionControl();
				xAngleControl();
				break;
			case LEFT:
			case RIGHT:
				yPositionControl();
				yAngleControl();
//				xAngleControl();//横走时利用垂直线回正车身
				break;
		}
}

//void LineFollowInit()
//{
//	for(xunhuan=0;xunhuan<DIRECTION;xunhuan++)
//		SegOFFSET_JG[xunhuan] = xunhuan*SENSORNUM;
//	
//	User_PID_Init(&xAnglePIDParameter4);
//	xAnglePIDParameter4.Kp = Angle_Kp;
//	xAnglePIDParameter4.Ki = Angle_Ki;
//	xAnglePIDParameter4.Kd = Angle_Kd;
//	User_PID_Init(&xPositionPIDParameter5);
//	xPositionPIDParameter5.Kp = Position_Kp;
//	xPositionPIDParameter5.Ki = Position_Ki;
//	xPositionPIDParameter5.Kd = Position_Kd;
//	User_PID_Init(&yPositionPIDParameter6);
//	yPositionPIDParameter6.Kp = Position_Kp;
//	yPositionPIDParameter6.Ki = Position_Ki;
//	yPositionPIDParameter6.Kd = Position_Kd;
//	
//	
//}

//void AngleControl(unsigned int dir)
//{
//		Error[dir] = DirectionError_Calc(dir);
//		Error[(dir+2)%4] = DirectionError_Calc((dir+2)%4);
//		HBPosition = Error[dir] +/*-*/ Error[(dir+2)%4];
//		Angle = HBPosition;	//approximately equ
//		a_speed = User_PID_Calc_angle(&xAnglePIDParameter4,Angle,AngleSet);
//}
//	
//	

//void PositionControl(unsigned int dir)
//{
//	Error[dir] = DirectionError_Calc(dir);
//	switch(dir)
//	{
//		case 0:
//		case 2:
//		y_speed = User_PID_Calc_angle(&yPositionPIDParameter6,Error[dir],PositionSet);
//		break;
//		case 1:
//		case 4:
//		x_speed = User_PID_Calc_angle(&xPositionPIDParameter5,Error[dir],PositionSet);
//	  break;
//	}
//}


int32_t DirectionError_Calc(int32_t direction)//0,1,2,3 前右后左
{
	temp=0;
	uint8_t cnt=0;
	for(xunhuan=SegOFFSET_JG[direction];xunhuan<SENSORNUM+SegOFFSET_JG[direction];xunhuan++)
	{
		temp += ErrorRank[xunhuan-SegOFFSET_JG[direction]] * Sensor_JG_Buffer[xunhuan];
		if(Sensor_JG_Buffer[xunhuan]==1) cnt++;
	}
	if(cnt>1)//有两个以上识别到黑线
	{
		if (temp>0) temp=ErrorRank[2];
		else if (temp<0) temp=ErrorRank[1];
//		temp=0;
	}
	if(direction%2==1)//左右时
	{
		if(temp==-1) temp=-2;
		else if(temp==1) temp=2;
		
	}
//	if(direction==1)//右时
//	{
//		if(temp==-1) temp=-2;
//	}
//	else if (direction==3)//左时
//	{
//		if(temp==1) temp=2;
//	}
	
//	if(direction == BACK||direction == LEFT)
//		temp = -temp;
	return temp;
}

void GetSensorData()
{
	//读16个灰度
//	Sensor_JG_Buffer[index_gray[0]] = HAL_GPIO_ReadPin(OUT1_Port,OUT1_Pin);
//	Sensor_JG_Buffer[index_gray[1]] = HAL_GPIO_ReadPin(OUT2_Port,OUT2_Pin);
//	Sensor_JG_Buffer[index_gray[2]] = HAL_GPIO_ReadPin(OUT3_Port,OUT3_Pin);
//	Sensor_JG_Buffer[index_gray[3]] = HAL_GPIO_ReadPin(OUT4_Port,OUT4_Pin);
//	Sensor_JG_Buffer[index_gray[4]] = HAL_GPIO_ReadPin(OUT5_Port,OUT5_Pin);
//	Sensor_JG_Buffer[index_gray[5]] = HAL_GPIO_ReadPin(OUT6_Port,OUT6_Pin);
//	Sensor_JG_Buffer[index_gray[6]] = HAL_GPIO_ReadPin(OUT7_Port,OUT7_Pin);
//	Sensor_JG_Buffer[index_gray[7]] = HAL_GPIO_ReadPin(OUT8_Port,OUT8_Pin);
//	Sensor_JG_Buffer[index_gray[8]] = HAL_GPIO_ReadPin(OUT9_Port,OUT9_Pin);
//	Sensor_JG_Buffer[index_gray[9]] = HAL_GPIO_ReadPin(OUT10_Port,OUT10_Pin);
//	Sensor_JG_Buffer[index_gray[10]] = HAL_GPIO_ReadPin(OUT11_Port,OUT11_Pin);
//	Sensor_JG_Buffer[index_gray[11]] = HAL_GPIO_ReadPin(OUT12_Port,OUT12_Pin);
//	Sensor_JG_Buffer[index_gray[12]] = HAL_GPIO_ReadPin(OUT13_Port,OUT13_Pin);
//	Sensor_JG_Buffer[index_gray[13]] = HAL_GPIO_ReadPin(OUT14_Port,OUT14_Pin);
//	Sensor_JG_Buffer[index_gray[14]] = HAL_GPIO_ReadPin(OUT15_Port,OUT15_Pin);
//	Sensor_JG_Buffer[index_gray[15]] = HAL_GPIO_ReadPin(OUT16_Port,OUT16_Pin);
	
//	Sensor_JG_Buffer[0] = HAL_GPIO_ReadPin(OUT1_Port,OUT1_Pin);
//	Sensor_JG_Buffer[1] = HAL_GPIO_ReadPin(OUT2_Port,OUT2_Pin);
//	Sensor_JG_Buffer[2] = HAL_GPIO_ReadPin(OUT3_Port,OUT3_Pin);
//	Sensor_JG_Buffer[3] = HAL_GPIO_ReadPin(OUT4_Port,OUT4_Pin);
//	Sensor_JG_Buffer[4] = HAL_GPIO_ReadPin(OUT5_Port,OUT5_Pin);
//	Sensor_JG_Buffer[5] = HAL_GPIO_ReadPin(OUT6_Port,OUT6_Pin);
//	Sensor_JG_Buffer[6] = HAL_GPIO_ReadPin(OUT7_Port,OUT7_Pin);
//	Sensor_JG_Buffer[7] = HAL_GPIO_ReadPin(OUT8_Port,OUT8_Pin);
//	Sensor_JG_Buffer[8] = HAL_GPIO_ReadPin(OUT9_Port,OUT9_Pin);
//	Sensor_JG_Buffer[9] = HAL_GPIO_ReadPin(OUT10_Port,OUT10_Pin);
//	Sensor_JG_Buffer[10] = HAL_GPIO_ReadPin(OUT11_Port,OUT11_Pin);
//	Sensor_JG_Buffer[11] = HAL_GPIO_ReadPin(OUT12_Port,OUT12_Pin);
//	Sensor_JG_Buffer[12] = HAL_GPIO_ReadPin(OUT13_Port,OUT13_Pin);
//	Sensor_JG_Buffer[13] = HAL_GPIO_ReadPin(OUT14_Port,OUT14_Pin);
//	Sensor_JG_Buffer[14] = HAL_GPIO_ReadPin(OUT15_Port,OUT15_Pin);
//	Sensor_JG_Buffer[15] = HAL_GPIO_ReadPin(OUT16_Port,OUT16_Pin);

	Sensor_JG_Buffer[0] = HAL_GPIO_ReadPin(OUT1_Port,OUT1_Pin);
	Sensor_JG_Buffer[1] = HAL_GPIO_ReadPin(OUT2_Port,OUT2_Pin);
	Sensor_JG_Buffer[2] = HAL_GPIO_ReadPin(OUT11_Port,OUT11_Pin);//HAL_GPIO_ReadPin(OUT3_Port,OUT3_Pin);
	Sensor_JG_Buffer[3] = HAL_GPIO_ReadPin(OUT10_Port,OUT10_Pin);//HAL_GPIO_ReadPin(OUT4_Port,OUT4_Pin);
	Sensor_JG_Buffer[4] = HAL_GPIO_ReadPin(OUT9_Port,OUT9_Pin);//HAL_GPIO_ReadPin(OUT5_Port,OUT5_Pin);
	Sensor_JG_Buffer[5] = HAL_GPIO_ReadPin(OUT12_Port,OUT12_Pin);//HAL_GPIO_ReadPin(OUT6_Port,OUT6_Pin);
	Sensor_JG_Buffer[6] = HAL_GPIO_ReadPin(OUT8_Port,OUT8_Pin);//HAL_GPIO_ReadPin(OUT7_Port,OUT7_Pin);
	Sensor_JG_Buffer[7] = HAL_GPIO_ReadPin(OUT16_Port,OUT16_Pin);//HAL_GPIO_ReadPin(OUT8_Port,OUT8_Pin);--
	Sensor_JG_Buffer[8] = HAL_GPIO_ReadPin(OUT5_Port,OUT5_Pin);//HAL_GPIO_ReadPin(OUT9_Port,OUT9_Pin);
	Sensor_JG_Buffer[9] = HAL_GPIO_ReadPin(OUT6_Port,OUT6_Pin);//HAL_GPIO_ReadPin(OUT10_Port,OUT10_Pin);
	Sensor_JG_Buffer[10] = HAL_GPIO_ReadPin(OUT15_Port,OUT15_Pin);//HAL_GPIO_ReadPin(OUT11_Port,OUT11_Pin);
	Sensor_JG_Buffer[11] = HAL_GPIO_ReadPin(OUT13_Port,OUT13_Pin);//HAL_GPIO_ReadPin(OUT12_Port,OUT12_Pin);
	Sensor_JG_Buffer[12] = HAL_GPIO_ReadPin(OUT14_Port,OUT14_Pin);//HAL_GPIO_ReadPin(OUT13_Port,OUT13_Pin);
	Sensor_JG_Buffer[13] = HAL_GPIO_ReadPin(OUT7_Port,OUT7_Pin);//HAL_GPIO_ReadPin(OUT14_Port,OUT14_Pin);
	Sensor_JG_Buffer[14] = HAL_GPIO_ReadPin(OUT3_Port,OUT3_Pin);//HAL_GPIO_ReadPin(OUT15_Port,OUT15_Pin);
	Sensor_JG_Buffer[15] = HAL_GPIO_ReadPin(OUT4_Port,OUT4_Pin);//HAL_GPIO_ReadPin(OUT16_Port,OUT16_Pin);

	//读6个激光
	Sensor_JG_Buffer[16] = HAL_GPIO_ReadPin(SJ1_GPIO_Port,SJ1_Pin);
	Sensor_JG_Buffer[17] = HAL_GPIO_ReadPin(SJ2_GPIO_Port,SJ2_Pin);
	Sensor_JG_Buffer[18] = HAL_GPIO_ReadPin(SJ3_GPIO_Port,SJ3_Pin);
	Sensor_JG_Buffer[19] = HAL_GPIO_ReadPin(SJ4_GPIO_Port,SJ4_Pin);
	Sensor_JG_Buffer[20] = HAL_GPIO_ReadPin(SJ5_GPIO_Port,SJ5_Pin);
	Sensor_JG_Buffer[21] = HAL_GPIO_ReadPin(SJ6_GPIO_Port,SJ6_Pin);

	for(xunhuan=0;xunhuan<TOTAL_SENSORNUM;xunhuan++)
	{
		if(Sensor_JG_Buffer[xunhuan] == BLACK)
			Sensor_JG_Buffer[xunhuan]= 1;
		else
			Sensor_JG_Buffer[xunhuan]= 0;
	}
	//1 -- online, 0 -- offline
}

static void GetSensorDataRaw()
{
	//读16个灰度
	Sensor_Check_Buffer[0] = HAL_GPIO_ReadPin(OUT1_Port,OUT1_Pin);
	Sensor_Check_Buffer[1] = HAL_GPIO_ReadPin(OUT2_Port,OUT2_Pin);
	Sensor_Check_Buffer[2] = HAL_GPIO_ReadPin(OUT3_Port,OUT3_Pin);
	Sensor_Check_Buffer[3] = HAL_GPIO_ReadPin(OUT4_Port,OUT4_Pin);
	Sensor_Check_Buffer[4] = HAL_GPIO_ReadPin(OUT5_Port,OUT5_Pin);
	Sensor_Check_Buffer[5] = HAL_GPIO_ReadPin(OUT6_Port,OUT6_Pin);
	Sensor_Check_Buffer[6] = HAL_GPIO_ReadPin(OUT7_Port,OUT7_Pin);
	Sensor_Check_Buffer[7] = HAL_GPIO_ReadPin(OUT8_Port,OUT8_Pin);
	Sensor_Check_Buffer[8] = HAL_GPIO_ReadPin(OUT9_Port,OUT9_Pin);
	Sensor_Check_Buffer[9] = HAL_GPIO_ReadPin(OUT10_Port,OUT10_Pin);
	Sensor_Check_Buffer[10] = HAL_GPIO_ReadPin(OUT11_Port,OUT11_Pin);
	Sensor_Check_Buffer[11] = HAL_GPIO_ReadPin(OUT12_Port,OUT12_Pin);
	Sensor_Check_Buffer[12] = HAL_GPIO_ReadPin(OUT13_Port,OUT13_Pin);
	Sensor_Check_Buffer[13] = HAL_GPIO_ReadPin(OUT14_Port,OUT14_Pin);
	Sensor_Check_Buffer[14] = HAL_GPIO_ReadPin(OUT15_Port,OUT15_Pin);
	Sensor_Check_Buffer[15] = HAL_GPIO_ReadPin(OUT16_Port,OUT16_Pin);

	//读6个激光
	Sensor_Check_Buffer[16] = HAL_GPIO_ReadPin(SJ1_GPIO_Port,SJ1_Pin);
	Sensor_Check_Buffer[17] = HAL_GPIO_ReadPin(SJ2_GPIO_Port,SJ2_Pin);
	Sensor_Check_Buffer[18] = HAL_GPIO_ReadPin(SJ3_GPIO_Port,SJ3_Pin);
	Sensor_Check_Buffer[19] = HAL_GPIO_ReadPin(SJ4_GPIO_Port,SJ4_Pin);
	Sensor_Check_Buffer[20] = HAL_GPIO_ReadPin(SJ5_GPIO_Port,SJ5_Pin);
	Sensor_Check_Buffer[21] = HAL_GPIO_ReadPin(SJ6_GPIO_Port,SJ6_Pin);

	for(xunhuan=0;xunhuan<TOTAL_SENSORNUM;xunhuan++)
	{
		if(Sensor_Check_Buffer[xunhuan] == BLACK)
			Sensor_Check_Buffer[xunhuan]= 1;
		else
			Sensor_Check_Buffer[xunhuan]= 0;
	}
	//1 -- online, 0 -- offline
}
//调用此函数前，确认所有传感器在白色环境，
//调用此函数,蜂鸣器会循环鸣响3声,
//听到蜂鸣器鸣响第三声时，使第一个传感器（前方最左侧）处于黑色状态
//待下一波3声鸣响开始第一声后，这个传感器的顺序数据就被保存了
//听到这一波蜂鸣器鸣响第三声时，使第二个传感器（前方自左向右第2个）处于黑色状态
//待下一波3声鸣响开始第一声后，这个传感器的顺序数据就被保存了
//依次顺时针重复上面的操作16次
//这样，16个传感器的连接关系便会由程序自动保存，车辆可正常循迹
void CheckSensorData()
{
	uint8_t i,j;
	for(j=0;j<16;j++){
		if(j%4==0) HAL_Delay(1000);
		music2Play();
		GetSensorDataRaw();
		
		for(i=0;i<16;i++)
			if(Sensor_Check_Buffer[i]==BLACK)
			{
				index_gray[j]=i;
				break;
			}
	}
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC), csd, CheckSensorData, CheckSensorData());

/**
  * @brief  将与传感器顺序有关的缓冲区变量存入内部Flash的
	* 扇区10(0x080C 0000 - 0x080D FFFF)中,此扇区可存储128kByte数据
  * @retval 无
  */
void SaveSensor2F()
{
	int i = 0;
	uint32_t PageError = 0;
	FLASH_EraseInitTypeDef a;
	HAL_StatusTypeDef status;
	uint32_t addr = 0x080C0000;
	uint32_t data_buf[10];
	
	/* 读取Flash内容 */
	memcpy(data_buf, (uint32_t*)addr, sizeof(uint32_t)*10);
	printf("read before erase:\r\n\t");
	for(i = 0;i < 10;i++)
	{
		printf("0x%08x ", data_buf[i]);
	}
	printf("\r\n");
	
	/*定义要擦除的部分*/
	a.TypeErase = FLASH_TYPEERASE_SECTORS;
	a.Banks = FLASH_BANK_1;
	a.Sector = FLASH_SECTOR_10;
	a.NbSectors = 1;
	a.VoltageRange = FLASH_VOLTAGE_RANGE_3;

	HAL_FLASH_Unlock();
	status = HAL_FLASHEx_Erase(&a,&PageError);/*擦除扇区11的内容*/
	HAL_FLASH_Lock();
	if(status != HAL_OK)
	{
		printf("erase fail, PageError = %d\r\n", PageError);
	}
	else
		printf("erase success\r\n");

	/* 读取Flash内容 */
	memcpy(data_buf, (uint32_t*)addr, sizeof(uint32_t)*10);
	printf("read after erase:\r\n\t");
	for(i = 0;i < 10;i++)
	{
		printf("0x%08x ", data_buf[i]);
	}
	printf("\r\n");
	
	//写入Flash内容
	HAL_FLASH_Unlock();
	
	{
		uint16_t cnt=0;//从第0个位置开始存数据
			
			/*----开始存传感器顺序数据，共16字节*/
			for(uint8_t i=0;i<16;i++)//遍历16个数据
			{
				HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, addr+cnt, index_gray[i]);//存ID
				cnt += 1;
			}
			//存数据结束
	}
	
	HAL_FLASH_Lock();
	
	if(status != HAL_OK)
	{
		printf("write fail\r\n");
	}
	else
	{
		printf("write success\r\n");
	}

	/* 读取Flash内容 */
	addr = 0x080C0000;
	memcpy(data_buf, (uint32_t*)addr, sizeof(uint32_t)*10);
	printf("read after write:\r\n\t");
	for(i = 0;i < 10;i++)
	{
		printf("0x%08x ", data_buf[i]);
	}
	printf("\r\n");

}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC), ssenf, SaveSensor2F, SaveSensor2F());

/**
  * @brief  将Flash中的传感器顺序数据读至缓冲区
  * @retval 无
  */
void readSensorF2ram()
{
	uint16_t cnt=0;
	uint32_t addr = 0x080C0000;
	//读传感器顺序数据，长度已知为16
	for(uint8_t i=0;i<16;i++)//遍历16个数据
	{
		index_gray[i] = *(uint8_t *)(addr+cnt);//读ID
		cnt += 1;
	}
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC), rsenf, readSensorF2ram, readSensorF2ram());

//判断是否在格点
unsigned int  AtGridPosition(unsigned int dir)//返回为1，代表小车达到格点
{
	/*
	buffer
	前行： 5 6 (左侧先后),14 13(右侧先后)
	后行：6 5 (左侧先后) 13 14 (右侧先后)
	左行：1 2 (前侧先后)10 9(后侧先后)
	右行：2 1(后侧先后) 9 10(后侧先后)
	*/
	//Sensor_JG_Buffer[0]
//	if(x_position == 6 && y_position == 0)
//		;
	
	switch(dir)
		{
			case 0://往前
				if(Sensor_JG_Buffer[5]||Sensor_JG_Buffer[14])
//				if(Sensor_JG_Buffer[4]||Sensor_JG_Buffer[15])
//			if((Sensor_JG_Buffer[5] && Sensor_JG_Buffer[4])||(Sensor_JG_Buffer[14] && Sensor_JG_Buffer[15] ))
					return 1;
				else
					return 0;
			case 1://往右
				if(Sensor_JG_Buffer[2]||Sensor_JG_Buffer[9])
//				if(Sensor_JG_Buffer[3]||Sensor_JG_Buffer[8])
					return 1;
				else
					return 0;
			case 2://往后
				if(Sensor_JG_Buffer[6]||Sensor_JG_Buffer[13])
//				if(Sensor_JG_Buffer[7]||Sensor_JG_Buffer[12])
					return 1;
				else
					return 0;
			case 3://往左
//				if(Sensor_JG_Buffer[0]||Sensor_JG_Buffer[11])
				if(Sensor_JG_Buffer[1]||Sensor_JG_Buffer[10])
					return 1;
				else
					return 0;	
		}
		return 0;
}
//判断是否在格点
unsigned int  AtGridPosition_sp(uint8_t dir,uint8_t next)//返回为1，代表小车达到格点
{
	uint8_t temp[2];
	/*
	buffer
	前行： 5 6 (左侧先后),14 13(右侧先后)
	后行：6 5 (左侧先后) 13 14 (右侧先后)
	左行：1 2 (前侧先后)10 9(后侧先后)
	右行：2 1(后侧先后) 9 10(后侧先后)
	*/
	//Sensor_JG_Buffer[0]
//	if(x_position == 6 && y_position == 0)
//		;

	switch(dir)
	{
		//temp[0]--当前行进方向右侧的灰度
		//temp[1]--当前行进方向左侧的灰度
		case 0://往前
				temp[0]=5;
				temp[1]=14;
		break;
		case 1://往右
				temp[0]=9;
				temp[1]=2;
		break;
		case 2://往后
				temp[0]=13;
				temp[1]=6;
		break;
		case 3://往左
				temp[0]=1;
				temp[1]=10;
		break;
	}
	if(next%2!=dir%2)
	{
		if(((3+next-dir)%4)/2)
		{//下一次右转，当前方向左侧的灰度到黑线，判断为为到达格点
			return(Sensor_JG_Buffer[temp[1]]);
		}
		else
		{
			return(Sensor_JG_Buffer[temp[0]]);
		}
	}
	else
		return(Sensor_JG_Buffer[temp[0]]||Sensor_JG_Buffer[temp[1]]);
}

unsigned int  FromGridPosition(unsigned int dir)//返回为1，代表小车离开格点
{
	/*
	buffer
	前行： 5 6 (左侧先后),14 13(右侧先后)
	后行：6 5 (左侧先后) 13 14 (右侧先后)
	左行：1 2 (前侧先后)10 9(后侧先后)
	右行：2 1(后侧先后) 9 10(后侧先后)
	*/

	switch(dir)
		{
			case 0://往前
			case 2://往后
//				if(Sensor_JG_Buffer[5]||Sensor_JG_Buffer[6]||Sensor_JG_Buffer[13]||Sensor_JG_Buffer[14])
				if(Sensor_JG_Buffer[4]||Sensor_JG_Buffer[5]||Sensor_JG_Buffer[6]||Sensor_JG_Buffer[7]||Sensor_JG_Buffer[12]||Sensor_JG_Buffer[13]||Sensor_JG_Buffer[14]||Sensor_JG_Buffer[15])
					return 0;
				else
					return 1;
			case 1://往右
			case 3://往左
//				if(Sensor_JG_Buffer[2]||Sensor_JG_Buffer[1]||Sensor_JG_Buffer[9]||Sensor_JG_Buffer[10])
				if(Sensor_JG_Buffer[3]||Sensor_JG_Buffer[2]||Sensor_JG_Buffer[1]||Sensor_JG_Buffer[0]||Sensor_JG_Buffer[8]||Sensor_JG_Buffer[9]||Sensor_JG_Buffer[10]||Sensor_JG_Buffer[11])
						return 0;
				else
					return 1;	
		}
	return 0;
}

//小车向任意方向行进一格，从格点开始到格点停止
void OneGrid(unsigned int dir,int32_t speedOffset)
{	//要不要提前给速度？？？要
	AngleAndPositionTIM=1;
	CarMovingTo = dir;
	switch(dir)
	{
		case 0:	
			x_speed=userSpeed_x+speedOffset;
			break;
		case 1:
			y_speed=userSpeed_y+speedOffset;
			break;
		case 2:
			x_speed=-userSpeed_x-speedOffset;
			break;
		case 3:
			y_speed=-userSpeed_y-speedOffset;
			break;
	}
//	while(FromGridPosition(dir)==0){}
	if(dir%2==0)
	HAL_Delay(400);
	else
	HAL_Delay(800);	
	while(AtGridPosition(dir)==0){}
	
	
	switch(dir)
		{
			case 0:
				CarMovingTo = 0;
				x_speed=0;
			  x_position++;
				break;
			case 1:
				CarMovingTo = 1;
				y_speed=0;
			  y_position--;
				break;
			case 2:
				CarMovingTo = 2;
				x_speed=0;
			  x_position--;
				break;
			case 3:
				CarMovingTo = 3;
				y_speed=0;
				y_position++;
				break;
		}
		
		user_main_printf("I am at (%d,%d).",x_position,y_position);
		
//		AnglePosControl((dir+1)%4);
//		HAL_Delay(20);
//		AnglePosControl((dir+1)%4);
//		HAL_Delay(20);
//		AnglePosControl((dir+1)%4);
//		HAL_Delay(20);
//		AnglePosControl((dir+1)%4);
//		HAL_Delay(20);	
//		AnglePosControl((dir+1)%4);
//		HAL_Delay(20);
//		AnglePosControl((dir+1)%4);
//		HAL_Delay(20);
//		AnglePosControl((dir+1)%4);
//		HAL_Delay(20);
		y_speed=0;
		x_speed=0;

}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC), og, OneGrid, OneGrid(dir,speedoffset));

void OneGrid_sp(unsigned int dir,uint8_t next,int32_t speedOffset)
{	//要不要提前给速度？？？要
	static uint8_t first=0;
	
	AngleAndPositionTIM=1;
	CarMovingTo = dir;
	switch(dir)
	{
		case 0:	
			x_speed=userSpeed_x+speedOffset;
			break;
		case 1:
			y_speed=userSpeed_y+speedOffset;
			break;
		case 2:
			x_speed=-userSpeed_x-speedOffset;
			break;
		case 3:
			y_speed=-userSpeed_y-speedOffset;
			break;
	}
//	while(FromGridPosition(dir)==0){}
	if(dir%2==0)
	HAL_Delay(400);
	else{
		if(!first){
			first=1;
			HAL_Delay(100);
		}
		else
			HAL_Delay(800);
	}
	
	while(AtGridPosition_sp(dir,next)==0){}
	
	
	switch(dir)
		{
			case 0:
				CarMovingTo = 0;
				x_speed=0;
			  x_position++;
				break;
			case 1:
				CarMovingTo = 1;
				y_speed=0;
			  y_position--;
				break;
			case 2:
				CarMovingTo = 2;
				x_speed=0;
			  x_position--;
				break;
			case 3:
				CarMovingTo = 3;
				y_speed=0;
				y_position++;
				break;
		}
		
		user_main_printf("I am at (%d,%d).",x_position,y_position);
	
		y_speed=0;
		x_speed=0;

}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC), ogsp, OneGrid, OneGrid(dir,next,speedoffset));


//填写障碍物数组
void FillObsArray()
{
	obs[x_position+1][y_position]=ObsOrNot(FRONT);
	obs[x_position][y_position-1]=ObsOrNot(RIGHT);
	obs[x_position-1][y_position]=ObsOrNot(BACK);
	obs[x_position][y_position+1]=ObsOrNot(LEFT);
}

//返回任意四个相邻格点上的障碍物情况，有障碍物为1，无障碍物为0
unsigned int ObsOrNot(unsigned int dir)
{
	switch(dir)
	{//前后左右是17-20,改改顺序8
		case FRONT:
			if(x_position==7)
				return 1;//超出地图的虚拟墙
			else
				return !HAL_GPIO_ReadPin(OUT17_Port,OUT17_Pin);
		case RIGHT:
			if(y_position==0)
				return 1;
			else
				return !HAL_GPIO_ReadPin(OUT18_Port,OUT18_Pin);
		case BACK:
			if(x_position==0)
				return 1;
			else 
				return !HAL_GPIO_ReadPin(OUT19_Port,OUT19_Pin);
		case LEFT:
			if(y_position==4)
				return 1;
			else
				return !HAL_GPIO_ReadPin(OUT20_Port,OUT20_Pin);
	}
	return 0;
}

//判断起点位置,整个工程只执行一次
void WhereAmI()//右侧出发，用手把前方最左侧的激光头挡住
{
  if(HAL_GPIO_ReadPin(OUT1_Port,OUT1_Pin) == 1)
		{//只要----，说明在右侧起点，否则为左侧起点
    BEGIN_X = 7;
    BEGIN_Y = 0;
    END_X = 0;
    END_Y = 4;
    START_MODE = 1;
		} 
	else
		{
    BEGIN_X = 0;
    BEGIN_Y = 0;
    END_X = 7;
    END_Y = 4;
    START_MODE = 0;
		}
  x_position = BEGIN_X;
  y_position = BEGIN_Y;
}

//迷宫算法走至终点
void MazeExplore()
{
	static unsigned int x_destination=0;
	static unsigned int y_destination=0;
	static unsigned dir=0;//指挥每一次迷宫的行进方向
	if(y_position==0)//去丢瓶子
		{
			x_destination=END_X;
			y_destination=END_Y;
			if( START_MODE )//给出迷宫算法的第一步，设定初始右手
				{//从右侧开始
					if(ObsOrNot(LEFT) == 0)
						{                           
							dir = LEFT;
						} 
					else
						{
							dir = BACK;
						}
				}
			else
				{//从左侧开始
					if(ObsOrNot(FRONT) == 0)
						{                           
							dir = FRONT;
						} 
					else 
						{
							dir = LEFT;   
						}
				}
		}
	else//回装瓶子
		{
			x_destination=BEGIN_X;
			y_destination=BEGIN_Y;
			if( START_MODE )//给出迷宫算法的第一步，设定初始右手
				{//从右侧开始
					if(ObsOrNot(RIGHT) == 0)
						{                           
							dir = RIGHT;
						} 
					else
						{
							dir = FRONT;
						}
				}
			else
				{//从左侧开始
					if(ObsOrNot(BACK) == 0)
						{                           
							dir = BACK;
						} 
					else 
						{
							dir = RIGHT;   
						}
				}
		
		
		}
  while(x_position!=x_destination||y_position!=y_destination)
		{ 
			OneGrid(dir,0);
			
			CarMovingTo_last = dir;//上次走完，存储上次行走的方向
			
			//计算下次行走方向
			dir=(dir+1)%4; 
			while(ObsOrNot(dir))
			{//没有障碍物
				dir=(dir+3)%4;
			}
			
			//下次行走方向与上一次不同（即下一次要转弯），则等一会再转，确保转向稳定
//			if(dir != CarMovingTo_last)
//				HAL_Delay(200);
		}
		
//		if(x_position == BEGIN_X && y_position == BEGIN_Y)	
//		{
//			AngleAndPositionTIM = 0;
//			x_speed = 0;
//			y_speed = 0;
//			a_speed = 0;
//			
//		}
		
}

/*
终点投递的两次转身
OUT1		OUT2		OUT3		OUT4
buffer0	buffer1	buffer2	buffer3
前方左2	前方左1	前方右1	前方右2

OUT5		OUT6		OUT7		OUT8
buffer4	buffer5	buffer6	buffer7
右方左2	右方左1	右方右1	右方右2

OUT9		OUT10		OUT11		OUT12
buffer8	buffer9 buffer10	buffer11
后方左2	后方左1	后方右1	后方右2

OUT13		OUT14		OUT15		OUT16
buffer12	buffer13	buffer14	buffer15
左方左2	左方左1	左方右1	左方右2
*/
void TurnAndBack()
{//需要分两个终点分别讨论
	AngleAndPositionTIM=0;
	if(x_position==7)
		{
			if(Sensor_JG_Buffer[5]||Sensor_JG_Buffer[6])//判断顺逆时针转，投递前逆转，投递完顺转
				{//右侧有线，准备逆转
					while(!Sensor_JG_Buffer[13]&&!Sensor_JG_Buffer[14])//左方左1和右1任意为1才跳出while
					{
						a_speed=30;
					}
					AnglePosControl(0);
					AnglePosControl(1);
					HAL_Delay(20);
					AnglePosControl(0);
					AnglePosControl(1);
					HAL_Delay(20);
					AnglePosControl(0);
					AnglePosControl(1);
					HAL_Delay(20);
					AnglePosControl(0);
					AnglePosControl(1);
					HAL_Delay(20);
					AnglePosControl(0);
					AnglePosControl(1);
					HAL_Delay(20);				
					a_speed=0;
					x_speed=0;
					y_speed=0;
					
//					a_speed=30;
//					HAL_Delay(200);
//					a_speed=0;
					
				}
			else
				{
					while(!Sensor_JG_Buffer[5]/*&&!Sensor_JG_Buffer[7]*/)//右方左1和右1任意为1才跳出while
						{
							a_speed=-30;
						}
					AnglePosControl(0);
					AnglePosControl(1);
					HAL_Delay(20);
					AnglePosControl(0);
					AnglePosControl(1);
					HAL_Delay(20);
					AnglePosControl(0);
					AnglePosControl(1);
					HAL_Delay(20);
					AnglePosControl(0);
					AnglePosControl(1);
					HAL_Delay(20);
					AnglePosControl(0);
					AnglePosControl(1);
					HAL_Delay(20);				
					a_speed=0;
					a_speed=0;
				}
		}
	else 
		{
			if(Sensor_JG_Buffer[1]||Sensor_JG_Buffer[2])//判断顺逆时针转，投递前逆转，投递完顺转
				{//前侧有线，准备逆转
					while(!Sensor_JG_Buffer[9]&&!Sensor_JG_Buffer[10])//后方左1和右1同时为1才跳出while
					{
						a_speed=30;
					}
					AnglePosControl(0);
					AnglePosControl(1);
					HAL_Delay(20);
					AnglePosControl(0);
					AnglePosControl(1);
					HAL_Delay(20);
					AnglePosControl(0);
					AnglePosControl(1);
					HAL_Delay(20);
					AnglePosControl(0);
					AnglePosControl(1);
					HAL_Delay(20);
					AnglePosControl(0);
					AnglePosControl(1);
					HAL_Delay(20);				
					a_speed=0;
					a_speed=0;
				}
			else
				{
					while(!Sensor_JG_Buffer[1]&&!Sensor_JG_Buffer[2])//前方左1和右1同时为1才跳出while
						{
							a_speed=-30;
						}
						AnglePosControl(0);
					AnglePosControl(1);
					HAL_Delay(20);
					AnglePosControl(0);
					AnglePosControl(1);
					HAL_Delay(20);
					AnglePosControl(0);
					AnglePosControl(1);
					HAL_Delay(20);
					AnglePosControl(0);
					AnglePosControl(1);
					HAL_Delay(20);
					AnglePosControl(0);
					AnglePosControl(1);
					HAL_Delay(20);				
					a_speed=0;
					a_speed=0;
				}
		}
		
	Time3_ms = 0;
	AngleAndPositionTIM=1;
}

void ni(int32_t speed_)
{
	AngleAndPositionTIM=0;
	x_speed=0;
	y_speed=0;
	a_speed=-speed_;
	HAL_Delay(900);
	while(Sensor_JG_Buffer[1]!=BLACK);
	a_speed=0;
	CarMovingTo=0;
	AngleAndPositionTIM=1;
}

void Grid_Lock()
{
	LockSetPID();
	lockFlag=1;
}

void Grid_UnLock()
{
	lockFlag=0;
	LockResetPID();
}

