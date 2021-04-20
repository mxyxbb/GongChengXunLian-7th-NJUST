#ifndef _lineFollowSensor_H
#define _lineFollowSensor_H

#include "stm32f4xx_hal.h"

#define SENSORNUM 4
#define DIRECTION 4

#define FRONT 0
#define RIGHT 1
#define BACK 2
#define LEFT 3
//k=((3+next-dir)%4)/2;	//k=0-下一次右转,k=1-下一次左转
//dir		0	0	0	0	1	1	1	1	2	2	2	2	3	3	3	3
//next	0	1	2	3	0	1	2	3	0	1	2	3	0	1	2	3	
//k			/	0	/	1	1	/	0 / / 1 / 0 0 / 1 /														

/*
端口连接
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
#define OUT1_Port GPIOE
#define OUT1_Pin	GPIO_PIN_2
#define OUT2_Port GPIOE
#define OUT2_Pin	GPIO_PIN_3
#define OUT3_Port GPIOE
#define OUT3_Pin	GPIO_PIN_4
#define OUT4_Port GPIOE
#define OUT4_Pin	GPIO_PIN_5
#define OUT5_Port GPIOE
#define OUT5_Pin	GPIO_PIN_6
#define OUT6_Port GPIOC
#define OUT6_Pin	GPIO_PIN_13
#define OUT7_Port GPIOC
#define OUT7_Pin	GPIO_PIN_14
#define OUT8_Port GPIOC
#define OUT8_Pin	GPIO_PIN_15
#define OUT9_Port GPIOB
#define OUT9_Pin	GPIO_PIN_8
#define OUT10_Port GPIOB
#define OUT10_Pin	GPIO_PIN_9
#define OUT11_Port GPIOE
#define OUT11_Pin	GPIO_PIN_0
#define OUT12_Port GPIOE
#define OUT12_Pin	GPIO_PIN_1
#define OUT13_Port GPIOC
#define OUT13_Pin	GPIO_PIN_0
#define OUT14_Port GPIOC
#define OUT14_Pin	GPIO_PIN_1
#define OUT15_Port GPIOC
#define OUT15_Pin	GPIO_PIN_2
#define OUT16_Port GPIOC
#define OUT16_Pin	GPIO_PIN_3
//加上四个光电开关
#define OUT17_Port GPIOE
#define OUT17_Pin	GPIO_PIN_14
#define OUT18_Port GPIOE
#define OUT18_Pin	GPIO_PIN_15
#define OUT19_Port GPIOB
#define OUT19_Pin	GPIO_PIN_10
#define OUT20_Port GPIOB
#define OUT20_Pin	GPIO_PIN_11
#define userSpeed 35
#define userSpeed_x 30 //30
#define userSpeed_y 15 //15
#define TURN90 800


extern int32_t LineSensor[SENSORNUM];
extern int32_t ErrorRank[SENSORNUM];
extern int32_t temp;
extern int32_t xunhuan;
extern int32_t Error[DIRECTION]; 
extern int32_t HBPosition;
extern int32_t Angle;
extern int32_t Sensor_JG_Buffer[30];//JG == ji guang
extern int32_t SegOFFSET_JG[DIRECTION];
extern int32_t Angle_PIDControlVal;
extern int32_t AngleSet;
extern int32_t PositionSet;
extern int32_t CarMovingTo;

void LineFollowInit(void);
void GetSensorData(void);
int32_t DirectionError_Calc(int32_t direction);//0,1,2,3 前右后左
//void AngleControl(unsigned int dir);
//void PositionControl(unsigned int dir);
void AnglePosControl(unsigned int dir);
void OneGrid(unsigned int dir,int32_t speedOffset);
unsigned int AtGridPosition(unsigned int dir);
unsigned int FromGridPosition(unsigned int dir);
unsigned int ObsOrNot(unsigned int dir);
void FillObsArray(void);
void WhereAmI(void);
void MazeExplore(void);
void TurnAndBack(void);
void WhereAmI(void);//右侧出发，用手把前方最左侧的激光头挡住
void ni(int32_t speed_);
void Grid_Lock(void);
void Grid_UnLock(void);
void OneGrid_sp(unsigned int dir,uint8_t next,int32_t speedOffset);
#endif

