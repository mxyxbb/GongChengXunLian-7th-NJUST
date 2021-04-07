#include "MotorControl.h"
#include "ControlTask.h"
#include "lineFollowSensor.h"
#include "PID.h"
#include "user_usart.h"
#include <stdio.h>
#include "main.h"
#include "letter_shell/src/shell_port.h"
#include "Buzzer/buzzerDriver.h"
#include <string.h> //ʹ�õ���memcpy


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

//�����Զ�У���Ҷȴ��������ᱻ�洢��flash��
uint8_t index_gray[16];

extern int32_t y_speed;
extern int32_t x_speed;
extern int32_t a_speed;
extern int32_t obs[5][8];
extern int32_t START_MODE;//ȡ���½���ʼΪ0�����½���ʼΪ1
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
//				xAngleControl();//����ʱ���ô�ֱ�߻�������
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


int32_t DirectionError_Calc(int32_t direction)//0,1,2,3 ǰ�Һ���
{
	temp=0;
	uint8_t cnt=0;
	for(xunhuan=SegOFFSET_JG[direction];xunhuan<SENSORNUM+SegOFFSET_JG[direction];xunhuan++)
	{
		temp += ErrorRank[xunhuan-SegOFFSET_JG[direction]] * Sensor_JG_Buffer[xunhuan];
		if(Sensor_JG_Buffer[xunhuan]==1) cnt++;
	}
	if(cnt>1)//����������ʶ�𵽺���
	{
		if (temp>0) temp=ErrorRank[2];
		else if (temp<0) temp=ErrorRank[1];
//		temp=0;
	}
	if(direction%2==1)//����ʱ
	{
		if(temp==-1) temp=-2;
		else if(temp==1) temp=2;
		
	}
//	if(direction==1)//��ʱ
//	{
//		if(temp==-1) temp=-2;
//	}
//	else if (direction==3)//��ʱ
//	{
//		if(temp==1) temp=2;
//	}
	
//	if(direction == BACK||direction == LEFT)
//		temp = -temp;
	return temp;
}

void GetSensorData()
{
	//��16���Ҷ�
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

	//��6������
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
	//��16���Ҷ�
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

	//��6������
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
//���ô˺���ǰ��ȷ�����д������ڰ�ɫ������
//���ô˺���,��������ѭ������3��,
//�������������������ʱ��ʹ��һ����������ǰ������ࣩ���ں�ɫ״̬
//����һ��3�����쿪ʼ��һ���������������˳�����ݾͱ�������
//������һ�����������������ʱ��ʹ�ڶ�����������ǰ���������ҵ�2�������ں�ɫ״̬
//����һ��3�����쿪ʼ��һ���������������˳�����ݾͱ�������
//����˳ʱ���ظ�����Ĳ���16��
//������16�������������ӹ�ϵ����ɳ����Զ����棬����������ѭ��
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
  * @brief  ���봫����˳���йصĻ��������������ڲ�Flash��
	* ����10(0x080C 0000 - 0x080D FFFF)��,�������ɴ洢128kByte����
  * @retval ��
  */
void SaveSensor2F()
{
	int i = 0;
	uint32_t PageError = 0;
	FLASH_EraseInitTypeDef a;
	HAL_StatusTypeDef status;
	uint32_t addr = 0x080C0000;
	uint32_t data_buf[10];
	
	/* ��ȡFlash���� */
	memcpy(data_buf, (uint32_t*)addr, sizeof(uint32_t)*10);
	printf("read before erase:\r\n\t");
	for(i = 0;i < 10;i++)
	{
		printf("0x%08x ", data_buf[i]);
	}
	printf("\r\n");
	
	/*����Ҫ�����Ĳ���*/
	a.TypeErase = FLASH_TYPEERASE_SECTORS;
	a.Banks = FLASH_BANK_1;
	a.Sector = FLASH_SECTOR_10;
	a.NbSectors = 1;
	a.VoltageRange = FLASH_VOLTAGE_RANGE_3;

	HAL_FLASH_Unlock();
	status = HAL_FLASHEx_Erase(&a,&PageError);/*��������11������*/
	HAL_FLASH_Lock();
	if(status != HAL_OK)
	{
		printf("erase fail, PageError = %d\r\n", PageError);
	}
	else
		printf("erase success\r\n");

	/* ��ȡFlash���� */
	memcpy(data_buf, (uint32_t*)addr, sizeof(uint32_t)*10);
	printf("read after erase:\r\n\t");
	for(i = 0;i < 10;i++)
	{
		printf("0x%08x ", data_buf[i]);
	}
	printf("\r\n");
	
	//д��Flash����
	HAL_FLASH_Unlock();
	
	{
		uint16_t cnt=0;//�ӵ�0��λ�ÿ�ʼ������
			
			/*----��ʼ�洫����˳�����ݣ���16�ֽ�*/
			for(uint8_t i=0;i<16;i++)//����16������
			{
				HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, addr+cnt, index_gray[i]);//��ID
				cnt += 1;
			}
			//�����ݽ���
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

	/* ��ȡFlash���� */
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
  * @brief  ��Flash�еĴ�����˳�����ݶ���������
  * @retval ��
  */
void readSensorF2ram()
{
	uint16_t cnt=0;
	uint32_t addr = 0x080C0000;
	//��������˳�����ݣ�������֪Ϊ16
	for(uint8_t i=0;i<16;i++)//����16������
	{
		index_gray[i] = *(uint8_t *)(addr+cnt);//��ID
		cnt += 1;
	}
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC), rsenf, readSensorF2ram, readSensorF2ram());

//�ж��Ƿ��ڸ��
unsigned int  AtGridPosition(unsigned int dir)//����Ϊ1������С���ﵽ���
{
	/*
	buffer
	ǰ�У� 5 6 (����Ⱥ�),14 13(�Ҳ��Ⱥ�)
	���У�6 5 (����Ⱥ�) 13 14 (�Ҳ��Ⱥ�)
	���У�1 2 (ǰ���Ⱥ�)10 9(����Ⱥ�)
	���У�2 1(����Ⱥ�) 9 10(����Ⱥ�)
	*/
	//Sensor_JG_Buffer[0]
//	if(x_position == 6 && y_position == 0)
//		;
	
	switch(dir)
		{
			case 0://��ǰ
				if(Sensor_JG_Buffer[5]||Sensor_JG_Buffer[14])
//				if(Sensor_JG_Buffer[4]||Sensor_JG_Buffer[15])
//			if((Sensor_JG_Buffer[5] && Sensor_JG_Buffer[4])||(Sensor_JG_Buffer[14] && Sensor_JG_Buffer[15] ))
					return 1;
				else
					return 0;
			case 1://����
				if(Sensor_JG_Buffer[2]||Sensor_JG_Buffer[9])
//				if(Sensor_JG_Buffer[3]||Sensor_JG_Buffer[8])
					return 1;
				else
					return 0;
			case 2://����
				if(Sensor_JG_Buffer[6]||Sensor_JG_Buffer[13])
//				if(Sensor_JG_Buffer[7]||Sensor_JG_Buffer[12])
					return 1;
				else
					return 0;
			case 3://����
//				if(Sensor_JG_Buffer[0]||Sensor_JG_Buffer[11])
				if(Sensor_JG_Buffer[1]||Sensor_JG_Buffer[10])
					return 1;
				else
					return 0;	
		}
		return 0;
}
//�ж��Ƿ��ڸ��
unsigned int  AtGridPosition_sp(uint8_t dir,uint8_t next)//����Ϊ1������С���ﵽ���
{
	uint8_t temp[2];
	/*
	buffer
	ǰ�У� 5 6 (����Ⱥ�),14 13(�Ҳ��Ⱥ�)
	���У�6 5 (����Ⱥ�) 13 14 (�Ҳ��Ⱥ�)
	���У�1 2 (ǰ���Ⱥ�)10 9(����Ⱥ�)
	���У�2 1(����Ⱥ�) 9 10(����Ⱥ�)
	*/
	//Sensor_JG_Buffer[0]
//	if(x_position == 6 && y_position == 0)
//		;

	switch(dir)
	{
		//temp[0]--��ǰ�н������Ҳ�ĻҶ�
		//temp[1]--��ǰ�н��������ĻҶ�
		case 0://��ǰ
				temp[0]=5;
				temp[1]=14;
		break;
		case 1://����
				temp[0]=9;
				temp[1]=2;
		break;
		case 2://����
				temp[0]=13;
				temp[1]=6;
		break;
		case 3://����
				temp[0]=1;
				temp[1]=10;
		break;
	}
	if(next%2!=dir%2)
	{
		if(((3+next-dir)%4)/2)
		{//��һ����ת����ǰ�������ĻҶȵ����ߣ��ж�ΪΪ������
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

unsigned int  FromGridPosition(unsigned int dir)//����Ϊ1������С���뿪���
{
	/*
	buffer
	ǰ�У� 5 6 (����Ⱥ�),14 13(�Ҳ��Ⱥ�)
	���У�6 5 (����Ⱥ�) 13 14 (�Ҳ��Ⱥ�)
	���У�1 2 (ǰ���Ⱥ�)10 9(����Ⱥ�)
	���У�2 1(����Ⱥ�) 9 10(����Ⱥ�)
	*/

	switch(dir)
		{
			case 0://��ǰ
			case 2://����
//				if(Sensor_JG_Buffer[5]||Sensor_JG_Buffer[6]||Sensor_JG_Buffer[13]||Sensor_JG_Buffer[14])
				if(Sensor_JG_Buffer[4]||Sensor_JG_Buffer[5]||Sensor_JG_Buffer[6]||Sensor_JG_Buffer[7]||Sensor_JG_Buffer[12]||Sensor_JG_Buffer[13]||Sensor_JG_Buffer[14]||Sensor_JG_Buffer[15])
					return 0;
				else
					return 1;
			case 1://����
			case 3://����
//				if(Sensor_JG_Buffer[2]||Sensor_JG_Buffer[1]||Sensor_JG_Buffer[9]||Sensor_JG_Buffer[10])
				if(Sensor_JG_Buffer[3]||Sensor_JG_Buffer[2]||Sensor_JG_Buffer[1]||Sensor_JG_Buffer[0]||Sensor_JG_Buffer[8]||Sensor_JG_Buffer[9]||Sensor_JG_Buffer[10]||Sensor_JG_Buffer[11])
						return 0;
				else
					return 1;	
		}
	return 0;
}

//С�������ⷽ���н�һ�񣬴Ӹ�㿪ʼ�����ֹͣ
void OneGrid(unsigned int dir,int32_t speedOffset)
{	//Ҫ��Ҫ��ǰ���ٶȣ�����Ҫ
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
{	//Ҫ��Ҫ��ǰ���ٶȣ�����Ҫ
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


//��д�ϰ�������
void FillObsArray()
{
	obs[x_position+1][y_position]=ObsOrNot(FRONT);
	obs[x_position][y_position-1]=ObsOrNot(RIGHT);
	obs[x_position-1][y_position]=ObsOrNot(BACK);
	obs[x_position][y_position+1]=ObsOrNot(LEFT);
}

//���������ĸ����ڸ���ϵ��ϰ�����������ϰ���Ϊ1�����ϰ���Ϊ0
unsigned int ObsOrNot(unsigned int dir)
{
	switch(dir)
	{//ǰ��������17-20,�ĸ�˳��8
		case FRONT:
			if(x_position==7)
				return 1;//������ͼ������ǽ
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

//�ж����λ��,��������ִֻ��һ��
void WhereAmI()//�Ҳ���������ְ�ǰ�������ļ���ͷ��ס
{
  if(HAL_GPIO_ReadPin(OUT1_Port,OUT1_Pin) == 1)
		{//ֻҪ----��˵�����Ҳ���㣬����Ϊ������
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

//�Թ��㷨�����յ�
void MazeExplore()
{
	static unsigned int x_destination=0;
	static unsigned int y_destination=0;
	static unsigned dir=0;//ָ��ÿһ���Թ����н�����
	if(y_position==0)//ȥ��ƿ��
		{
			x_destination=END_X;
			y_destination=END_Y;
			if( START_MODE )//�����Թ��㷨�ĵ�һ�����趨��ʼ����
				{//���Ҳ࿪ʼ
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
				{//����࿪ʼ
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
	else//��װƿ��
		{
			x_destination=BEGIN_X;
			y_destination=BEGIN_Y;
			if( START_MODE )//�����Թ��㷨�ĵ�һ�����趨��ʼ����
				{//���Ҳ࿪ʼ
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
				{//����࿪ʼ
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
			
			CarMovingTo_last = dir;//�ϴ����꣬�洢�ϴ����ߵķ���
			
			//�����´����߷���
			dir=(dir+1)%4; 
			while(ObsOrNot(dir))
			{//û���ϰ���
				dir=(dir+3)%4;
			}
			
			//�´����߷�������һ�β�ͬ������һ��Ҫת�䣩�����һ����ת��ȷ��ת���ȶ�
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
�յ�Ͷ�ݵ�����ת��
OUT1		OUT2		OUT3		OUT4
buffer0	buffer1	buffer2	buffer3
ǰ����2	ǰ����1	ǰ����1	ǰ����2

OUT5		OUT6		OUT7		OUT8
buffer4	buffer5	buffer6	buffer7
�ҷ���2	�ҷ���1	�ҷ���1	�ҷ���2

OUT9		OUT10		OUT11		OUT12
buffer8	buffer9 buffer10	buffer11
����2	����1	����1	����2

OUT13		OUT14		OUT15		OUT16
buffer12	buffer13	buffer14	buffer15
����2	����1	����1	����2
*/
void TurnAndBack()
{//��Ҫ�������յ�ֱ�����
	AngleAndPositionTIM=0;
	if(x_position==7)
		{
			if(Sensor_JG_Buffer[5]||Sensor_JG_Buffer[6])//�ж�˳��ʱ��ת��Ͷ��ǰ��ת��Ͷ����˳ת
				{//�Ҳ����ߣ�׼����ת
					while(!Sensor_JG_Buffer[13]&&!Sensor_JG_Buffer[14])//����1����1����Ϊ1������while
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
					while(!Sensor_JG_Buffer[5]/*&&!Sensor_JG_Buffer[7]*/)//�ҷ���1����1����Ϊ1������while
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
			if(Sensor_JG_Buffer[1]||Sensor_JG_Buffer[2])//�ж�˳��ʱ��ת��Ͷ��ǰ��ת��Ͷ����˳ת
				{//ǰ�����ߣ�׼����ת
					while(!Sensor_JG_Buffer[9]&&!Sensor_JG_Buffer[10])//����1����1ͬʱΪ1������while
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
					while(!Sensor_JG_Buffer[1]&&!Sensor_JG_Buffer[2])//ǰ����1����1ͬʱΪ1������while
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

