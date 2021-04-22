/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SCS_SERVO_H
#define __SCS_SERVO_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"


typedef struct
{
	int16_t  pos_id;
	int16_t  angle[5];//20-1003 mid:511
	int16_t  timems;
}Pos;
//anglep[0]��1�Ŷ���Ƕȣ�צ�ӣ�
//anglep[1]��2�Ŷ���Ƕȣ�С�ۣ�
//anglep[2]��3�Ŷ���Ƕȣ��бۣ�
//anglep[3]��4�Ŷ���Ƕȣ���ۣ�
//anglep[4]��5�Ŷ���Ƕȣ����̣�


void ArmForceEnable(uint8_t ID_,uint8_t Enable_);
void SavePos(int16_t ID_,int16_t timems_);
void GoPos(int16_t ID_);
void GoPosSP(int16_t ID_);
void ArmInit(void);
void DoGroup(uint8_t ID_);
void ArmGoMiddle(void);
void readF2ram(void);
void calibrateEndArea();
void calibrateMidArea();
	
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

