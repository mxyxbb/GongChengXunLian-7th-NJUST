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



extern void ArmForceEnable(uint8_t ID_,uint8_t Enable_);
extern void SavePos(int16_t ID_,int16_t timems_);
extern void GoPos(int16_t ID_);
	


#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

