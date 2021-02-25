#ifndef _GONGXUN_H
#define _GONGXUN_H

#include "stm32f4xx_hal.h"

typedef struct node {
	uint8_t itsOrder;//from 1-6
	uint8_t itsColor;//RGB
}Meterial;

extern Meterial meterial[6];
extern uint8_t queue[6];
extern int32_t y_speed;
extern int32_t x_speed;
extern uint8_t Max7219_String[8];

void OnTheWay(unsigned int vectorFrom,unsigned int vectorTo);
void ManufacturingProcesses();
#endif
