#ifndef _ControlTask_H
#define _ControlTask_H

#include "stm32f4xx_hal.h"

#define zhMotorEncoder 0
#define zqMotorEncoder 1
#define yhMotorEncoder 3
#define yqMotorEncoder 2

extern uint8_t lockFlag;
extern uint8_t AngleAndPositionTIM;
extern int32_t motorspeed[4];
extern uint8_t tim6enable;
extern uint8_t lockFlag;
extern uint8_t musicenable2;

extern uint8_t distanceStart;
extern int32_t distance;

extern uint16_t Time1_ms;
extern uint16_t Time2_ms;
extern uint16_t Time3_ms;

#endif
