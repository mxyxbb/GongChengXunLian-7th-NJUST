#ifndef _MotorControl_H
#define _MotorControl_H

#include "stm32f4xx_hal.h"

#define maxLinearSpeed 60

#define ZQ_OFFSET 0
#define YQ_OFFSET 0
#define ZH_OFFSET 0
#define YH_OFFSET 0

//according to the real situation which is listed in yuque doc
//#define YOUQIAN_TIM ((TIM_TypeDef *) TIM1_BASE)
//#define ZUOHOU_TIM ((TIM_TypeDef *) TIM1_BASE)
//#define YOUHOU_TIM ((TIM_TypeDef *) TIM1_BASE)
//#define ZUOQIAN_TIM ((TIM_TypeDef *) TIM8_BASE)

//#define YOUQIAN_CCR ((uint32_t) CCR1)
//#define ZUOHOU_CCR ((uint32_t) CCR2)
//#define YOUHOU_CCR ((uint32_t) CCR3)
//#define ZUOQIAN_CCR ((uint32_t) CCR1)



//下面的数值由编码器和MCU的接线方式决定
//若车轮正转，脉冲计数值增加，则定义脉冲相序为1
//若车轮正转，脉冲计数值减小，则定义脉冲相序为0
#define CoderPhase0 1
#define CoderPhase1 1
#define CoderPhase2 0
#define CoderPhase3 0

#define YOUQIAN_TIM_ARR 16799
#define ZUOHOU_TIM_ARR 16799
#define YOUHOU_TIM_ARR 16799
#define ZUOQIAN_TIM_ARR 16799

#define PWMUPPER 16799
#define PWMDOWN	0

#define YOUQIAN_CONTROL_DIRECTION 0
#define ZUOHOU_CONTROL_DIRECTION 1
#define YOUHOU_CONTROL_DIRECTION 1
#define ZUOQIAN_CONTROL_DIRECTION 0

void SetMotorPWM(uint32_t zq,uint32_t yq,uint32_t zh,uint32_t yh);
void MotorSpeed2RealSpeed(void);
uint32_t Incremental_PI0 (int32_t MotorSpeedFromEncoder_,int32_t Target);
uint32_t Incremental_PI1 (int32_t MotorSpeedFromEncoder_,int32_t Target);
uint32_t Incremental_PI2 (int32_t MotorSpeedFromEncoder_,int32_t Target);
uint32_t Incremental_PI3 (int32_t MotorSpeedFromEncoder_,int32_t Target);
void SetMotorSPEED(int32_t zq,int32_t yq,int32_t zh,int32_t yh);
void mecanumRun(int32_t xSpeed, int32_t ySpeed, int32_t aSpeed);

#endif
