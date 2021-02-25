/*
 * PID.h
 *
 *  Created on: Jul 15, 2019
 *      Author: XinLiu
 */

#ifndef PID_H_
#define PID_H_

/* Includes ------------------------------------------------------------------*/
#include "main.h"

typedef struct {
    float SetValue;			 //Target Value

    float Kp;
    float Ki;
    float Kd;

    float error;
    float error_last;
    float error_pre;

    float integral;

    float pid_Calc_out; 	 //PID out
} PID;

extern PID udPIDParameter0;
extern PID udPIDParameter1;
extern PID udPIDParameter2;
extern PID udPIDParameter3;
extern PID xAnglePIDParameter4;
extern PID xPositionPIDParameter5;
extern PID yAnglePIDParameter6;
extern PID yPositionPIDParameter7;

extern void User_PID_Init(PID *userPara);
extern float User_PID_Calc(PID *uPID,float TargetVal,float CurrentVal);
extern float User_PID_ADJ(PID *uPID,float TargetVal);
extern float User_PID_Calc_angle(PID *uPID,float TargetVal,float CurrentVal);
extern float yPosUser_PID_Calc_angle(PID *uPID,float TargetVal,float CurrentVal);

#endif /* PID_H_ */
