/*
 * PID.c
 *
 *  Created on: Jul 15, 2019
 *      Author: XinLiu
 */
#include "PID.h"
float middle = 8400; 
#define KP 100
#define KI 60
#define KD 10

#define UPPER  16799
#define LOWER  0

#define UPPER_angle  1000
#define LOWER_angle  -1000

#define UPPER_yPosIntegral 10000
#define LOWER_yPosIntegral 1000

PID udPIDParameter0;
PID udPIDParameter1;
PID udPIDParameter2;
PID udPIDParameter3;
PID xAnglePIDParameter4;
PID xPositionPIDParameter5;
PID yAnglePIDParameter6;
PID yPositionPIDParameter7;



void User_PID_Init(PID *userPara)
{
	userPara->Kp = KP;
	userPara->Ki = KI;
	userPara->Kd = KD;
	userPara->SetValue = 0.0;
	userPara->error = 0.0;
	userPara->error_last = 0.0;
	userPara->error_pre = 0.0;
	userPara->integral = 0.0;
	userPara->pid_Calc_out = 0.0;
}


float User_PID_Calc(PID *uPID,float TargetVal,float CurrentVal)
{
    uPID->SetValue = TargetVal;

    uPID->error = uPID->SetValue - CurrentVal;
    uPID->integral += uPID->error;
    uPID->pid_Calc_out = (uPID->Kp * uPID->error) + (uPID->Ki * uPID->integral)
    					+ uPID->Kd * (uPID->error - uPID->error_last);
    uPID->error_last = uPID->error;

    if(uPID->pid_Calc_out > UPPER)	uPID->pid_Calc_out = UPPER;
    else if(uPID->pid_Calc_out < LOWER) uPID->pid_Calc_out = LOWER;

    return (uPID->pid_Calc_out * 1.0f);
}

float User_PID_Calc_angle(PID *uPID,float TargetVal,float CurrentVal)
{
    uPID->SetValue = TargetVal;

    uPID->error = uPID->SetValue - CurrentVal;
    uPID->integral += uPID->error;
    uPID->pid_Calc_out = (uPID->Kp * uPID->error) + (uPID->Ki * uPID->integral)
    					+ uPID->Kd * (uPID->error - uPID->error_last);
    uPID->error_last = uPID->error;

    if(uPID->pid_Calc_out > UPPER_angle)	uPID->pid_Calc_out = UPPER_angle;
    else if(uPID->pid_Calc_out < LOWER_angle) uPID->pid_Calc_out = LOWER_angle;

    return (uPID->pid_Calc_out * 1.0f);
}

float yPosUser_PID_Calc_angle(PID *uPID,float TargetVal,float CurrentVal)
{
    uPID->SetValue = TargetVal;

    uPID->error = uPID->SetValue - CurrentVal;
    uPID->integral += uPID->error;
	
		if(uPID->integral > UPPER_yPosIntegral)	uPID->integral = UPPER_yPosIntegral;
    else if(uPID->integral < LOWER_yPosIntegral) uPID->integral = 0;
	
    uPID->pid_Calc_out = (uPID->Kp * uPID->error) + (uPID->Ki * uPID->integral)
    					+ uPID->Kd * (uPID->error - uPID->error_last);
    uPID->error_last = uPID->error;

    if(uPID->pid_Calc_out > UPPER_angle)	uPID->pid_Calc_out = UPPER_angle;
    else if(uPID->pid_Calc_out < LOWER_angle) uPID->pid_Calc_out = LOWER_angle;
		
	
	
    return (uPID->pid_Calc_out * 1.0f);
}

float User_PID_ADJ(PID *uPID,float TargetVal)
{
    float Out;
    uPID->SetValue = TargetVal;
    Out = (uPID->Kp * (uPID->error - uPID->error_last))  + (uPID->Ki * uPID->error)
    			+ uPID->Kd * (uPID->error-2 * uPID->error_last + uPID->error_pre);
    uPID->error_pre = uPID->error_last;
    uPID->error_last = uPID->error;

    uPID->pid_Calc_out += Out;
    return (uPID->pid_Calc_out);
}

