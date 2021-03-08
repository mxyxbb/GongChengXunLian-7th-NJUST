#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "MotorControl.h"
#include "ControlTask.h"
#include "PID.h"
#include "lineFollowSensor.h"
#include "Buzzer/buzzerDriver.h"
#include "letter_shell/src/shell_port.h"

int32_t CoderData[4] = {0};
int32_t CoderData_last[4] = {0};
int32_t motordirection[4] = {0};
int32_t motorspeed[4] = {0};
SHELL_EXPORT_VAR(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_VAR_CHAR), msp0, &motorspeed[0], motorspeed[0]);
SHELL_EXPORT_VAR(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_VAR_CHAR), msp1, &motorspeed[1], motorspeed[1]);
SHELL_EXPORT_VAR(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_VAR_CHAR), msp2, &motorspeed[2], motorspeed[2]);
SHELL_EXPORT_VAR(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_VAR_CHAR), msp3, &motorspeed[3], motorspeed[3]);
int32_t motorspeed_set[4] = {0};//change this value to change target speed of PI controller 改变这个数组的值来改变PI速度环的设定值
uint32_t motorpwm_set[4] = {8400,8400,8400,8400};
int32_t MaxSpeed[4] = {0};
uint8_t cx=0;//循环变量
uint8_t AngleAndPositionTIM=1;
uint8_t lockFlag=0;
SHELL_EXPORT_VAR(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_VAR_CHAR), fol, &AngleAndPositionTIM, AngleAndPositionTIM);
SHELL_EXPORT_VAR(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_VAR_CHAR), lkf, &lockFlag, lockFlag);


uint16_t Time1_ms = 0;
uint16_t Time2_ms = 0;
uint16_t Time3_ms = 0;
uint8_t tim6enable=0;
SHELL_EXPORT_VAR(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_VAR_CHAR), t6e, &tim6enable, tim6enable);
uint8_t musicenable=0;
SHELL_EXPORT_VAR(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_VAR_CHAR), emu, &musicenable, musicenable);


extern int32_t y_speed;
extern int32_t x_speed;
extern int32_t a_speed;
void quietTest()
{
	tim6enable=1;
	AngleAndPositionTIM=0;
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC), qte, quietTest, quietTest());

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

	
	
    if (htim == (&htim6) && tim6enable)//中断频率为1kHz
    {
			Time1_ms++;			// 每1ms增一
			Time2_ms++;			// 每1ms增一
			Time3_ms++;
			if(musicenable)
				musicPlay();
			if(Time2_ms == 20)//传感器读入，周期为20ms(50Hz)
			{
				GetSensorData();
				Time2_ms = 0;
			}
			if(Time3_ms == 20&&AngleAndPositionTIM)//周期为20ms(50Hz)
			{
//				xAngleControl();
//				PositionControl(CarMovingTo);
				AnglePosControl(CarMovingTo);
				if(lockFlag==1)
				{
					AnglePosControl(CarMovingTo+1);
				}
				Time3_ms = 0;
			}
			
			if(Time1_ms == 20)//获取车轮转角增量，周期为20ms(50Hz)
			{
				//获取编码器脉冲计数值
				CoderData[0] = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
				CoderData[1] = (int32_t)__HAL_TIM_GET_COUNTER(&htim3);
				CoderData[2] = (int32_t)__HAL_TIM_GET_COUNTER(&htim4);
				CoderData[3] = (int32_t)__HAL_TIM_GET_COUNTER(&htim5);
				//获取电机转向
				motordirection[0] = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2);//zh
				motordirection[1] = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim3);//zq
				motordirection[2] = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim4);//yq
				motordirection[3] = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim5);//yh
				//计算电机转速

				for(cx=0; cx<4; cx++)
				{
						motorspeed[cx] = CoderData[cx] - CoderData_last[cx];
						if( motorspeed[cx] >50000 )
								motorspeed[cx] -= 65536;
						else if( motorspeed[cx] <-50000 )
								motorspeed[cx] += 65536;
						else
								;
						//记录最大转速，测试用
						if( motorspeed[cx] > MaxSpeed[cx] )
								MaxSpeed[cx] = motorspeed[cx];
				}
				MotorSpeed2RealSpeed();
				 //保存当前脉冲数，用于下次计算速度
				for(cx=0; cx<4; cx++)
					CoderData_last[cx] = CoderData[cx];
				#if 1 //pid调节部分
				//增量PID调节车轮速度
				motorpwm_set[0] = Incremental_PI0(motorspeed[0],motorspeed_set[0]);
				motorpwm_set[1] = Incremental_PI1(motorspeed[1],motorspeed_set[1]);
				motorpwm_set[2] = Incremental_PI2(motorspeed[2],motorspeed_set[2]);
				motorpwm_set[3] = Incremental_PI3(motorspeed[3],motorspeed_set[3]);

				//位置式PID调节车轮速度
//				motorpwm_set[0] = User_PID_Calc(&udPIDParameter0,motorspeed_set[0],motorspeed[0]);	
//				motorpwm_set[1] = User_PID_Calc(&udPIDParameter1,motorspeed_set[1],motorspeed[1]);	
//				motorpwm_set[2] = User_PID_Calc(&udPIDParameter2,motorspeed_set[2],motorspeed[2]);	
//				motorpwm_set[3] = User_PID_Calc(&udPIDParameter3,motorspeed_set[3],motorspeed[3]);	
				
				SetMotorPWM(motorpwm_set[zqMotorEncoder],motorpwm_set[yqMotorEncoder],motorpwm_set[zhMotorEncoder],motorpwm_set[yhMotorEncoder]);
				#endif
				mecanumRun(y_speed,x_speed,a_speed);
				
				Time1_ms = 0;
			}
			if(Time1_ms == 20)//任务x，周期为20ms(50Hz)
			{
				Time1_ms = 0;
			}
			if(Time2_ms==20)//任务x，周期为20ms(50Hz)
			{
				Time2_ms = 0;
			}
			if(Time3_ms==20)//任务x，周期为20ms(50Hz)
			{
				Time3_ms = 0;
			}
			
			
    }
}

//void throwTheBottle()
//{
//	int32_t step=10;
//	int32_t cx=10;
//	
//	TIM9->CCR1=500-1;
//	HAL_Delay(2000);
//	for(cx=500;cx<1500;cx+=step)
//	{
//		TIM9->CCR1=cx;
//		if(cx==1000) 	
//			step=5;
//		if(cx==1300) 
//			step=3;
//		HAL_Delay(10);
//	}
//	
//	TIM9->CCR1=500-1;
//}
