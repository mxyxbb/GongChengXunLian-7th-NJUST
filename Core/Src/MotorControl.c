#include "MotorControl.h"
#include "ControlTask.h"
#include "letter_shell/src/shell_port.h"

extern int32_t motorspeed_set[4];//change this value to change target speed of PI controller 改变这个数组的值来改变PI速度环的设定值


//@fuction - motorpwm
//@para:(zq,yq,zh,yh)(0-8399-16799)
//zq - zuoqian pwm value : big forward
//yq - youqian pwm value : big forward
//zh - zuohou pwm value : big forward
//yh - youhou pwm value : big forward
void SetMotorPWM(uint32_t zq,uint32_t yq,uint32_t zh,uint32_t yh)
{
    #if (ZUOQIAN_CONTROL_DIRECTION == 1) 
//        ZUOQIAN_TIM->ZUOQIAN_CCR = zq;
	if(zq>8400){
					TIM8->CCR2 = (zq+zq-ZUOQIAN_TIM_ARR);
					TIM8->CCR1 = 0;
	}else{
					TIM8->CCR2 = 0;
					TIM8->CCR1 = (ZUOQIAN_TIM_ARR-zq-zq+1);
	}
    #else
//        ZUOQIAN_TIM->ZUOQIAN_CCR = ZUOQIAN_TIM_ARR - zq;
//改过
	if(zq>8400){
					TIM8->CCR1 = (zq+zq-ZUOQIAN_TIM_ARR);
					TIM8->CCR2 = 0;
	}else{
					TIM8->CCR1 = 0;
					TIM8->CCR2 = (ZUOQIAN_TIM_ARR-zq-zq+1);
	}
    #endif

    #if (YOUQIAN_CONTROL_DIRECTION == 1) 
//        YOUQIAN_TIM->YOUQIAN_CCR = yq;
	if(yq>8400){
					TIM1->CCR2 = (yq+yq-YOUQIAN_TIM_ARR);
					TIM1->CCR1 = 0;
	}else{
					TIM1->CCR2 = 0;
					TIM1->CCR1 = (YOUQIAN_TIM_ARR-yq-yq+1);
	}
    #else
//        YOUQIAN_TIM->YOUQIAN_CCR = YOUQIAN_TIM_ARR - yq;
//改过
	if(yq>8400){
					TIM1->CCR1 = (yq+yq-YOUQIAN_TIM_ARR);
					TIM1->CCR2 = 0;
	}else{
					TIM1->CCR1 = 0;
					TIM1->CCR2 = (YOUQIAN_TIM_ARR-yq-yq+1);
	}
    #endif

    #if (ZUOHOU_CONTROL_DIRECTION == 1) 
//        ZUOHOU_TIM->ZUOHOU_CCR = zh;
	if(zh>8400){
					TIM1->CCR3 = (zh+zh-ZUOHOU_TIM_ARR);
					TIM1->CCR4 = 0;
	}else{
					TIM1->CCR3 = 0;
					TIM1->CCR4 = (ZUOHOU_TIM_ARR-zh-zh+1);
	}
    #else
//        ZUOHOU_TIM->ZUOHOU_CCR = ZUOHOU_TIM_ARR - zh;
//改过
	if(zh>8400){
					TIM1->CCR4 = (zh+zh-ZUOHOU_TIM_ARR);
					TIM1->CCR3 = 0;
	}else{
					TIM1->CCR4 = 0;
					TIM1->CCR3 = (ZUOHOU_TIM_ARR-zh-zh+1);
	}
    #endif

    #if (YOUHOU_CONTROL_DIRECTION == 1) 
//        YOUHOU_TIM->YOUHOU_CCR = yh;
	if(yh>8400){
					TIM8->CCR4 = (yh+yh-YOUHOU_TIM_ARR);
					TIM8->CCR3 = 0;
	}else{
					TIM8->CCR4 = 0;
					TIM8->CCR3 = (YOUHOU_TIM_ARR-yh-yh+1);
	}
    #else
//        YOUHOU_TIM->YOUHOU_CCR = YOUHOU_TIM_ARR - yh;
//改过
	if(yh>8400){
					TIM8->CCR3 = (yh+yh-YOUHOU_TIM_ARR);
					TIM8->CCR4 = 0;
	}else{
					TIM8->CCR3 = 0;
					TIM8->CCR4 = (YOUHOU_TIM_ARR-yh-yh+1);
	}
    #endif
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC), smp, SetMotorPWM, SetMotorPWM(zq,yq,zh,yh));

void SetMotorSPEED(int32_t zq,int32_t yq,int32_t zh,int32_t yh)
{
	motorspeed_set[zqMotorEncoder] = zq+ZQ_OFFSET;
	motorspeed_set[yqMotorEncoder] = yq+YQ_OFFSET;
	motorspeed_set[zhMotorEncoder] = zh+ZH_OFFSET;
	motorspeed_set[yhMotorEncoder] = yh+YH_OFFSET;
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC), sms, SetMotorSPEED, SetMotorSPEED(zq,yq,zh,yh));

int32_t Velocity_Kp0 = -0;
int32_t Velocity_Ki0 = -60;
SHELL_EXPORT_VAR(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_VAR_INT), vkp0, &Velocity_Kp0, Velocity_Kp0);
SHELL_EXPORT_VAR(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_VAR_INT), vki0, &Velocity_Ki0, Velocity_Ki0);
uint32_t Incremental_PI0 (int32_t MotorSpeedFromEncoder_,int32_t Target)
{
	static int32_t Bias0,Last_bias0;
	static int32_t Pwm0=8400;
	
	Bias0 = MotorSpeedFromEncoder_- Target;									//计算偏差
	Pwm0 += Velocity_Kp0*(Bias0-Last_bias0) + Velocity_Ki0*Bias0;	//增量式PI控制器
	Last_bias0=Bias0;																					//保存上一次偏差
	if( Pwm0 > PWMUPPER )
		Pwm0 = PWMUPPER;
	if( Pwm0 < PWMDOWN )
		Pwm0 = PWMDOWN;
	return Pwm0;																							//增量输出
}
int32_t Velocity_Kp1 = 250;
int32_t Velocity_Ki1 = 250;
uint32_t Incremental_PI1 (int32_t MotorSpeedFromEncoder_,int32_t Target)
{
	static int32_t Bias1,Last_bias1;
	static int32_t Pwm1=8400;
	
	Bias1 = MotorSpeedFromEncoder_- Target;									//计算偏差
	Pwm1 += Velocity_Kp1*(Bias1-Last_bias1) + Velocity_Ki1*Bias1;	//增量式PI控制器
	Last_bias1=Bias1;																					//保存上一次偏差
	if( Pwm1 > PWMUPPER )
		Pwm1 = PWMUPPER;
	if( Pwm1 < PWMDOWN )
		Pwm1 = PWMDOWN;
	return Pwm1;																							//增量输出
}
int32_t Velocity_Kp2 = -250;
int32_t Velocity_Ki2 = -250;
uint32_t Incremental_PI2 (int32_t MotorSpeedFromEncoder_,int32_t Target)
{
	static int32_t Bias2,Last_bias2;
	static int32_t Pwm2=8400;
	
	Bias2 = MotorSpeedFromEncoder_- Target;									//计算偏差
	Pwm2 += Velocity_Kp2*(Bias2-Last_bias2) + Velocity_Ki2*Bias2;	//增量式PI控制器
	Last_bias2=Bias2;																					//保存上一次偏差
	if( Pwm2 > PWMUPPER )
		Pwm2 = PWMUPPER;
	if( Pwm2 < PWMDOWN )
		Pwm2 = PWMDOWN;
	return Pwm2;																							//增量输出
}
int32_t Velocity_Kp3 = -250;
int32_t Velocity_Ki3 = -250;
uint32_t Incremental_PI3 (int32_t MotorSpeedFromEncoder_,int32_t Target)
{
	static int32_t Bias3,Last_bias3;
	static int32_t Pwm3=8400;
	
	Bias3 = MotorSpeedFromEncoder_- Target;									//计算偏差
	Pwm3 += Velocity_Kp3*(Bias3-Last_bias3) + Velocity_Ki3*Bias3;	//增量式PI控制器
	Last_bias3=Bias3;																					//保存上一次偏差
	if( Pwm3 > PWMUPPER )
		Pwm3 = PWMUPPER;
	if( Pwm3 < PWMDOWN )
		Pwm3 = PWMDOWN;
	return Pwm3;																							//增量输出
}




extern int32_t motorspeed[4];
//本段代码功能：将上面计算到的速度与四个轮子的实际速度对应起来
void MotorSpeed2RealSpeed()
{
    #if (CoderPhase0 == 0)
        motorspeed[0] = -motorspeed[0];
    #endif
    
    #if (CoderPhase1 == 0)
        motorspeed[1] = -motorspeed[1];
    #endif
    
    #if (CoderPhase2 == 0)
        motorspeed[2] = -motorspeed[2];
    #endif
    
    #if (CoderPhase3 == 0)
        motorspeed[3] = -motorspeed[3];
    #endif
}

void mecanumRun(int32_t xSpeed, int32_t ySpeed, int32_t aSpeed)
{
    int32_t speed1 = ySpeed - xSpeed + aSpeed; 
    int32_t speed2 = ySpeed + xSpeed - aSpeed;
    int32_t speed3 = ySpeed - xSpeed - aSpeed;
    int32_t speed4 = ySpeed + xSpeed + aSpeed;
	
    
//    int32_t max = speed1;
//    if (max < speed2)   max = speed2;
//    if (max < speed3)   max = speed3;
//    if (max < speed4)   max = speed4;
//    
//    if (max > maxLinearSpeed)
//    {
//        speed1 = (float)speed1/ max * maxLinearSpeed;
//        speed2 = (float)speed2/ max * maxLinearSpeed;
//        speed3 = (float)speed3/ max * maxLinearSpeed;
//        speed4 = (float)speed4/ max * maxLinearSpeed;
//    }
    
    SetMotorSPEED(speed2, speed1, speed3, speed4);
}

