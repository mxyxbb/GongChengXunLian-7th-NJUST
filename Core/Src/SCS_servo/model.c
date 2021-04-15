#include "stm32f4xx_hal.h"
#include "math.h"
#include <stdio.h>

#define PI 3.1415926f

float targetX = 0.2, targetY = 0.0;
float lastX = 0.2, lastY = 0.0;

#define l0 0.1035f	//2
#define l1 0.088f		//1
#define l2 0.17f		//3


uint8_t model(float x,float y,float alpha)
{  
	      float m,n,k,a,b,c,theta1,theta2,theta3,s1ps2;

				m=l2*cos(alpha)-x;   
				n=l2*sin(alpha)-y;   
	      k=(l1*l1-l0*l0-m*m-n*n)/2/l0;//中间变量
	      a=m*m+n*n;             //解一元二次方程
	      b=-2*n*k;
	      c=k*k-m*m;

	      if(b*b-4*a*c<=0)   //b^2-4ac 小于0即无实根，直接返回
        { 
				targetX=lastX;		
			  targetY=lastY;		
				return 1; //返回1， 作错误
				}
				

	      theta1=(-b+sqrt(b*b-4*a*c))/2/a;  //求解二元一次方程，只取其中一个，另外一个解是(-b+sqrt(b*b-4*a*c))/2/a
	      theta1=asin(theta1)*180/PI;       //将弧度换算为角度  

	      if(theta1>90)theta1=90;           //限制最大角度为正负90度
				if(theta1<-90)theta1=-90;
	
	      k=(l0*l0-l1*l1-m*m-n*n)/2/l1;     
				a=m*m+n*n;                        //解一元二次方程
				b=-2*n*k;
				c=k*k-m*m;
				
	      if(b*b-4*a*c<=0)   //方程无实根就不做求解
				{	
				targetX=lastX;		
			  targetY=lastY;
				return 2;          //返回2， 作错误标记
				}
	      s1ps2=(-b-sqrt(b*b-4*a*c))/2/a;      
	      s1ps2=asin(s1ps2)*180/PI;          //将弧度换算为角度  
	
				if(s1ps2>90)theta2=90;    
				if(s1ps2<-90)theta2=-90;  
				
	      theta2=s1ps2-theta1;      
			  if(theta2>90)theta2=90;       //限制最大角度为正负90度
				if(theta2<-90)theta2=-90;    //限制最大角度为正负90度
	
	      theta3=alpha*180/PI-theta1-theta2;   //求5号舵机角度
				if(theta3>180)theta3=180;
				if(theta3<0)theta3=0;	    //控制舵机的最大角度180

				
				//将求得的三个角度，转换为对应的脉宽，然后控制舵机转动。
				//需要注意的是机械臂的舵机开机后为1500位置。我们将此位置定为0度，即0度为180度舵机的90度位置。
				//因舵机安装方向的不同，舵机角度的正负方向要根据舵机安装方向调整
				//将舵机角度转为公式为
				//   (2000 * 角度 / 180 + 500)
				//上式的角度的范围是0-180度
				printf("%f\r\n",theta3);
//				ServoSetPluseAndTime(5,(2000 * (90.0 - theta1) / 180.0 + 500.0),50);  //控制舵机转动， 给根据公式转换脉宽， 以舵机的中位为基准。
//				ServoSetPluseAndTime(4,(2000 * (90.0 + theta2) / 180.0 + 500.0) ,50);
//				ServoSetPluseAndTime(3,(2000 * (theta3) / 180.0 + 500.0),50);

				lastX=targetX;  //将当前位置，替换为目标位置
				lastY=targetY;
				return 0; //一切正常返回0
}

