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
	      k=(l1*l1-l0*l0-m*m-n*n)/2/l0;//�м����
	      a=m*m+n*n;             //��һԪ���η���
	      b=-2*n*k;
	      c=k*k-m*m;

	      if(b*b-4*a*c<=0)   //b^2-4ac С��0����ʵ����ֱ�ӷ���
        { 
				targetX=lastX;		
			  targetY=lastY;		
				return 1; //����1�� ������
				}
				

	      theta1=(-b+sqrt(b*b-4*a*c))/2/a;  //����Ԫһ�η��̣�ֻȡ����һ��������һ������(-b+sqrt(b*b-4*a*c))/2/a
	      theta1=asin(theta1)*180/PI;       //�����Ȼ���Ϊ�Ƕ�  

	      if(theta1>90)theta1=90;           //�������Ƕ�Ϊ����90��
				if(theta1<-90)theta1=-90;
	
	      k=(l0*l0-l1*l1-m*m-n*n)/2/l1;     
				a=m*m+n*n;                        //��һԪ���η���
				b=-2*n*k;
				c=k*k-m*m;
				
	      if(b*b-4*a*c<=0)   //������ʵ���Ͳ������
				{	
				targetX=lastX;		
			  targetY=lastY;
				return 2;          //����2�� ��������
				}
	      s1ps2=(-b-sqrt(b*b-4*a*c))/2/a;      
	      s1ps2=asin(s1ps2)*180/PI;          //�����Ȼ���Ϊ�Ƕ�  
	
				if(s1ps2>90)theta2=90;    
				if(s1ps2<-90)theta2=-90;  
				
	      theta2=s1ps2-theta1;      
			  if(theta2>90)theta2=90;       //�������Ƕ�Ϊ����90��
				if(theta2<-90)theta2=-90;    //�������Ƕ�Ϊ����90��
	
	      theta3=alpha*180/PI-theta1-theta2;   //��5�Ŷ���Ƕ�
				if(theta3>180)theta3=180;
				if(theta3<0)theta3=0;	    //���ƶ�������Ƕ�180

				
				//����õ������Ƕȣ�ת��Ϊ��Ӧ������Ȼ����ƶ��ת����
				//��Ҫע����ǻ�е�۵Ķ��������Ϊ1500λ�á����ǽ���λ�ö�Ϊ0�ȣ���0��Ϊ180�ȶ����90��λ�á�
				//������װ����Ĳ�ͬ������Ƕȵ���������Ҫ���ݶ����װ�������
				//������Ƕ�תΪ��ʽΪ
				//   (2000 * �Ƕ� / 180 + 500)
				//��ʽ�ĽǶȵķ�Χ��0-180��
				printf("%f\r\n",theta3);
//				ServoSetPluseAndTime(5,(2000 * (90.0 - theta1) / 180.0 + 500.0),50);  //���ƶ��ת���� �����ݹ�ʽת������ �Զ������λΪ��׼��
//				ServoSetPluseAndTime(4,(2000 * (90.0 + theta2) / 180.0 + 500.0) ,50);
//				ServoSetPluseAndTime(3,(2000 * (theta3) / 180.0 + 500.0),50);

				lastX=targetX;  //����ǰλ�ã��滻ΪĿ��λ��
				lastY=targetY;
				return 0; //һ����������0
}

