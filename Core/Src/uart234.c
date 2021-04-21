#include "uart234.h"
#include "user_usart.h"
#include "usart.h"
#include <stdio.h>
#include "i2c.h"
#include "MAX7219/max7219_matrix.h"
#include "MAX7219/max7219.h"
#include <string.h>
#include "SCS_servo/SCS_servo.h"
#include "letter_shell/src/shell_port.h"
#include "mymath.h"

uint8_t uart3ok=0;
SHELL_EXPORT_VAR(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_VAR_CHAR), u3ok, &uart3ok, uart3ok);
uint8_t readcolor_mode=1;//1--���η���0---�ظ���
SHELL_EXPORT_VAR(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_VAR_CHAR), rc1111, &readcolor_mode, readcolor_mode);


uint8_t Ov3Mode = Ov3Mode_QrCode;
uint8_t Max7219_String[]="123--213";
uint8_t CMD_Serial[] = "CMD_Serial";
uint8_t CMD_qrcode[] = "qrcode";
uint8_t CMD_Material[] = "CMD_Material\r\n";
uint8_t CMD_color[] = "color";
uint8_t colororder[]={0,0,0,0,0,0};//����LED�������ʾ���������ҵ�˳����ʾ����Ļ��

extern int32_t y_speed;
extern int32_t x_speed;	
	
/*
----------------�°涯���鶨��--------------
PS:���ϵ�������ɫ�̶����񡢢򡢢�Ŵ��������졢�����ӣ�ͬ��������ҵ���ɫ˳��
���ƻ�е��ִ���ض����� (����ʽ)

DOG		CMD			action
			---ԭ����---
0			1				ץ��������������ɫ����+1����+2��
3			4				ץ���У�����������ɫ
6			7				ץ���ң�����������ɫ
9			10 			ץ��������������ɫ
12		13			ץ���У�����������ɫ
15		16			ץ���ң�����������ɫ
			---�ּӹ�---
18		19			�Ţ���������ϣ�ץ+1��
20		21			�Ţ����������
22		23			�Ţ����������
			---���ӹ�---
24		25			�Ţ���������ϣ��ϣ�
25		26      �Ţ����������
26		27      �Ţ����������
27		28      �Ţ���������ϣ��£�
28		29      �Ţ���������� 
29		30      �Ţ����������
!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!���ϣ���Ӧ����²�
!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!���£���Ӧ����ϲ�
30            ��һ������������̨���ϲ�λ��1
31
*/
void Uart2_servoCtr(uint8_t CMD){
	DoGroup(CMD-1);
	led_shan();
}

uint8_t RxBuff[1];      //�����жϽ������ݵ�����
uint8_t DataBuff[50]; //������յ������ݵ����飬���ս������
uint8_t Color[6]; //������յ�����ɫ������

int RxLine=0;           //���յ������ݳ���

void Uart3_readQRcode()
{
	printf("reading qrcode\n\r");
	Ov3Mode=Ov3Mode_QrCode;
	HAL_UART_Receive_IT(&huart3, (uint8_t *)RxBuff, 1); //�򿪴����жϽ���
	HAL_UART_Transmit(&huart3, CMD_qrcode, 6, 0xffff);
	while(uart3ok==0)
	{
		__ExecuteOnce(user_main_printf("%s",CMD_qrcode));
	}
	x_speed=0;
	colororder[0]=Max7219_String[0]-'0';
	colororder[1]=Max7219_String[1]-'0';
	colororder[2]=Max7219_String[2]-'0';
	colororder[3]=Max7219_String[5]-'0';
	colororder[4]=Max7219_String[6]-'0';
	colororder[5]=Max7219_String[7]-'0';
	MAX7219_mywrite(colororder);
	MAX7219_MatrixUpdate();
	uart3ok=0;
	printf("qrcode:%s\n\r",Max7219_String);
	led_shan();
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC), u3rqr, Uart3_readQRcode, Uart3_readQRcode());

char led_str[6]="RGBRGB";

void Uart3_readColor()
{
	user_main_printf("%s",CMD_color);
	Ov3Mode=Ov3Mode_ColorBlock;
	HAL_UART_Receive_IT(&huart3, (uint8_t *)RxBuff, 1); //�򿪴����жϽ���
	while(uart3ok==0){
		if(readcolor_mode)
			__ExecuteOnce(HAL_UART_Transmit(&huart3, CMD_color, 5, 0xffff));
		else
			HAL_UART_Transmit(&huart3, CMD_color, 5, 0xffff);
		HAL_Delay(40);
	}
	sprintf(led_str,"%c%c%c%c%c%c",Color[0],Color[1],Color[2],Color[3],Color[4],Color[5]);
	user_main_printf("color result:%s",led_str);
	uart3ok=0;
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC), u3rco, Uart3_readColor, Uart3_readColor());

void Uart2_sendColor()
{
	user_main_printf("color result:%s",led_str);
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC), u2sco, Uart2_sendColor, Uart2_sendColor());

void HAL_UART_RxCpltCallback_color()//�ڴ���3�Ľ��ջص������е��ô˺���������openMV���ص���ɫʶ������
{
	static char setVal,RxLine_Copy,i;
		
		if(RxLine==0)//��ʼ��
		{
			RxLine_Copy=0;//��ʼ��
			uart3ok=0;//��ʼ����Ĭ��״̬����δ���
			switch(Ov3Mode)
			{
				case Ov3Mode_QrCode:
					setVal = 11;//"123+321"\r\n
				break;
				case Ov3Mode_ColorBlock:
					setVal = 10;//"RGBBRG"\r\n
				break;
			}
		}
	
		
    RxLine++;                      //ÿ���յ�һ�����ݣ�����ص����ݳ��ȼ�1
    DataBuff[RxLine-1]=RxBuff[0];  //��ÿ�ν��յ������ݱ��浽��������
    
    if((DataBuff[RxLine-1] == 0x0A)&&(DataBuff[RxLine-2] == 0x0D))            //���ս�����־λ��������ݿ����Զ��壬����ʵ����������ֻ��ʾ��ʹ�ã���һ����0xff
    {
        //printf("Uart3-RXLen=%d\r\n",RxLine); 
        for(int i=0;i<RxLine;i++)
					//printf("UART DataBuff[%d] = 0x%x\r\n",i,DataBuff[i]);                               

				RxLine_Copy=RxLine;
				RxLine=0;  //��ս��ճ���
				
				
				
				if(RxLine_Copy<setVal)
				{	
					//HAL_UART_Receive_IT(&huart3, (uint8_t *)RxBuff, 1); //���ܳ��Ȳ�����Ҫ�������գ�������һ�δ����жϽ��գ�����ֻ�����һ�����ݾ�ֹͣ����
				}
				else
				{
					uart3ok=1;
					RxLine_Copy=0;
					switch(Ov3Mode)
					{
						case Ov3Mode_QrCode:
							Max7219_String[0]=DataBuff[1];
							Max7219_String[1]=DataBuff[2];
							Max7219_String[2]=DataBuff[3];
							Max7219_String[5]=DataBuff[5];
							Max7219_String[6]=DataBuff[6];
							Max7219_String[7]=DataBuff[7];
						break;
						case Ov3Mode_ColorBlock:
							for(i=0;i<6;i++)
							{
								Color[i]=DataBuff[i+1];
							}
						break;
					}
					memset(DataBuff,0,sizeof(DataBuff));  //��ջ�������
					RxBuff[0]=0;
				}
				
		}
		

		if(RxLine_Copy<setVal&&!uart3ok)
		{	
			HAL_UART_Receive_IT(&huart3, (uint8_t *)RxBuff, 1); //���ܳ��Ȳ�����Ҫ�������գ�������һ�δ����жϽ��գ�����ֻ�����һ�����ݾ�ֹͣ����
		}
		
		
}
