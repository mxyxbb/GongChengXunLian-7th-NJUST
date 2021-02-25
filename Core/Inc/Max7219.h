#ifndef _Max7219_H
#define _Max7219_H

#include "stm32f4xx_hal.h"

#define uchar unsigned char
#define uint  unsigned int

#define Max7219_pinCLK GPIO_PIN_10
#define Max7219_portCLK GPIOC
#define Max7219_pinCS GPIO_PIN_11
#define Max7219_portCS GPIOC
#define Max7219_pinDIN GPIO_PIN_0
#define Max7219_portDIN GPIOD

void Write_Max7219_byte(uchar DATA);//功能：向MAX7219(U3)写入字节
void Write_Max7219(uchar address,uchar dat);//功能：向MAX7219写入数据
void Init_MAX7219(void);
void WriteNum_Max7219(uint8_t *pData);
void WriteClear_Max7219(void);

/*example

 void main(void)
{
 Delay_xms(50);
 Init_MAX7219();
 Delay_xms(2000);
 Write_Max7219(0x0f, 0x00);       //显示测试：1；测试结束，正常显示：0
 Write_Max7219(1,8);
 Write_Max7219(2,7);
 Write_Max7219(3,6);
 Write_Max7219(4,5); 
 Write_Max7219(5,4);
 Write_Max7219(6,3);
 Write_Max7219(7,2);
 Write_Max7219(8,1);
 while(1);
}
 
*/

#endif

