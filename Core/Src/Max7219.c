#include"Max7219.h"


//--------------------------------------------
//功能：向MAX7219(U3)写入字节
//入口参数：DATA 
//出口参数：无
//说明：
void Write_Max7219_byte(uchar DATA)         
{
    	uchar i;    
		HAL_GPIO_WritePin(Max7219_portCS,Max7219_pinCS,GPIO_PIN_RESET);//Max7219_pinCS=0;		
	    for(i=8;i>=1;i--)
          {		  
            HAL_GPIO_WritePin(Max7219_portCLK,Max7219_pinCLK,GPIO_PIN_RESET);//Max7219_pinCLK=0;
            HAL_GPIO_WritePin(Max7219_portDIN,Max7219_pinDIN,DATA&0x80);//Max7219_pinDIN=DATA&0x80;
            DATA=DATA<<1;
            HAL_GPIO_WritePin(Max7219_portCLK,Max7219_pinCLK,GPIO_PIN_SET);//Max7219_pinCLK=1;
           }                                 
}
//-------------------------------------------
//功能：向MAX7219写入数据
//入口参数：address、dat
//出口参数：无
//说明：
void Write_Max7219(uchar address,uchar dat)
{ 
   HAL_GPIO_WritePin(Max7219_portCS,Max7219_pinCS,GPIO_PIN_RESET);//Max7219_pinCS=0;
	 Write_Max7219_byte(address);           //写入地址，即数码管编号
   Write_Max7219_byte(dat);               //写入数据即数码管显示数字 
	 HAL_GPIO_WritePin(Max7219_portCS,Max7219_pinCS,GPIO_PIN_SET);//Max7219_pinCS=1;                        
}

void Init_MAX7219(void)
{
	Write_Max7219(0x09, 0xff);       //译码方式：BCD译码
 Write_Max7219(0x0a, 0x03);       //亮度00-0f
 Write_Max7219(0x0b, 0x07);       //扫描界限；4个数码管显示
 Write_Max7219(0x0c, 0x01);       //掉电模式：0，普通模式：1
 Write_Max7219(0x0f, 0x01);       //显示测试：1；测试结束，正常显示：0
}

void WriteNum_Max7219(uint8_t *pData)
{
//	int8_t i=0;
	Write_Max7219(1,pData[7]);
	Write_Max7219(2,pData[6]);
	Write_Max7219(3,pData[5]);
	Write_Max7219(4,10);
	Write_Max7219(5,10);
	Write_Max7219(6,pData[2]);
	Write_Max7219(7,pData[1]);
	Write_Max7219(8,pData[0]);
	
}

void WriteClear_Max7219()
{
	int8_t i=0;
	for(i=1;i<9;i++)
	{
		Write_Max7219(i,15);
	}
}
