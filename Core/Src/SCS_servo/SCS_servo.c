//-------------------------
// File Name: SCS_servo.c
// Author: Ma Xueyang
// Date: 2021.2.15
//-------------------------

#include "SCS_servo.h"
#include "../SCSLib/SCServo.h"
#include "../SCSLib/uart.h"
#include "../SCSLib/wiring.h"
#include "../letter_shell/src/shell_port.h"
#include "stdio.h"
#include "../mymath.h"
#include "../ee24/ee24.h"
#include <string.h> //使用到了memcpy
#include "user_usart.h"

#define POS_LEN 100
#define GROUP_LEN 40//30
#define GROUP_POS_LEN 15

#define POS_LEN_EX 800


/*-------第一缓冲区-------*/
/*开辟机械臂"位置"缓冲区*/
Pos postion[POS_LEN];
/*开辟机械臂"动作组"缓冲区*/
int16_t group[GROUP_LEN][GROUP_POS_LEN]; 

/*-------第二缓冲区-------*/
/*开辟机械臂第二"位置"缓冲区,该区数据将存至stm32的片内flash*/
Pos postion_ex[POS_LEN_EX];


/**
  * @brief  初始化函数
  * @retval 无
  */
void ArmInit()
{
	for(uint8_t i=0;i<GROUP_LEN;i++)
	{
		for(uint8_t j=0;j<GROUP_POS_LEN;j++)
		{
			group[i][j]=-1;
		}
	}

	for(uint8_t i=0;i<POS_LEN;i++)
  {
		postion[i].pos_id=-1;
  }
	for(uint16_t i=0;i<POS_LEN_EX;i++)
  {
		postion_ex[i].pos_id=-1;
  }
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC), ami, ArmInit, ArmInit());

/**
  * @brief  单个舵机力矩控制
  * @param  ID_:被控制的舵机ID
  * @param  Enable_:置0关闭力矩输出、置1开启力矩输出
  * @retval 无
  */
void ArmForceEnable(uint8_t ID_,uint8_t Enable_){
	unLockEprom(ID_);//打开EPROM保存功能
	EnableTorque(ID_, Enable_);
	LockEprom(ID_);//关闭EPROM保存功能
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC), amf, ArmForceEnable, ArmForceEnable(id,en));
/**
  * @brief  全部舵机力矩控制
  * @param  Enable_:置0关闭力矩输出、置1开启力矩输出
  * @retval 无
  */
void ForceAll(uint8_t Enable_){
	for(uint8_t ID_=0;ID_<5;ID_++){
		unLockEprom(ID_+1);//打开EPROM保存功能
		EnableTorque(ID_+1, Enable_);
		LockEprom(ID_+1);//关闭EPROM保存功能
	}
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC), fll, ForceAll, forceAll(en));
/**
  * @brief  存储机械臂当前位置信息
  * @param  ID_:当前位置信息存储到数组中的位置
  * @param  timems_:当前位置的执行时间（与舵机转速有关）
  * @retval 无
  */
void SavePos(int16_t ID_,int16_t timems_)
{
	Pos postion0;
	for(uint8_t temp=0;temp<5;temp++)//读取并存储5个舵机的角度
  {
		postion0.angle[temp]=ReadPos(temp+1);
  }
	postion0.pos_id=ID_;//设置动作id
	postion0.timems=timems_;//设置动作时间
	/*存储*/
	for(uint8_t i=0;i<5;i++)
	user_main_printf("angle%d: %d",i,postion0.angle[i]);
	if(ID_<POS_LEN)
		postion[ID_]=postion0;//存储至缓存区1
	else
		postion_ex[ID_-POS_LEN]=postion0;//存储至缓存区2
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC), sap, SavePos, savePos(id,time));
/**
  * @brief  使机械臂动作到指定位置
  * @param  ID_:指定位置信息存储在数组中的位置
  * @retval 无
  */
void GoPos(int16_t ID_)
{
	if(ID_<POS_LEN){
		for(uint8_t temp=0;temp<5;temp++)//写5个舵机的角度
		{
			WritePos(temp+1, postion[ID_].angle[temp], postion[ID_].timems, 0);//舵机(IDtemp),以时间timems毫秒,运行至postion[ID_].angle[temp]角度
		}
		delay(postion[ID_].timems);//堵塞式等待动作完成
	}
	else{
		ID_ -= POS_LEN;
		for(uint8_t temp=0;temp<5;temp++)//写5个舵机的角度
		{
			WritePos(temp+1, postion_ex[ID_].angle[temp], postion_ex[ID_].timems, 0);//舵机(IDtemp),以时间timems毫秒,运行至postion_ex[ID_].angle[temp]角度
		}
		delay(postion_ex[ID_].timems);//堵塞式等待动作完成
	}
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC), gop, GoPos, goPos(id));

/**
  * @brief  修改缓冲区机械臂动作组序列
  * @param  ID_:动作组信息存储在数组中的位置
  * @param  PosIDs:按顺序存储着动作组各个动作的数组
  * @retval 无
  */
//int8_t order[]={0,1,2,3,4,5};
//saveGroup(0,order);
void Pos2Group(uint8_t G_ID_,uint8_t GP_ID_,uint16_t P_ID)
{
		group[G_ID_][GP_ID_]=P_ID;
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC), p2g, Pos2Group, Pos2Group(gid,gpid,pid));

/**
  * @brief  执行缓冲区的某个动作组
  * @param  ID_:动作组信息存储在数组中的位置
  * @retval 无
  */
void DoGroup(uint8_t ID_)
{
	for(uint8_t i=0;group[ID_][i]!=-1;i++)
  {
		GoPos(group[ID_][i]);
  }
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC), dog, DoGroup, DoGroup(id));

/**
  * @brief  将与机械臂有关的缓冲区变量存入内部Flash的
	* 扇区11(0x080E 0000 - 0x080F FFFF)中,此扇区可存储128kByte数据
  * @retval 无
  */
void SaveAll2F()
{
	int i = 0;
	uint32_t PageError = 0;
	FLASH_EraseInitTypeDef a;
	HAL_StatusTypeDef status;
	uint32_t addr = 0x080E0000;
	uint32_t data_buf[10];
	
	/* 读取Flash内容 */
	memcpy(data_buf, (uint32_t*)addr, sizeof(uint32_t)*10);
	printf("read before erase:\r\n\t");
	for(i = 0;i < 10;i++)
	{
		printf("0x%08x ", data_buf[i]);
	}
	printf("\r\n");
	
	/*定义要擦除的部分*/
	a.TypeErase = FLASH_TYPEERASE_SECTORS;
	a.Banks = FLASH_BANK_1;
	a.Sector = FLASH_SECTOR_11;
	a.NbSectors = 1;
	a.VoltageRange = FLASH_VOLTAGE_RANGE_3;

	HAL_FLASH_Unlock();
	status = HAL_FLASHEx_Erase(&a,&PageError);/*擦除扇区11的内容*/
	HAL_FLASH_Lock();
	if(status != HAL_OK)
	{
		printf("erase fail, PageError = %d\r\n", PageError);
	}
	else
		printf("erase success\r\n");

	/* 读取Flash内容 */
	memcpy(data_buf, (uint32_t*)addr, sizeof(uint32_t)*10);
	printf("read after erase:\r\n\t");
	for(i = 0;i < 10;i++)
	{
		printf("0x%08x ", data_buf[i]);
	}
	printf("\r\n");
	
	//写入Flash内容
	HAL_FLASH_Unlock();
	
	{
		uint16_t lenadd=0;
		uint16_t cnt=6;//前面空6个位置，用来存三部分的数据长度
			
			/*----开始存位置1，最大1400字节*/
			for(uint8_t i=0;postion[i].pos_id != -1&& i < POS_LEN;i++)//遍历位置ID
			{
				HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, addr+cnt, postion[i].pos_id);//存ID
				cnt += 2;
				for(uint8_t j=0;j<5;j++)
				{
					HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, addr+cnt, postion[i].angle[j]);//存舵机角
					cnt += 2;
				}
				HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, addr+cnt, postion[i].timems);//存动作时间
				cnt += 2;
			}
			//存位置结束
			HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, addr+lenadd, cnt);//在本扇区的开头存位置数据总长度
			lenadd += 2;
			//----开始存动作组，最大900字节
			for(uint8_t i=0;group[i][0] != -1&& i < GROUP_LEN;i++)//遍历动作组ID
			{
				for(uint8_t j=0;j<GROUP_POS_LEN;j++)
				{
					HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, addr+cnt, group[i][j]);//存储该动作组的位置序列
					cnt += 2;
				}
			}
			//存动作组结束
			status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, addr+lenadd, cnt);//在本扇区的开头存动作组数据总长度
			lenadd += 2;
			
			/*----开始存位置ex，最大11200字节*/
			for(uint16_t i=0;postion_ex[i].pos_id!=-1&& i < POS_LEN_EX;i++)//遍历位置ID
			{
				HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, addr+cnt, postion_ex[i].pos_id);//存ID
				cnt += 2;
				for(uint8_t j=0;j<5;j++)
				{
					HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, addr+cnt, postion_ex[i].angle[j]);//存舵机角
					cnt += 2;
				}
				HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, addr+cnt, postion_ex[i].timems);//存动作时间
				cnt += 2;
			}
			//存位置结束
			HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, addr+lenadd, cnt);//在本扇区的开头存位置2数据总总长度
			lenadd += 2;
	}
	
	HAL_FLASH_Lock();
	
	if(status != HAL_OK)
	{
		printf("write fail\r\n");
	}
	else
	{
		printf("write success\r\n");
	}

	/* 读取Flash内容 */
	addr = 0x080E0000;
	memcpy(data_buf, (uint32_t*)addr, sizeof(uint32_t)*10);
	printf("read after write:\r\n\t");
	for(i = 0;i < 10;i++)
	{
		printf("0x%08x ", data_buf[i]);
	}
	printf("\r\n");

}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC), sf, SaveAll2F, SaveAll2F());

/**
  * @brief  将Flash中的数据读至缓冲区
  * @retval 无
  */
void readF2ram()
{
	uint16_t cnt=0;
	int16_t len[3];
	uint32_t addr = 0x080E0000;
	//读数据长度
	len[0] = *(uint16_t *)(addr+cnt);//读半字--位置1数据总长度
	cnt += 2;
	len[1] = *(uint16_t *)(addr+cnt);//读半字--动作组数据总长度
	cnt += 2;
	len[2] = *(uint16_t *)(addr+cnt);//读半字--位置2数据总长度
	cnt += 2;

	//----开始读位置1----
	for(uint8_t i=0;i<len[0]/sizeof(postion[0]);i++)//遍历位置ID
	{
		postion[i].pos_id = *(uint16_t *)(addr+cnt);//读ID
		cnt += 2;
		for(uint8_t j=0;j<5;j++)
		{
			postion[i].angle[j] = *(uint16_t *)(addr+cnt);//读舵机角
			cnt += 2;
		}
		postion[i].timems = *(uint16_t *)(addr+cnt);//读动作时间
		cnt += 2;
	}
	//----开始读动作组----
	for(uint8_t i=0;i<(len[1]-len[0])/sizeof(group[0]);i++)//遍历动作组ID
	{
		for(uint8_t j=0;j<GROUP_POS_LEN;j++)
		{	
			group[i][j] = *(uint16_t *)(addr+cnt);//读取该动作组的位置序列
			cnt += 2;
		}
	}
	//----开始读位置ex----
	for(uint16_t i=0;i<(len[2]-len[1])/sizeof(postion_ex[0]);i++)//遍历位置ID
	{
		postion_ex[i].pos_id = *(uint16_t *)(addr+cnt);//读ID
		cnt += 2;
		for(uint8_t j=0;j<5;j++)
		{
			postion_ex[i].angle[j] = *(uint16_t *)(addr+cnt);//读舵机角
			cnt += 2;
		}
		postion_ex[i].timems = *(uint16_t *)(addr+cnt);//读动作时间
		cnt += 2;
	}
	
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC), rf, readF2ram, readF2ram());

/**
  * @brief  5个舵机全部归中
  * @retval 无
  */
void ArmGoMiddle()
{
	for(uint8_t temp=0;temp<5;temp++)
  {
		WritePos(temp+1, 510, 2000, 0);
  }
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC), mi, ArmGoMiddle, ArmGoMiddle());

/**
  * @brief  内部debug，不可用于其他用途
  * @retval 无
  */
void ArmSetBuff()
{
	for(uint8_t temp=0;temp<40;temp++)
  {
		for(uint8_t j=0;j<15;j++)
			group[temp][j]=j;
  }
	for(uint8_t temp=0;temp<100;temp++)
  {
		postion[temp].pos_id=temp;
		for(uint8_t j=0;j<5;j++)
			postion[temp].angle[j]=j;
  }
	for(uint16_t temp=0;temp<800;temp++)
  {
		postion_ex[temp].pos_id=temp;
		for(uint8_t j=0;j<5;j++)
			postion_ex[temp].angle[j]=j;
  }
	
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC), de1, ArmSetBuff, ArmSetBuff());


/*--------letter shell example begin--------*/
//void fun(char en)
//{
//    printf("Test function！\r\n");
//		ArmForceEnable(2,en);
//}
////键入func 以执行fun
//SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC), func, fun, test);
/*--------letter shell example end--------*/
