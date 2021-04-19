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
#include "Buzzer/buzzerDriver.h"

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
	if(ID_!=5)
	{
		unLockEprom(ID_);//打开EPROM保存功能
		EnableTorque(ID_, Enable_);
		LockEprom(ID_);//关闭EPROM保存功能
	}
	else if(ID_==5)
	{
		unLockEpromEx(ID_);//打开EPROM保存功能
		EnableTorque(ID_, Enable_);
		LockEpromEx(ID_);//关闭EPROM保存功能
	}
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
	if(!Enable_)//卸力时
		music2Play();//嘀嘀嘀
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
  * @brief  使某舵机动作到指定位置，并将写入的指定位置存为该舵机之后的动作目标位置
  * @param  PID:动作结构体序号
  * @param  SID:指定舵机的序号（1、2、3、4、5）
  * @param  Position:指定舵机要运动到的目标位置（20-1000）
  * @param  Position:指定舵机运动到的目标位置花费的时间（ms）
  * @retval 无
  */
void WritePos_sp(int16_t PID,uint8_t SID, uint16_t Position, uint16_t Time)
{	
	if(SID!=5)
		WritePos(SID,Position,Time,0);
	else
		WritePosEx(SID,Position,0,0);
	Pos postion0;
	/*读取*/
	if(PID<POS_LEN)
		postion0=postion[PID];//存储至缓存区1
	else
		postion0=postion_ex[PID-POS_LEN];//存储至缓存区2
	/*修改*/
	postion0.angle[SID-1]=Position;
	/*存储*/
	for(uint8_t i=0;i<5;i++)
	user_main_printf("angle%d: %d",i,postion0.angle[i]);
	if(PID<POS_LEN)
		postion[PID]=postion0;//存储至缓存区1
	else
		postion_ex[PID-POS_LEN]=postion0;//存储至缓存区2
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC), wwp, WritePos_sp, WritePos_sp(int16_t PID,uint8_t SID, uint16_t Position, uint16_t Time));
/**
  * @brief  查看指定动作结构体的数据
  * @param  PID:动作结构体序号
  * @retval 无
  */
void seePos(int16_t PID)
{	
	Pos postion0;
	/*读取*/
	if(PID<POS_LEN)
		postion0=postion[PID];//存储至缓存区1
	else
		postion0=postion_ex[PID-POS_LEN];//存储至缓存区2
	/*打印*/
	user_main_printf("PID=%d",postion0.pos_id);
	user_main_printf("time=%d",postion0.timems);
	for(uint8_t i=0;i<5;i++)
	user_main_printf("angle%d:%d",i,postion0.angle[i]);
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC), seep, seePos, seePos(int16_t PID));

void changePosTime(int16_t PID,uint16_t Time)
{
	Pos postion0;
	/*读取*/
	if(PID<POS_LEN)
		postion0=postion[PID];//存储至缓存区1
	else
		postion0=postion_ex[PID-POS_LEN];//存储至缓存区2
	user_main_printf("time before=%d",postion0.timems);
	postion0.timems=Time;
	user_main_printf("time after=%d",postion0.timems);
	/*存储*/
	for(uint8_t i=0;i<5;i++)
	user_main_printf("angle%d: %d",i,postion0.angle[i]);
	if(PID<POS_LEN)
		postion[PID]=postion0;//存储至缓存区1
	else
		postion_ex[PID-POS_LEN]=postion0;//存储至缓存区2
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC), cpt, changePosTime, changePosTime(int16_t PID,,uint16_t Time));


/**
  * @brief  复制动作结构体
  * @param  Pfrom:复制的源动作结构体序号
  * @param  Pto:复制的目的动作结构体序号
  * @retval 无
  */
void copyPos(int16_t Pfrom,int16_t Pto)
{	
	Pos postion0;
	/*读取*/
	if(Pfrom<POS_LEN)
		postion0=postion[Pfrom];//从缓存区1读取
	else
		postion0=postion_ex[Pfrom-POS_LEN];//从缓存区2读取
	/*打印*/
	for(uint8_t i=0;i<5;i++)
	user_main_printf("angle%d: %d",i,postion0.angle[i]);
	if(Pto<POS_LEN)
		postion[Pto]=postion0;//存储至缓存区1
	else
		postion_ex[Pto-POS_LEN]=postion0;//存储至缓存区2
	user_main_printf("copy complete!");
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC), cop, copyPos, copyPos(int16_t Pfrom,int16_t Pto));


/**
  * @brief  使机械臂动作到指定位置
  * @param  ID_:指定位置信息存储在数组中的位置
  * @retval 无
  */
void GoPos(int16_t ID_)
{
	if(ID_<POS_LEN){
		if(postion[ID_].pos_id==-1)
			return;
	}
	else{
		if(postion_ex[ID_-POS_LEN].pos_id==-1)
			return;
	}
	if(ID_<POS_LEN){
		for(uint8_t temp=0;temp<5;temp++)//写5个舵机的角度
		{
			if(temp!=4)
				WritePos(temp+1, postion[ID_].angle[temp], postion[ID_].timems, 0);//舵机(IDtemp),以时间timems毫秒,运行至postion[ID_].angle[temp]角度
			else if(temp==4)
				WritePosEx(temp+1, postion[ID_].angle[temp],2000, 50);//舵机(IDtemp),以时间timems毫秒,运行至postion[ID_].angle[temp]角度
		}
		delay(postion[ID_].timems);//堵塞式等待动作完成
	}
	else{
		ID_ -= POS_LEN;
		for(uint8_t temp=0;temp<5;temp++)//写5个舵机的角度
		{
			if(temp!=4)
				WritePos(temp+1, postion_ex[ID_].angle[temp], postion[ID_].timems, 0);//舵机(IDtemp),以时间timems毫秒,运行至postion[ID_].angle[temp]角度
			else if(temp==4)
				WritePosEx(temp+1, postion_ex[ID_].angle[temp],2000, 50);//舵机(IDtemp),以时间timems毫秒,运行至postion[ID_].angle[temp]角度
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
	for(uint8_t temp=0;temp<4;temp++)
  {
		WritePos(temp+1, 510, 2000, 0);
  }
	WritePosEx(5, 2000, 2000, 50);
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
/*
typedef struct
{
	int16_t  pos_id;
	int16_t  angle[5];//20-1003 mid:511
	int16_t  timems;
}Pos;
*/
void Write5Angle(uint16_t PID, int16_t angle0, int16_t angle1, int16_t angle2, int16_t angle3, int16_t angle4, int16_t timems_)
{
	if(PID<POS_LEN){
			postion[PID].pos_id=PID;
			postion[PID].angle[0]=angle0;
			postion[PID].angle[1]=angle1;
			postion[PID].angle[2]=angle2;
			postion[PID].angle[3]=angle3;
			postion[PID].angle[4]=angle4;
			postion[PID].timems=timems_;
	}
	else{
			uint16_t k=PID-POS_LEN;
			postion_ex[k].pos_id=PID;
			postion_ex[k].angle[0]=angle0;
			postion_ex[k].angle[1]=angle1;
			postion_ex[k].angle[2]=angle2;
			postion_ex[k].angle[3]=angle3;
			postion_ex[k].angle[4]=angle4;
			postion_ex[k].timems=timems_;
	}
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC), w5a, Write5Angle, Write5Angle(PID,angle0~4,timems_));
void Write5Position(int16_t angle0, int16_t angle1, int16_t angle2, int16_t angle3, int16_t angle4, int16_t timems_)
{
	WritePos(1,angle0,timems_,0);
	WritePos(2,angle1,timems_,0);
	WritePos(3,angle2,timems_,0);
	WritePos(4,angle3,timems_,0);
	WritePosEx(5,angle4,timems_,0);
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC), w5p, Write5Position, Write5Position(angle0~4,timems_));
void readGroup(uint8_t GID)
{
	for(uint8_t i=0;i<15&&group[GID][i]!=-1;i++)
	{
		user_main_printf("%d,", group[GID][i]);
	}
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC), rg, readGroup, readGroup(GID));
void readAllGroupTime()
{
	uint16_t time;
	for(uint8_t i=0;i<GROUP_LEN&&group[i][0]!=-1;i++)//0-39
	{
		for(uint8_t j=0;j<GROUP_POS_LEN&&group[i][j]!=-1;j++)//0-15
		{
			if(group[i][j]<POS_LEN)
				time += postion[group[i][j]].timems;
			else
				time += postion_ex[group[i][j]-POS_LEN].timems;
		}
	}
	user_main_printf("total arm time: %d", time);
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC), ragt, readAllGroupTime, readAllGroupTime(GID));

/*--------letter shell example begin--------*/
//void fun(char en)
//{
//    printf("Test function！\r\n");
//		ArmForceEnable(2,en);
//}
////键入func 以执行fun
//SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC), func, fun, test);
/*--------letter shell example end--------*/
