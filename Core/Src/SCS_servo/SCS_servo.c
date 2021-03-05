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

#define POS_LEN 100
#define GROUP_LEN 30
#define GROUP_POS_LEN 15
//#define ID_START 1
//#define ID_END 5



//开辟机械臂"位置"缓冲区
Pos postion[POS_LEN];
//开辟机械臂"动作组"缓冲区
int8_t group[GROUP_LEN][GROUP_POS_LEN]; 


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
	for(uint8_t temp=0;temp<5;temp++)//读取5个舵机的角度
  {
		postion0.angle[temp]=ReadPos(temp+1);
  }
	postion0.pos_id=ID_;//设置动作id
	postion0.timems=timems_;//设置动作时间
	postion[ID_]=postion0;//存储至缓存区
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC), sap, SavePos, savePos(id,time));
/**
  * @brief  使机械臂动作到指定位置
  * @param  ID_:指定位置信息存储在数组中的位置
  * @retval 无
  */
void GoPos(int16_t ID_)
{
	for(uint8_t temp=0;temp<5;temp++)//写5个舵机的角度
  {
		WritePos(temp+1, postion[ID_].angle[temp], postion[ID_].timems, 0);//舵机(IDtemp),以时间timems毫秒,运行至postion[ID_].angle[temp]角度
	}
  delay(postion[ID_].timems);//堵塞式等待动作完成
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
void Pos2Group(uint8_t G_ID_,uint8_t GP_ID_,uint8_t P_ID)
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
  * @brief  存储一个16bit数据，仅用于SaveAll2ee()函数中
  * @param  *cnt:要存储的地址
  * @param  dt:要存储的16bit数据
  * @retval 无
  */
static void Save2ee16(uint16_t* cnt,int16_t dt)
{
	uint8_t s[2];
	s[0]=LSB(dt);
	s[1]=MSB(dt);
	ee24_write(*cnt,s,2,0xffff);
	*cnt+=2;
}
/**
  * @brief  读取一个16bit数据，仅用于readAll2ram()函数中
  * @param  *cnt:要读取的地址
  * @param  dt:读取得到的16bit数据句柄
  * @retval 无
  */
static void Readee16(uint16_t* cnt,int16_t *dt)
{
	uint8_t s[2];
	ee24_read(*cnt, s, 2, 1000);
	*dt = (int16_t) (s[1] << 8) | s[0]; 
	*cnt+=2;
}
/**
  * @brief  将与机械臂有关的缓冲区变量存入EEPROM中
  * @retval 无
  */
void SaveAll2ee()
{
	uint16_t lenadd=0;
	uint16_t cnt=4;//前面空4个位置，用来存两部分的数据长度
	if (ee24_isConnected()){
		
		//----开始存位置，最大1400字节
		for(uint8_t i=0;postion[i].pos_id!=-1;i++)//遍历位置ID
		{
			Save2ee16(&cnt,postion[i].pos_id);//存ID
			for(uint8_t j=0;j<5;j++)
			{
				Save2ee16(&cnt,postion[i].angle[j]);//存舵机角
			}
			Save2ee16(&cnt,postion[i].timems);//存动作时间
		}
		//存位置结束
		Save2ee16(&lenadd,cnt);//在EEPROM的开头存位置数据总长度
		
		//----开始存动作组，最大450字节
		for(uint8_t i=0;group[i][0]!=-1;i++)//遍历动作组ID
		{
			ee24_write(cnt,(uint8_t*)group[i],GROUP_POS_LEN,0xffff);//存储该动作组的位置序列
			cnt+=GROUP_POS_LEN;
		}
		//存动作组结束
		Save2ee16(&lenadd,cnt);//在EEPROM的开头存动作组数据总长度
	}
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC), sa, SaveAll2ee, SaveAll2ee());

/**
  * @brief  将EEPROM中的数据读至缓冲区
  * @retval 无
  */
void readAll2ram()
{
	uint16_t cnt=0;
	int16_t len[2];
	if (ee24_isConnected()){
		//读数据长度
		Readee16(&cnt,len);//位置数据总长度
		Readee16(&cnt,len+1);//动作组数据总长度
		
		//----开始读位置----
		for(uint8_t i=0;i<len[0]/sizeof(postion[0]);i++)//遍历位置ID
		{
			Readee16(&cnt,&postion[i].pos_id);//读ID
			for(uint8_t j=0;j<5;j++)
			{
				Readee16(&cnt,&postion[i].angle[j]);//读舵机角
			}
			Readee16(&cnt,&postion[i].timems);//读ID
		}
		//----开始读动作组----
		for(uint8_t i=0;i<len[1]/GROUP_POS_LEN;i++)//遍历动作组ID
		{
			ee24_read(cnt,(uint8_t*)group[i],GROUP_POS_LEN,0xffff);//读取该动作组的位置序列
			cnt+=GROUP_POS_LEN;
		}
	}
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC), ra, readAll2ram, readAll2ram());

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



/*--------letter shell example begin--------*/
//void fun(char en)
//{
//    printf("Test function！\r\n");
//		ArmForceEnable(2,en);
//}
////键入func 以执行fun
//SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC), func, fun, test);
/*--------letter shell example end--------*/
