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
#include <string.h> //ʹ�õ���memcpy
#include "user_usart.h"
#include "Buzzer/buzzerDriver.h"

#define POS_LEN 100
#define GROUP_LEN 40//30
#define GROUP_POS_LEN 15

#define POS_LEN_EX 800


/*-------��һ������-------*/
/*���ٻ�е��"λ��"������*/
Pos postion[POS_LEN];
/*���ٻ�е��"������"������*/
int16_t group[GROUP_LEN][GROUP_POS_LEN]; 

/*-------�ڶ�������-------*/
/*���ٻ�е�۵ڶ�"λ��"������,�������ݽ�����stm32��Ƭ��flash*/
Pos postion_ex[POS_LEN_EX];


/**
  * @brief  ��ʼ������
  * @retval ��
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
  * @brief  ����������ؿ���
  * @param  ID_:�����ƵĶ��ID
  * @param  Enable_:��0�ر������������1�����������
  * @retval ��
  */
void ArmForceEnable(uint8_t ID_,uint8_t Enable_){
	if(ID_!=5)
	{
		unLockEprom(ID_);//��EPROM���湦��
		EnableTorque(ID_, Enable_);
		LockEprom(ID_);//�ر�EPROM���湦��
	}
	else if(ID_==5)
	{
		unLockEpromEx(ID_);//��EPROM���湦��
		EnableTorque(ID_, Enable_);
		LockEpromEx(ID_);//�ر�EPROM���湦��
	}
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC), amf, ArmForceEnable, ArmForceEnable(id,en));
/**
  * @brief  ȫ��������ؿ���
  * @param  Enable_:��0�ر������������1�����������
  * @retval ��
  */
void ForceAll(uint8_t Enable_){
	for(uint8_t ID_=0;ID_<5;ID_++){
		unLockEprom(ID_+1);//��EPROM���湦��
		EnableTorque(ID_+1, Enable_);
		LockEprom(ID_+1);//�ر�EPROM���湦��
	}
	if(!Enable_)//ж��ʱ
		music2Play();//������
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC), fll, ForceAll, forceAll(en));
/**
  * @brief  �洢��е�۵�ǰλ����Ϣ
  * @param  ID_:��ǰλ����Ϣ�洢�������е�λ��
  * @param  timems_:��ǰλ�õ�ִ��ʱ�䣨����ת���йأ�
  * @retval ��
  */
void SavePos(int16_t ID_,int16_t timems_)
{
	Pos postion0;
	for(uint8_t temp=0;temp<5;temp++)//��ȡ���洢5������ĽǶ�
  {
		postion0.angle[temp]=ReadPos(temp+1);
  }
	postion0.pos_id=ID_;//���ö���id
	postion0.timems=timems_;//���ö���ʱ��
	/*�洢*/
	for(uint8_t i=0;i<5;i++)
	user_main_printf("angle%d: %d",i,postion0.angle[i]);
	if(ID_<POS_LEN)
		postion[ID_]=postion0;//�洢��������1
	else
		postion_ex[ID_-POS_LEN]=postion0;//�洢��������2
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC), sap, SavePos, savePos(id,time));
/**
  * @brief  ʹĳ���������ָ��λ�ã�����д���ָ��λ�ô�Ϊ�ö��֮��Ķ���Ŀ��λ��
  * @param  PID:�����ṹ�����
  * @param  SID:ָ���������ţ�1��2��3��4��5��
  * @param  Position:ָ�����Ҫ�˶�����Ŀ��λ�ã�20-1000��
  * @param  Position:ָ������˶�����Ŀ��λ�û��ѵ�ʱ�䣨ms��
  * @retval ��
  */
void WritePos_sp(int16_t PID,uint8_t SID, uint16_t Position, uint16_t Time)
{	
	if(SID!=5)
		WritePos(SID,Position,Time,0);
	else
		WritePosEx(SID,Position,0,0);
	Pos postion0;
	/*��ȡ*/
	if(PID<POS_LEN)
		postion0=postion[PID];//�洢��������1
	else
		postion0=postion_ex[PID-POS_LEN];//�洢��������2
	/*�޸�*/
	postion0.angle[SID-1]=Position;
	/*�洢*/
	for(uint8_t i=0;i<5;i++)
	user_main_printf("angle%d: %d",i,postion0.angle[i]);
	if(PID<POS_LEN)
		postion[PID]=postion0;//�洢��������1
	else
		postion_ex[PID-POS_LEN]=postion0;//�洢��������2
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC), wwp, WritePos_sp, WritePos_sp(int16_t PID,uint8_t SID, uint16_t Position, uint16_t Time));
/**
  * @brief  �鿴ָ�������ṹ�������
  * @param  PID:�����ṹ�����
  * @retval ��
  */
void seePos(int16_t PID)
{	
	Pos postion0;
	/*��ȡ*/
	if(PID<POS_LEN)
		postion0=postion[PID];//�洢��������1
	else
		postion0=postion_ex[PID-POS_LEN];//�洢��������2
	/*��ӡ*/
	user_main_printf("PID=%d",postion0.pos_id);
	user_main_printf("time=%d",postion0.timems);
	for(uint8_t i=0;i<5;i++)
	user_main_printf("angle%d:%d",i,postion0.angle[i]);
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC), seep, seePos, seePos(int16_t PID));

void changePosTime(int16_t PID,uint16_t Time)
{
	Pos postion0;
	/*��ȡ*/
	if(PID<POS_LEN)
		postion0=postion[PID];//�洢��������1
	else
		postion0=postion_ex[PID-POS_LEN];//�洢��������2
	user_main_printf("time before=%d",postion0.timems);
	postion0.timems=Time;
	user_main_printf("time after=%d",postion0.timems);
	/*�洢*/
	for(uint8_t i=0;i<5;i++)
	user_main_printf("angle%d: %d",i,postion0.angle[i]);
	if(PID<POS_LEN)
		postion[PID]=postion0;//�洢��������1
	else
		postion_ex[PID-POS_LEN]=postion0;//�洢��������2
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC), cpt, changePosTime, changePosTime(int16_t PID,,uint16_t Time));


/**
  * @brief  ���ƶ����ṹ��
  * @param  Pfrom:���Ƶ�Դ�����ṹ�����
  * @param  Pto:���Ƶ�Ŀ�Ķ����ṹ�����
  * @retval ��
  */
void copyPos(int16_t Pfrom,int16_t Pto)
{	
	Pos postion0;
	/*��ȡ*/
	if(Pfrom<POS_LEN)
		postion0=postion[Pfrom];//�ӻ�����1��ȡ
	else
		postion0=postion_ex[Pfrom-POS_LEN];//�ӻ�����2��ȡ
	/*��ӡ*/
	for(uint8_t i=0;i<5;i++)
	user_main_printf("angle%d: %d",i,postion0.angle[i]);
	if(Pto<POS_LEN)
		postion[Pto]=postion0;//�洢��������1
	else
		postion_ex[Pto-POS_LEN]=postion0;//�洢��������2
	user_main_printf("copy complete!");
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC), cop, copyPos, copyPos(int16_t Pfrom,int16_t Pto));


/**
  * @brief  ʹ��е�۶�����ָ��λ��
  * @param  ID_:ָ��λ����Ϣ�洢�������е�λ��
  * @retval ��
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
		for(uint8_t temp=0;temp<5;temp++)//д5������ĽǶ�
		{
			if(temp!=4)
				WritePos(temp+1, postion[ID_].angle[temp], postion[ID_].timems, 0);//���(IDtemp),��ʱ��timems����,������postion[ID_].angle[temp]�Ƕ�
			else if(temp==4)
				WritePosEx(temp+1, postion[ID_].angle[temp],2000, 50);//���(IDtemp),��ʱ��timems����,������postion[ID_].angle[temp]�Ƕ�
		}
		delay(postion[ID_].timems);//����ʽ�ȴ��������
	}
	else{
		ID_ -= POS_LEN;
		for(uint8_t temp=0;temp<5;temp++)//д5������ĽǶ�
		{
			if(temp!=4)
				WritePos(temp+1, postion_ex[ID_].angle[temp], postion[ID_].timems, 0);//���(IDtemp),��ʱ��timems����,������postion[ID_].angle[temp]�Ƕ�
			else if(temp==4)
				WritePosEx(temp+1, postion_ex[ID_].angle[temp],2000, 50);//���(IDtemp),��ʱ��timems����,������postion[ID_].angle[temp]�Ƕ�
		}
		delay(postion_ex[ID_].timems);//����ʽ�ȴ��������
	}
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC), gop, GoPos, goPos(id));

/**
  * @brief  �޸Ļ�������е�۶���������
  * @param  ID_:��������Ϣ�洢�������е�λ��
  * @param  PosIDs:��˳��洢�Ŷ������������������
  * @retval ��
  */
//int8_t order[]={0,1,2,3,4,5};
//saveGroup(0,order);
void Pos2Group(uint8_t G_ID_,uint8_t GP_ID_,uint16_t P_ID)
{
		group[G_ID_][GP_ID_]=P_ID;
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC), p2g, Pos2Group, Pos2Group(gid,gpid,pid));

/**
  * @brief  ִ�л�������ĳ��������
  * @param  ID_:��������Ϣ�洢�������е�λ��
  * @retval ��
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
  * @brief  �����е���йصĻ��������������ڲ�Flash��
	* ����11(0x080E 0000 - 0x080F FFFF)��,�������ɴ洢128kByte����
  * @retval ��
  */
void SaveAll2F()
{
	int i = 0;
	uint32_t PageError = 0;
	FLASH_EraseInitTypeDef a;
	HAL_StatusTypeDef status;
	uint32_t addr = 0x080E0000;
	uint32_t data_buf[10];
	
	/* ��ȡFlash���� */
	memcpy(data_buf, (uint32_t*)addr, sizeof(uint32_t)*10);
	printf("read before erase:\r\n\t");
	for(i = 0;i < 10;i++)
	{
		printf("0x%08x ", data_buf[i]);
	}
	printf("\r\n");
	
	/*����Ҫ�����Ĳ���*/
	a.TypeErase = FLASH_TYPEERASE_SECTORS;
	a.Banks = FLASH_BANK_1;
	a.Sector = FLASH_SECTOR_11;
	a.NbSectors = 1;
	a.VoltageRange = FLASH_VOLTAGE_RANGE_3;

	HAL_FLASH_Unlock();
	status = HAL_FLASHEx_Erase(&a,&PageError);/*��������11������*/
	HAL_FLASH_Lock();
	if(status != HAL_OK)
	{
		printf("erase fail, PageError = %d\r\n", PageError);
	}
	else
		printf("erase success\r\n");

	/* ��ȡFlash���� */
	memcpy(data_buf, (uint32_t*)addr, sizeof(uint32_t)*10);
	printf("read after erase:\r\n\t");
	for(i = 0;i < 10;i++)
	{
		printf("0x%08x ", data_buf[i]);
	}
	printf("\r\n");
	
	//д��Flash����
	HAL_FLASH_Unlock();
	
	{
		uint16_t lenadd=0;
		uint16_t cnt=6;//ǰ���6��λ�ã������������ֵ����ݳ���
			
			/*----��ʼ��λ��1�����1400�ֽ�*/
			for(uint8_t i=0;postion[i].pos_id != -1&& i < POS_LEN;i++)//����λ��ID
			{
				HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, addr+cnt, postion[i].pos_id);//��ID
				cnt += 2;
				for(uint8_t j=0;j<5;j++)
				{
					HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, addr+cnt, postion[i].angle[j]);//������
					cnt += 2;
				}
				HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, addr+cnt, postion[i].timems);//�涯��ʱ��
				cnt += 2;
			}
			//��λ�ý���
			HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, addr+lenadd, cnt);//�ڱ������Ŀ�ͷ��λ�������ܳ���
			lenadd += 2;
			//----��ʼ�涯���飬���900�ֽ�
			for(uint8_t i=0;group[i][0] != -1&& i < GROUP_LEN;i++)//����������ID
			{
				for(uint8_t j=0;j<GROUP_POS_LEN;j++)
				{
					HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, addr+cnt, group[i][j]);//�洢�ö������λ������
					cnt += 2;
				}
			}
			//�涯�������
			status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, addr+lenadd, cnt);//�ڱ������Ŀ�ͷ�涯���������ܳ���
			lenadd += 2;
			
			/*----��ʼ��λ��ex�����11200�ֽ�*/
			for(uint16_t i=0;postion_ex[i].pos_id!=-1&& i < POS_LEN_EX;i++)//����λ��ID
			{
				HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, addr+cnt, postion_ex[i].pos_id);//��ID
				cnt += 2;
				for(uint8_t j=0;j<5;j++)
				{
					HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, addr+cnt, postion_ex[i].angle[j]);//������
					cnt += 2;
				}
				HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, addr+cnt, postion_ex[i].timems);//�涯��ʱ��
				cnt += 2;
			}
			//��λ�ý���
			HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, addr+lenadd, cnt);//�ڱ������Ŀ�ͷ��λ��2�������ܳ���
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

	/* ��ȡFlash���� */
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
  * @brief  ��Flash�е����ݶ���������
  * @retval ��
  */
void readF2ram()
{
	uint16_t cnt=0;
	int16_t len[3];
	uint32_t addr = 0x080E0000;
	//�����ݳ���
	len[0] = *(uint16_t *)(addr+cnt);//������--λ��1�����ܳ���
	cnt += 2;
	len[1] = *(uint16_t *)(addr+cnt);//������--�����������ܳ���
	cnt += 2;
	len[2] = *(uint16_t *)(addr+cnt);//������--λ��2�����ܳ���
	cnt += 2;

	//----��ʼ��λ��1----
	for(uint8_t i=0;i<len[0]/sizeof(postion[0]);i++)//����λ��ID
	{
		postion[i].pos_id = *(uint16_t *)(addr+cnt);//��ID
		cnt += 2;
		for(uint8_t j=0;j<5;j++)
		{
			postion[i].angle[j] = *(uint16_t *)(addr+cnt);//�������
			cnt += 2;
		}
		postion[i].timems = *(uint16_t *)(addr+cnt);//������ʱ��
		cnt += 2;
	}
	//----��ʼ��������----
	for(uint8_t i=0;i<(len[1]-len[0])/sizeof(group[0]);i++)//����������ID
	{
		for(uint8_t j=0;j<GROUP_POS_LEN;j++)
		{	
			group[i][j] = *(uint16_t *)(addr+cnt);//��ȡ�ö������λ������
			cnt += 2;
		}
	}
	//----��ʼ��λ��ex----
	for(uint16_t i=0;i<(len[2]-len[1])/sizeof(postion_ex[0]);i++)//����λ��ID
	{
		postion_ex[i].pos_id = *(uint16_t *)(addr+cnt);//��ID
		cnt += 2;
		for(uint8_t j=0;j<5;j++)
		{
			postion_ex[i].angle[j] = *(uint16_t *)(addr+cnt);//�������
			cnt += 2;
		}
		postion_ex[i].timems = *(uint16_t *)(addr+cnt);//������ʱ��
		cnt += 2;
	}
	
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC), rf, readF2ram, readF2ram());

/**
  * @brief  5�����ȫ������
  * @retval ��
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
  * @brief  �ڲ�debug����������������;
  * @retval ��
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
//    printf("Test function��\r\n");
//		ArmForceEnable(2,en);
//}
////����func ��ִ��fun
//SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC), func, fun, test);
/*--------letter shell example end--------*/
