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
	unLockEprom(ID_);//��EPROM���湦��
	EnableTorque(ID_, Enable_);
	LockEprom(ID_);//�ر�EPROM���湦��
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
  * @brief  ʹ��е�۶�����ָ��λ��
  * @param  ID_:ָ��λ����Ϣ�洢�������е�λ��
  * @retval ��
  */
void GoPos(int16_t ID_)
{
	if(ID_<POS_LEN){
		for(uint8_t temp=0;temp<5;temp++)//д5������ĽǶ�
		{
			WritePos(temp+1, postion[ID_].angle[temp], postion[ID_].timems, 0);//���(IDtemp),��ʱ��timems����,������postion[ID_].angle[temp]�Ƕ�
		}
		delay(postion[ID_].timems);//����ʽ�ȴ��������
	}
	else{
		ID_ -= POS_LEN;
		for(uint8_t temp=0;temp<5;temp++)//д5������ĽǶ�
		{
			WritePos(temp+1, postion_ex[ID_].angle[temp], postion_ex[ID_].timems, 0);//���(IDtemp),��ʱ��timems����,������postion_ex[ID_].angle[temp]�Ƕ�
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
	for(uint8_t temp=0;temp<5;temp++)
  {
		WritePos(temp+1, 510, 2000, 0);
  }
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


/*--------letter shell example begin--------*/
//void fun(char en)
//{
//    printf("Test function��\r\n");
//		ArmForceEnable(2,en);
//}
////����func ��ִ��fun
//SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC), func, fun, test);
/*--------letter shell example end--------*/
