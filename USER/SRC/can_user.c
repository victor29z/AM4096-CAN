#include "can_user.h"
#include "stm32f10x.h"
#include "core_cm3.h"
#include "stm32f10x_gpio.h"
#include "main.h"
#include "stm32f10x_exti.h"
#include "bsp.h"
#include "SMBus.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_can.h"
#include "stm32f10x_flash.h"

#include <stdio.h>
#include "serial.h"

//-----------------------------extern global varibles---------------------------------------
extern unsigned char can_data[8];
extern unsigned char new_can_data;
extern unsigned char SMB_status;

extern unsigned int CPU_ID0;
extern unsigned int CPU_ID1;
extern unsigned int CPU_ID2;
extern unsigned int can_id;
//extern __no_init const unsigned int can_id_mem @ 0x0800FC00;



void CAN_init(void)
{
	CAN_InitTypeDef        CAN_InitStructure;
	CAN_FilterInitTypeDef  CAN_FilterInitStructure;
	//FLASH_Unlock();
	//can_id = (*(unsigned int *)(DATA_SPACE_BASE + DATA_OFFSET_CANID))& 0x7ff;
	//can_id = can_id_mem & 0x7ff;
	//FLASH_Lock();
	//  CAN register init 
	CAN_DeInit(CAN1);
	CAN_StructInit(&CAN_InitStructure);
	// CAN cell init
	CAN_InitStructure.CAN_TTCM=DISABLE;
	CAN_InitStructure.CAN_ABOM=ENABLE;
	CAN_InitStructure.CAN_AWUM=ENABLE;
	CAN_InitStructure.CAN_NART=ENABLE;
	CAN_InitStructure.CAN_RFLM=DISABLE;
	CAN_InitStructure.CAN_TXFP=DISABLE;
	CAN_InitStructure.CAN_Mode=CAN_Mode_Normal;// ����ģʽ
	CAN_InitStructure.CAN_SJW=CAN_SJW_1tq;
	CAN_InitStructure.CAN_BS1=CAN_BS1_8tq;
	CAN_InitStructure.CAN_BS2=CAN_BS2_7tq;
	//CAN bit rate = RCC_APB1PeriphClock/CAN_SJW+CAN_BS1+CAN_BS2/CAN_Prescaler; 
	CAN_InitStructure.CAN_Prescaler=10; //8MHz/(1+8+7)/10 =50Kbps
	CAN_Init(CAN1,&CAN_InitStructure);
	// CAN filter init 
	CAN_FilterInitStructure.CAN_FilterNumber=0; 					//ѡ�������0
	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;	//��ʶ������λģʽ
	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;	//32λ������

	CAN_FilterInitStructure.CAN_FilterIdHigh =CAN_BCST_ID << 5;	//���հ��CAN��ַ
	CAN_FilterInitStructure.CAN_FilterIdLow = 0x0004;				//ѡ����չ��ʶ�������ֲ�CAN_RIxR��
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0;//0xffff;			//���հ�ĵ�ַҪ��tempidһ��
	CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;			//�����н���

	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_FIFO0; 	//ѡ��FIFO0
	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;			//ʹ�ܹ�����
	CAN_FilterInit(&CAN_FilterInitStructure); 						//�����ʼ������

	// CAN FIFO0 message pending interrupt enable 
	CAN_ITConfig(CAN1,CAN_IT_FMP0, ENABLE);  //FIFO0 ��Ϣ�Һ��ж�����
	CAN_ITConfig(CAN1,CAN_IT_TME, ENABLE);           //����������ж�����
	CAN_ITConfig(CAN1,CAN_IT_BOF, ENABLE);   //�����ж�����
}

void Set_Can_id(unsigned int id)
{
	id &= 0x7ff;
	FLASH_Unlock();
	FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);
	FLASH_ErasePage(DATA_SPACE_BASE);
	FLASH_ProgramHalfWord((DATA_SPACE_BASE + DATA_OFFSET_CANID),id); 
	FLASH_Lock();
}
void Can_Msg_Process(void)
{
	unsigned int recv_cpu_id;
	unsigned int recv_can_id;
	CanTxMsg TxMessage; //�������ݽṹ���ͱ���
	unsigned char TransmitMailbox = 0;
	unsigned char i;
	switch(can_data[0])
	{
		case CAN_MSG_SETID:
			for(i = 0; i < 4; i++)
			{
				recv_cpu_id <<= 8;
				recv_cpu_id |= can_data[i + 1];
			}
			for(i = 0; i < 2; i++)
			{
				recv_can_id <<= 8;
				recv_can_id |= can_data[i + 5];
			}
			
			if(recv_cpu_id == CPU_ID0)
			{
				Set_Can_id(recv_can_id & 0x7ff);
				can_id = recv_can_id & 0x7ff;
			}
			new_can_data = 0;
			break;
		case CAN_MSG_UPLOAD:
			Delay_nms(CPU_ID0 & 0x1ff);
			TxMessage.StdId=0x301;        //���ܰ��ַ
			TxMessage.IDE=CAN_ID_STD;//ʹ�ñ�׼֡
			TxMessage.RTR=CAN_RTR_DATA;//��������֡
			TxMessage.DLC= 4; //���÷������ݵĳ���

			TxMessage.Data[0] = (CPU_ID0 >> 24) & 0xff;
			TxMessage.Data[1] = (CPU_ID0 >> 16) & 0xff;
			TxMessage.Data[2] = (CPU_ID0 >> 8) & 0xff;
			TxMessage.Data[3] = CPU_ID0 & 0xff;
			TransmitMailbox = CAN_Transmit(CAN1,&TxMessage);//��ʼ��������
			i = 0;
			//ͨ�����CANTXOKλ��ȷ�Ϸ����Ƿ�ɹ�
			while((CAN_TransmitStatus(CAN1,TransmitMailbox) != CANTXOK) && (i != 0xFF))
			{
				i++;
			}
			break;	
			new_can_data = 0;
		case CAN_MSG_SETEMY:
			if(SMB_status == SMB_IDLE)
			{
				MLX_Write_Emissivity(can_data[1]);
				Delay_nms(500);
				GPIO_ResetBits(SEN_ONOFF_PORT,SEN_ONOFF_PIN);
				Delay_nms(500);
				GPIO_SetBits(SEN_ONOFF_PORT,SEN_ONOFF_PIN);
				Delay_nms(500);
				SMBus_Init();
				new_can_data = 0;
			}
			break;	
			
	}

			
}

void Can_Send_Temp(unsigned int temp, unsigned char mag,unsigned int id)
{
	CanTxMsg TxMessage; //�������ݽṹ���ͱ���
	unsigned char TransmitMailbox = 0;
	unsigned char i;

	TxMessage.StdId=can_id;        //���ܰ��ַ
	TxMessage.IDE=CAN_ID_STD;//ʹ�ñ�׼֡
	TxMessage.RTR=CAN_RTR_DATA;//��������֡
	TxMessage.DLC= 2; //���÷������ݵĳ���

	TxMessage.Data[0] = (temp >> 8) & 0xff;
	TxMessage.Data[1] = temp & 0xff;
	TxMessage.Data[2] = mag;

	TransmitMailbox = CAN_Transmit(CAN1,&TxMessage);//��ʼ��������
	i = 0;
	//ͨ�����CANTXOKλ��ȷ�Ϸ����Ƿ�ɹ�
	while((CAN_TransmitStatus(CAN1,TransmitMailbox) != CANTXOK) && (i != 0xFF))
	{
		i++;
	}
}
