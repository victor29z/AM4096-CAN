#include "can_user.h"


//-----------------------------extern global varibles---------------------------------------
extern unsigned char can_data[8];
extern unsigned char new_can_data;
extern unsigned char SMB_status;

extern unsigned int CPU_ID0;
extern unsigned int CPU_ID1;
extern unsigned int CPU_ID2;
extern unsigned int can_id;

u32 slave_id = HAND_PWM_CANID;
//extern __no_init const unsigned int can_id_mem @ 0x0800FC00;




void CAN_init(void)
{
	CAN_InitTypeDef        CAN_InitStructure;
	CAN_FilterInitTypeDef  CAN_FilterInitStructure;
/*	FLASH_Unlock();
	can_id = (*(unsigned int *)(DATA_SPACE_BASE + DATA_OFFSET_CANID))& 0x7ff;
	can_id = can_id_mem & 0x7ff;
	FLASH_Lock();
	*/
	//  CAN register init 
	CAN_DeInit(CAN1);
	CAN_StructInit(&CAN_InitStructure);
	// CAN cell init
	CAN_InitStructure.CAN_TTCM=DISABLE;
	CAN_InitStructure.CAN_ABOM=ENABLE;
	CAN_InitStructure.CAN_AWUM=ENABLE;
	CAN_InitStructure.CAN_NART=DISABLE; // enable auto re-transmit mode
	CAN_InitStructure.CAN_RFLM=DISABLE;
	CAN_InitStructure.CAN_TXFP=DISABLE;
	CAN_InitStructure.CAN_Mode=CAN_Mode_Normal;// 正常模式
	CAN_InitStructure.CAN_SJW=CAN_SJW_1tq;
	CAN_InitStructure.CAN_BS1=CAN_BS1_6tq;
	CAN_InitStructure.CAN_BS2=CAN_BS2_5tq;
	//CAN bit rate = RCC_APB1PeriphClock/CAN_SJW+CAN_BS1+CAN_BS2/CAN_Prescaler; 
	CAN_InitStructure.CAN_Prescaler=6;//72MHz/(1+6+5)/48 =125Kbps
	CAN_Init(CAN1,&CAN_InitStructure);
	// CAN filter init 
	CAN_FilterInitStructure.CAN_FilterNumber=0; 					//选择过滤器0
	//CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;	//标识符屏蔽位模式
	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdList;
	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;	//32位过滤器


CAN_FilterInitStructure.CAN_FilterIdHigh =(((u32)(slave_id )<<21)&0xffff0000)>>16;;	//接收板的CAN地址
CAN_FilterInitStructure.CAN_FilterIdLow =  (((u32)(slave_id)<<21)|CAN_ID_STD|CAN_RTR_DATA)&0xffff;				//选择扩展标识符（见手册CAN_RIxR）
//CAN_FilterInitStructure.CAN_FilterIdHigh =(u16)slave_id << 5;	//接收板的CAN地址
//CAN_FilterInitStructure.CAN_FilterIdLow = 0 | CAN_ID_STD;				//选择扩展标识符（见手册CAN_RIxR）

CAN_FilterInitStructure.CAN_FilterMaskIdHigh =(((u32)(can_id )<<21)&0xffff0000)>>16;;	//接收板的CAN地址
CAN_FilterInitStructure.CAN_FilterMaskIdLow =  (((u32)(can_id)<<21)|CAN_ID_STD|CAN_RTR_REMOTE)&0xffff;				//选择扩展标识符（见手册CAN_RIxR）

//	CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0xFFFF;//0xffff;			//接收板的地址要和tempid一致
//	CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0xFFFF;			//下面有介绍

	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_FIFO0; 	//选择FIFO0
	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;			//使能过滤器
	CAN_FilterInit(&CAN_FilterInitStructure); 						//进入初始化函数

	// CAN FIFO0 message pending interrupt enable 
	CAN_ITConfig(CAN1,CAN_IT_FMP0, ENABLE);  //FIFO0 消息挂号中断屏蔽
	CAN_ITConfig(CAN1,CAN_IT_TME, ENABLE);           //发送邮箱空中断屏蔽
	CAN_ITConfig(CAN1,CAN_IT_BOF, ENABLE);   //离线中断允许
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
	CanTxMsg TxMessage; //定义数据结构类型变量
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
			TxMessage.StdId=0x301;        //接受板地址
			TxMessage.IDE=CAN_ID_STD;//使用标准帧
			TxMessage.RTR=CAN_RTR_DATA;//发送数据帧
			TxMessage.DLC= 4; //设置发送数据的长度

			TxMessage.Data[0] = (CPU_ID0 >> 24) & 0xff;
			TxMessage.Data[1] = (CPU_ID0 >> 16) & 0xff;
			TxMessage.Data[2] = (CPU_ID0 >> 8) & 0xff;
			TxMessage.Data[3] = CPU_ID0 & 0xff;
			TransmitMailbox = CAN_Transmit(CAN1,&TxMessage);//开始发送数据
			i = 0;
			//通过检查CANTXOK位来确认发送是否成功
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

void Can_Send_Temp(u32 temp, unsigned char mag,unsigned int id)
{
	CanTxMsg TxMessage; //定义数据结构类型变量
	unsigned char TransmitMailbox = 0;
	unsigned char i;

	TxMessage.StdId=can_id;        //接受板地址
	TxMessage.IDE=CAN_ID_STD;//使用标准帧
	TxMessage.RTR=CAN_RTR_DATA;//发送数据帧
	TxMessage.DLC= 4; //设置发送数据的长度

	TxMessage.Data[0] = (temp >> 24) & 0xff;
	TxMessage.Data[1] = (temp >> 16) & 0xff;
	TxMessage.Data[2] = (temp >> 8) & 0xff;
	TxMessage.Data[3] = temp & 0xff;

	TransmitMailbox = CAN_Transmit(CAN1,&TxMessage);//开始发送数据
	i = 0;
	//通过检查CANTXOK位来确认发送是否成功
	while((CAN_TransmitStatus(CAN1,TransmitMailbox) != CANTXOK) && (i != 0xFF))
	{
		i++;
	}
}

