#include "stm32f10x_i2c.h"
#include "bsp.h"
#include "smbus.h"
#include "main.h"
#include <stdio.h>
#include "stm32f10x_usart.h"
#include "stm32f10x.h"

extern volatile unsigned int Timer_MLX; //time out counter
unsigned char StepNum;



SMB_data_strcut SMB_data_buffer;
unsigned char SMB_status = SMB_IDLE;


void SMBus_Init(void){
	I2C_InitTypeDef  I2C_InitStructure; 

	//i2c_status=I2CST_NULL;

	/* I2C configuration */
	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStructure.I2C_OwnAddress1 = ((I2C_SLAVE_ADDR)<<1);
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_InitStructure.I2C_ClockSpeed = I2C_Speed;
  
	/* Apply I2C configuration after enabling it */
	I2C_SoftwareResetCmd(I2C_DEVICE,ENABLE);
	I2C_SoftwareResetCmd(I2C_DEVICE,DISABLE);
	I2C_Cmd(I2C_DEVICE, ENABLE);
	I2C_AcknowledgeConfig(I2C_DEVICE, ENABLE);
	I2C_Init(I2C_DEVICE, &I2C_InitStructure);
	
	I2C_ITConfig(I2C_DEVICE, I2C_IT_BUF | I2C_IT_EVT, ENABLE);
	
}

u8 I2C_MLX_InData(u8 SlaveAddr,u8 Command,uint8_t* pbuff)
{
	Timer_MLX = 1000; //100ms of Time Out
    StepNum = 0;

	while(Timer_MLX && I2C_GetFlagStatus(I2C_DEVICE, I2C_FLAG_BUSY) );
	StepNum = 1;
	if(!Timer_MLX) return ERROR;
	I2C_GenerateSTART(I2C_DEVICE, ENABLE);
	while(Timer_MLX && !I2C_CheckEvent(I2C_DEVICE, I2C_EVENT_MASTER_MODE_SELECT)) ;   
	StepNum = 2;
	if(!Timer_MLX) return ERROR;
	I2C_DEVICE->DR = (SlaveAddr<<1);
	while(Timer_MLX && !I2C_CheckEvent(I2C_DEVICE, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
	StepNum = 3;
	if(!Timer_MLX) return ERROR;
	I2C_DEVICE->DR = Command;
	while(Timer_MLX && !I2C_CheckEvent(I2C_DEVICE, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) ;
	StepNum = 4;
	if(!Timer_MLX) return ERROR;
	I2C_GenerateSTART(I2C_DEVICE, ENABLE);
	while(Timer_MLX && !I2C_CheckEvent(I2C_DEVICE, I2C_EVENT_MASTER_MODE_SELECT));
	StepNum = 5;
	if(!Timer_MLX) return ERROR;
	I2C_DEVICE->DR = ((SlaveAddr<<1) | 1);
	//checkflag(I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED);
	while(Timer_MLX && !I2C_CheckEvent(I2C_DEVICE, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
	StepNum = 6;
	if(!Timer_MLX) return ERROR;
	(void)(I2C_GetFlagStatus(I2C_DEVICE, I2C_FLAG_ADDR));
	(void)(I2C_GetFlagStatus(I2C_DEVICE, I2C_FLAG_BUSY));
	while(Timer_MLX && !I2C_GetFlagStatus(I2C_DEVICE, I2C_FLAG_RXNE));
	StepNum = 7;
	if(!Timer_MLX) return ERROR;
	pbuff[0] = I2C_DEVICE->DR;
	(void)(I2C_GetFlagStatus(I2C_DEVICE, I2C_FLAG_ADDR));
	(void)(I2C_GetFlagStatus(I2C_DEVICE, I2C_FLAG_BUSY));
	while(Timer_MLX && !I2C_GetFlagStatus(I2C_DEVICE, I2C_FLAG_RXNE));
	StepNum = 8;
	if(!Timer_MLX) return ERROR;
	pbuff[1] = I2C_DEVICE->DR;
	I2C_AcknowledgeConfig(I2C_DEVICE, DISABLE);
	I2C_GenerateSTOP(I2C_DEVICE, ENABLE);
	(void)(I2C_GetFlagStatus(I2C_DEVICE, I2C_FLAG_ADDR));
	(void)(I2C_GetFlagStatus(I2C_DEVICE, I2C_FLAG_BUSY));
	while(Timer_MLX && !I2C_GetFlagStatus(I2C_DEVICE, I2C_FLAG_RXNE));
	StepNum = 9;
	if(!Timer_MLX) return ERROR;
	pbuff[2] = I2C_DEVICE->DR;
	I2C_AcknowledgeConfig(I2C_DEVICE, ENABLE);
	if(!Timer_MLX) return ERROR;
	return SUCCESS;  
}

ErrorStatus checkflag(unsigned int I2C_EVENT)
{
	unsigned int lastevent = 0;
	unsigned int flag1 = 0, flag2 = 0;
	ErrorStatus status = ERROR;


	/* Read the I2Cx status register */
	flag1 = I2C_DEVICE->SR1;
	flag2 = I2C_DEVICE->SR2;
	flag2 = flag2 << 16;

	/* Get the last event value from I2C status register */
	lastevent = (flag1 | flag2) & FLAG_Mask;

	/* Check whether the last event contains the I2C_EVENT */
	if ((lastevent & I2C_EVENT) == I2C_EVENT)
	{
	/* SUCCESS: last event is equal to I2C_EVENT */
	status = SUCCESS;
	}
	else
	{
	/* ERROR: last event is different from I2C_EVENT */
	status = ERROR;
	}
	/* Return status */
	return status;
}


int SMB_Read_Word(unsigned char addr, unsigned char type)
{
	if(SMB_status != SMB_IDLE)	return -1;
	SMB_data_buffer.command = addr | type;
	SMB_data_buffer.data = 0;
	SMB_data_buffer.dir = SLAVE_TO_MASTER;
	SMB_data_buffer.pec = 0;

	SMB_status = SMB_W_ADDRESS;
	Timer_MLX = 1000;
	I2C_GenerateSTART(I2C_DEVICE, ENABLE);
	return 0;
}

int SMB_Write_Word(unsigned char addr, unsigned char type,unsigned int data)
{
	unsigned char arr[6];				 //Buffer for the sent bytes

	if(SMB_status != SMB_IDLE)	return -1;
	SMB_data_buffer.command = addr | type;
	SMB_data_buffer.data = data;
	SMB_data_buffer.dir = MASTER_TO_SLAVE;

	SMB_status = SMB_W_ADDRESS;
	Timer_MLX = 1000;
	I2C_GenerateSTART(I2C_DEVICE, ENABLE);
	return 0;
}

int MLX_Write_Emissivity(unsigned char emi)
{
	unsigned int emi_param;
	if(emi > 100) emi = 100;
	emi_param = 65535 * emi / 100;
	Timer_MLX = 1000;
	while(Timer_MLX && (SMB_status != SMB_IDLE));
	if(Timer_MLX == 0) return -1;
	if(SMB_Write_Word(EE_EMI,TYPE_EE,0x0000) == -1) return -1;
	Delay_nms(100);
	SMB_status = SMB_IDLE;
	if(SMB_Write_Word(EE_EMI,TYPE_EE,emi_param) == -1) return -1;
	Delay_nms(100);
	SMB_status = SMB_IDLE;
	if(SMB_Read_Word(EE_EMI,TYPE_EE) == -1) return -1;
	while(Timer_MLX && (SMB_status != SMB_R_FINISHED));
	if(Timer_MLX == 0) return -1;
	SMB_status = SMB_IDLE;
	if(emi_param == SMB_data_buffer.data)return 0;
	else return -1;
	
}

int Caculate_Temp(void)
{
	int temp;
	
	temp = SMB_data_buffer.data & 0x0fff;
	return temp;
}

//---------------------------------------
//      CALCULATE THE PEC PACKET
//Name: PEC_cal
//Function: Calculate the PEC of received bytes
//Parameters: unsigned char pec[], int n
//Return: pec[0] - This byte contains calculated crc value
//Comments:	Refer to "System Management BUS specification Version 2.0" and " AN "SMBus communication with MLX90614"
//---------------------------------------

unsigned char PEC_cal(unsigned char pec[],int n)

{
     unsigned char crc[6];
     unsigned char Bitposition=47;
     unsigned char shift;
     unsigned char i;
     unsigned char j;
     unsigned char temp;

 do{
          crc[5]=0;           			        //Load CRC value 0x000000000107
          crc[4]=0;
          crc[3]=0;
          crc[2]=0;
          crc[1]=0x01;
          crc[0]=0x07;
          Bitposition=47;     		                //Set maximum bit position at 47
          shift=0;

          //Find first 1 in the transmitted bytes

          i=5;                				//Set highest index (package byte index)
          j=0;                			        //Byte bit index, from lowest
          while((pec[i]&(0x80>>j))==0 && (i>0))
		  
		  {
             Bitposition--;
             if(j<7)
			     {
                    j++;
                 }
             else
			     {
                   j=0x00;
                   i--;
                 }
           }//End of while, and the position of highest "1" bit in Bitposition is calculated 
       
          shift=Bitposition-8;                          //Get shift value for CRC value

		                                        //Shift CRC value left with "shift" bits

          while(shift)
		  {
              for(i=5;i<0xFF;i--)
			     {  
                    if((crc[i-1]&0x80) && (i>0))        //Check if the MSB of the byte lower is "1"
			 {   		                //Yes - current byte + 1
                          temp=1;			//No - current byte + 0
                         }				//So that "1" can shift between bytes
                    else
			 {
                          temp=0;
                         }
                     crc[i]<<=1;
                     crc[i]+=temp;
                  } 

                  shift--;
             } 

           //Exclusive OR between pec and crc

           for(i=0;i<=5;i++)
		 {
                   pec[i]^=crc[i];
			 }  

		}while(Bitposition>8); 

	return pec[0];
}
 
//---------------------------------------

void I2C_interrupt(void)
{
	u32 event;
	static unsigned int bad_event_cnt = 0;
	unsigned char event_targeted;
	event = I2C_GetLastEvent(I2C_DEVICE);
	event_targeted = 0;
	if((event & I2C_EVENT_MASTER_MODE_SELECT) == I2C_EVENT_MASTER_MODE_SELECT)
	{
		if(SMB_status == SMB_W_ADDRESS)
		{
			SMB_status = SMB_W_COMMAND;
			I2C_DEVICE->DR = AM4096_W;
			return;
		}

		if(SMB_status == SMB_R_ADDRESS)
		{
			SMB_status = SMB_WAIT_READ;
			I2C_DEVICE->DR = AM4096_R;
			return;
		}
		bad_event_cnt = 0;
		event_targeted = 1;
		
	}
	if((event & I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) == I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)
	{
		if(SMB_status == SMB_W_COMMAND)
		{
			if(SMB_data_buffer.dir == SLAVE_TO_MASTER)
			{
				SMB_status = SMB_RESTART;
			}
			else
			{
				SMB_status = SMB_W_BYTE_LOW;
			}
			I2C_DEVICE->DR = SMB_data_buffer.command;
			return;	
			
		}
		bad_event_cnt = 0;
		event_targeted = 1;
	}
	if((event & I2C_EVENT_MASTER_BYTE_TRANSMITTED) == I2C_EVENT_MASTER_BYTE_TRANSMITTED)
	{
		if(SMB_status == SMB_RESTART)
		{
			SMB_status = SMB_R_ADDRESS;
			I2C_GenerateSTART(I2C_DEVICE, ENABLE);
			return;
		}

		if(SMB_status == SMB_W_BYTE_LOW)
		{
			SMB_status = SMB_W_BYTE_HIGH;
			(void)(I2C_GetFlagStatus(I2C_DEVICE, I2C_FLAG_TRA));
    		(void)(I2C_GetFlagStatus(I2C_DEVICE, I2C_FLAG_BTF));
			I2C_DEVICE->DR = SMB_data_buffer.data >> 8;
			
			return;	
		}

		if(SMB_status == SMB_W_BYTE_HIGH)
		{
			SMB_status = SMB_W_STOP;
			(void)(I2C_GetFlagStatus(I2C_DEVICE, I2C_FLAG_TRA));
    		(void)(I2C_GetFlagStatus(I2C_DEVICE, I2C_FLAG_BTF));
			I2C_DEVICE->DR = SMB_data_buffer.data & 0xff;
			return;	
		}

		if(SMB_status == SMB_W_PEC)
		{
			SMB_status = SMB_W_STOP;
			(void)(I2C_GetFlagStatus(I2C_DEVICE, I2C_FLAG_TRA));
    		(void)(I2C_GetFlagStatus(I2C_DEVICE, I2C_FLAG_BTF));
			I2C_DEVICE->DR = SMB_data_buffer.pec;
			return;	
		}

		if(SMB_status == SMB_W_STOP)
		{
			SMB_status = SMB_W_FINISHED;
			I2C_GenerateSTOP(I2C_DEVICE, ENABLE);
			return;	
		}
		bad_event_cnt = 0;
		event_targeted = 1;
	}
	if((event & I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED) == I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)
	{
		
		if(SMB_status == SMB_WAIT_READ)
		{
			(void)(I2C_GetFlagStatus(I2C_DEVICE, I2C_FLAG_ADDR));
    		(void)(I2C_GetFlagStatus(I2C_DEVICE, I2C_FLAG_BUSY));
			SMB_status = SMB_R_BYTE_LOW;
			return;
			
		}
		bad_event_cnt = 0;
		event_targeted = 1;
	}
	if((event & I2C_EVENT_MASTER_BYTE_RECEIVED) == I2C_EVENT_MASTER_BYTE_RECEIVED)
	{
		if(SMB_status == SMB_R_BYTE_LOW)
		{
			SMB_data_buffer.data = I2C_DEVICE->DR;
			SMB_status = SMB_R_BYTE_HIGH;
			(void)(I2C_GetFlagStatus(I2C_DEVICE, I2C_FLAG_ADDR));
    		(void)(I2C_GetFlagStatus(I2C_DEVICE, I2C_FLAG_BUSY));
			return;
		}

		if(SMB_status == SMB_R_BYTE_HIGH)
		{
			SMB_data_buffer.data = (I2C_DEVICE->DR) + SMB_data_buffer.data * 256;
			SMB_status = SMB_R_PEC;
			
			I2C_AcknowledgeConfig(I2C_DEVICE, DISABLE);
			
			I2C_GenerateSTOP(I2C_DEVICE, ENABLE);
			(void)(I2C_GetFlagStatus(I2C_DEVICE, I2C_FLAG_ADDR));
    		(void)(I2C_GetFlagStatus(I2C_DEVICE, I2C_FLAG_BUSY));
			
			//I2C_AcknowledgeConfig(I2C_DEVICE, ENABLE);
			
			
			return;
		}
		
		if(SMB_status == SMB_R_PEC)
		{
			SMB_data_buffer.pec= I2C_DEVICE->DR;
			I2C_AcknowledgeConfig(I2C_DEVICE, ENABLE);
			SMB_status = SMB_R_FINISHED;
			
			return;
		}
		bad_event_cnt = 0;
		event_targeted = 1;
	}

	if(event_targeted == 0){	// no target event happened
		bad_event_cnt ++ ;
	}
	
	if(bad_event_cnt > 200){
		printf("bad event cnt:%d\r\n",bad_event_cnt);
		NVIC_GenerateSystemReset();
	}
	//	
}


