#ifndef __SMBUS_H__
#define __SMBUS_H__

#include "stm32f10x.h"
#define I2C_SLAVE_ADDR		0x3C	//[6:0]
#define I2C_Speed			20000
#define I2C_DEVICE			I2C1

#define FLAG_Mask               ((uint32_t)0x00FFFFFF)

#define SMB_W_ADDRESS 		0x01
#define SMB_W_COMMAND 		0x02
#define SMB_W_BYTE_LOW 		0x03
#define SMB_W_BYTE_HIGH 	0x04
#define SMB_W_PEC 			0x05
#define SMB_R_ADDRESS 		0x06
#define SMB_R_BYTE_LOW 		0x07
#define SMB_R_BYTE_HIGH 	0x08
#define SMB_R_PEC 			0x09
#define SMB_IDLE			0x0a
#define SMB_RESTART			0x0b
#define SMB_WAIT_READ		0x0c
#define SMB_R_FINISHED		0x0d
#define SMB_W_FINISHED		0x0e
#define SMB_ERROR			0x0f
#define SMB_W_STOP			0x10


#define RAM_RAW_IR1			0x04
#define RAM_RAW_IR2			0x05
#define RAM_TMP_AMB			0x06
#define RAM_TMP_IR1			0x07
#define RAM_TMP_IR2			0x08

#define EE_TOMAX			0x00
#define EE_TOMIN			0x01
#define EE_PWM				0x02
#define EE_TA_RANGE			0x03
#define EE_EMI				0x04
#define EE_CTRL				0x05
#define EE_ADDR				0x0e


#define TYPE_EE 			0x20
#define TYPE_RAM			0x00
#define TYPE_FLAG			0xf0
#define TYPE_SLEEP			0xff

#define SLAVE_TO_MASTER		0x01
#define MASTER_TO_SLAVE		0x02

#define MLX90614_W			0xB4
#define MLX90614_R			0xB5

#define AM4096_W			0
#define AM4096_R			(AM4096_W + 1)

#define REG_APOS			33

typedef struct{
	unsigned char command;
	unsigned char dir;
	unsigned int data;
	unsigned char pec;
}SMB_data_strcut;




void SMBus_Init(void);
u8 I2C_MLX_InData(u8 SlaveAddr,u8 Command,uint8_t* pbuff);
ErrorStatus checkflag(unsigned int I2C_EVENT);
void I2C_interrupt(void);
int SMB_Read_Word(unsigned char addr, unsigned char type);
int SMB_Write_Word(unsigned char addr, unsigned char type,unsigned int data);
int Caculate_Temp(void);
unsigned char PEC_cal(unsigned char pec[],int n);
int MLX_Write_Emissivity(unsigned char emi);


#endif

