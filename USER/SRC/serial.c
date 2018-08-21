#include "serial.h"
#include "main.h"
#include "smbus.h"
#include <stdio.h>
#include "stm32f10x_usart.h"
#include "can_user.h"

unsigned char Serial_Buffer[SERIAL_BUFF_MAX];
unsigned char Serial_Buffer_index = 0;
unsigned char Console_Status = CONSOLE_STAT_MENU;
extern unsigned int serial_cmd_timeout;
extern unsigned char serial_cmd_status;
extern unsigned int PWMCnt;
void Show_Menu(void)
{

	printf("****************MLX90614 Testing****************\n");
	printf("1. Show temperature\n");
	printf("2. Modify emissivity\n");	
}

void Console(void)
{
	unsigned char key;
	switch(Console_Status)
	{
		case CONSOLE_STAT_IDLE:
			return;
		break;
		case CONSOLE_STAT_MENU:
			key = Serial_Buffer[--Serial_Buffer_index];
			USART_SendData(USART1, 0x0c);
			switch(key)
			{
				case '1':
					printf("Press any key return...\n");
					printf("Temperature: \n");
					Console_Status = CONSOLE_STAT_TEMP;
				break;
				case '2':
					printf("Input emissivity within 0~100\n");
					Console_Status = CONSOLE_STAT_EMI;
				break;
			}
			return;
		break;
		case CONSOLE_STAT_EMI:
			return;
		break;
		case CONSOLE_STAT_TEMP:
			return;
		break;
		
	}
}

/*
		Serial Command Format
		0x55 0xaa CMD dat_H dat_L 0xaa 0x55

		CMD: 	a1 -- set can ID
				a2 -- set data offset

*/
void Serial_cmd_parse(void){
	unsigned int dat;
	if(Serial_Buffer[0]!= 0x55 || Serial_Buffer[1]!= 0xaa 
		|| Serial_Buffer[5]!= 0xaa || Serial_Buffer[6]!= 0x55 ){
		printf("bad command,%d\r\n",PWMCnt);
		serial_cmd_status = SERIAL_CMD_IDLE;
		Serial_Buffer_index = 0;
		return;
	}

	switch(Serial_Buffer[2]){
		case 0xa1:	//set can id
			dat = Serial_Buffer[3] ;
			dat <<= 8;
			dat += Serial_Buffer[4];
			Set_Can_id(dat);
			printf("set can id = 0x%x\r\n",dat);
		break;
		case 0xa2:	//set data offset
			dat = Serial_Buffer[3] ;
			dat <<= 8;
			dat += Serial_Buffer[4];
			printf("set data offset = %d\r\n",dat);
		break;
		default:
			printf("bad command,%d\r\n",PWMCnt);
		break;
	}
	
	serial_cmd_status = SERIAL_CMD_IDLE;
	Serial_Buffer_index = 0;
		

}

