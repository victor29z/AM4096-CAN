#include "serial.h"
#include "main.h"
#include "smbus.h"
#include <stdio.h>
#include "stm32f10x_usart.h"


unsigned char Serial_Buffer[SERIAL_BUFF_MAX];
unsigned char Serial_Buffer_index = 0;
unsigned char Console_Status = CONSOLE_STAT_MENU;

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
			key = Serial_Buffer[Serial_Buffer_index--];
			USART_SendData(USART1, 0x0c);
			switch(key)
			{
				case 1:
					printf("Press any key return...\n");
					printf("Temperature: \n");
					Console_Status = CONSOLE_STAT_TEMP;
				break;
				case 2:
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

