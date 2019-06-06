#ifndef __MAIN_H__
#define __MAIN_H__
#include "stm32f10x.h"

/* Micro defines
--------------------------------------------*/
//#define USE_INTERNAL_OSC
#define AIRCR_VECTKEY_MASK    ((u32)0x05FA0000)
#define MAIN_CLOCK	72000000

#define TIMER_STATUS_STOP		0x01
#define TIMER_STATUS_RUNNING	0x02
#define TIMER_STATUS_UP			0x03

#define SERIAL_CMD_IDLE			0x01
#define SERIAL_CMD_RECV			0x02
#define SERIAL_CMD_FINISHED		0x03

#define ADC1_DR_Address    	((u32)0x4001244C)


/*Structs
---------------------------------------------*/
typedef struct{
	unsigned char status;
	unsigned int remaining;
}TIMER_STRUCT;


/* Private prototypes
--------------------------------------------*/
static void clock_init(void);
static void port_init(void);
static void hw_init(void);
void NVIC_GenerateSystemReset(void);
static void Check_Flash(void);
static void Systick_init(void);
static void NVIC_init(void);
void Systick_Procedure(void);
void Delay_nms(unsigned int n);
void Serial_Init(void);
u32 get_ssi_value(void);
void ADC_Configuration(void);
void Set_Data_Offset(int offset);
void TIM_PWM_Config(void);
void TIM_ENC_Config(void);
u8 get_key_value(void);




#endif
