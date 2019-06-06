#ifndef __CAN_USER_H__
#define __CAN_USER_H__
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

/* Micro defines
--------------------------------------------*/
#define CAN_BCST_ID	0x300
#define DATA_SPACE_BASE	0x800FC00
#define DATA_OFFSET_CANID	0
#define CAN_MSG_UPLOAD	0x01
#define CAN_MSG_SETID	0x02
#define CAN_MSG_SETEMY	0x03


/* Private prototypes
--------------------------------------------*/
void CAN_init(void);
void Set_Can_id(unsigned int id);
void Can_Msg_Process(void);
void Can_Send_Temp(u32 temp, unsigned char mag,unsigned int id);










#endif
