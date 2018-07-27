#ifndef __BSP_H__
#define __BSP_H__

/* Micro defines
--------------------------------------------*/
#define LED0_PORT GPIOA
#define LED1_PORT GPIOD
#define LED0_PIN GPIO_Pin_8
#define LED1_PIN GPIO_Pin_2

#define SEN_ONOFF_PORT	GPIOB
#define SEN_ONOFF_PIN	GPIO_Pin_0

#define KEY1_PORT GPIOB
#define KEY2_PORT GPIOB
#define KEY3_PORT GPIOB
#define KEY4_PORT GPIOB

#define KEY1_PIN GPIO_Pin_11
#define KEY2_PIN GPIO_Pin_10
#define KEY3_PIN GPIO_Pin_1
#define KEY4_PIN GPIO_Pin_0

#define SSICLK_PORT GPIOB
#define SSIDAT_PORT GPIOB
#define SSICLK_PIN GPIO_Pin_8
#define SSIDAT_PIN GPIO_Pin_9



#define SCL_PORT GPIOB
#define SDA_PORT GPIOB
#define SCL_PIN GPIO_Pin_6
#define SDA_PIN GPIO_Pin_7

#define MAG_PORT GPIOB
#define MAG_PIN GPIO_Pin_1

#ifdef BOARD_HAND		//board hand
#define SERIAL_TX_PORT	GPIOA
#define SERIAL_TX_PIN   GPIO_Pin_9
#define SERIAL_RX_PORT	GPIOA
#define SERIAL_RX_PIN   GPIO_Pin_10
#define SERIAL_NAME		USART1


#else	//board wrist
#define SERIAL_TX_PORT	GPIOA
#define SERIAL_TX_PIN   GPIO_Pin_2
#define SERIAL_RX_PORT	GPIOA
#define SERIAL_RX_PIN   GPIO_Pin_3
#define SERIAL_NAME		USART2

#endif

/* Private prototypes
--------------------------------------------*/





#endif
