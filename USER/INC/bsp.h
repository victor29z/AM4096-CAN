#ifndef __BSP_H__
#define __BSP_H__
#define BOARD_HAND 1
//#define USE_SSI 1
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

#define KEY1_PIN GPIO_Pin_8
#define KEY2_PIN GPIO_Pin_9
#define KEY3_PIN GPIO_Pin_10
#define KEY4_PIN GPIO_Pin_11

#define SSICLK_PORT GPIOA
#define SSIDAT_PORT GPIOA
#define SSICLK_PIN GPIO_Pin_6
#define SSIDAT_PIN GPIO_Pin_7



#define SCL_PORT GPIOB
#define SDA_PORT GPIOB
#define SCL_PIN GPIO_Pin_6
#define SDA_PIN GPIO_Pin_7



#ifdef BOARD_HAND		//board hand


#define PWM1_PORT GPIOA
#define PWM1_PIN  GPIO_Pin_1
#define PWM2_PORT GPIOA
#define PWM2_PIN  GPIO_Pin_2

#define PWM_TIM	TIM2
#define PWM_TIM_CLK	RCC_APB1Periph_TIM2


#endif


#define SERIAL_TX_PORT	GPIOA
#define SERIAL_TX_PIN   GPIO_Pin_9
#define SERIAL_RX_PORT	GPIOA
#define SERIAL_RX_PIN   GPIO_Pin_10
#define SERIAL_NAME		USART1






#define ENCA_PORT GPIOA
#define ENCA_PIN GPIO_Pin_6
#define ENCB_PORT GPIOA
#define ENCB_PIN GPIO_Pin_7
#define ENCZ_PORT GPIOA
#define ENCZ_PIN GPIO_Pin_5

#define ENC_TIM TIM3
#define ENC_TIM_CLK RCC_APB1Periph_TIM3

#define CAN_TX_PORT GPIOA
#define CAN_TX_PIN  GPIO_Pin_12
#define CAN_RX_PORT GPIOA
#define CAN_RX_PIN  GPIO_Pin_11


#define HAND_PWM_CANID	0x78
#define HAND_PWM_LH		1
#define HAND_PWM_RH		2


#define SSI_DATA_LEN	12
#define ENC_CNT_MAX		65535
#define ENC_CNT_PRESCALER	0

/* Private prototypes
--------------------------------------------*/





#endif
