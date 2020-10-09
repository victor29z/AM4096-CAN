/* Includes
--------------------------------------------*/
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
#include "stm32f10x_i2c.h"
#include "stm32f10x_adc.h"
#include "stm32f10x_dma.h"
#include "stm32f10x_tim.h"


#include "can_user.h"

#include <stdio.h>
#include "serial.h"

#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)


PUTCHAR_PROTOTYPE
{
/* Place your implementation of fputc here */
/* e.g. write a character to the USART */
USART_SendData(SERIAL_NAME, (u8) ch);

/* Loop until the end of transmission */
while(USART_GetFlagStatus(SERIAL_NAME, USART_FLAG_TC) == RESET)
{
}

return ch;
}



/*Global varibles
---------------------------------------------*/
unsigned char NewPulse = 0;
unsigned char buf[3];
unsigned int temp;
TIMER_STRUCT soft_timer;
int result;
unsigned char MAG_Status;
unsigned int Output_timer = 0;
unsigned char read_request = 0;
unsigned char Receive_data = 0;
unsigned char can_data[8];
unsigned char new_can_data = 0;
unsigned char new_can_rtr = 0;

volatile unsigned int CPU_ID0;
volatile unsigned int CPU_ID1;
volatile unsigned int CPU_ID2;
unsigned int can_id;
unsigned long can_status;
__no_init const unsigned int can_id_mem @ 0x0800FC00;
unsigned char key_result=0;

unsigned int bus_busy_cnt = 0;

extern SMB_data_strcut SMB_data_buffer;

unsigned int serial_cmd_timeout = 0;
unsigned char serial_cmd_status = SERIAL_CMD_IDLE;

volatile u16 ADC1Result[2];
volatile unsigned int PWMCnt = 50;

CanRxMsg can_rx_msg;

const unsigned char hand_lr = HAND_PWM_LH;	//used to specify left or right hand 

u32 ssi_res;	// used to store the encoder result from ssi interface
u32 key_res;		// used to store the key value

/*Extern varibles
---------------------------------------------*/

extern unsigned char SMB_status;
extern volatile unsigned int Timer_MLX; //time out counter
extern unsigned char Serial_Buffer_index;

/*Main function
---------------------------------------------*/
int main(void)
{
/*Local varibles declaration*/

	
	
	
/*System Initial*/
	CPU_ID0 = *(u32*)(0x1FFFF7E8);	 
	CPU_ID1 = *(u32*)(0x1FFFF7EC);   
	CPU_ID2 = *(u32*)(0x1FFFF7F0);
	
	
	hw_init();
	//SMBus_Init();

	printf("joint board, id = 0x%x\r\n",can_id);

	
	Systick_init();
	soft_timer.status = TIMER_STATUS_STOP;
	Delay_nms((can_id % 10) * 275 + (can_id % 100) * 33);// different start time to avoid message collision
	
	ENC_TIM->CNT = 0x8000;//;//wait for hand motor moving to initial place, then set encoder value to 0xff as initial value
	PWMCnt = 8; // set motor torque to a low value to maintain position
	while (1)
	{	
		//MAG_Status = GPIO_ReadInputDataBit(MAG_PORT,MAG_PIN);
		if(read_request == 1){
			read_request = 0;
			
			
#ifdef USE_SSI
			ssi_res = get_ssi_value();
#else
			ssi_res = ENC_TIM->CNT;			
#endif

			key_res = 0;
			key_res = get_key_value();	

#ifdef BOARD_HANDLE

			ssi_res = (ADC1Result[0] & 0x0fff) | ((ADC1Result[1] & 0x0fff) << 12) | (key_res << 24);
#else
			key_res = 0;
			key_res = get_key_value();
			key_res <<= 28;  // use the highest 4 bits of can message to store the key value
			ssi_res |= key_res;
			//printf("ssi: %d, mag: %d\r\n",ssi_res, MAG_Status);

#endif
			Can_Send_Temp(ssi_res,MAG_Status,can_id);
			

		}
		// use rtr to retrieve data
		/*
		if(new_can_rtr){
				Can_Send_Temp(ssi_res,MAG_Status,can_id);
				new_can_rtr = 0;
		}
		*/
		

		if(serial_cmd_status == SERIAL_CMD_FINISHED){
			Serial_cmd_parse();
		}
		if(new_can_data){
			unsigned char can_pwm;
			new_can_data = 0;
			
			if(can_id == HAND_PWM_LH){	// if the hand id matched 
				can_pwm = can_rx_msg.Data[0];
			}
			if(can_id == HAND_PWM_RH){	// if the hand id matched 
				can_pwm = can_rx_msg.Data[1];			}
			if(can_pwm > 99) can_pwm = 99;
				PWMCnt = can_pwm;
		}
		
		
	
	}
}

static void clock_init(void)
{
/////////init pll/////////////////////////////////////////
	RCC_DeInit();
#ifdef USE_INTERNAL_OSC
	RCC_HSICmd(ENABLE);
#else
	RCC_HSEConfig(RCC_HSE_ON);
	RCC_WaitForHSEStartUp();
#endif

	RCC_HCLKConfig(RCC_SYSCLK_Div1);	//8M
	RCC_PCLK2Config(RCC_HCLK_Div1);		//8M
	RCC_PCLK1Config(RCC_HCLK_Div1);		//8M
	RCC_ADCCLKConfig(RCC_PCLK2_Div6);
	FLASH_SetLatency(FLASH_Latency_2);
	FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);
	Check_Flash();
    
#ifdef USE_INTERNAL_OSC
	//internal OSC is 8MHz, HCLK is 8/2*2=8MHz
	RCC_PLLConfig(RCC_PLLSource_HSI_Div2, RCC_PLLMul_2);
#else
#ifdef STM32F10X_CL
    /* Configure PLLs ------------------------------------------------------*/
    /* PLL2 configuration: PLL2CLK = (HSE / 5) * 8 = 40 MHz */
    /* PREDIV1 configuration: PREDIV1CLK = PLL2 / 5 = 8 MHz */
	RCC_PREDIV1Config(RCC_CFGR2_PREDIV1SRC_PLL2, RCC_CFGR2_PREDIV1_DIV5);
	RCC_PREDIV2Config(RCC_CFGR2_PREDIV2_DIV5);
	RCC_PLL2Config(RCC_CFGR2_PLL2MUL8);
  
    RCC_PLL2Cmd(ENABLE);
    /* Wait till PLL2 is ready */
	while(RCC_GetFlagStatus(RCC_FLAG_PLL2RDY) == RESET);
   
	/* PLL configuration: PLLCLK = PREDIV1 * 9 = 72 MHz */
	RCC_PLLConfig(RCC_PLLSource_PREDIV1, RCC_PLLMul_9);
#else
	RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);
#endif
#endif
	RCC_PLLCmd(ENABLE);//PLL is disabled for saving energy

	while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET);
	//RCC_SYSCLKConfig(RCC_SYSCLKSource_HSI);//Use HSI as sysclk for saving energy
	RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
	while (RCC_GetSYSCLKSource()!= 0x08);//Check sysclk source 0x00-HSI, 0x04-HSE, 0x08-PLL

/////////////////////////////////////////////////////////////////////////////
/*Enable used periph clocks*/
	/* APB2 clock enable */
	//RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR , ENABLE);//enable rtc clock
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
    RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOA 
    						| RCC_APB2Periph_GPIOB 
    						| RCC_APB2Periph_GPIOC 
    						| RCC_APB2Periph_GPIOD 
    						//| RCC_APB2Periph_GPIOE 
    						//| RCC_APB2Periph_GPIOF 
    						//| RCC_APB2Periph_GPIOG 
    						| RCC_APB2Periph_AFIO  
    						| RCC_APB2Periph_ADC1  
    						//| RCC_APB2Periph_ADC3  
    						//| RCC_APB2Periph_TIM1  
    						//| RCC_APB2Periph_TIM8  
    						//| RCC_APB2Periph_USART1
    						
                                                ,ENABLE);
	/* APB1 clock enable */
	RCC_APB1PeriphClockCmd(	//RCC_APB1Periph_I2C1
							RCC_APB1Periph_CAN1
							//| RCC_APB1Periph_USART2
							//| RCC_APB1Periph_TIM2 
							//| RCC_APB1Periph_TIM3 
							//| RCC_APB1Periph_TIM4 
							//| RCC_APB1Periph_TIM6 
							//| RCC_APB1Periph_TIM7  
                                                        ,ENABLE);

}

static void port_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	//LED port
	GPIO_InitStructure.GPIO_Pin = LED0_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(LED0_PORT,&GPIO_InitStructure);

	//USART
	GPIO_InitStructure.GPIO_Pin = SERIAL_TX_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(SERIAL_TX_PORT, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = SERIAL_RX_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(SERIAL_RX_PORT, &GPIO_InitStructure);

	
#ifdef USE_SSI	
	//ssi port
	GPIO_InitStructure.GPIO_Pin = SSICLK_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(SSICLK_PORT,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = SSIDAT_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(SSIDAT_PORT,&GPIO_InitStructure);

	
	GPIO_SetBits(SSICLK_PORT,SSICLK_PIN);
#else
	GPIO_InitStructure.GPIO_Pin = ENCA_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(ENCA_PORT,&GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = ENCB_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(ENCB_PORT,&GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = ENCZ_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(ENCZ_PORT,&GPIO_InitStructure);
#endif
#ifdef BOARD_HANDLE
	//KEY PORT
	GPIO_InitStructure.GPIO_Pin = TOPKEY1_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(TOPKEY1_PORT,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = TOPKEY2_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(TOPKEY2_PORT,&GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = TOPKEY3_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(TOPKEY3_PORT,&GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = BUTKEY1_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(BUTKEY1_PORT,&GPIO_InitStructure);
	
	
	GPIO_InitStructure.GPIO_Pin = BUTKEY2_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(BUTKEY2_PORT,&GPIO_InitStructure);

// AD PORT	
	GPIO_InitStructure.GPIO_Pin = OUT_X_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(OUT_X_PORT,&GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = OUT_Y_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(OUT_Y_PORT,&GPIO_InitStructure);
#endif

	//CAN
	GPIO_InitStructure.GPIO_Pin = CAN_RX_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(CAN_RX_PORT, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = CAN_TX_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	//GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(CAN_TX_PORT, &GPIO_InitStructure);
	

//PWM OUT
/*
	GPIO_InitStructure.GPIO_Pin = PWM1_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(PWM1_PORT, &GPIO_InitStructure);

	
	
	GPIO_InitStructure.GPIO_Pin = PWM2_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(PWM2_PORT, &GPIO_InitStructure);

	GPIO_ResetBits(PWM2_PORT,PWM2_PIN);

	
	GPIO_InitStructure.GPIO_Pin = M_DIR_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(M_DIR_PORT, &GPIO_InitStructure);
	GPIO_SetBits(M_DIR_PORT,M_DIR_PIN);

	GPIO_InitStructure.GPIO_Pin = M_DIS_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(M_DIS_PORT, &GPIO_InitStructure);

	
	GPIO_SetBits(M_DIS_PORT,M_DIS_PIN);
*/	
}

static void hw_init(void)
{
	clock_init();
	port_init();
	Serial_Init();
	//Systick_init();
	NVIC_init();
	
	//Check_Flash();
	asm("CPSIE   I");	//enable cpu interrupt
	
	FLASH_Unlock();
	can_id = (*(unsigned int *)(DATA_SPACE_BASE + DATA_OFFSET_CANID))& 0x7ff;
	can_id = can_id_mem & 0x7ff;
	CAN_init();
	FLASH_Lock();
	ADC_Configuration();
	
	TIM_PWM_Config();

#ifdef USE_SSI
#else
	TIM_ENC_Config();
#endif
}

void NVIC_GenerateSystemReset(void)
{
	SCB->AIRCR = AIRCR_VECTKEY_MASK | (u32)0x04;
}


void Check_Flash(void)
{
	FlagStatus status = RESET;  
	status = FLASH_GetReadOutProtectionStatus();
#ifdef NDEBUG
	if(status != SET){
		FLASH_Unlock();  /* Flash 解锁 */ 
		/* ENABLE the ReadOut Protection */  
		FLASH_ReadOutProtection(ENABLE);   //读保护使能 
	//	FLASH_EnableWriteProtection(FLASH_WRProt_AllPages);  //写保护使能 
		NVIC_GenerateSystemReset();
	}
#else
	if(status == SET){
		FLASH_Unlock();  /* Flash 解锁 */ 
		/* ENABLE the ReadOut Protection */  
		FLASH_ReadOutProtection(DISABLE);   //读保护使能 
	//	FLASH_EnableWriteProtection(FLASH_WRProt_AllPages);  //写保护使能 
		NVIC_GenerateSystemReset();
	}
        
#endif
}

void NVIC_init(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	/*
	NVIC_InitStructure.NVIC_IRQChannel = I2C1_EV_IRQn;                     //NVIC配置 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	*/

	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	 // Enable CAN RX0 interrupt IRQ channel  //接收中断
	NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	//发送中断，如果是查询方式发送的话，下面代码不要
/*	NVIC_InitStructure.NVIC_IRQChannel=USB_HP_CAN1_TX_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;//抢先中断优先
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;//响应中断优先
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
*/	

}

void Serial_Init(void)
{

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);

	
	USART_InitTypeDef USART_InitStructure;
	
	USART_InitStructure.USART_BaudRate = 9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	USART_Init(SERIAL_NAME, &USART_InitStructure);
	USART_ITConfig(SERIAL_NAME,USART_IT_RXNE,ENABLE);
	
	USART_ITConfig(SERIAL_NAME, USART_IT_PE, ENABLE);	//开启PE错误接收中断Bit 8PEIE: PE interrupt enable
	
	USART_ITConfig(SERIAL_NAME, USART_IT_ERR, ENABLE);
	USART_Cmd(SERIAL_NAME, ENABLE);
}

void Systick_init(void)
{
	RCC_ClocksTypeDef RCC_Clocks;
	//1ms
	RCC_GetClocksFreq(&RCC_Clocks);
	
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK);
	SysTick_Config(RCC_Clocks.HCLK_Frequency/1000);
}

void Systick_Procedure(void)
{
	static unsigned char led_status = 1;
	static unsigned long int counter;
	
	
	
	if(counter < 500){
		counter++;
	}
	else
	{
		counter = 0;
		if(led_status == 1){
			led_status = 0;
			//GPIO_SetBits(LED1_PORT,LED1_PIN);
			GPIO_ResetBits(LED0_PORT,LED0_PIN);
		}
		else {
			led_status = 1;
			GPIO_SetBits(LED0_PORT,LED0_PIN);
			//GPIO_ResetBits(LED1_PORT,LED1_PIN);
		}
			
	}
	

	if(counter % 30 == 0){
		read_request = 1;
	}

	
	if(serial_cmd_status != SERIAL_CMD_IDLE){
		if(serial_cmd_timeout < 20)
			serial_cmd_timeout++;
		else{
			serial_cmd_timeout = 0;
			serial_cmd_status = SERIAL_CMD_FINISHED;
		}
	}

	
	if(soft_timer.status == TIMER_STATUS_RUNNING){
		if(	soft_timer.remaining>0)
			soft_timer.remaining--;
		else
			soft_timer.status = TIMER_STATUS_UP;
	}

	TIM_SetCompare2(TIM2, PWMCnt);
	
	
	
}

void Delay_nms(unsigned int n)
{
	if(soft_timer.status == TIMER_STATUS_STOP)
	{
		soft_timer.remaining = n;
		soft_timer.status = TIMER_STATUS_RUNNING;
	}
	while(soft_timer.status == TIMER_STATUS_RUNNING);
	soft_timer.status = TIMER_STATUS_STOP;
}

u32 get_ssi_value(void){
	u32 tmp = 0;
	
	unsigned char i;
	volatile int t;
	asm("CPSID   I");	//disable cpu interrupt
	GPIO_SetBits(SSICLK_PORT,SSICLK_PIN);
	//t = GPIO_ReadInputDataBit(SSIDAT_PORT,SSIDAT_PIN);
	GPIO_ResetBits(SSICLK_PORT,SSICLK_PIN);
	//t = GPIO_ReadInputDataBit(SSIDAT_PORT,SSIDAT_PIN);
	for(i = 0; i < SSI_DATA_LEN; i++){

		GPIO_SetBits(SSICLK_PORT,SSICLK_PIN);
		tmp <<= 1;
		//t = GPIO_ReadInputDataBit(SSIDAT_PORT,SSIDAT_PIN);

		GPIO_ResetBits(SSICLK_PORT,SSICLK_PIN);
		if(GPIO_ReadInputDataBit(SSIDAT_PORT,SSIDAT_PIN))
			
			tmp |= 0x00000001;
		else
			tmp &= 0xfffffffe;

	}
	GPIO_SetBits(SSICLK_PORT,SSICLK_PIN);
	while(!GPIO_ReadInputDataBit(SSIDAT_PORT,SSIDAT_PIN));
	asm("CPSIE   I");	//enable cpu interrupt
	//ret = tmp/2;
	return tmp;

}

u8 get_key_value(void){
	u8 result = 0;
	if(!GPIO_ReadInputDataBit(TOPKEY1_PORT,TOPKEY1_PIN))
		result |= 0x01;
	if(!GPIO_ReadInputDataBit(TOPKEY2_PORT,TOPKEY2_PIN))
		result |= 0x02;
	if(!GPIO_ReadInputDataBit(TOPKEY3_PORT,TOPKEY3_PIN))
		result |= 0x04;
	if(!GPIO_ReadInputDataBit(BUTKEY1_PORT,BUTKEY1_PIN))
		result |= 0x08;
	if(!GPIO_ReadInputDataBit(BUTKEY2_PORT,BUTKEY2_PIN))
		result |= 0x10;

	return result;

}

void ADC_Configuration(void)
{
	ADC_InitTypeDef ADC_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;

	/* DMA1 channel1 configuration ----------------------------------------------*/
	DMA_DeInit(DMA1_Channel1);
	
    DMA_InitStructure.DMA_PeripheralBaseAddr = ADC1_DR_Address;
	DMA_InitStructure.DMA_MemoryBaseAddr = (u32 )ADC1Result;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_BufferSize = 2;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel1, &DMA_InitStructure);

	/* Enable DMA1 channel1 */
	DMA_Cmd(DMA1_Channel1, ENABLE);

	/* ADC1 configuration ------------------------------------------------------*/
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfChannel = 2;
	ADC_Init(ADC1, &ADC_InitStructure);
	/* ADC1 regular channel14 configuration */ 
	//ADC_RegularChannelConfig(ADC1, ADC_Channel_16, 1, ADC_SampleTime_55Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 1, ADC_SampleTime_239Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 2, ADC_SampleTime_239Cycles5);
/*	ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 3, ADC_SampleTime_7Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 4, ADC_SampleTime_7Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 5, ADC_SampleTime_7Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_11, 6, ADC_SampleTime_7Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_12, 7, ADC_SampleTime_7Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_13, 8, ADC_SampleTime_7Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_14, 9, ADC_SampleTime_7Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_15, 10, ADC_SampleTime_7Cycles5);
*/	
	/* Enable the temperature sensor and vref internal channel */ 
	//ADC_TempSensorVrefintCmd(ENABLE); 
	/* Enable ADC1 DMA */
	ADC_DMACmd(ADC1, ENABLE);

	/* Enable ADC1 */
	ADC_Cmd(ADC1, ENABLE);
	/* Enable ADC1 reset calibaration register */
	ADC_ResetCalibration(ADC1);
	/* Check the end of ADC1 reset calibration register */
	while(ADC_GetResetCalibrationStatus(ADC1));
	/* Start ADC1 calibaration */
	ADC_StartCalibration(ADC1);
	/* Check the end of ADC1 calibration */
	while(ADC_GetCalibrationStatus(ADC1));

	/* Start ADC1 Software Conversion */ 
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
	
}

void TIM_PWM_Config(void){
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;//定义初始化结构体
	RCC_APB1PeriphClockCmd(PWM_TIM_CLK, ENABLE); //使能定时器2时钟
	//初始化TIM2
	TIM_TimeBaseStructure.TIM_Period = 99; //自动重装载寄存器的值
	TIM_TimeBaseStructure.TIM_Prescaler =35; //TIMX预分频的值  频率为：72*10^6/(99+1)/(143+1)=5000Hz
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //时钟分割
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //向上计数
	TIM_TimeBaseInit(PWM_TIM, &TIM_TimeBaseStructure); //根据以上功能对定时器进行初始化

	TIM_OCInitTypeDef  TIM_OCInitStructure;//定义结构体
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;//选择定时器模式，TIM脉冲宽度调制模式2
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;//比较输出使能
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;//输出比较极性低
	TIM_OC2Init(PWM_TIM, &TIM_OCInitStructure);//根据结构体信息进行初始化
	//TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);  //使能定时器TIM2在CCR2上的预装载值

	TIM_Cmd(PWM_TIM, ENABLE);	//使能定时器TIM2

	TIM_SetCompare2(PWM_TIM, 8);

	
}
void TIM_ENC_Config(void){
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;//定义初始化结构体
	TIM_ICInitTypeDef TIM_ICInitStructure;   
	RCC_APB1PeriphClockCmd(ENC_TIM_CLK, ENABLE); //使能定时器2时钟
	TIM_DeInit(ENC_TIM);
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Period = ENC_CNT_MAX;  //设定计数器重装值	TIMx_ARR = 359*4
	TIM_TimeBaseStructure.TIM_Prescaler = ENC_CNT_PRESCALER; //TIM3时钟预分频值
	TIM_TimeBaseStructure.TIM_ClockDivision =TIM_CKD_DIV1 ;//设置时钟分割 T_dts = T_ck_int	
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Down; //TIM向上计数 
	TIM_TimeBaseInit(ENC_TIM, &TIM_TimeBaseStructure); 			 
				 
	TIM_EncoderInterfaceConfig(ENC_TIM, TIM_EncoderMode_TI12, TIM_ICPolarity_Falling,TIM_ICPolarity_Falling);//使用编码器模式3，上升下降都计数
	TIM_ICStructInit(&TIM_ICInitStructure);//将结构体中的内容缺省输入
	TIM_ICInitStructure.TIM_ICFilter = 6;  //选择输入比较滤波器
	
	
	TIM_ICInit(ENC_TIM, &TIM_ICInitStructure);//将TIM_ICInitStructure中的指定参数初始化TIM3
	
//	TIM_ARRPreloadConfig(TIM4, ENABLE);//使能预装载
	TIM_ClearFlag(ENC_TIM, TIM_FLAG_Update);//清除TIM3的更新标志位
	//TIM_ITConfig(ENC_TIM, TIM_IT_Update, ENABLE);//运行更新中断
	//Reset counter
	ENC_TIM->CNT = 0x8000;//

	TIM_Cmd(ENC_TIM, ENABLE);	 //启动TIM4定时器

}

void Set_Data_Offset(int offset)
{
	

}


