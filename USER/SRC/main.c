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

#include "can_user.h"

#include <stdio.h>
#include "serial.h"

#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)


PUTCHAR_PROTOTYPE
{
/* Place your implementation of fputc here */
/* e.g. write a character to the USART */
USART_SendData(USART1, (u8) ch);

/* Loop until the end of transmission */
while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
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
volatile unsigned int CPU_ID0;
volatile unsigned int CPU_ID1;
volatile unsigned int CPU_ID2;
unsigned int can_id;
unsigned long can_status;
//__no_init const unsigned int can_id_mem @ 0x0800FC00;
unsigned char key_result=0;

unsigned int bus_busy_cnt = 0;

extern SMB_data_strcut SMB_data_buffer;

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
	//CanTxMsg TxMessage; //�������ݽṹ���ͱ���
	//unsigned char TransmitMailbox = 0;
	//unsigned int i = 0;
	
	
/*System Initial*/
	CPU_ID0 = *(u32*)(0x1FFFF7E8);	 
	CPU_ID1 = *(u32*)(0x1FFFF7EC);   
	CPU_ID2 = *(u32*)(0x1FFFF7F0);
	Delay_nms(CPU_ID0 & 0x1ff);
	can_id = CPU_ID0 & 0x7ff;
	hw_init();
	SMBus_Init();

	printf("AM4096-can board, id = 0x%x\r\n",can_id);

	
	
	//GPIO_SetBits(LED1_PORT,LED1_PIN);
	//GPIO_ResetBits(LED0_PORT,LED0_PIN);
	//GPIO_SetBits(SEN_ONOFF_PORT,SEN_ONOFF_PIN);

	//Delay_nms(CPU_ID0 & 0x1ff);

	/*Main loop*/
	unsigned int ssi_res;
	
	SMB_Write_Word(50,TYPE_RAM,0x0080);
	while(SMB_status != SMB_W_FINISHED);
	SMB_status = SMB_IDLE;
	Systick_init();
	while (1)
	{	
		MAG_Status = GPIO_ReadInputDataBit(MAG_PORT,MAG_PIN);
		if(read_request == 1 && SMB_status == SMB_IDLE){
			read_request = 0;
			//ssi_res = get_ssi_value();
			//printf(" ssi: %d\r\n",ssi_res);
			SMB_Read_Word(REG_APOS,TYPE_RAM);
			//SMB_Read_Word(50,TYPE_RAM);
			

		}
		if(SMB_status == SMB_R_FINISHED)
		{

			
			SMB_status = SMB_IDLE;
			result = Caculate_Temp();
			
			printf("ssi: %d, cnt: %d, mag: %d\r\n",result,bus_busy_cnt, MAG_Status);
			//printf("reg: 0x%x\r\n",SMB_data_buffer.data);
			bus_busy_cnt = 0;
			//not a default id then send temp
			can_status = CAN1->ESR;
			
			
			Can_Send_Temp(result,MAG_Status,can_id);
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
	//RCC_PLLCmd(ENABLE);//PLL is disabled for saving energy

	//while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET);
	RCC_SYSCLKConfig(RCC_SYSCLKSource_HSI);//Use HSI as sysclk for saving energy
    while (RCC_GetSYSCLKSource()!= 0x00);//Check sysclk source 0x00-HSI, 0x04-HSE, 0x08-PLL

/////////////////////////////////////////////////////////////////////////////
/*Enable used periph clocks*/
	/* APB2 clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR , ENABLE);//enable rtc clock
    //RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
    RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOA 
    						| RCC_APB2Periph_GPIOB 
    						| RCC_APB2Periph_GPIOC 
    						| RCC_APB2Periph_GPIOD 
    						//| RCC_APB2Periph_GPIOE 
    						//| RCC_APB2Periph_GPIOF 
    						//| RCC_APB2Periph_GPIOG 
    						| RCC_APB2Periph_AFIO  
    						//| RCC_APB2Periph_ADC1  
    						//| RCC_APB2Periph_ADC3  
    						//| RCC_APB2Periph_TIM1  
    						//| RCC_APB2Periph_TIM8  
    						| RCC_APB2Periph_USART1
                                                ,ENABLE);
	/* APB1 clock enable */
	RCC_APB1PeriphClockCmd(	RCC_APB1Periph_I2C1
							| RCC_APB1Periph_CAN1
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
	//GPIO_Init(LED0_PORT,&GPIO_InitStructure);

	//USART
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	//KEY PORT
/*
	GPIO_InitStructure.GPIO_Pin = KEY1_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(KEY1_PORT,&GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = KEY2_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(KEY2_PORT,&GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = KEY3_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(KEY3_PORT,&GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = KEY4_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(KEY4_PORT,&GPIO_InitStructure);
	*/
	//ssi port
	GPIO_InitStructure.GPIO_Pin = SSICLK_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(SSICLK_PORT,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = SSIDAT_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(SSIDAT_PORT,&GPIO_InitStructure);

	
	GPIO_SetBits(SSICLK_PORT,SSICLK_PIN);

	
	//I2C port
	GPIO_InitStructure.GPIO_Pin = SCL_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
	GPIO_Init(SCL_PORT, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = SDA_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
	GPIO_Init(SDA_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = MAG_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(MAG_PORT,&GPIO_InitStructure);

	//CAN
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}

static void hw_init(void)
{
	clock_init();
	port_init();
	Serial_Init();
	//Systick_init();
	NVIC_init();
	CAN_init();
	//Check_Flash();
	asm("CPSIE   I");	//enable cpu interrupt
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
		FLASH_Unlock();  /* Flash ���� */ 
		/* ENABLE the ReadOut Protection */  
		FLASH_ReadOutProtection(ENABLE);   //������ʹ�� 
	//	FLASH_EnableWriteProtection(FLASH_WRProt_AllPages);  //д����ʹ�� 
		NVIC_GenerateSystemReset();
	}
#else
	if(status == SET){
		FLASH_Unlock();  /* Flash ���� */ 
		/* ENABLE the ReadOut Protection */  
		FLASH_ReadOutProtection(DISABLE);   //������ʹ�� 
	//	FLASH_EnableWriteProtection(FLASH_WRProt_AllPages);  //д����ʹ�� 
		NVIC_GenerateSystemReset();
	}
        
#endif
}

void NVIC_init(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	
	NVIC_InitStructure.NVIC_IRQChannel = I2C1_EV_IRQn;                     //NVIC���� 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	/**/
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	 // Enable CAN RX0 interrupt IRQ channel  //�����ж�
	NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	//�����жϣ�����ǲ�ѯ��ʽ���͵Ļ���������벻Ҫ
/*	NVIC_InitStructure.NVIC_IRQChannel=USB_HP_CAN1_TX_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;//�����ж�����
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;//��Ӧ�ж�����
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
*/	

}

void Serial_Init(void)
{
	USART_InitTypeDef USART_InitStructure;
	
	USART_InitStructure.USART_BaudRate = 9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	USART_Init(USART1, &USART_InitStructure);
	USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);
	
	USART_ITConfig(USART1, USART_IT_PE, ENABLE);	//����PE��������ж�Bit 8PEIE: PE interrupt enable
	
	USART_ITConfig(USART1, USART_IT_ERR, ENABLE);
	USART_Cmd(USART1, ENABLE);
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
	static unsigned int counter;

	
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
	

	if(counter % 10 == 0){
		read_request = 1;
	}

	if(SMB_status != SMB_IDLE){
			bus_busy_cnt++;
	}

	if(bus_busy_cnt > 10){
		bus_busy_cnt = 0;
		//NVIC_GenerateSystemReset();
	}
	if(soft_timer.status == TIMER_STATUS_RUNNING){
		if(	soft_timer.remaining>0)
			soft_timer.remaining--;
		else
			soft_timer.status = TIMER_STATUS_UP;
	}
	
	
	
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

unsigned int get_ssi_value(void){
	unsigned int tmp = 0;
	unsigned char i;
	volatile int t;
	asm("CPSID   I");	//disable cpu interrupt
	GPIO_SetBits(SSICLK_PORT,SSICLK_PIN);
	//t = GPIO_ReadInputDataBit(SSIDAT_PORT,SSIDAT_PIN);
	GPIO_ResetBits(SSICLK_PORT,SSICLK_PIN);
	//t = GPIO_ReadInputDataBit(SSIDAT_PORT,SSIDAT_PIN);
	for(i = 0; i < 12; i++){

		GPIO_SetBits(SSICLK_PORT,SSICLK_PIN);
		tmp <<= 1;
		//t = GPIO_ReadInputDataBit(SSIDAT_PORT,SSIDAT_PIN);

		GPIO_ResetBits(SSICLK_PORT,SSICLK_PIN);
		if(GPIO_ReadInputDataBit(SSIDAT_PORT,SSIDAT_PIN))
			
			tmp |= 0x0001;
		else
			tmp &= 0xfffe;

	}
	GPIO_SetBits(SSICLK_PORT,SSICLK_PIN);
	while(!GPIO_ReadInputDataBit(SSIDAT_PORT,SSIDAT_PIN));
	asm("CPSIE   I");	//enable cpu interrupt
	return tmp;

}

