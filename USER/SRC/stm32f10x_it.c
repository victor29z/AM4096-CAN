/******************** (C) COPYRIGHT 2011 STMicroelectronics ********************
* File Name          : stm32f10x_it.c
* Author             : MCD Application Team
* Version            : V3.3.0
* Date               : 21-March-2011
* Description        : Main Interrupt Service Routines.
*                      This file provides template for all exceptions handler
*                      and peripherals interrupt service routine.
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_can.h"

#include "main.h"
#include "smbus.h"
#include "serial.h"
#include <string.h>




/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
volatile unsigned int Timer_MLX; //time out counter

/* Extern varibles------------------------------------------------------------*/
extern unsigned char NewPulse;
extern TIMER_STRUCT soft_timer;
extern unsigned char Serial_Buffer[SERIAL_BUFF_MAX];
extern unsigned char Serial_Buffer_index;
extern unsigned char read_request;
extern unsigned char SMB_status;
extern unsigned char Receive_data;
extern unsigned char can_data[8];
extern unsigned char new_can_data;
extern unsigned int serial_cmd_timeout;
extern unsigned char serial_cmd_status;
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/*******************************************************************************
* Function Name  : NMI_Handler
* Description    : This function handles NMI exception.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void NMI_Handler(void)
{
}

/*******************************************************************************
* Function Name  : HardFault_Handler
* Description    : This function handles Hard Fault exception.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/*******************************************************************************
* Function Name  : MemManage_Handler
* Description    : This function handles Memory Manage exception.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/*******************************************************************************
* Function Name  : BusFault_Handler
* Description    : This function handles Bus Fault exception.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/*******************************************************************************
* Function Name  : UsageFault_Handler
* Description    : This function handles Usage Fault exception.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/*******************************************************************************
* Function Name  : SVC_Handler
* Description    : This function handles SVCall exception.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SVC_Handler(void)
{
}

/*******************************************************************************
* Function Name  : DebugMon_Handler
* Description    : This function handles Debug Monitor exception.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void DebugMon_Handler(void)
{
}

/*******************************************************************************
* Function Name  : PendSV_Handler
* Description    : This function handles PendSV_Handler exception.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void PendSV_Handler(void)
{
}

/*******************************************************************************
* Function Name  : SysTick_Handler
* Description    : This function handles SysTick Handler.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SysTick_Handler(void)
{
	Systick_Procedure();
			
}

void I2C1_EV_IRQHandler(void)
{
	I2C_interrupt();
}

/*******************************************************************************
* Function Name  : USART2_IRQHandler
* Description    : This function handles USART1 interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/

void USART1_IRQHandler(void)
{
	
	//开启CR3,bit0的EIE: Error interrupt enable, 处理USART_IT_ERR,USART_IT_ORE_ER,USART_IT_NE,USART_IT_FE	错误
	if(USART_GetFlagStatus(USART1, USART_FLAG_ORE) != RESET)
	{//同  @arg USART_IT_ORE_ER : OverRun Error interrupt if the EIE bit is set  
	char ushTemp = USART_ReceiveData(USART1); //取出来扔掉
	USART_ClearFlag(USART1, USART_FLAG_ORE);
	}
	
	if(USART_GetFlagStatus(USART1, USART_FLAG_NE) != RESET)
	{//同  @arg USART_IT_NE 	: Noise Error interrupt
	USART_ClearFlag(USART1, USART_FLAG_NE);
	}
	
	
	if(USART_GetFlagStatus(USART1, USART_FLAG_FE) != RESET)
	{//同	@arg USART_IT_FE	 : Framing Error interrupt
	USART_ClearFlag(USART1, USART_FLAG_FE);
	}
	
	if(USART_GetFlagStatus(USART1, USART_FLAG_PE) != RESET)
	{//同  @arg USART_IT_PE 	: Parity Error interrupt
	USART_ClearFlag(USART1, USART_FLAG_PE);
	}

    u8 data_receive;
	data_receive=USART_ReceiveData(USART1);
	if(Serial_Buffer_index < (SERIAL_BUFF_MAX - 1)) Serial_Buffer[Serial_Buffer_index++] = data_receive;
	serial_cmd_status = SERIAL_CMD_RECV;
	
	//USART_SendData(USART1,data_receive);
	//if(SMB_status == SMB_IDLE)read_request = 1;    
}

void USART2_IRQHandler(void)
{
	
	//开启CR3,bit0的EIE: Error interrupt enable, 处理USART_IT_ERR,USART_IT_ORE_ER,USART_IT_NE,USART_IT_FE	错误
	if(USART_GetFlagStatus(USART2, USART_FLAG_ORE) != RESET)
	{//同  @arg USART_IT_ORE_ER : OverRun Error interrupt if the EIE bit is set  
	char ushTemp = USART_ReceiveData(USART2); //取出来扔掉
	USART_ClearFlag(USART2, USART_FLAG_ORE);
	}
	
	if(USART_GetFlagStatus(USART2, USART_FLAG_NE) != RESET)
	{//同  @arg USART_IT_NE 	: Noise Error interrupt
	USART_ClearFlag(USART2, USART_FLAG_NE);
	}
	
	
	if(USART_GetFlagStatus(USART2, USART_FLAG_FE) != RESET)
	{//同	@arg USART_IT_FE	 : Framing Error interrupt
	USART_ClearFlag(USART2, USART_FLAG_FE);
	}
	
	if(USART_GetFlagStatus(USART2, USART_FLAG_PE) != RESET)
	{//同  @arg USART_IT_PE 	: Parity Error interrupt
	USART_ClearFlag(USART2, USART_FLAG_PE);
	}

    u8 data_receive;
	data_receive=USART_ReceiveData(USART2);
	if(Serial_Buffer_index < (SERIAL_BUFF_MAX - 1)) Serial_Buffer[Serial_Buffer_index++] = data_receive;
	serial_cmd_status = SERIAL_CMD_RECV;
	
	//USART_SendData(USART2,data_receive);
	//if(SMB_status == SMB_IDLE)read_request = 1;    
}


void  USB_LP_CAN1_RX0_IRQHandler(void)
{
	CanRxMsg  can_rx_one_frame; //定义接收数据变量
	CAN_Receive(CAN1,CAN_FIFO0,&can_rx_one_frame);//接收数据函数
	Receive_data = can_rx_one_frame.Data[0];//接收到的数据转存给变量Receive_data
	memcpy(can_data, can_rx_one_frame.Data, can_rx_one_frame.DLC);
	new_can_data = 1;
	CAN_FIFORelease(CAN1,CAN_FIFO0);// 释放一 FIFO0
}



/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
