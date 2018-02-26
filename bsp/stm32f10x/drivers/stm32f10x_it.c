/**
  ******************************************************************************
  * @file    Project/STM32F10x_StdPeriph_Template/stm32f10x_it.c
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"
#include <board.h>
#include <rtthread.h>
#include "bxcan.h"
#include "sdio_sdcard.h"
#include "RTC.h"

vu32 TimeDisplay = 0;

/** @addtogroup Template_Project
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief   This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
    /* Go to infinite loop when Memory Manage exception occurs */
    while (1)
    {
    }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
    /* Go to infinite loop when Bus Fault exception occurs */
    while (1)
    {
    }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
    /* Go to infinite loop when Usage Fault exception occurs */
    while (1)
    {
    }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

void SDIO_IRQHandler(void)
{
  /* Process All SDIO Interrupt Sources */
  SD_ProcessIRQSrc();
}

void RTC_IRQHandler(void)
{
  if(RTC_GetITStatus(RTC_IT_SEC) != RESET)				 //读取秒中断状态
  {
    RTC_ClearITPendingBit(RTC_IT_SEC);					 //清除秒中断标志
//    GPIO_WriteBit(GPIOB, GPIO_Pin_5, (BitAction)(1-GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_5)));   //LED1闪烁	  

    /* 时钟更新标志置位 */
    TimeDisplay = 1;	  
    RTC_WaitForLastTask();							     //等待上一次对RTC寄存器的写操作是否已经完成    
    if(RTC_GetCounter() == 0x00015180)				     //当前时间是23:59:59时 复位为0:0:0 	    
    {
	  RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);
	  PWR->CR|=1<<8;                  					 //取消备份区写保护
	  RTC_EnterConfigMode();						     //允许配置 	  				
	  RTC_WaitForLastTask();                             //等待上一次对RTC寄存器的写操作是否已经完成 
      RTC_SetCounter(0x0);								 //写入复位值
      RTC_WaitForLastTask();							 //等待上一次对RTC寄存器的写操作是否已经完成    
    }
	else if(RTC_GetCounter() > 0x00015180)				 //当再次上电后计数值超过0x00015180， 复位为当前值取模0x00015180。	    
    {
	  RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);
	  PWR->CR|=1<<8;                                     //取消备份区写保护
	  RTC_EnterConfigMode();			                 //允许配置 
	  RTC_WaitForLastTask();                             //等待上一次对RTC寄存器的写操作是否已经完成    
      RTC_SetCounter(RTC_GetCounter()%0x00015180);		 //写入复位值
      RTC_WaitForLastTask();							 //等待上一次对RTC寄存器的写操作是否已经完成    
    }
  }
}

//void SysTick_Handler(void)
//{
//    // definition in boarc.c
//}

/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

#ifdef  RT_USING_LWIP
/*******************************************************************************
* Function Name  : EXTI4_IRQHandler
* Description    : This function handles External lines 9 to 5 interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void EXTI4_IRQHandler(void)
{
    extern void rt_dm9000_isr(void);

    /* enter interrupt */
    rt_interrupt_enter();

    /* Clear the DM9000A EXTI line pending bit */
    EXTI_ClearITPendingBit(EXTI_Line4);

    rt_dm9000_isr();

    /* leave interrupt */
    rt_interrupt_leave();
}
#endif /* RT_USING_LWIP */

#ifndef STM32F10X_CL
/* CAN and USB IRQ for stm32 none connectivity line devices
 */
void USB_LP_CAN1_RX0_IRQHandler(void)
{
#ifdef RT_USING_CAN
    CAN1_RX0_IRQHandler();
#endif
}
void USB_HP_CAN1_TX_IRQHandler(void)
{
#ifdef RT_USING_CAN
    CAN1_TX_IRQHandler();
#endif
}
#endif


/**
  * @}
  */


/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
