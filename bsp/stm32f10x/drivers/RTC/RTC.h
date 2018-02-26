/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "stm32f10x_bkp.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_rtc.h" 
#include "stm32f10x_pwr.h"
#include "misc.h"
#include <stdio.h>


void RCC_Configuration_RTC(void);
//void GPIO_Configuration(void);
//void Usart1_Init(void);
void RTC_Configuration(void);
void NVIC_Configuration_RTC(void);
u32 Time_Regulate(void);
void Time_Adjust(void);
void Time_Show(void);
void Time_Display(u32 TimeVar);
//u8 USART_Scanf(u32 value);
void Clock_ini(void);






