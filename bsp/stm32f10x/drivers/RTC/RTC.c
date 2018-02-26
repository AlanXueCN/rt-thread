#include "RTC.h"

//#define RTCClockSource_LSI   /* 用内置的32K 时钟晶振源 */
#define RTCClockSource_LSE   /* 用外置的32.768K 时钟晶振源 */

//vu32 TimeDisplay = 0;
ErrorStatus HSEStartUpStatus;

/****************************************************************************
* 名    称：void Clock_ini(void)
* 功    能：实时时钟初始化
* 入口参数：无
* 出口参数：无
* 说    明：
* 调用方法：无 
****************************************************************************/ 
void Clock_ini(void){
  if(BKP_ReadBackupRegister(BKP_DR1) != 0xA5A5)		     //判断保存在备份寄存器的RTC标志是否已经被配置过
  {
//     printf("\r\n\n RTC not yet configured....");
     RTC_Configuration();							      //RTC初始化	 
//     printf("\r\n RTC configured....");
//     Time_Adjust();										  //设置RTC 时钟参数
     BKP_WriteBackupRegister(BKP_DR1, 0xA5A5);    	      //RTC设置后，将已配置标志写入备份数据寄存器 
  }
  else
  {	     
     if(RCC_GetFlagStatus(RCC_FLAG_PORRST) != RESET)	  //检查是否掉电重启
     {
//       printf("\r\n\n Power On Reset occurred....");
     }												     
     else if(RCC_GetFlagStatus(RCC_FLAG_PINRST) != RESET) //检查是否reset复位
     {
//       printf("\r\n\n External Reset occurred....");
     }
//     printf("\r\n No need to configure RTC....");  
     RTC_WaitForSynchro();								   //等待RTC寄存器被同步 
     RTC_ITConfig(RTC_IT_SEC, ENABLE);					   //使能秒中断
     RTC_WaitForLastTask();								   //等待写入完成
  }
  RCC_ClearFlag();										   //清除复位标志
}
/*******************************************************************************
* Function Name  : main
* Description    : Main program.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
//int main(void)
//{
//  RCC_Configuration_RTC();								  //系统时钟初始化及外设时钟使能
//  NVIC_Configuration_RTC();								  //中断源配置
//  GPIO_Configuration();								  //LED1控制初始化
//  Usart1_Init();							          //串口1初始化	      
//  Clock_ini();                                        //实时时钟初始化	
//  Time_Show();									      //串口输出时钟
//}

/****************************************************************************
* 名    称：void RCC_Configuration(void)
* 功    能：系统时钟配置为72MHZ， 外设时钟配置
* 入口参数：无
* 出口参数：无
* 说    明：
* 调用方法：无 
****************************************************************************/ 
void RCC_Configuration_RTC(void){
//  SystemInit();
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);			   //复用功能使能
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC 
  						| RCC_APB2Periph_GPIOD| RCC_APB2Periph_GPIOE , ENABLE);
 }

/****************************************************************************
* 名    称：void NVIC_Configuration(void)
* 功    能：中断源配置
* 入口参数：无
* 出口参数：无
* 说    明：
* 调用方法：无 
****************************************************************************/
void NVIC_Configuration_RTC(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;	

  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);	
   
  /* Enable the RTC Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = RTC_IRQn;					//配置外部中断源（秒中断） 
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 7;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);  
}


/****************************************************************************
* 名    称：void GPIO_Configuration(void)
* 功    能：通用IO口配置
* 入口参数：无
* 出口参数：无
* 说    明：
* 调用方法：
****************************************************************************/  
//void GPIO_Configuration(void)
//{
//  GPIO_InitTypeDef GPIO_InitStructure;

//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;				       //LED1闪烁控制
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//  GPIO_Init(GPIOB, &GPIO_InitStructure);	
//  
//}

/****************************************************************************
* 名    称：void RTC_Configuration(void)
* 功    能：RTC初始化函数
* 入口参数：无
* 出口参数：无
* 说    明：
* 调用方法：
****************************************************************************/
void RTC_Configuration(void)
{ 
  /* 使能 PWR 和 BKP 的时钟 */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);
  
  /* 允许访问BKP区域 */
  PWR_BackupAccessCmd(ENABLE);

  /* 复位BKP */
  BKP_DeInit();

#ifdef RTCClockSource_LSI
  /* 使能内部RTC时钟 */ 
  RCC_LSICmd(ENABLE);
  /* 等待RTC内部时钟就绪 */
  while(RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == RESET)
  {
  }
  /* 选择RTC内部时钟为RTC时钟 */
  RCC_RTCCLKConfig(RCC_RTCCLKSource_LSI);  
#elif defined	RTCClockSource_LSE  
  /* 使能RTC外部时钟 */
  RCC_LSEConfig(RCC_LSE_ON);
  /* 等待RTC外部时钟就绪 */
  while(RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET)
  {	    
  }

  /* 选择RTC外部时钟为RTC时钟 */
  RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);  
#endif
  /* 使能RTC时钟 */
  RCC_RTCCLKCmd(ENABLE);


#ifdef RTCClockOutput_Enable  
  /* Disable the Tamper Pin */
  BKP_TamperPinCmd(DISABLE); /* To output RTCCLK/64 on Tamper pin, the tamper
                               functionality must be disabled */
                               
  /* 使能在TAMPER脚输出RTC时钟 */
  BKP_RTCCalibrationClockOutputCmd(ENABLE);
#endif 

  /* 等待RTC寄存器同步 */
  RTC_WaitForSynchro();

  /* 等待写RTC寄存器完成 */
  RTC_WaitForLastTask();
  
  /* 使能RTC秒中断 */  
  RTC_ITConfig(RTC_IT_SEC, ENABLE);

  /* 等待写RTC寄存器完成 */
  RTC_WaitForLastTask();
  
  /* 设置RTC预分频 */
#ifdef RTCClockSource_LSI
  RTC_SetPrescaler(31999);            /* RTC period = RTCCLK/RTC_PR = (32.000 KHz)/(31999+1) */
#elif defined	RTCClockSource_LSE
  RTC_SetPrescaler(32767);            /* RTC period = RTCCLK/RTC_PR = (32.768 KHz)/(32767+1) */
#endif
  
  /* 等待写RTC寄存器完成 */
  RTC_WaitForLastTask();
}

/****************************************************************************
* 名    称：u32 Time_Regulate(void)
* 功    能：时间校正函数
* 入口参数：无
* 出口参数：无
* 说    明：
* 调用方法：
****************************************************************************/
u32 Time_Regulate(void)
{
  u32 Tmp_HH = 0xFF, Tmp_MM = 0xFF, Tmp_SS = 0xFF;

//  printf("\r\n==============Time Settings=====================================");
//  printf("\r\n  Please Set Hours");
  
  while(Tmp_HH == 0xFF)				      
  {
//    Tmp_HH = USART_Scanf(23);	         
  }
//  printf(":  %d", Tmp_HH); 
//  printf("\r\n  Please Set Minutes");
  while(Tmp_MM == 0xFF)
  {
//    Tmp_MM = USART_Scanf(59);
  }
//  printf(":  %d", Tmp_MM); 
//  printf("\r\n  Please Set Seconds");
  while(Tmp_SS == 0xFF)
  {
//    Tmp_SS = USART_Scanf(59);
  }
//  printf(":  %d", Tmp_SS); 

  /* 返回保存在RTC计数寄存器里的值 */
  return((Tmp_HH*3600 + Tmp_MM*60 + Tmp_SS));
}

/****************************************************************************
* 名    称：void Time_Adjust(void)
* 功    能：时间调整函数
* 入口参数：无
* 出口参数：无
* 说    明：
* 调用方法：
****************************************************************************/
void Time_Adjust(void)
{
  /* 等待写RTC寄存器完成 */
  RTC_WaitForLastTask(); 
  /* 改变当前时间 */
  RTC_SetCounter(Time_Regulate());
  /* 等待写RTC寄存器完成 */
  RTC_WaitForLastTask();   
}

/****************************************************************************
* 名    称：void Time_Display(u32 TimeVar)
* 功    能：显示当前时间
* 入口参数：无
* 出口参数：无
* 说    明：
* 调用方法：
****************************************************************************/
void Time_Display(u32 TimeVar)
{ 
  u32 THH = 0, TMM = 0, TSS = 0;

  /* 计算小时 */
  THH = TimeVar/3600;
  /* 计算分钟 */
  TMM = (TimeVar % 3600)/60;
  /* 计算秒 */
  TSS = (TimeVar % 3600)% 60;

//  printf("Time: %0.2d:%0.2d:%0.2d\r",THH, TMM, TSS);
}

/****************************************************************************
* 名    称：void Time_Show(void)
* 功    能：循环显示当前时间
* 入口参数：无
* 出口参数：无
* 说    明：
* 调用方法：
****************************************************************************/
//void Time_Show(void)
//{
//  printf("\n\r");	   
//  while(1)
//  {
//    /* 秒更新发生 */
//    if(TimeDisplay == 1)
//    {    
//      /* 显示当前时间 */
//      Time_Display(RTC_GetCounter());
//      TimeDisplay = 0;
//    }
//  }
//}

/****************************************************************************
* 名    称：void Usart1_Init(void)
* 功    能：串口1初始化函数
* 入口参数：无
* 出口参数：无
* 说    明：
* 调用方法：无 
****************************************************************************/
//void Usart1_Init(void)
//{
//  GPIO_InitTypeDef GPIO_InitStructure;
//  USART_InitTypeDef USART_InitStructure;
// 
//  RCC_APB2PeriphClockCmd( RCC_APB2Periph_USART1 , ENABLE);	 //使能串口1时钟

//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;	         		 //USART1 TX
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;    		 //复用推挽输出
//  GPIO_Init(GPIOA, &GPIO_InitStructure);		    		 //A端口 

//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;	         	 //USART1 RX
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;   	 //复用开漏输入
//  GPIO_Init(GPIOA, &GPIO_InitStructure);		         	 //A端口 

//  USART_InitStructure.USART_BaudRate = 115200;						//速率115200bps
//  USART_InitStructure.USART_WordLength = USART_WordLength_8b;		//数据位8位
//  USART_InitStructure.USART_StopBits = USART_StopBits_1;			//停止位1位
//  USART_InitStructure.USART_Parity = USART_Parity_No;				//无校验位
//  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;   //无硬件流控
//  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;					//收发模式

//  /* Configure USART1 */
//  USART_Init(USART1, &USART_InitStructure);							//配置串口参数函数   

//  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);					//接收中断使能

//   /* Enable the USART1 */
//  USART_Cmd(USART1, ENABLE);	
//  
//}


/****************************************************************************
* 名    称：int fputc(int ch, FILE *f)
* 功    能：printf函数的输出驱动函数
* 入口参数：无
* 出口参数：无
* 说    明：
* 调用方法：无 
****************************************************************************/

//int fputc(int ch, FILE *f)
//{
//  USART_SendData(USART1, (unsigned char) ch);
//  while (!(USART1->SR & USART_FLAG_TXE));
//  return (ch);
//}

/****************************************************************************
* 名    称：u8 USART_Scanf(u32 value)
* 功    能：串口1输入查询函数
* 入口参数：无
* 出口参数：无
* 说    明：
* 调用方法：无 
****************************************************************************/
//u8 USART_Scanf(u32 value)
//{
//  u32 index = 0;
//  u32 tmp[2] = {0, 0};     
//  
//  while(index < 2)
//  {
//    /* Loop until RXNE = 1 */
//    while(USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET)
//    {
//    }
//    tmp[index++] = (USART_ReceiveData(USART1));
//    if((tmp[index - 1] < 0x30) || (tmp[index - 1] > 0x39))
//    {
//      printf("\n\rPlease enter valid number between 0 and 9");
//      index--;
//    }
//  }
//  /* Calculate the Corresponding value */
//  index = (tmp[1] - 0x30) + ((tmp[0] - 0x30) * 10); 
//  /* Checks */
//  if(index > value)
//  {
//    printf("\n\rPlease enter valid number between 0 and %d", value);
//    return 0xFF;
//  }
//  return index;
//}



