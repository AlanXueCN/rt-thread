#include "RTC.h"

//#define RTCClockSource_LSI   /* �����õ�32K ʱ�Ӿ���Դ */
#define RTCClockSource_LSE   /* �����õ�32.768K ʱ�Ӿ���Դ */

//vu32 TimeDisplay = 0;
ErrorStatus HSEStartUpStatus;

/****************************************************************************
* ��    �ƣ�void Clock_ini(void)
* ��    �ܣ�ʵʱʱ�ӳ�ʼ��
* ��ڲ�������
* ���ڲ�������
* ˵    ����
* ���÷������� 
****************************************************************************/ 
void Clock_ini(void){
  if(BKP_ReadBackupRegister(BKP_DR1) != 0xA5A5)		     //�жϱ����ڱ��ݼĴ�����RTC��־�Ƿ��Ѿ������ù�
  {
//     printf("\r\n\n RTC not yet configured....");
     RTC_Configuration();							      //RTC��ʼ��	 
//     printf("\r\n RTC configured....");
//     Time_Adjust();										  //����RTC ʱ�Ӳ���
     BKP_WriteBackupRegister(BKP_DR1, 0xA5A5);    	      //RTC���ú󣬽������ñ�־д�뱸�����ݼĴ��� 
  }
  else
  {	     
     if(RCC_GetFlagStatus(RCC_FLAG_PORRST) != RESET)	  //����Ƿ��������
     {
//       printf("\r\n\n Power On Reset occurred....");
     }												     
     else if(RCC_GetFlagStatus(RCC_FLAG_PINRST) != RESET) //����Ƿ�reset��λ
     {
//       printf("\r\n\n External Reset occurred....");
     }
//     printf("\r\n No need to configure RTC....");  
     RTC_WaitForSynchro();								   //�ȴ�RTC�Ĵ�����ͬ�� 
     RTC_ITConfig(RTC_IT_SEC, ENABLE);					   //ʹ�����ж�
     RTC_WaitForLastTask();								   //�ȴ�д�����
  }
  RCC_ClearFlag();										   //�����λ��־
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
//  RCC_Configuration_RTC();								  //ϵͳʱ�ӳ�ʼ��������ʱ��ʹ��
//  NVIC_Configuration_RTC();								  //�ж�Դ����
//  GPIO_Configuration();								  //LED1���Ƴ�ʼ��
//  Usart1_Init();							          //����1��ʼ��	      
//  Clock_ini();                                        //ʵʱʱ�ӳ�ʼ��	
//  Time_Show();									      //�������ʱ��
//}

/****************************************************************************
* ��    �ƣ�void RCC_Configuration(void)
* ��    �ܣ�ϵͳʱ������Ϊ72MHZ�� ����ʱ������
* ��ڲ�������
* ���ڲ�������
* ˵    ����
* ���÷������� 
****************************************************************************/ 
void RCC_Configuration_RTC(void){
//  SystemInit();
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);			   //���ù���ʹ��
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC 
  						| RCC_APB2Periph_GPIOD| RCC_APB2Periph_GPIOE , ENABLE);
 }

/****************************************************************************
* ��    �ƣ�void NVIC_Configuration(void)
* ��    �ܣ��ж�Դ����
* ��ڲ�������
* ���ڲ�������
* ˵    ����
* ���÷������� 
****************************************************************************/
void NVIC_Configuration_RTC(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;	

  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);	
   
  /* Enable the RTC Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = RTC_IRQn;					//�����ⲿ�ж�Դ�����жϣ� 
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 7;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);  
}


/****************************************************************************
* ��    �ƣ�void GPIO_Configuration(void)
* ��    �ܣ�ͨ��IO������
* ��ڲ�������
* ���ڲ�������
* ˵    ����
* ���÷�����
****************************************************************************/  
//void GPIO_Configuration(void)
//{
//  GPIO_InitTypeDef GPIO_InitStructure;

//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;				       //LED1��˸����
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//  GPIO_Init(GPIOB, &GPIO_InitStructure);	
//  
//}

/****************************************************************************
* ��    �ƣ�void RTC_Configuration(void)
* ��    �ܣ�RTC��ʼ������
* ��ڲ�������
* ���ڲ�������
* ˵    ����
* ���÷�����
****************************************************************************/
void RTC_Configuration(void)
{ 
  /* ʹ�� PWR �� BKP ��ʱ�� */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);
  
  /* �������BKP���� */
  PWR_BackupAccessCmd(ENABLE);

  /* ��λBKP */
  BKP_DeInit();

#ifdef RTCClockSource_LSI
  /* ʹ���ڲ�RTCʱ�� */ 
  RCC_LSICmd(ENABLE);
  /* �ȴ�RTC�ڲ�ʱ�Ӿ��� */
  while(RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == RESET)
  {
  }
  /* ѡ��RTC�ڲ�ʱ��ΪRTCʱ�� */
  RCC_RTCCLKConfig(RCC_RTCCLKSource_LSI);  
#elif defined	RTCClockSource_LSE  
  /* ʹ��RTC�ⲿʱ�� */
  RCC_LSEConfig(RCC_LSE_ON);
  /* �ȴ�RTC�ⲿʱ�Ӿ��� */
  while(RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET)
  {	    
  }

  /* ѡ��RTC�ⲿʱ��ΪRTCʱ�� */
  RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);  
#endif
  /* ʹ��RTCʱ�� */
  RCC_RTCCLKCmd(ENABLE);


#ifdef RTCClockOutput_Enable  
  /* Disable the Tamper Pin */
  BKP_TamperPinCmd(DISABLE); /* To output RTCCLK/64 on Tamper pin, the tamper
                               functionality must be disabled */
                               
  /* ʹ����TAMPER�����RTCʱ�� */
  BKP_RTCCalibrationClockOutputCmd(ENABLE);
#endif 

  /* �ȴ�RTC�Ĵ���ͬ�� */
  RTC_WaitForSynchro();

  /* �ȴ�дRTC�Ĵ������ */
  RTC_WaitForLastTask();
  
  /* ʹ��RTC���ж� */  
  RTC_ITConfig(RTC_IT_SEC, ENABLE);

  /* �ȴ�дRTC�Ĵ������ */
  RTC_WaitForLastTask();
  
  /* ����RTCԤ��Ƶ */
#ifdef RTCClockSource_LSI
  RTC_SetPrescaler(31999);            /* RTC period = RTCCLK/RTC_PR = (32.000 KHz)/(31999+1) */
#elif defined	RTCClockSource_LSE
  RTC_SetPrescaler(32767);            /* RTC period = RTCCLK/RTC_PR = (32.768 KHz)/(32767+1) */
#endif
  
  /* �ȴ�дRTC�Ĵ������ */
  RTC_WaitForLastTask();
}

/****************************************************************************
* ��    �ƣ�u32 Time_Regulate(void)
* ��    �ܣ�ʱ��У������
* ��ڲ�������
* ���ڲ�������
* ˵    ����
* ���÷�����
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

  /* ���ر�����RTC�����Ĵ������ֵ */
  return((Tmp_HH*3600 + Tmp_MM*60 + Tmp_SS));
}

/****************************************************************************
* ��    �ƣ�void Time_Adjust(void)
* ��    �ܣ�ʱ���������
* ��ڲ�������
* ���ڲ�������
* ˵    ����
* ���÷�����
****************************************************************************/
void Time_Adjust(void)
{
  /* �ȴ�дRTC�Ĵ������ */
  RTC_WaitForLastTask(); 
  /* �ı䵱ǰʱ�� */
  RTC_SetCounter(Time_Regulate());
  /* �ȴ�дRTC�Ĵ������ */
  RTC_WaitForLastTask();   
}

/****************************************************************************
* ��    �ƣ�void Time_Display(u32 TimeVar)
* ��    �ܣ���ʾ��ǰʱ��
* ��ڲ�������
* ���ڲ�������
* ˵    ����
* ���÷�����
****************************************************************************/
void Time_Display(u32 TimeVar)
{ 
  u32 THH = 0, TMM = 0, TSS = 0;

  /* ����Сʱ */
  THH = TimeVar/3600;
  /* ������� */
  TMM = (TimeVar % 3600)/60;
  /* ������ */
  TSS = (TimeVar % 3600)% 60;

//  printf("Time: %0.2d:%0.2d:%0.2d\r",THH, TMM, TSS);
}

/****************************************************************************
* ��    �ƣ�void Time_Show(void)
* ��    �ܣ�ѭ����ʾ��ǰʱ��
* ��ڲ�������
* ���ڲ�������
* ˵    ����
* ���÷�����
****************************************************************************/
//void Time_Show(void)
//{
//  printf("\n\r");	   
//  while(1)
//  {
//    /* ����·��� */
//    if(TimeDisplay == 1)
//    {    
//      /* ��ʾ��ǰʱ�� */
//      Time_Display(RTC_GetCounter());
//      TimeDisplay = 0;
//    }
//  }
//}

/****************************************************************************
* ��    �ƣ�void Usart1_Init(void)
* ��    �ܣ�����1��ʼ������
* ��ڲ�������
* ���ڲ�������
* ˵    ����
* ���÷������� 
****************************************************************************/
//void Usart1_Init(void)
//{
//  GPIO_InitTypeDef GPIO_InitStructure;
//  USART_InitTypeDef USART_InitStructure;
// 
//  RCC_APB2PeriphClockCmd( RCC_APB2Periph_USART1 , ENABLE);	 //ʹ�ܴ���1ʱ��

//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;	         		 //USART1 TX
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;    		 //�����������
//  GPIO_Init(GPIOA, &GPIO_InitStructure);		    		 //A�˿� 

//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;	         	 //USART1 RX
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;   	 //���ÿ�©����
//  GPIO_Init(GPIOA, &GPIO_InitStructure);		         	 //A�˿� 

//  USART_InitStructure.USART_BaudRate = 115200;						//����115200bps
//  USART_InitStructure.USART_WordLength = USART_WordLength_8b;		//����λ8λ
//  USART_InitStructure.USART_StopBits = USART_StopBits_1;			//ֹͣλ1λ
//  USART_InitStructure.USART_Parity = USART_Parity_No;				//��У��λ
//  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;   //��Ӳ������
//  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;					//�շ�ģʽ

//  /* Configure USART1 */
//  USART_Init(USART1, &USART_InitStructure);							//���ô��ڲ�������   

//  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);					//�����ж�ʹ��

//   /* Enable the USART1 */
//  USART_Cmd(USART1, ENABLE);	
//  
//}


/****************************************************************************
* ��    �ƣ�int fputc(int ch, FILE *f)
* ��    �ܣ�printf�����������������
* ��ڲ�������
* ���ڲ�������
* ˵    ����
* ���÷������� 
****************************************************************************/

//int fputc(int ch, FILE *f)
//{
//  USART_SendData(USART1, (unsigned char) ch);
//  while (!(USART1->SR & USART_FLAG_TXE));
//  return (ch);
//}

/****************************************************************************
* ��    �ƣ�u8 USART_Scanf(u32 value)
* ��    �ܣ�����1�����ѯ����
* ��ڲ�������
* ���ڲ�������
* ˵    ����
* ���÷������� 
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



