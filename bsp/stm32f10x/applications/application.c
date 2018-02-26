/*
 * File      : application.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2006, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2009-01-05     Bernard      the first version
 * 2013-07-12     aozima       update for auto initial.
 */

/**
 * @addtogroup STM32
 */
/*@{*/

#include <board.h>
#include <rtthread.h>

#ifdef RT_USING_DFS
/* dfs filesystem:ELM filesystem init */
#include <dfs_elm.h>
/* dfs Filesystem APIs */
#include <dfs_fs.h>
#endif

#ifdef RT_USING_RTGUI
#include <rtgui/rtgui.h>
#include <rtgui/rtgui_server.h>
#include <rtgui/rtgui_system.h>
#include <rtgui/driver.h>
#include <rtgui/calibration.h>
#endif

#include "led.h"
#include "sdio_sdcard.h"
#include "integer.h"
#include "ff.h"
#include "ffconf.h"
#include "diskio.h"
#include "demo.h"
#include "RTC.h"

void SD_TEST(void);
void NVIC_Configuration_SD(void);
void OutPutFile(void);
void OutPutFile2(void);
unsigned int FindStr(char*str1,char*str2);
int f(char*s1,char*s2);
char* GetStr(char* str1,char*str2);

#define _DF1S	0x81

SD_CardInfo SDCardInfo;
uint32_t Buffer_Block_Tx[512]={3,6,9}, Buffer_Block_Rx[512]; 
unsigned char* write_buf = "mzc";
SD_Error Status = SD_OK;
FATFS fs;                      // �߼��������ı�־
FIL fsrc, fdst;                // �ļ���־ 
unsigned char buffer[512];     // �ļ����ݻ�����
unsigned char* buffers;     // �ļ����ݻ�����
FRESULT res;                   // FatFs ���ܺ������ؽ������
unsigned int br, bw;           // �ļ���/д������

ALIGN(RT_ALIGN_SIZE)
static rt_uint8_t sd_stack[ 512 ];
static struct rt_thread sd_thread;
static void sd_thread_entry(void* parameter)
{
  NVIC_Configuration_SD();
	FindStr("helloword123","word");
	f("helloword123","word");
	SD_TEST();
//	OutPutFile();//���Դ�SD��������
	OutPutFile2();//������SD��д����
	while (1)
	{
		rt_hw_led_on(1);
		rt_thread_delay( RT_TICK_PER_SECOND/2 );
		rt_hw_led_off(1);
		rt_thread_delay( RT_TICK_PER_SECOND/2 );
	}
}

extern vu32 TimeDisplay;
ALIGN(RT_ALIGN_SIZE)
static rt_uint8_t RTC_stack[ 512 ];
static struct rt_thread RTC_thread;
static void RTC_thread_entry(void* parameter)
{
	NVIC_Configuration_RTC();								  //�ж�Դ����
	RCC_Configuration_RTC();								  //ϵͳʱ�ӳ�ʼ��������ʱ��ʹ��
	Clock_ini();                                        //ʵʱʱ�ӳ�ʼ��
	while(1)
  {
    /* ����·��� */
    if(TimeDisplay == 1)
    {    
      /* ��ʾ��ǰʱ�� */
      Time_Display(RTC_GetCounter());
      TimeDisplay = 0;
    }
  }
}

ALIGN(RT_ALIGN_SIZE)
static rt_uint8_t led_stack[ 512 ];
static struct rt_thread led_thread;
static void led_thread_entry(void* parameter)
{
    unsigned int count=0;

    rt_hw_led_init();

    while (1)
    {
        /* led1 on */
#ifndef RT_USING_FINSH
        rt_kprintf("led on, count : %d\r\n",count);
#endif
        count++;
        rt_hw_led_on(0);
        rt_thread_delay( RT_TICK_PER_SECOND/2 ); /* sleep 0.5 second and switch to other thread */

        /* led1 off */
#ifndef RT_USING_FINSH
        rt_kprintf("led off\r\n");
#endif
        rt_hw_led_off(0);
        rt_thread_delay( RT_TICK_PER_SECOND/2 );
    }
}

#ifdef RT_USING_RTGUI
rt_bool_t cali_setup(void)
{
    rt_kprintf("cali setup entered\n");
    return RT_FALSE;
}

void cali_store(struct calibration_data *data)
{
    rt_kprintf("cali finished (%d, %d), (%d, %d)\n",
               data->min_x,
               data->max_x,
               data->min_y,
               data->max_y);
}
#endif /* RT_USING_RTGUI */

void rt_init_thread_entry(void* parameter)
{
#ifdef RT_USING_COMPONENTS_INIT
    /* initialization RT-Thread Components */
    rt_components_init();
#endif

    /* Filesystem Initialization */
#if defined(RT_USING_DFS) && defined(RT_USING_DFS_ELMFAT)
    /* mount sd card fat partition 1 as root directory */
    if (dfs_mount("sd0", "/", "elm", 0, 0) == 0)
    {
        rt_kprintf("File System initialized!\n");
    }
    else
        rt_kprintf("File System initialzation failed!\n");
#endif  /* RT_USING_DFS */

#ifdef RT_USING_RTGUI
    {
        extern void rt_hw_lcd_init();
        extern void rtgui_touch_hw_init(void);

        rt_device_t lcd;

        /* init lcd */
        rt_hw_lcd_init();

        /* init touch panel */
        rtgui_touch_hw_init();

        /* find lcd device */
        lcd = rt_device_find("lcd");

        /* set lcd device as rtgui graphic driver */
        rtgui_graphic_set_device(lcd);

#ifndef RT_USING_COMPONENTS_INIT
        /* init rtgui system server */
        rtgui_system_server_init();
#endif

        calibration_set_restore(cali_setup);
        calibration_set_after(cali_store);
        calibration_init();
    }
#endif /* #ifdef RT_USING_RTGUI */
}

int rt_application_init(void)
{
    rt_thread_t init_thread;

    rt_err_t result;

    /* init led thread */
    result = rt_thread_init(&led_thread,
                            "led",
                            led_thread_entry,
                            RT_NULL,
                            (rt_uint8_t*)&led_stack[0],
                            sizeof(led_stack),
                            20,
                            5);
    if (result == RT_EOK)
    {
        rt_thread_startup(&led_thread);
    }
		
		/* init sd thread */
    result = rt_thread_init(&sd_thread,
                            "sd",
                            sd_thread_entry,
                            RT_NULL,
                            (rt_uint8_t*)&sd_stack[0],
                            sizeof(sd_stack),
                            20,
                            5);
    if (result == RT_EOK)
    {
        rt_thread_startup(&sd_thread);
    }
		
		/* init rtc thread */
    result = rt_thread_init(&RTC_thread,
                            "RTC",
                            RTC_thread_entry,
                            RT_NULL,
                            (rt_uint8_t*)&RTC_stack[0],
                            sizeof(RTC_stack),
                            20,
                            5);
    if (result == RT_EOK)
    {
        rt_thread_startup(&RTC_thread);
    }
		

#if (RT_THREAD_PRIORITY_MAX == 32)
    init_thread = rt_thread_create("init",
                                   rt_init_thread_entry, RT_NULL,
                                   2048, 8, 20);
#else
    init_thread = rt_thread_create("init",
                                   rt_init_thread_entry, RT_NULL,
                                   2048, 80, 20);
#endif

    if (init_thread != RT_NULL)
        rt_thread_startup(init_thread);

    return 0;
}

void OutPutFile(void)
{ 
	unsigned int a;
  FILINFO finfo;
  DIR dirs;

	int baud = 0;
	float data = 0.0;
	char* baud_buf;
	char name[50]={0};
	
  char path[50]={""}; 	  						 //Ŀ¼��Ϊ�գ���ʾ�Ǹ�Ŀ¼
  char *result1, *result2; 
  char EXT1[4]=".txt"; 	 
  char EXT2[4]=".TXT"; 	
  /*�������ļ�������ʱ�� ҪԤ�ȳ�ʼ���ļ����������ĳ��� */
  #if _USE_LFN
    static char lfn[_MAX_LFN * (_DF1S ? 2 : 1) + 1];
    finfo.lfname = lfn;
    finfo.lfsize = sizeof(lfn);
  #endif
  
  disk_initialize(0);						     //fatfs���Թ��������ʵķ����� ���԰�����洢����SST25VF016B��ʾΪ0�����൱�ڴ��̱��  
  f_mount(0, &fs);							     //���ļ�ϵͳ���õ�0�� 

	if (f_opendir(&dirs, path) == FR_OK)            //��ȡ�ô��̵ĸ�Ŀ¼        
  {
    while (f_readdir(&dirs, &finfo) == FR_OK)  	 //ѭ�����ζ�ȡ�ļ���
    {	 
      if (finfo.fattrib & AM_ARC) 			     //�ж��ļ������Ƿ�Ϊ�浵��	 TXT�ļ�һ�㶼Ϊ�浵��
      {
        if(!finfo.fname[0])	 break;    		     //������ļ���Ϊ�ձ�ʾ��Ŀ¼��ĩβ���˳�	 
				if(finfo.lfname[0])//���ļ���
				{	 											 	
				result1=strstr(finfo.lfname,EXT1);								 //�ж��Ƿ���txt��Сд����׺���ļ���
				result2=strstr(finfo.lfname,EXT2);	     						 //�ȽϺ�׺�Ƿ�TXT����д����׺���ļ���
				}
				else
				{						  			    	                     //8.3��ʽ�ļ���
					result1=strstr(finfo.fname,EXT1);								 //�ж��Ƿ���txt��Сд����׺���ļ���
					result2=strstr(finfo.fname,EXT2);	     						 //�ȽϺ�׺�Ƿ�TXT����д����׺���ļ���		
				}			
				if(result1!=NULL||result2!=NULL)//�ǵĻ�������ļ������� 
				{	 								  
					if(finfo.lfname[0])//���ļ���
					{	 										  		
						res = f_open(&fsrc, finfo.lfname, FA_OPEN_EXISTING | FA_READ);//�Զ��ķ�ʽ���ļ�
					}
					else//8.3��ʽ�ļ���
					{										  		 			  
						res = f_open(&fsrc, finfo.fname, FA_OPEN_EXISTING | FA_READ); //�Զ��ķ�ʽ���ļ�
					}
       	 	br=1;
					a=0;
					for (;;) 
					{														  //ѭ�����������ļ�������
						for(a=0; a<512; a++) buffer[a]=0; 							  //��Ϊ����һ�ζ���512�ֽڣ���������ݻ����� 
						res = f_read(&fsrc, buffer, 512, &br);			  //���ļ����ݶ��������ݻ�����
						if(f(buffer,"123456")!=6)break;
						baud_buf = GetStr(buffer,"baud");
						baud = atoi(baud_buf);
						
						baud_buf = GetStr(buffer,"data");
						data = atof(baud_buf);
						
//						baud_buf = GetStr(buffer,"name");//�����޷���ʾ
//						baud_buf = &name[0];
						
						if (res || br == 0) break;   // error or eof				  //�ж��Ƿ��ļ�����
					}
					f_close(&fsrc);													   //�ر�Դ�ļ�
				}												   
      }
    }  
  } 
}

void OutPutFile2(void)
{ 
//	unsigned int a;
  FILINFO finfo;
  DIR dirs;

  char path[50]={""}; 	  						 //Ŀ¼��Ϊ�գ���ʾ�Ǹ�Ŀ¼
//  char *result1, *result2; 
//  char EXT1[4]=".txt"; 	 
//  char EXT2[4]=".TXT"; 	
  /*�������ļ�������ʱ�� ҪԤ�ȳ�ʼ���ļ����������ĳ��� */
  #if _USE_LFN
    static char lfn[_MAX_LFN * (_DF1S ? 2 : 1) + 1];
    finfo.lfname = lfn;
    finfo.lfsize = sizeof(lfn);
  #endif
  
  disk_initialize(0);						     //fatfs���Թ��������ʵķ����� ���԰�����洢����SST25VF016B��ʾΪ0�����൱�ڴ��̱��  
  f_mount(0, &fs);							     //���ļ�ϵͳ���õ�0�� 

	if (f_opendir(&dirs, path) == FR_OK)            //��ȡ�ô��̵ĸ�Ŀ¼        
  {
		res = f_open(&fdst, "Config.txt", FA_CREATE_ALWAYS | FA_WRITE);//��д�ķ�ʽ���ļ�����������ڣ���ֱ�ӽ�����������ھ͸��ǵ�
																																		//ע�͵���Ϊ�������ѧϰ�ļ��������ܣ�Ҳͬ��������
																																		//�ļ��������������ݡ�
		res = f_write(&fdst, write_buf, sizeof(write_buf)-1, &br);	//д���ַ�����ʾ�ǶԵģ�д��������ʾ����
		f_close(&fdst);	    											   //�ر�Ŀ���ļ� 
  } 
}

void SD_TEST(void){
	
	Status = SD_Init();
	
 	if (Status == SD_OK)
	{
			// �ӵ�ַ0��ʼ��ȡ512�ֽ�  
		Status = SD_ReadBlock(Buffer_Block_Rx, 0x00,  1); 
//		Status = SD_WriteBlock(Buffer_Block_Tx,0x0a,3);
	}
	
}

void NVIC_Configuration_SD(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;

  /* ���ȼ���1 */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);	  

  NVIC_InitStructure.NVIC_IRQChannel = SDIO_IRQn;			     //SDIO�ж�
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;		 //�������ȼ�0  ��Χ��0��1
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;			 //�����ȼ�0	��Χ��0-7
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

/*
str1:Դ�ַ���    helloword123
str2:�����ҵ��ַ��� word
��str1�в���str2������str2��str1�е�λ��
*/
unsigned int FindStr(char* str1,char* str2)
{
	unsigned int lo = 0;
	unsigned int i,j;
	i = sizeof(str1);
	j = sizeof(str2);
	
	for(i=0;i<sizeof(str1);i++)
	{
		for(j=0;j<sizeof(str2);j++)
		{
			if(str1[i]!=str2[j]) 
			{
				j = sizeof(str2);
				break;
			}
			if(j==sizeof(str2)) 
			{
				lo = i;
				i = sizeof(str1);
			}
		}
		
	}
	return lo;
}

int f(char*s1,char*s2)
{
	char *p,*q;
	int flag = 0,flag2=0;
	int lo = 0;
	
	for(;*s1!=0x00;s1++)//'\0'
	{
		if (*s2==*s1) /*�ж�Դ�ַ������Ƿ��к�Ŀ���ַ�������ĸ��ͬ���ַ�*/
		{ 
			flag=1;
			p=s1 ; /*s1��pΪ��һ����ͬ�ַ��ĵ�ַ*/
			q=s2;
			for(;*q!='\0';) /*��������жϽ��������ַ��Ƿ���ͬ*/
			{ 
				if (*q++!=*p++)
				{ 
					flag=0;
					break;
				}
				lo++;
				flag2++;
			}
		}

		if (flag==1)break;
		else if(flag2==0) lo++;
		else
		{
			lo = lo+1-flag2;
			flag2=0;
		}
	}
	if(flag==0) lo=0;
	
	return(lo);//����Դ�ַ�����Ŀ���ַ�����λ�õ���һλ
}

/*
��ȡstr2�����Ч����
*/
char* GetStr(char* str1,char*str2)
{
	static char var[20]={0};
//	char* var_buf;
	int lo=0,i=0;
	
	lo = f(str1,str2);//=
	if(lo>0)
	{
		while(str1[lo+1]!=0x0D && str1[lo+2]!=0x0A)
		{
			lo++;
			var[i++] = str1[lo];
		}
	}
//	var_buf = &var[0];
	
	return var;
}

///*
//��ȡstr2�����Ч����
//*/
//char* GetStr(char* str1,char*str2)
//{
//	char* var;
//	int lo=0;
//	
//	lo = f(str1,str2);//=
//	if(lo>0)
//	{
//		while(*(str1+lo+1)!=0x0D && *(str1+lo+2)!=0x0A)
//		{
//			*(var++) = *(str1+lo);
//			lo++;
//		}
//	}
//	
//	return var;
//}

/*@}*/
