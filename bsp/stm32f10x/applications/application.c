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
FATFS fs;                      // 逻辑驱动器的标志
FIL fsrc, fdst;                // 文件标志 
unsigned char buffer[512];     // 文件内容缓冲区
unsigned char* buffers;     // 文件内容缓冲区
FRESULT res;                   // FatFs 功能函数返回结果变量
unsigned int br, bw;           // 文件读/写计数器

ALIGN(RT_ALIGN_SIZE)
static rt_uint8_t sd_stack[ 512 ];
static struct rt_thread sd_thread;
static void sd_thread_entry(void* parameter)
{
  NVIC_Configuration_SD();
	FindStr("helloword123","word");
	f("helloword123","word");
	SD_TEST();
//	OutPutFile();//测试从SD卡读数据
	OutPutFile2();//测试向SD卡写数据
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
	NVIC_Configuration_RTC();								  //中断源配置
	RCC_Configuration_RTC();								  //系统时钟初始化及外设时钟使能
	Clock_ini();                                        //实时时钟初始化
	while(1)
  {
    /* 秒更新发生 */
    if(TimeDisplay == 1)
    {    
      /* 显示当前时间 */
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
	
  char path[50]={""}; 	  						 //目录名为空，表示是根目录
  char *result1, *result2; 
  char EXT1[4]=".txt"; 	 
  char EXT2[4]=".TXT"; 	
  /*开启长文件名功能时， 要预先初始化文件名缓冲区的长度 */
  #if _USE_LFN
    static char lfn[_MAX_LFN * (_DF1S ? 2 : 1) + 1];
    finfo.lfname = lfn;
    finfo.lfsize = sizeof(lfn);
  #endif
  
  disk_initialize(0);						     //fatfs可以管理多个介质的分区， 所以把物理存储介质SST25VF016B标示为0区，相当于磁盘编号  
  f_mount(0, &fs);							     //将文件系统设置到0区 

	if (f_opendir(&dirs, path) == FR_OK)            //读取该磁盘的根目录        
  {
    while (f_readdir(&dirs, &finfo) == FR_OK)  	 //循环依次读取文件名
    {	 
      if (finfo.fattrib & AM_ARC) 			     //判断文件属性是否为存档型	 TXT文件一般都为存档型
      {
        if(!finfo.fname[0])	 break;    		     //如果是文件名为空表示到目录的末尾。退出	 
				if(finfo.lfname[0])//长文件名
				{	 											 	
				result1=strstr(finfo.lfname,EXT1);								 //判断是否是txt（小写）后缀的文件名
				result2=strstr(finfo.lfname,EXT2);	     						 //比较后缀是否TXT（大写）后缀的文件名
				}
				else
				{						  			    	                     //8.3格式文件名
					result1=strstr(finfo.fname,EXT1);								 //判断是否是txt（小写）后缀的文件名
					result2=strstr(finfo.fname,EXT2);	     						 //比较后缀是否TXT（大写）后缀的文件名		
				}			
				if(result1!=NULL||result2!=NULL)//是的话就输出文件的内容 
				{	 								  
					if(finfo.lfname[0])//长文件名
					{	 										  		
						res = f_open(&fsrc, finfo.lfname, FA_OPEN_EXISTING | FA_READ);//以读的方式打开文件
					}
					else//8.3格式文件名
					{										  		 			  
						res = f_open(&fsrc, finfo.fname, FA_OPEN_EXISTING | FA_READ); //以读的方式打开文件
					}
       	 	br=1;
					a=0;
					for (;;) 
					{														  //循环读出被打开文件的扇区
						for(a=0; a<512; a++) buffer[a]=0; 							  //因为可以一次读出512字节，先清空数据缓冲区 
						res = f_read(&fsrc, buffer, 512, &br);			  //将文件内容读出到数据缓冲区
						if(f(buffer,"123456")!=6)break;
						baud_buf = GetStr(buffer,"baud");
						baud = atoi(baud_buf);
						
						baud_buf = GetStr(buffer,"data");
						data = atof(baud_buf);
						
//						baud_buf = GetStr(buffer,"name");//汉字无法显示
//						baud_buf = &name[0];
						
						if (res || br == 0) break;   // error or eof				  //判断是否到文件结束
					}
					f_close(&fsrc);													   //关闭源文件
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

  char path[50]={""}; 	  						 //目录名为空，表示是根目录
//  char *result1, *result2; 
//  char EXT1[4]=".txt"; 	 
//  char EXT2[4]=".TXT"; 	
  /*开启长文件名功能时， 要预先初始化文件名缓冲区的长度 */
  #if _USE_LFN
    static char lfn[_MAX_LFN * (_DF1S ? 2 : 1) + 1];
    finfo.lfname = lfn;
    finfo.lfsize = sizeof(lfn);
  #endif
  
  disk_initialize(0);						     //fatfs可以管理多个介质的分区， 所以把物理存储介质SST25VF016B标示为0区，相当于磁盘编号  
  f_mount(0, &fs);							     //将文件系统设置到0区 

	if (f_opendir(&dirs, path) == FR_OK)            //读取该磁盘的根目录        
  {
		res = f_open(&fdst, "Config.txt", FA_CREATE_ALWAYS | FA_WRITE);//以写的方式打开文件，如果不存在，就直接建立，如果存在就覆盖掉
																																		//注释掉是为了如果有学习文件拷贝功能，也同样具有了
																																		//文件建立及增加内容。
		res = f_write(&fdst, write_buf, sizeof(write_buf)-1, &br);	//写入字符串显示是对的，写入数组显示乱码
		f_close(&fdst);	    											   //关闭目标文件 
  } 
}

void SD_TEST(void){
	
	Status = SD_Init();
	
 	if (Status == SD_OK)
	{
			// 从地址0开始读取512字节  
		Status = SD_ReadBlock(Buffer_Block_Rx, 0x00,  1); 
//		Status = SD_WriteBlock(Buffer_Block_Tx,0x0a,3);
	}
	
}

void NVIC_Configuration_SD(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;

  /* 优先级组1 */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);	  

  NVIC_InitStructure.NVIC_IRQChannel = SDIO_IRQn;			     //SDIO中断
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;		 //抢先优先级0  范围：0或1
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;			 //子优先级0	范围：0-7
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

/*
str1:源字符串    helloword123
str2:待查找的字符串 word
在str1中查找str2，返回str2在str1中的位置
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
		if (*s2==*s1) /*判断源字符串中是否有和目的字符串首字母相同的字符*/
		{ 
			flag=1;
			p=s1 ; /*s1中p为第一个相同字符的地址*/
			q=s2;
			for(;*q!='\0';) /*如果有则判断接下来的字符是否相同*/
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
	
	return(lo);//返回源字符串中目标字符串的位置的下一位
}

/*
获取str2后的有效数据
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
//获取str2后的有效数据
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
