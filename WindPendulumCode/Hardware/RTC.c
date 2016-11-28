#include "RTC.h"
#if __RTC_ENABLE

/***************************************************************************
函数名称：RTC_Set_Time
函数功能：RTC时间设置
函数备注：输入顺序——时，分，秒，上午/下午
返回值  ：SUCESS(1),成功
					ERROR(0),进入初始化模式失败
***************************************************************************/
ErrorStatus RTC_Set_Time(u8 hour,u8 min,u8 sec,u8 ampm)
{
	RTC_TimeTypeDef RTC_TimeTypeInitStructure;
	
	RTC_TimeTypeInitStructure.RTC_Hours=hour;
	RTC_TimeTypeInitStructure.RTC_Minutes=min;
	RTC_TimeTypeInitStructure.RTC_Seconds=sec;
	RTC_TimeTypeInitStructure.RTC_H12=ampm;
	
	return RTC_SetTime(RTC_Format_BIN,&RTC_TimeTypeInitStructure);
	
}
/***************************************************************************
函数名称：RTC_Set_Date
函数功能：RTC日期设置
函数备注：输入顺序——年，月，日，星期
返回值  ：SUCESS(1),成功
					ERROR(0),进入初始化模式失败
***************************************************************************/
ErrorStatus RTC_Set_Date(u8 year,u8 month,u8 date,u8 week)
{
	
	RTC_DateTypeDef RTC_DateTypeInitStructure;
	RTC_DateTypeInitStructure.RTC_Date=date;
	RTC_DateTypeInitStructure.RTC_Month=month;
	RTC_DateTypeInitStructure.RTC_WeekDay=week;
	RTC_DateTypeInitStructure.RTC_Year=year;
	return RTC_SetDate(RTC_Format_BIN,&RTC_DateTypeInitStructure);
}

/***************************************************************************
函数名称：The_RTC_Init
函数功能：RTC初始化
函数备注：
返回值:0,初始化成功
       1,LSE开启失败
       2,进入初始化模式失败
***************************************************************************/
u8 The_RTC_Init(void)
{
	
	RTC_InitTypeDef   RTC_InitStructure;
	GPIO_InitTypeDef  GPIO_InitStructure;
	
	u16 retry = 0x1fff;
	

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);//使能GPIOF时钟

  //GPIOF9,F10初始化设置
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;//LED0和LED1对应IO口
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
  GPIO_Init(GPIOF, &GPIO_InitStructure);//初始化GPIO
	
	GPIO_SetBits(GPIOF,GPIO_Pin_9 | GPIO_Pin_10);//GPIOF9,F10设置高，灯灭
	
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR,ENABLE);     //使能PWR时钟
	PWR_BackupAccessCmd(ENABLE);                           //使能后备寄存器访问
	
	if(RTC_ReadBackupRegister(RTC_BKP_DR0) != 0x1530)      //检查是否第一次配置
	{
		RCC_LSEConfig(RCC_LSE_ON);                           //LSE 开启 
		while(RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET)   //检查指定的RCC标志位设置与否,等待低速晶振就绪
		{
			retry++;
			Delay_ms(10);
		}
		
	if(retry == 0)
		return 1;                                            //LSE 开启失败.
		
	RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);                //设置RTC时钟(RTCCLK),选择LSE作为RTC时钟  
	RCC_RTCCLKCmd(ENABLE);                                 //使能RTC时钟 
	
	RTC_InitStructure.RTC_AsynchPrediv = 0x7f;             //RTC异步分频系数(1~0X7F)
	RTC_InitStructure.RTC_SynchPrediv  = 0XFF;             //RTC同步分频系数(0~7FFF)
	RTC_InitStructure.RTC_HourFormat   = RTC_HourFormat_24;//RTC设置为,24小时格式
	
	RTC_Set_Time(23,59,59,RTC_H12_AM);
	RTC_Set_Date(15,3,23,Monday);
	RTC_WriteBackupRegister(RTC_BKP_DR0,0x1530);
	}
	
	return 0;
	
}

/***************************************************************************
函数名称：RTC_Set_AlarmA
函数功能：闹钟设置，以一个星期为周期
函数备注：输入顺序-星期，时，分，秒
返回值:
***************************************************************************/
void RTC_Set_AlarmA(u8 week,u8 hour,u8 min,u8 sec)
{
	
	NVIC_InitTypeDef   NVIC_InitStructure;
	EXTI_InitTypeDef   EXTI_InitStructure;
	RTC_AlarmTypeDef   RTC_AlarmTypeInitStructure;
	RTC_TimeTypeDef    RTC_TimeTypeInitStructure;
	
	RTC_AlarmCmd(RTC_Alarm_A,DISABLE);                                                  //关闭闹钟A 
	
  RTC_TimeTypeInitStructure.RTC_Hours=hour;                                           //小时
	RTC_TimeTypeInitStructure.RTC_Minutes=min;                                          //分钟
	RTC_TimeTypeInitStructure.RTC_Seconds=sec;                                          //秒
	RTC_TimeTypeInitStructure.RTC_H12=RTC_H12_AM;
  
	RTC_AlarmTypeInitStructure.RTC_AlarmDateWeekDay=week;                               //星期
	RTC_AlarmTypeInitStructure.RTC_AlarmDateWeekDaySel=RTC_AlarmDateWeekDaySel_WeekDay; //按星期闹
	RTC_AlarmTypeInitStructure.RTC_AlarmMask=RTC_AlarmMask_None;                        //精确匹配星期，时分秒
	RTC_AlarmTypeInitStructure.RTC_AlarmTime=RTC_TimeTypeInitStructure;
  RTC_SetAlarm(RTC_Format_BIN,RTC_Alarm_A,&RTC_AlarmTypeInitStructure);
 
	
	RTC_ClearITPendingBit(RTC_IT_ALRA);                                                 //清除RTC闹钟A的标志
  EXTI_ClearITPendingBit(EXTI_Line17);                                                //清除LINE17上的中断标志位 
	
	RTC_ITConfig(RTC_IT_ALRA,ENABLE);                                                   //开启闹钟A中断
	RTC_AlarmCmd(RTC_Alarm_A,ENABLE);                                                   //开启闹钟A 
	
	EXTI_InitStructure.EXTI_Line = EXTI_Line17;                                         //LINE17
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;                                 //中断事件
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;                              //上升沿触发 
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;                                           //使能LINE17
  EXTI_Init(&EXTI_InitStructure);                                                     //配置

	NVIC_InitStructure.NVIC_IRQChannel = RTC_Alarm_IRQn; 
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = RTC_PP;                        //抢占优先级1
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = RTC_PS;                               //子优先级2
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;                                     //使能外部中断通道
  NVIC_Init(&NVIC_InitStructure);                                                     //配置
	
}


/***************************************************************************
函数名称：RTC_Set_WakeUp
函数功能：周期性唤醒定时器设置
函数备注：唤醒的是中断；cnt自动重装载，到0产生中断
					wksel:  @ref RTC_Wakeup_Timer_Definitions
					#define RTC_WakeUpClock_RTCCLK_Div16        ((uint32_t)0x00000000)
					#define RTC_WakeUpClock_RTCCLK_Div8         ((uint32_t)0x00000001)
					#define RTC_WakeUpClock_RTCCLK_Div4         ((uint32_t)0x00000002)
					#define RTC_WakeUpClock_RTCCLK_Div2         ((uint32_t)0x00000003)
					#define RTC_WakeUpClock_CK_SPRE_16bits      ((uint32_t)0x00000004)
					#define RTC_WakeUpClock_CK_SPRE_17bits      ((uint32_t)0x00000006)
返回值:
***************************************************************************/
void RTC_Set_WakeUp(u32 wksel,u16 cnt)
{ 
	
	NVIC_InitTypeDef   NVIC_InitStructure;
	EXTI_InitTypeDef   EXTI_InitStructure;
	
	RTC_WakeUpCmd(DISABLE);                                      //关闭WAKE UP
	
	RTC_WakeUpClockConfig(wksel);                                //唤醒时钟选择
	
	RTC_SetWakeUpCounter(cnt);                                   //设置WAKE UP自动重装载寄存器
	
	
	RTC_ClearITPendingBit(RTC_IT_WUT);                           //清除RTC WAKE UP的标志
  EXTI_ClearITPendingBit(EXTI_Line22);                         //清除LINE22上的中断标志位 
	 
	RTC_ITConfig(RTC_IT_WUT,ENABLE);                             //开启WAKE UP 定时器中断
	RTC_WakeUpCmd( ENABLE);                                      //开启WAKE UP 定时器　
	
	EXTI_InitStructure.EXTI_Line = EXTI_Line22;                  //LINE22
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;          //中断事件
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;       //上升沿触发 
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;                    //使能LINE22
  EXTI_Init(&EXTI_InitStructure);                              //配置
 
 
	NVIC_InitStructure.NVIC_IRQChannel = RTC_WKUP_IRQn; 
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;//抢占优先级1
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;       //子优先级2
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;             //使能外部中断通道
  NVIC_Init(&NVIC_InitStructure);                             //配置
	
}

/***************************************************************************
函数名称：RTC_Alarm_IRQHandler
函数功能：RTC闹钟中断服务函数
函数备注：
返回值: 
***************************************************************************/
void RTC_Alarm_IRQHandler(void)
{    
	if(RTC_GetFlagStatus(RTC_FLAG_ALRAF)==SET)         //ALARM A中断?
	{
		RTC_ClearFlag(RTC_FLAG_ALRAF);                   //清除中断标志
	}   
	EXTI_ClearITPendingBit(EXTI_Line17);	             //清除中断线17的中断标志 											 
}

/***************************************************************************
函数名称：RTC_WKUP_IRQHandler
函数功能：RTC 唤醒中断服务
函数备注：
返回值: 
***************************************************************************/
void RTC_WKUP_IRQHandler(void)
{    
	if(RTC_GetFlagStatus(RTC_FLAG_WUTF)==SET)    //WK_UP中断?
	{ 
		RTC_ClearFlag(RTC_FLAG_WUTF);	             //清除中断标志
		LED1=!LED1; 
	}   
	EXTI_ClearITPendingBit(EXTI_Line22);         //清除中断线22的中断标志 								
}








#endif
