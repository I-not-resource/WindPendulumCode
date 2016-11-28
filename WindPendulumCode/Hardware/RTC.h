#ifndef __RTC_H_
#define __RTC_H_
#include "sys.h"
#if __RTC_ENABLE

#define LED1 PFout(10)

typedef enum
{
	Monday   = 1, 
	Tuesday,
	Wednesday,
	Thursday,
	Friday,
	Saturday,
	Sunday
}Week_TypeDef;

/*****************************Ó¦ÓÃº¯Êý**********************************/
ErrorStatus RTC_Set_Time(u8 hour,u8 min,u8 sec,u8 ampm);
ErrorStatus RTC_Set_Date(u8 year,u8 month,u8 date,u8 week);
u8 The_RTC_Init(void);
void RTC_Set_WakeUp(u32 wksel,u16 cnt);	

#endif

#endif
