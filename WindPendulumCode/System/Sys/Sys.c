#include "Sys.h"

/***************************************************************************
函数名称：SysTick_Init
函数功能：系统滴答时钟启动
函数备注：
***************************************************************************/
static u16 fac_us = 0;
static u32 fac_ms = 0;
void SysTick_Init(void)
{

	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);
	fac_us = SystemCoreClock / 8000000;
	fac_ms = SystemCoreClock / 8000;

}

/***************************************************************************
函数名称：Delay_us
函数功能：微秒级延时，会占用CPU运行时间
函数备注：最大值——798915
***************************************************************************/
void Delay_us(__IO u32 nTime)
{
	u32 State;
	SysTick -> LOAD = nTime * fac_us;    //Load time
	SysTick -> VAL  = 0X00;      //Clear Counter
	SysTick -> CTRL = 0X01;       //Start time
	do                           //Wait the time will over
	{
		State = SysTick -> CTRL;
	}
	while((State & 0x01) && !(State & (1 << 16)));
	SysTick -> CTRL = 0X00;      //Close Counter
	SysTick -> VAL  = 0x00;      //Clear Counter	
}

/***************************************************************************
函数名称：毫秒级延时
函数功能：毫秒级延时，会占用CPU运行时间
函数备注：最大值——798
***************************************************************************/
void Delay_ms(__IO u16 nTime)
{
	u32 State;
	SysTick -> LOAD = nTime * fac_ms;    //Load time
	SysTick -> VAL  = 0X00;      //Clear Counter
	SysTick -> CTRL = 0X01;       //Start time
	do                           //Wait the time will over
	{
		State = SysTick -> CTRL;
	}
	while((State & 0x01) && !(State & (1 << 16)));
	SysTick -> CTRL = 0X00;      //Close Counter
	SysTick -> VAL  = 0x00;      //Clear Counter	
}


