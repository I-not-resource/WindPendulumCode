#include "TIMControl.h"
#if __TIM_ENABLE
/***************************************************************************
函数名称：TIM51Line_Init
函数功能：定时器5中断配置函数，配置成每5ms执行一次,T=PSC/(42MHz/ARR)，倾角计算
函数备注：
***************************************************************************/
void TIM5Line_Init(void)
{
  TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef        NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,ENABLE);
	
	TIM_TimeBaseInitStructure.TIM_Period       =  (T5ARR - 1);          //自动重装载值
	TIM_TimeBaseInitStructure.TIM_Prescaler    =  (T5PSC - 1);          //定时器分频
	TIM_TimeBaseInitStructure.TIM_CounterMode  =  TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	
  TIM_TimeBaseInit(TIM5,&TIM_TimeBaseInitStructure);
	
	TIM_ITConfig(TIM5,TIM_IT_Update,ENABLE);
	
	NVIC_InitStructure.NVIC_IRQChannel                   = TIM5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = TIM5_PP;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority        = TIM5_PS;
	NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	TIM_Cmd(TIM5,ENABLE);
}

/***************************************************************************
函数名称：TIM7Line_Init
函数功能：定时器7中断配置函数，配置成每10ms执行一次,T=PSC/(42MHz/ARR),用于AD按键采集
函数备注：
***************************************************************************/
void TIM7Line_Init()
{
  TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef        NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7,ENABLE);
	
	TIM_TimeBaseInitStructure.TIM_Period       =  (T7ARR - 1);          //自动重装载值
	TIM_TimeBaseInitStructure.TIM_Prescaler    =  (T7PSC - 1);          //定时器分频
	TIM_TimeBaseInitStructure.TIM_CounterMode  =  TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	
  TIM_TimeBaseInit(TIM7,&TIM_TimeBaseInitStructure);
	
	TIM_ITConfig(TIM7,TIM_IT_Update,ENABLE);
	
	NVIC_InitStructure.NVIC_IRQChannel                   = TIM7_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = TIM7_PP;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority        = TIM7_PS;
	NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	TIM_Cmd(TIM7,ENABLE);
}

/***************************************************************************
函数名称：TIM9Line_Init
函数功能：定时器9中断配置函数，配置成每200ms执行一次,T=PSC/(84MHz/ARR),用于状态更新
函数备注：
***************************************************************************/
void TIM9Line_Init(void)
{
  TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef        NVIC_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9,ENABLE);
	
	TIM_TimeBaseInitStructure.TIM_Period       =  (T9ARR - 1);          //自动重装载值
	TIM_TimeBaseInitStructure.TIM_Prescaler    =  (T9PSC - 1);          //定时器分频
	TIM_TimeBaseInitStructure.TIM_CounterMode  =  TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	
  TIM_TimeBaseInit(TIM9,&TIM_TimeBaseInitStructure);
	
	TIM_ITConfig(TIM9,TIM_IT_Update,ENABLE);
	
	NVIC_InitStructure.NVIC_IRQChannel                   = TIM1_BRK_TIM9_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = TIM9_PP;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority        = TIM9_PS;
	NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	TIM_Cmd(TIM9,ENABLE);
}

/***************************************************************************
函数名称：TIM11Line_Init
函数功能：定时器11中断配置函数，配置成每200ms执行一次,T=PSC/(84MHz/ARR)
函数备注：
***************************************************************************/
void TIM11Line_Init(void)
{
  TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef        NVIC_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM11,ENABLE);
	
	TIM_TimeBaseInitStructure.TIM_Period       =  (T11ARR - 1);          //自动重装载值
	TIM_TimeBaseInitStructure.TIM_Prescaler    =  (T11PSC - 1);          //定时器分频
	TIM_TimeBaseInitStructure.TIM_CounterMode  =  TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	
  TIM_TimeBaseInit(TIM11,&TIM_TimeBaseInitStructure);
	
	TIM_ITConfig(TIM11,TIM_IT_Update,ENABLE);
	
	NVIC_InitStructure.NVIC_IRQChannel                   = TIM1_TRG_COM_TIM11_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = TIM11_PP;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority        = TIM11_PS;
	NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	TIM_Cmd(TIM11,ENABLE);
}










#endif  //__TIM_ENABLE



