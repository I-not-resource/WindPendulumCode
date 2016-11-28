#include "TIMControl.h"
#if __TIM_ENABLE
/***************************************************************************
�������ƣ�TIM51Line_Init
�������ܣ���ʱ��5�ж����ú��������ó�ÿ5msִ��һ��,T=PSC/(42MHz/ARR)����Ǽ���
������ע��
***************************************************************************/
void TIM5Line_Init(void)
{
  TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef        NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,ENABLE);
	
	TIM_TimeBaseInitStructure.TIM_Period       =  (T5ARR - 1);          //�Զ���װ��ֵ
	TIM_TimeBaseInitStructure.TIM_Prescaler    =  (T5PSC - 1);          //��ʱ����Ƶ
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
�������ƣ�TIM7Line_Init
�������ܣ���ʱ��7�ж����ú��������ó�ÿ10msִ��һ��,T=PSC/(42MHz/ARR),����AD�����ɼ�
������ע��
***************************************************************************/
void TIM7Line_Init()
{
  TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef        NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7,ENABLE);
	
	TIM_TimeBaseInitStructure.TIM_Period       =  (T7ARR - 1);          //�Զ���װ��ֵ
	TIM_TimeBaseInitStructure.TIM_Prescaler    =  (T7PSC - 1);          //��ʱ����Ƶ
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
�������ƣ�TIM9Line_Init
�������ܣ���ʱ��9�ж����ú��������ó�ÿ200msִ��һ��,T=PSC/(84MHz/ARR),����״̬����
������ע��
***************************************************************************/
void TIM9Line_Init(void)
{
  TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef        NVIC_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9,ENABLE);
	
	TIM_TimeBaseInitStructure.TIM_Period       =  (T9ARR - 1);          //�Զ���װ��ֵ
	TIM_TimeBaseInitStructure.TIM_Prescaler    =  (T9PSC - 1);          //��ʱ����Ƶ
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
�������ƣ�TIM11Line_Init
�������ܣ���ʱ��11�ж����ú��������ó�ÿ200msִ��һ��,T=PSC/(84MHz/ARR)
������ע��
***************************************************************************/
void TIM11Line_Init(void)
{
  TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef        NVIC_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM11,ENABLE);
	
	TIM_TimeBaseInitStructure.TIM_Period       =  (T11ARR - 1);          //�Զ���װ��ֵ
	TIM_TimeBaseInitStructure.TIM_Prescaler    =  (T11PSC - 1);          //��ʱ����Ƶ
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



