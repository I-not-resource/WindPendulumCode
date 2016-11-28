#include "pwm.h"
#if __PWM_ENABLE

/***************************************************************************
函数名称：PWMx4_Init
函数功能：PWM设置，速度为1KHz,调节范围0-999。
函数备注：
***************************************************************************/
void PWMx4_Init(void)
{
	GPIO_InitTypeDef        GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef       TIM_OCInitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);

/***GPIO口定义********************************************************/
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource6,GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource7,GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource8,GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource9,GPIO_AF_TIM4);
	
	
/***定时器定、模式义***************************************************/	
  TIM_TimeBaseStructure.TIM_ClockDivision     = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode       = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period            = 999;
	TIM_TimeBaseStructure.TIM_Prescaler         = 84-1;
	TIM_TimeBaseInit(TIM4,&TIM_TimeBaseStructure);
	
	TIM_OCInitStructure.TIM_OCMode       = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OCPolarity   = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OutputState  = TIM_OutputState_Enable;
	TIM_OC1Init(TIM4,&TIM_OCInitStructure);
	TIM_OC2Init(TIM4,&TIM_OCInitStructure);
	TIM_OC3Init(TIM4,&TIM_OCInitStructure);
	TIM_OC4Init(TIM4,&TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM4,TIM_OCPreload_Enable);
	TIM_OC2PreloadConfig(TIM4,TIM_OCPreload_Enable);
	TIM_OC3PreloadConfig(TIM4,TIM_OCPreload_Enable);
	TIM_OC4PreloadConfig(TIM4,TIM_OCPreload_Enable);
	TIM_ARRPreloadConfig(TIM4,ENABLE);
	
	TIM_Cmd(TIM4,ENABLE);	
	
	TIM_SetCompare1(TIM4,100);
	TIM_SetCompare2(TIM4,100);		
	TIM_SetCompare3(TIM4,100);	
	TIM_SetCompare4(TIM4,100);	
}

#endif
