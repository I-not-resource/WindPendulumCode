#include "Ort_encoding.h"
#if     __ORT_ENABLE
/***************************************************************************
�������ƣ�ORT_ENC_Init
�������ܣ������������IO�ڳ�ʼ��
������ע��TIM8_CH1->PC6��TIM8_CH2->PC7
***************************************************************************/
void ORT0_ENC_Init(void)
{
	GPIO_InitTypeDef        GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_ICInitTypeDef       TIM_ICInitStructure;
	
/***��ʱ��************************************************************/	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8,ENABLE);

/***GPIO�ڶ���********************************************************/
  GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOC,&GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource6,GPIO_AF_TIM8);
  GPIO_PinAFConfig(GPIOC,GPIO_PinSource7,GPIO_AF_TIM8);

	
/***��ʱ������ģʽ��***************************************************/	
  TIM_DeInit(TIM8);
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);

	TIM_TimeBaseStructure.TIM_Prescaler     = 0x00;                //��ʱ����Ƶ
	TIM_TimeBaseStructure.TIM_CounterMode   = TIM_CounterMode_Up | TIM_CounterMode_Down;  //���ϼ���ģʽ
	TIM_TimeBaseStructure.TIM_Period        = ENCODER_TIM_PERIOD;  //�Զ���װ��ֵ
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;        //ʱ�ӷ�Ƶ
	TIM_TimeBaseInit(TIM8,&TIM_TimeBaseStructure);
	
	TIM_EncoderInterfaceConfig(TIM8,
	                           TIM_EncoderMode_TI12,
														 TIM_ICPolarity_Rising,
														 TIM_ICPolarity_Rising);
	TIM_ICStructInit(&TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_ICFilter = 0;                           //���������˲��������˲�
	TIM_ICInit(TIM8,&TIM_ICInitStructure);
	
	TIM8 -> CNT = 0;
	
	TIM_Cmd(TIM8,ENABLE);
}

/***************************************************************************
�������ƣ�ORT_ENC_Init
�������ܣ������������IO�ڳ�ʼ��
������ע��TIM8_CH1->PC6��TIM8_CH2->PC7
***************************************************************************/
void ORT1_ENC_Init(void)
{
	GPIO_InitTypeDef        GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_ICInitTypeDef       TIM_ICInitStructure;
	
/***��ʱ��************************************************************/	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);

/***GPIO�ڶ���********************************************************/
  GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource14,GPIO_AF_TIM1);
  GPIO_PinAFConfig(GPIOB,GPIO_PinSource15,GPIO_AF_TIM1);

	
/***��ʱ������ģʽ��***************************************************/	
  TIM_DeInit(TIM1);
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);

	TIM_TimeBaseStructure.TIM_Prescaler     = 0x00;                //��ʱ����Ƶ
	TIM_TimeBaseStructure.TIM_CounterMode   = TIM_CounterMode_Up | TIM_CounterMode_Down;  //���ϼ���ģʽ
	TIM_TimeBaseStructure.TIM_Period        = ENCODER_TIM_PERIOD;  //�Զ���װ��ֵ
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;        //ʱ�ӷ�Ƶ
	TIM_TimeBaseInit(TIM1,&TIM_TimeBaseStructure);
	
	TIM_EncoderInterfaceConfig(TIM1,
	                           TIM_EncoderMode_TI12,
														 TIM_ICPolarity_Rising,
														 TIM_ICPolarity_Rising);
	TIM_ICStructInit(&TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_ICFilter = 0;                           //���������˲��������˲�
	TIM_ICInit(TIM1,&TIM_ICInitStructure);
	
	TIM1 -> CNT = 0;
	
	TIM_Cmd(TIM1,ENABLE);
}


/***************************************************************************
�������ƣ�ORT_ENC_GetData
�������ܣ��������������������ݻ�ȡ
������ע��ʹ�ö�ʱ��8
***************************************************************************/
s16 ORT0_ENC_GetData(void)
{
	
	static u16 LastCount = 0;
	u16 CurCount         = 0;
	s32 dAngle           = 0;
	CurCount = TIM_GetCounter(TIM8);
	dAngle   = CurCount - LastCount;
	if(dAngle > MAX_COUNT)
	  dAngle += ENCODER_TIM_PERIOD;
	else if(dAngle < -MAX_COUNT)
		dAngle -= ENCODER_TIM_PERIOD;
	
	LastCount = CurCount;
	return (s16)dAngle;
	
}

/***************************************************************************
�������ƣ�ORT_ENC_GetData
�������ܣ��������������������ݻ�ȡ
������ע��ʹ�ö�ʱ��12
***************************************************************************/
s16 ORT1_ENC_GetData(void)
{
	
	static u16 LastCount = 0;
	u16 CurCount         = 0;
	s32 dAngle           = 0;
	CurCount = TIM_GetCounter(TIM1);
	dAngle   = CurCount - LastCount;
	if(dAngle > MAX_COUNT)
	  dAngle += ENCODER_TIM_PERIOD;
	else if(dAngle < -MAX_COUNT)
		dAngle -= ENCODER_TIM_PERIOD;
	
	LastCount = CurCount;
	return (s16)dAngle;
	
}

#endif  //__ORT_ENABLE





