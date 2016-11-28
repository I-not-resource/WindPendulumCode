#include "ADC.h"
#if __ADC_ENABLE
/***************************************************************************
函数名称：ADCKey_Init
函数功能：ADC功能初始化
函数备注：
***************************************************************************/
void ADCKey_Init(void)
{
  GPIO_InitTypeDef      GPIO_InitStructure;
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	ADC_InitTypeDef       ADC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE);
	
	GPIO_InitStructure.GPIO_Pin     = GPIO_Pin_0;        //通道10
	GPIO_InitStructure.GPIO_Mode    = GPIO_Mode_AN;      //模拟输入
	GPIO_InitStructure.GPIO_PuPd    = GPIO_PuPd_NOPULL;  //浮空
	GPIO_Init(GPIOC,&GPIO_InitStructure);
	
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1,ENABLE);  //ADC1复位
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1,DISABLE); //复位结束
	
	ADC_TempSensorVrefintCmd(ENABLE);                    //使能内部温度传感器
	
	ADC_CommonInitStructure.ADC_Mode             = ADC_Mode_Independent;         //独立模式
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles; //两个采样阶段周期
	ADC_CommonInitStructure.ADC_DMAAccessMode    = ADC_DMAAccessMode_Disabled;   //DMA失能
	ADC_CommonInitStructure.ADC_Prescaler        = ADC_Prescaler_Div4;           //4分频，36MHz
	ADC_CommonInit(&ADC_CommonInitStructure);
	
	ADC_InitStructure.ADC_Resolution           = ADC_Resolution_12b;             //12B模式
	ADC_InitStructure.ADC_ScanConvMode         = ENABLE;                         //扫描：开
	ADC_InitStructure.ADC_ContinuousConvMode   = ENABLE;                         //连续转换：开
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;  //触发检测：关
	ADC_InitStructure.ADC_DataAlign            = ADC_DataAlign_Right;            //对齐：右
	ADC_InitStructure.ADC_NbrOfConversion      = 2;                              //1个转换在规则序列中
	ADC_Init(ADC1,&ADC_InitStructure);
	
	/* ADC1 regular channel10 configuration ------------------------------------*/
	ADC_RegularChannelConfig(ADC1,ADC_Channel_10,1,ADC_SampleTime_480Cycles);	
	ADC_RegularChannelConfig(ADC1, ADC_Channel_16, 2, ADC_SampleTime_480Cycles );	//ADC16,ADC通道,480个周期,提高采样时间可以提高精确度	
	
	
	ADC_DMARequestAfterLastTransferCmd(ADC1,ENABLE);                             //源数据变化时开启DMA传输
	
	ADC_DMACmd(ADC1,ENABLE);
	
	ADC_Cmd(ADC1,ENABLE);
}

/***************************************************************************
函数名称：ADCKey_Init
函数功能：ADC功能初始化
函数备注：
***************************************************************************/
void ADCTem_Init(void)
{
  GPIO_InitTypeDef      GPIO_InitStructure;
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	ADC_InitTypeDef       ADC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE);
	
	GPIO_InitStructure.GPIO_Pin     = GPIO_Pin_6;        //通道6
	GPIO_InitStructure.GPIO_Mode    = GPIO_Mode_AN;      //模拟输入
	GPIO_InitStructure.GPIO_PuPd    = GPIO_PuPd_NOPULL;  //浮空
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1,ENABLE);  //ADC1复位
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1,DISABLE); //复位结束
	
  ADC_TempSensorVrefintCmd(ENABLE);                    //使能内部温度传感器
	
	ADC_CommonInitStructure.ADC_Mode             = ADC_Mode_Independent;         //独立模式
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles; //两个采样阶段周期
	ADC_CommonInitStructure.ADC_DMAAccessMode    = ADC_DMAAccessMode_Disabled;   //DMA失能
	ADC_CommonInitStructure.ADC_Prescaler        = ADC_Prescaler_Div4;           //4分频，36MHz
	ADC_CommonInit(&ADC_CommonInitStructure);
	
	ADC_InitStructure.ADC_Resolution           = ADC_Resolution_12b;             //12B模式
	ADC_InitStructure.ADC_ScanConvMode         = DISABLE;                        //扫描：关
	ADC_InitStructure.ADC_ContinuousConvMode   = DISABLE;                        //连续转换：关
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;  //触发检测：关
	ADC_InitStructure.ADC_DataAlign            = ADC_DataAlign_Right;            //对齐：右
	ADC_InitStructure.ADC_NbrOfConversion      = 2;                              //2个转换在规则序列中
	ADC_Init(ADC1,&ADC_InitStructure);
	
	/* ADC1 regular channel10 configuration ------------------------------------*/
	ADC_RegularChannelConfig(ADC1,ADC_Channel_6,1,ADC_SampleTime_480Cycles);
	
	ADC_Cmd(ADC1,ENABLE);
}
/***************************************************************************
函数名称：
函数功能：
函数备注：
***************************************************************************/









#endif  //__ADC_ENABLE

