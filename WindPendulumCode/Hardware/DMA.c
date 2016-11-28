#include "DMA.h"
#if __DMA_ENABLE
__IO u16 ADC1ConvertedValue[2];
/***************************************************************************
�������ƣ�DMA_Config
�������ܣ�DMAx�ĸ�ͨ�����ã��Ӵ洢��->����ģʽ/8λ���ݿ��/�洢������ģʽ
������ע������˳�򡪡���������DMAͨ��ѡ�������ַ���洢����ַ�����ݴ�����
***************************************************************************/
void DMAOUT_Config(DMA_Stream_TypeDef *DMA_Streamx,u32 chx,u32 par,u32 mar,u16 ndtr)
{
	
	DMA_InitTypeDef DMA_InitStructure;
	
	if((u32)DMA_Streamx > (u32)DMA2)
	{
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2,ENABLE);
	}
	else
	{
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1,ENABLE);
	}
	
	DMA_DeInit(DMA_Streamx);
	
	DMA_InitStructure.DMA_Channel            = chx;                         //ͨ��ѡ��
	DMA_InitStructure.DMA_PeripheralBaseAddr = par;                         //DMA�����ַ
	DMA_InitStructure.DMA_Memory0BaseAddr    = mar;                         //DMA�洢 ��ַ
	DMA_InitStructure.DMA_DIR                = DMA_DIR_MemoryToPeripheral;  //�洢��������ģʽ
	DMA_InitStructure.DMA_BufferSize         = ndtr;                        //���ݴ�����
	DMA_InitStructure.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;   //���������ģʽ
	DMA_InitStructure.DMA_MemoryInc          = DMA_MemoryInc_Enable;        //�洢������ģʽ
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; //�������ݳ���    8λ
	DMA_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;     //�洢�����ݳ���  8λ
	DMA_InitStructure.DMA_Mode               = DMA_Mode_Normal;             //ʹ����ͨģʽ
	DMA_InitStructure.DMA_Priority           = DMA_Priority_Medium;         //�е����ȼ�
	DMA_InitStructure.DMA_FIFOMode           = DMA_FIFOMode_Disable;        //FIFOģʽ��ֹ
	DMA_InitStructure.DMA_FIFOThreshold      = DMA_FIFOThreshold_Full;      //FIFO��ֵ
	DMA_InitStructure.DMA_MemoryBurst        = DMA_MemoryBurst_Single;      //�洢��ͻ�����δ���
	DMA_InitStructure.DMA_PeripheralBurst    = DMA_PeripheralBurst_Single;  //����ͻ�����δ���
	DMA_Init(DMA_Streamx,&DMA_InitStructure);
		
}

/***************************************************************************
�������ƣ�DMA_Enable
�������ܣ�����һ��DMA����
������ע������˳�򡪡�DMA�����������ݴ�����
***************************************************************************/
void DMA_Enable(DMA_Stream_TypeDef *DMA_Streamx,u16 ndtr)
{

	DMA_Cmd(DMA_Streamx,DISABLE);
	while(DMA_GetCmdStatus(DMA_Streamx) != DISABLE);
	DMA_SetCurrDataCounter(DMA_Streamx,ndtr);
	DMA_Cmd(DMA_Streamx,ENABLE);

}

/***************************************************************************
�������ƣ�ADC_DMA_Config
�������ܣ����ڴ���ADC�ɼ������ĵ�ѹ
������ע��
***************************************************************************/
void ADC_DMA_Config(void)
{
  DMA_InitTypeDef  DMA_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

  /* Enable DMA2 clocks -----------------------------------------------------*/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2,ENABLE);

  /* DMA2 Stream0 channe0 configuration -------------------------------------*/
	DMA_InitStructure.DMA_Channel            = DMA_Channel_0;                       //DMA1ͨ��1���� 
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)ADC1_DR_ADDRESS;                //�ڴ��ַ 
	DMA_InitStructure.DMA_Memory0BaseAddr    = (u32)ADC1ConvertedValue;             //dma���䷽���� 
	DMA_InitStructure.DMA_DIR                = DMA_DIR_PeripheralToMemory;          //���赽�洢��
	DMA_InitStructure.DMA_BufferSize         = 2;                                   //���ݴ����� 
	DMA_InitStructure.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;           //���������ģʽ
	DMA_InitStructure.DMA_MemoryInc          = DMA_MemoryInc_Enable;                //�洢������ģʽ
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;     //�������ݳ���    8λ
	DMA_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_HalfWord;         //�洢�����ݳ���  8λ
	DMA_InitStructure.DMA_Mode               = DMA_Mode_Circular;                   //ʹ��ѭ��ģʽ
	DMA_InitStructure.DMA_Priority           = DMA_Priority_High;                   //�е����ȼ�
	DMA_InitStructure.DMA_FIFOMode           = DMA_FIFOMode_Disable;                //FIFOģʽ��ֹ
	DMA_InitStructure.DMA_FIFOThreshold      = DMA_FIFOThreshold_HalfFull;          //FIFO��ֵ
	DMA_InitStructure.DMA_MemoryBurst        = DMA_MemoryBurst_Single;              //�洢��ͻ�����δ���
	DMA_InitStructure.DMA_PeripheralBurst    = DMA_PeripheralBurst_Single;          //����ͻ�����δ���
	DMA_Init(DMA2_Stream0,&DMA_InitStructure);
	
	DMA_Cmd(DMA2_Stream0,ENABLE);
	
	DMA_ITConfig(DMA2_Stream0,DMA_IT_TC,ENABLE);
	
	/* DMA2  interrupt configuration -------------------------------------------*/
	NVIC_InitStructure.NVIC_IRQChannel                   = DMA2_Stream0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = DMA2S0_PP;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority        = DMA2S0_PS;
	NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
}



#endif  //__DMA_ENABLE

