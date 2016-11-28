#include "DMA.h"
#if __DMA_ENABLE
__IO u16 ADC1ConvertedValue[2];
/***************************************************************************
函数名称：DMA_Config
函数功能：DMAx的各通道配置；从存储器->外设模式/8位数据宽度/存储器增量模式
函数备注：输入顺序——数据流、DMA通道选择、外设地址、存储器地址、数据传输量
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
	
	DMA_InitStructure.DMA_Channel            = chx;                         //通道选择
	DMA_InitStructure.DMA_PeripheralBaseAddr = par;                         //DMA外设地址
	DMA_InitStructure.DMA_Memory0BaseAddr    = mar;                         //DMA存储 地址
	DMA_InitStructure.DMA_DIR                = DMA_DIR_MemoryToPeripheral;  //存储器到外设模式
	DMA_InitStructure.DMA_BufferSize         = ndtr;                        //数据传输量
	DMA_InitStructure.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;   //外设非增量模式
	DMA_InitStructure.DMA_MemoryInc          = DMA_MemoryInc_Enable;        //存储器增量模式
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; //外设数据长度    8位
	DMA_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;     //存储器数据长度  8位
	DMA_InitStructure.DMA_Mode               = DMA_Mode_Normal;             //使用普通模式
	DMA_InitStructure.DMA_Priority           = DMA_Priority_Medium;         //中等优先级
	DMA_InitStructure.DMA_FIFOMode           = DMA_FIFOMode_Disable;        //FIFO模式禁止
	DMA_InitStructure.DMA_FIFOThreshold      = DMA_FIFOThreshold_Full;      //FIFO阈值
	DMA_InitStructure.DMA_MemoryBurst        = DMA_MemoryBurst_Single;      //存储器突发单次传输
	DMA_InitStructure.DMA_PeripheralBurst    = DMA_PeripheralBurst_Single;  //外设突发单次传输
	DMA_Init(DMA_Streamx,&DMA_InitStructure);
		
}

/***************************************************************************
函数名称：DMA_Enable
函数功能：开启一次DMA传输
函数备注：输入顺序——DMA数据流、数据传输量
***************************************************************************/
void DMA_Enable(DMA_Stream_TypeDef *DMA_Streamx,u16 ndtr)
{

	DMA_Cmd(DMA_Streamx,DISABLE);
	while(DMA_GetCmdStatus(DMA_Streamx) != DISABLE);
	DMA_SetCurrDataCounter(DMA_Streamx,ndtr);
	DMA_Cmd(DMA_Streamx,ENABLE);

}

/***************************************************************************
函数名称：ADC_DMA_Config
函数功能：用于传输ADC采集回来的电压
函数备注：
***************************************************************************/
void ADC_DMA_Config(void)
{
  DMA_InitTypeDef  DMA_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

  /* Enable DMA2 clocks -----------------------------------------------------*/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2,ENABLE);

  /* DMA2 Stream0 channe0 configuration -------------------------------------*/
	DMA_InitStructure.DMA_Channel            = DMA_Channel_0;                       //DMA1通道1配置 
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)ADC1_DR_ADDRESS;                //内存地址 
	DMA_InitStructure.DMA_Memory0BaseAddr    = (u32)ADC1ConvertedValue;             //dma传输方向单向 
	DMA_InitStructure.DMA_DIR                = DMA_DIR_PeripheralToMemory;          //外设到存储器
	DMA_InitStructure.DMA_BufferSize         = 2;                                   //数据传输量 
	DMA_InitStructure.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;           //外设非增量模式
	DMA_InitStructure.DMA_MemoryInc          = DMA_MemoryInc_Enable;                //存储器增量模式
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;     //外设数据长度    8位
	DMA_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_HalfWord;         //存储器数据长度  8位
	DMA_InitStructure.DMA_Mode               = DMA_Mode_Circular;                   //使用循环模式
	DMA_InitStructure.DMA_Priority           = DMA_Priority_High;                   //中等优先级
	DMA_InitStructure.DMA_FIFOMode           = DMA_FIFOMode_Disable;                //FIFO模式禁止
	DMA_InitStructure.DMA_FIFOThreshold      = DMA_FIFOThreshold_HalfFull;          //FIFO阈值
	DMA_InitStructure.DMA_MemoryBurst        = DMA_MemoryBurst_Single;              //存储器突发单次传输
	DMA_InitStructure.DMA_PeripheralBurst    = DMA_PeripheralBurst_Single;          //外设突发单次传输
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

