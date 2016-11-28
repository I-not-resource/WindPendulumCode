#ifndef __DMA_H_
#define __DMA_H_
#include "Sys.h"
#if __DMA_ENABLE
#define ADC1_DR_ADDRESS ((u32)0x4001204C)


extern __IO u16 ADC1ConvertedValue[2];

void DMAOUT_Config(DMA_Stream_TypeDef *DMA_Streamx,u32 chx,u32 par,u32 mar,u16 ndtr);
void DMA_Enable(DMA_Stream_TypeDef *DMA_Streamx,u16 ndtr);
void ADC_DMA_Config(void);


#endif   //__DMA_ENABLE

#endif   //__DMA_H_
