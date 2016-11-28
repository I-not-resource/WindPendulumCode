#include "IRQ_CELL.h"
#include "includes.h"
#include "string.h"
#if __IRQ_CELL_ENABLE

u16 Value = 0;
u8  i     = 0;
int Tem   = 0;
static u8 _Temp[11];
/***************************************************************************
函数名称：TIM7_IRQHandler
函数功能：
函数备注：
***************************************************************************/
void TIM7_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM7,TIM_IT_Update) == SET)
	{
		Keyopt(Value);
    Fuction_choose();
	}
	
	TIM_ClearITPendingBit(TIM7,TIM_IT_Update);            //清除定时器7溢出中断标志位
}

/***************************************************************************
函数名称：TIM1_BRK_TIM9_IRQHandler
函数功能：1、用于更新各种非实时性状态
函数备注：
***************************************************************************/
void TIM1_BRK_TIM9_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM9,TIM_IT_Update) == SET)
	{

	}
	
	TIM_ClearITPendingBit(TIM9,TIM_IT_Update);            //清除定时器7溢出中断标志位
}


/***************************************************************************
函数名称：TIM5_IRQHandler
函数功能：
函数备注：
***************************************************************************/
void TIM5_IRQHandler(void)
{

	if(TIM_GetITStatus(TIM5,TIM_IT_Update) == SET)
	{

	}
	
	TIM_ClearITPendingBit(TIM5,TIM_IT_Update);            //清除定时器7溢出中断标志位
}

/***************************************************************************
函数名称：DMA2_Stream0_IRQHandler
函数功能：用于检测按键
函数备注：
***************************************************************************/
void DMA2_Stream0_IRQHandler(void)
{
  if(DMA_GetITStatus(DMA2_Stream0,DMA_IT_TCIF0) == SET)
	{
      Value = ADC1ConvertedValue[0] * 330 / 0xFFF;
    	DMA_ClearITPendingBit(DMA2_Stream0,DMA_IT_TCIF0);    //清除DMA2通道0传输完成标志位
	}

	else if(DMA_GetITStatus(DMA2_Stream0,DMA_IT_HTIF0) == SET)
	{


    	DMA_ClearITPendingBit(DMA2_Stream0,DMA_IT_HTIF0);    //清除DMA2通道0传输完成标志位
	}
	
	else if(DMA_GetITStatus(DMA2_Stream0,DMA_IT_TEIF0) == SET)
	{


    	DMA_ClearITPendingBit(DMA2_Stream0,DMA_IT_TEIF0);    //清除DMA2通道0传输完成标志位
	}
}


/***************************************************************************
函数名称：DMA2_Stream0_IRQHandler
函数功能：用于检测按键
函数备注：
***************************************************************************/
void USART6_IRQHandler(void)
{
	   if(USART_GetITStatus(USART6, USART_IT_RXNE) != RESET)  //接收中断有效,若接收数据寄存器满
     {
      _Temp[counter] = USART_ReceiveData(USART6);   //接收数据
      //网购给的程序
      //if(counter == 0 && Re_buf[0] != 0x55) return;      //第 0 号数据不是帧头，跳过
	  if(counter == 0 && _Temp[0] != 0x55) return;      //第 0 号数据不是帧头，跳过
      counter++; 
      if(counter==11) //接收到 11 个数据
      { 
         memcpy(Re_buf,_Temp,11);
         counter=0; //重新赋值，准备下一帧数据的接收
         sign=1;
      }    
   }
}


#endif  //__IRQ_CELL_ENABLE

