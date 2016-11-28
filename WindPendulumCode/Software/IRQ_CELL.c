#include "IRQ_CELL.h"
#include "includes.h"
#include "string.h"
#if __IRQ_CELL_ENABLE

u16 Value = 0;
u8  i     = 0;
int Tem   = 0;
static u8 _Temp[11];
/***************************************************************************
�������ƣ�TIM7_IRQHandler
�������ܣ�
������ע��
***************************************************************************/
void TIM7_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM7,TIM_IT_Update) == SET)
	{
		Keyopt(Value);
    Fuction_choose();
	}
	
	TIM_ClearITPendingBit(TIM7,TIM_IT_Update);            //�����ʱ��7����жϱ�־λ
}

/***************************************************************************
�������ƣ�TIM1_BRK_TIM9_IRQHandler
�������ܣ�1�����ڸ��¸��ַ�ʵʱ��״̬
������ע��
***************************************************************************/
void TIM1_BRK_TIM9_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM9,TIM_IT_Update) == SET)
	{

	}
	
	TIM_ClearITPendingBit(TIM9,TIM_IT_Update);            //�����ʱ��7����жϱ�־λ
}


/***************************************************************************
�������ƣ�TIM5_IRQHandler
�������ܣ�
������ע��
***************************************************************************/
void TIM5_IRQHandler(void)
{

	if(TIM_GetITStatus(TIM5,TIM_IT_Update) == SET)
	{

	}
	
	TIM_ClearITPendingBit(TIM5,TIM_IT_Update);            //�����ʱ��7����жϱ�־λ
}

/***************************************************************************
�������ƣ�DMA2_Stream0_IRQHandler
�������ܣ����ڼ�ⰴ��
������ע��
***************************************************************************/
void DMA2_Stream0_IRQHandler(void)
{
  if(DMA_GetITStatus(DMA2_Stream0,DMA_IT_TCIF0) == SET)
	{
      Value = ADC1ConvertedValue[0] * 330 / 0xFFF;
    	DMA_ClearITPendingBit(DMA2_Stream0,DMA_IT_TCIF0);    //���DMA2ͨ��0������ɱ�־λ
	}

	else if(DMA_GetITStatus(DMA2_Stream0,DMA_IT_HTIF0) == SET)
	{


    	DMA_ClearITPendingBit(DMA2_Stream0,DMA_IT_HTIF0);    //���DMA2ͨ��0������ɱ�־λ
	}
	
	else if(DMA_GetITStatus(DMA2_Stream0,DMA_IT_TEIF0) == SET)
	{


    	DMA_ClearITPendingBit(DMA2_Stream0,DMA_IT_TEIF0);    //���DMA2ͨ��0������ɱ�־λ
	}
}


/***************************************************************************
�������ƣ�DMA2_Stream0_IRQHandler
�������ܣ����ڼ�ⰴ��
������ע��
***************************************************************************/
void USART6_IRQHandler(void)
{
	   if(USART_GetITStatus(USART6, USART_IT_RXNE) != RESET)  //�����ж���Ч,���������ݼĴ�����
     {
      _Temp[counter] = USART_ReceiveData(USART6);   //��������
      //�������ĳ���
      //if(counter == 0 && Re_buf[0] != 0x55) return;      //�� 0 �����ݲ���֡ͷ������
	  if(counter == 0 && _Temp[0] != 0x55) return;      //�� 0 �����ݲ���֡ͷ������
      counter++; 
      if(counter==11) //���յ� 11 ������
      { 
         memcpy(Re_buf,_Temp,11);
         counter=0; //���¸�ֵ��׼����һ֡���ݵĽ���
         sign=1;
      }    
   }
}


#endif  //__IRQ_CELL_ENABLE

