#include "includes.h"



/***************************************************************************
�������ƣ�main
�������ܣ�����������յ�
������ע��
***************************************************************************/
int main(void)
{
/*------------------------------������ʼ����------------------------------*/

/*------------------------------ϵͳ��ʼ����------------------------------*/	
  SysTick_Init();
  Key_Config();
	POW_KEY = ON;
	LCD_Init();
//	ADCKey_Init();
//	ADC_DMA_Config();
	
	PWMx4_Init();	
	TIM7Line_Init();
 
/*------------------------------Ӧ�ó�ʼ����------------------------------*/ 


/*------------------------------���Ƴ�ʼ����------------------------------*/
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	ADC_SoftwareStartConv(ADC1);

/*------------------------------����ʼ����------------------------------*/
  First_Show();
	Delay_ms(500);
	Second_Show();
  while(1)
  {
	if(BACK_ESC == 1)
	{
		Second_Show();
		BACK_ESC = 0;
		}
	while(Mode == 1)
	{
		Base1();
		Mode     = 0;
	  side     = 0;
	  BACK_ESC = 1;
	}
	while(Mode == 2)
	{	Base2();
		Mode     = 0;
	  side     = 0;
	  BACK_ESC = 1;
		MEMU_SURE = 0;
	}
	while(Mode == 3)
	{Base3();
		Mode     = 0;
	  side     = 0;
	  BACK_ESC = 1;
		MEMU_SURE = 0;
	}
	while(Mode == 4)
	{Base4();
		Mode     = 0;
	  side     = 0;
	  BACK_ESC = 1;
		MEMU_SURE = 0;
	}
	while(Mode == 5)
	{Improve();
		Mode     = 0;
	  side     = 0;
	  BACK_ESC = 1;
		MEMU_SURE = 0;
	}
  
	Delay_ms(500);
 
}
} 
