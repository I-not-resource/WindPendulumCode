#include "includes.h"



/***************************************************************************
函数名称：main
函数功能：程序的起点和终点
函数备注：
***************************************************************************/
int main(void)
{
/*------------------------------变量初始化层------------------------------*/

/*------------------------------系统初始化层------------------------------*/	
  SysTick_Init();
  Key_Config();
	POW_KEY = ON;
	LCD_Init();
//	ADCKey_Init();
//	ADC_DMA_Config();
	
	PWMx4_Init();	
	TIM7Line_Init();
 
/*------------------------------应用初始化层------------------------------*/ 


/*------------------------------控制初始化层------------------------------*/
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	ADC_SoftwareStartConv(ADC1);

/*------------------------------程序开始运行------------------------------*/
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
