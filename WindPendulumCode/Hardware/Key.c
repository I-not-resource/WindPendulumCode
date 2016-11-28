#include "Key.h"
#include "display.h"
#include "stdlib.h"
#include "run.h"
#if __KEY_ENABLE

u8 Key       = 11,
   _KeyRead  = 0,
	 mc        = 0,
	 Mode      = 0,
	 side      = 0;
u8 MoveLive  = 50,
   MoveLiveB = 50,
   MEMU_SURE = 0,
	 BACK_ESC  = 0;
/***************************************************************************
�������ƣ�Key_Config
�������ܣ�����������ų�ʼ��
������ע��PA0��PA8
***************************************************************************/
void Key_Config(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOF,ENABLE);
	
	GPIO_InitStructure.GPIO_Pin      = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Speed    = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType    = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd     = GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_Mode     = GPIO_Mode_OUT;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	POW_KEY = OFF;
	
	GPIO_InitStructure.GPIO_Pin      = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Speed    = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd     = GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_Mode     = GPIO_Mode_IN;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin      = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Speed    = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType    = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd     = GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_Mode     = GPIO_Mode_OUT;
	GPIO_Init(GPIOF,&GPIO_InitStructure);
	
	BEEP = 1;
}


/***************************************************************************
�������ƣ�Keyopt
�������ܣ�����ѡ��,���жϺ���DMA2_Stream0_IRQHandler����ʹ��
         ������Ϊ10
������ע��
***************************************************************************/
void Keyopt(u16 _k)
{
	u8 _key;
	_k  += KEYAREA;
	_key = _k / KEYNUM;
	if((_KeyRead == _key)&&(Key != _key))
	  mc++;
	else
	{
    _KeyRead = 0;
		mc = 0;
	}
	if(mc == 3)
	{
     Key = _key;
		 mc  =0;
	}
	
	_KeyRead = _key;
}

/***************************************************************************
�������ƣ�Fuction_choose
�������ܣ�����ѡ��
������ע��
***************************************************************************/
void Fuction_choose(void)
{
  switch(Key)
	{
    case 0:{if((Mode == 1)&&(BACK_ESC == 0))
	          BACK_ESC  = 1;
	          else Mode = 0;}side = 0;break;
		case 1:{if((Mode == 1)&&(MEMU_SURE == 0))
	          MEMU_SURE = 1;
	          else Mode = 1;}side = 1;break;
		case 2:{if((Mode == 0) && (side == 0)) 
	          Improve4_Function();
	           else MoveLive--;}side = 1;break;
	  case 3:{if((Mode == 0) && (side == 0)) 
	          Improve3_Function();
	          else MoveLiveB--;}side = 1;break;
		case 4:{if((Mode == 0) && (side == 0))
	         Improve2_Function();
	         else MoveLive--;}side = 1;break;
		case 5:{if((Mode == 0) && (side == 0)) 
	         {Improve1_Function();Mode = 5;}
	         else MoveLiveB++;}side = 1;break;
		case 6:if(side == 0)
	         { Base4_Fuction();Mode = 4;}side = 1;
	         break;
		case 7:if(side == 0)
	         {Base3_Fuction();Mode = 3;}side = 1;break;
		case 8:{if((Mode == 0) && (side == 0))
           {Base2_Fuction();Mode = 2;}else MoveLive++;}side = 1;break;
		case 9:if(side == 0)
	         {Base1_Fuction();Mode = 1;}side = 1;break;
		default:break;
	}
	
}

/***************************************************************************
�������ƣ�Fuction_choose
�������ܣ�����ѡ��
������ע��
***************************************************************************/











#endif  //__KEY_ENABLE
