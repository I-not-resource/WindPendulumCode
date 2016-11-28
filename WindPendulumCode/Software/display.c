#include "display.h"
#include "DMA.h"
#if __DISPLAY_ENABLE
  u8 flag = 0;
/***************************************************************************
函数名称：First_Show
函数功能：首页显示
函数备注：
***************************************************************************/
void First_Show(void)
{
  LCD_Fonts_Med(300,11,_a);
	LCD_Fonts_Med(480,2,b);
	LCD_Fonts_Med(505,7,c);
}

/***************************************************************************
函数名称：Second_Show
函数功能：第二页页显示
函数备注：
***************************************************************************/
void Second_Show(void)
{
  LCD_Clear();
	LCD_Fonts_Med(100,11,_a);
	LCD_Fonts_Med(130,2,b);
	POINT_COLOR = TURQUOISE;
  LCD_ShowWord(20,300,24,"Base Function 1",0);
	LCD_ShowWord(20,348,24,"Base Function 2",0);
	LCD_ShowWord(20,396,24,"Base Function 3",0);
	LCD_ShowWord(20,444,24,"Base Function 4",0);
  POINT_COLOR = GGREEN;
  LCD_ShowWord(254,300,24,"Improve Function 1",0);
	LCD_ShowWord(254,348,24,"Improve Function 2",0);
	LCD_ShowWord(254,396,24,"Improve Function 3",0);
	LCD_ShowWord(254,444,24,"Improve Function 4",0);
	POINT_COLOR = GREEN;
	LCD_Location_Med(700,24,"MEMU");
	POINT_COLOR = GREEN;
}

/***************************************************************************
函数名称：MEMU_Show
函数功能：菜单页页显示
函数备注：
***************************************************************************/
void MEMU_Show(void)
{
  LCD_Clear();
	POINT_COLOR = ORANGE;
	LCD_Location_Med(30,24,"MEMU");
	LCD_ShowWord(360,30,24,"T:     C",0);
	LCD_DrawRectangle(24, 60, 456, 776);
	POINT_COLOR = WHITE;
	LCD_ShowWord(36,72,24,"Debug    |",0);
	LCD_ShowWord(36,96,24,"NRF24L01 |",0);
	POINT_COLOR = GREEN;
}

/***************************************************************************
函数名称：BF1_Show
函数功能：基础功能1页显示
函数备注：
***************************************************************************/
void BF1_Show(void)
{
  LCD_Clear();
	POINT_COLOR = TURQUOISE;
	LCD_Location_Med(30,24,"Base Function 1");
	LCD_DrawRectangle(24, 60, 456, 776);
	POINT_COLOR = YELLOW;
	LCD_ShowWord(36,96,24,"On the ground to draw alengt",0);
	LCD_ShowWord(36,120,24,"shorter than 50cm of line segments,",0);
	LCD_ShowWord(36,144,24,"error 2.5cm, and with good reproducibility.",0);
	LCD_ShowWord(36,200,24,"Status :",0);
	LCD_ShowWord(36,224,24,"Angle  :",0);
	LCD_ShowWord(36,248,24,"length :",0);
	POINT_COLOR = GREEN;
}
/***************************************************************************
函数名称：BF2_Show
函数功能：基础功能2页显示
函数备注：
***************************************************************************/
void BF2_Show(void)
{
  LCD_Clear();
	POINT_COLOR = TURQUOISE;
	LCD_Location_Med(30,24,"Base Function 2");
	LCD_DrawRectangle(24, 60, 456, 776);
	POINT_COLOR = YELLOW;
	LCD_ShowWord(36,96,24,"Shown in the 30-60cm length can be",0);
	LCD_ShowWord(36,120,24,"set,",0);
	LCD_ShowWord(36,144,24,"error 2.5cm, and with good reproducibility.",0);
	LCD_ShowWord(36,200,24,"Status       :",0);
	LCD_ShowWord(36,224,24,"goal length  :",0);
	LCD_ShowWord(36,248,24,"Angle        :",0);
	LCD_ShowWord(36,272,24,"length       :",0);
	POINT_COLOR = GREEN;
}

/***************************************************************************
函数名称：BF3_Show
函数功能：基础功能3页显示
函数备注：
***************************************************************************/
void BF3_Show(void)
{
  LCD_Clear();
	POINT_COLOR = TURQUOISE;
	LCD_Location_Med(30,24,"Base Function 3");
	LCD_DrawRectangle(24, 60, 456, 776);
	POINT_COLOR = YELLOW;
	LCD_ShowWord(36,96,24,"Set the direction of the swing,",0);
	LCD_ShowWord(36,120,24,"draw a straight line not less than 20cm.",0);
	LCD_ShowWord(36,200,24,"Status         :",0);
	LCD_ShowWord(36,224,24,"goal direction :",0);
	LCD_ShowWord(36,248,24,"Angle          :",0);
	LCD_ShowWord(36,272,24,"length         :",0);
	POINT_COLOR = GREEN;
}

/***************************************************************************
函数名称：BF4_Show
函数功能：基础功能4页显示
函数备注：
***************************************************************************/
void BF4_Show(void)
{
  LCD_Clear();
	POINT_COLOR = TURQUOISE;
	LCD_Location_Med(30,24,"Base Function 4");
	LCD_DrawRectangle(24, 60, 456, 776);
	POINT_COLOR = YELLOW;
	LCD_ShowWord(36,96,24,"The wind pulled at an angle swing",0);
	LCD_ShowWord(36,120,24,"(30-45) open within 5 seconds",0);
	LCD_ShowWord(36,144,24,"so that the wind swing brake",0);
	LCD_ShowWord(36,200,24,"Status         :",0);
	LCD_ShowWord(36,248,24,"Angle          :",0);
	LCD_ShowWord(36,272,24,"length         :",0);
	POINT_COLOR = GREEN;
}

/***************************************************************************
函数名称：IF1_Show
函数功能：提升功能1页显示
函数备注：
***************************************************************************/
void IF1_Show(void)
{
  LCD_Clear();
  POINT_COLOR = GGREEN;
  LCD_Location_Med(30,24,"Improve Function 1");
	LCD_DrawRectangle(24, 60, 456, 776);
	POINT_COLOR = YELLOW;
	LCD_ShowWord(36,96,24,"Set radius 15-35cm round, laser",0);
	LCD_ShowWord(36,120,24,"pointer to draw the track, repeated",0);
	LCD_ShowWord(36,144,24,"three times within 30 seconds,",0);
	LCD_ShowWord(36,168,24,"error 2.5cm.",0);
	LCD_ShowWord(36,200,24,"Status         :",0);
	LCD_ShowWord(36,248,24,"Set Radius     :",0);
	LCD_ShowWord(36,272,24,"Radius         :",0);
	LCD_Draw_Circle(240,600,150);
	POINT_COLOR = GREEN;
}

/***************************************************************************
函数名称：IF2_Show
函数功能：提升功能2页显示
函数备注：
***************************************************************************/
void IF2_Show(void)
{
  LCD_Clear();
  POINT_COLOR = GGREEN;
  LCD_Location_Med(30,24,"Improve Function 2");
	LCD_DrawRectangle(24, 60, 456, 776);
	POINT_COLOR = YELLOW;
	LCD_ShowWord(36,96,24,"Set radius 15-35cm round, laser",0);
	LCD_ShowWord(36,120,24,"pointer to draw the track, repeated",0);
	LCD_ShowWord(36,144,24,"three times within 30 seconds,",0);
	LCD_ShowWord(36,168,24,"error 2.5cm.",0);
	LCD_ShowWord(36,200,24,"Status         :",0);
	LCD_ShowWord(36,248,24,"Set Radius     :",0);
	LCD_ShowWord(36,272,24,"Radius         :",0);
	LCD_Draw_Circle(240,600,150);
	POINT_COLOR = GREEN;
}

/***************************************************************************
函数名称：IF3_Show
函数功能：提升功能3页显示
函数备注：
***************************************************************************/
void IF3_Show(void)
{
  LCD_Clear();
  POINT_COLOR = GGREEN;
  LCD_Location_Med(30,24,"Improve Function 3");
	LCD_DrawRectangle(24, 60, 456, 776);
	POINT_COLOR = GREEN;
}

/***************************************************************************
函数名称：IF4_Show
函数功能：提升功能4页显示
函数备注：
***************************************************************************/
void IF4_Show(void)
{
  LCD_Clear();
  POINT_COLOR = GGREEN;
  LCD_Location_Med(30,24,"Improve Function 4");
	LCD_DrawRectangle(24, 60, 456, 776);
	POINT_COLOR = GREEN;
}








#endif  //__DISPLAY_ENABLE
