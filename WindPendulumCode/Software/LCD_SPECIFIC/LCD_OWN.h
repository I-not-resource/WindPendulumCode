#ifndef __LCD_OWN_H_
#define __LCD_OWN_H_
#if __SPECIALLY_ENABLE
#include "sys.h"

void LCD_Draw_Circle(u16 x0,u16 y0,u8 r);                               //画圆
void LCD_DrawLine(u16 x1, u16 y1, u16 x2, u16 y2);                      //画线
void LCD_DrawRectangle(u16 x1, u16 y1, u16 x2, u16 y2);                 //画矩形
void LCD_Fill(u16 sx,u16 sy,u16 ex,u16 ey,u16 color);                   //填充单色
void LCD_Color_Fill(u16 sx,u16 sy,u16 ex,u16 ey,u16 *color);            //填充指定颜色
void LCD_ShowChar(u16 x,u16 y,u8 num,u8 size,u8 mode);					        //显示一个字符
void LCD_ShowNum(u16 x,u16 y,u32 num,u8 len,u8 size);  						      //显示一个数字
void LCD_ShowxNum(u16 x,u16 y,u32 num,u8 len,u8 size,u8 mode);				  //显示 数字

void LCD_ShowWord(u16 x,u16 y,u8 size,u8 *font,u8 mode);
void LCD_Location_Med(u16 y,u8 size,u8 *font);
void LCD_Fonts_Med(u16 y,u8 size,u8 fon[][72]);
void LCD_ShowFonts(u16 x,u16 y,u8 font[][72],u8 num,u8 mode);

#endif   //__SPECIALLY_ENABLE

#endif   //__LCD_OWN_H_


