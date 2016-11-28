#ifndef __LCD_DRIVE_H_
#define __LCD_DRIVE_H_
#include "Sys.h"
#if     __LCD_ENABLE 
////////////////////////////////////////////////////////////////////////////
//警告！！！
//此应用程序，专为5510优化设计
//因此不具备良好的兼容性
////////////////////////////////////////////////////////////////////////////

//LCD的画笔颜色和背景色	   
extern u16  POINT_COLOR;//默认红色    
extern u16  BACK_COLOR; //背景颜色.默认为白色

//////////////////////////
/******LCD颜色对照表******/
//////////////////////////
#define RED       0XF800  //红
#define ORANGE    0XFBF7  //橙
#define YELLOW    0XFFE0  //黄
#define GREEN     0X07E0  //绿
#define TURQUOISE 0X07FF  //青
#define BLUE      0X001F  //蓝
#define PURPLE    0X8393  //紫
#define DPURPLE   0X7C8F  //深紫
#define WHITE     0XFFFF  //白
#define BLACK     0X0000  //黑
#define LGRAY     0XBFF7  //浅灰
#define DGRAY     0X7FFF  //深灰
#define GRAY      0X76BE  //灰
#define PINK      0XF81F  //粉红
#define BROWN     0X7DF0  //棕
#define GGREEN    0X41FF  //草绿
#define FGREEN    0X7FF0  //固绿
#define DBLUE     0X0013  //深蓝
    



//////////////////////////
/******LCD操作结构体******/
//////////////////////////
typedef struct
{
	
	u16 LCD_REG;
	u16 LCD_RAM; 
	
}LCD_TypeDef;
#define LCD_BASE       ((u32)(0X6C000000 | 0X0000007E))
#define LCD            ((LCD_TypeDef*)LCD_BASE)

//////////////////////////
/******LCD重要参数集******/
//////////////////////////
typedef struct
{

	u16 width;                   //LCD 宽度
	u16 height;                  //LCD 高度
	u16 id;                      //LCD ID
	u8  dir;                     //0：竖屏  1：横屏
	u16 wramcmd;                 //开始写gram指令
	u16 setxcmd;                 //设置x坐标指令
	u16 setycmd;                 //设置y坐标指令
	
}_lcd_dev;
extern _lcd_dev lcddev;


//////////////////////////
/*****LCD扫描方向定义*****/
//////////////////////////
#define L2R_U2D  0             //从左到右,从上到下
#define L2R_D2U  1             //从左到右,从下到上
#define R2L_U2D  2             //从右到左,从上到下
#define R2L_D2U  3             //从右到左,从下到上

#define U2D_L2R  4             //从上到下,从左到右
#define U2D_R2L  5             //从上到下,从右到左
#define D2U_L2R  6             //从下到上,从左到右
#define D2U_R2L  7             //从下到上,从右到左	

#define DFT_SCAN_DIR  L2R_U2D  //默认的扫描方向

#define	LCD_LED PBout(11)  		//LCD背光    		 PB15 

void LCD_Init(void);                                                    //初始化
void LCD_DisplayOn(void);                                               //开显示
void LCD_DisplayOff(void);                                              //关显示
void LCD_Clear(void);                                                   //清屏
void LCD_SetCursor(u16 Xpos, u16 Ypos);                                 //设置光标
void LCD_DrawPoint(u16 x,u16 y);                                        //画点
void LCD_Fast_DrawPoint(u16 x,u16 y,u16 color);	                        //快速画点
u16  LCD_ReadPoint(u16 x,u16 y);                                        //读点 

void LCD_WriteReg(u16 LCD_Reg, u16 LCD_RegValue);
u16  LCD_ReadReg(u16 LCD_Reg);
void LCD_WriteRAM_Prepare(void);
void LCD_WriteRAM(u16 RGB_Code);		  
void LCD_Scan_Dir(u8 dir);							                                //设置屏扫描方向
void LCD_Display_Dir(u8 dir);						                                //设置屏幕显示方向
void LCD_Set_Window(u16 sx,u16 sy,u16 width,u16 height);                //设置窗口	


#endif   //__LCD_ENABLE

#endif
