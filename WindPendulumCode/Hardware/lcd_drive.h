#ifndef __LCD_DRIVE_H_
#define __LCD_DRIVE_H_
#include "Sys.h"
#if     __LCD_ENABLE 
////////////////////////////////////////////////////////////////////////////
//���棡����
//��Ӧ�ó���רΪ5510�Ż����
//��˲��߱����õļ�����
////////////////////////////////////////////////////////////////////////////

//LCD�Ļ�����ɫ�ͱ���ɫ	   
extern u16  POINT_COLOR;//Ĭ�Ϻ�ɫ    
extern u16  BACK_COLOR; //������ɫ.Ĭ��Ϊ��ɫ

//////////////////////////
/******LCD��ɫ���ձ�******/
//////////////////////////
#define RED       0XF800  //��
#define ORANGE    0XFBF7  //��
#define YELLOW    0XFFE0  //��
#define GREEN     0X07E0  //��
#define TURQUOISE 0X07FF  //��
#define BLUE      0X001F  //��
#define PURPLE    0X8393  //��
#define DPURPLE   0X7C8F  //����
#define WHITE     0XFFFF  //��
#define BLACK     0X0000  //��
#define LGRAY     0XBFF7  //ǳ��
#define DGRAY     0X7FFF  //���
#define GRAY      0X76BE  //��
#define PINK      0XF81F  //�ۺ�
#define BROWN     0X7DF0  //��
#define GGREEN    0X41FF  //����
#define FGREEN    0X7FF0  //����
#define DBLUE     0X0013  //����
    



//////////////////////////
/******LCD�����ṹ��******/
//////////////////////////
typedef struct
{
	
	u16 LCD_REG;
	u16 LCD_RAM; 
	
}LCD_TypeDef;
#define LCD_BASE       ((u32)(0X6C000000 | 0X0000007E))
#define LCD            ((LCD_TypeDef*)LCD_BASE)

//////////////////////////
/******LCD��Ҫ������******/
//////////////////////////
typedef struct
{

	u16 width;                   //LCD ���
	u16 height;                  //LCD �߶�
	u16 id;                      //LCD ID
	u8  dir;                     //0������  1������
	u16 wramcmd;                 //��ʼдgramָ��
	u16 setxcmd;                 //����x����ָ��
	u16 setycmd;                 //����y����ָ��
	
}_lcd_dev;
extern _lcd_dev lcddev;


//////////////////////////
/*****LCDɨ�跽����*****/
//////////////////////////
#define L2R_U2D  0             //������,���ϵ���
#define L2R_D2U  1             //������,���µ���
#define R2L_U2D  2             //���ҵ���,���ϵ���
#define R2L_D2U  3             //���ҵ���,���µ���

#define U2D_L2R  4             //���ϵ���,������
#define U2D_R2L  5             //���ϵ���,���ҵ���
#define D2U_L2R  6             //���µ���,������
#define D2U_R2L  7             //���µ���,���ҵ���	

#define DFT_SCAN_DIR  L2R_U2D  //Ĭ�ϵ�ɨ�跽��

#define	LCD_LED PBout(11)  		//LCD����    		 PB15 

void LCD_Init(void);                                                    //��ʼ��
void LCD_DisplayOn(void);                                               //����ʾ
void LCD_DisplayOff(void);                                              //����ʾ
void LCD_Clear(void);                                                   //����
void LCD_SetCursor(u16 Xpos, u16 Ypos);                                 //���ù��
void LCD_DrawPoint(u16 x,u16 y);                                        //����
void LCD_Fast_DrawPoint(u16 x,u16 y,u16 color);	                        //���ٻ���
u16  LCD_ReadPoint(u16 x,u16 y);                                        //���� 

void LCD_WriteReg(u16 LCD_Reg, u16 LCD_RegValue);
u16  LCD_ReadReg(u16 LCD_Reg);
void LCD_WriteRAM_Prepare(void);
void LCD_WriteRAM(u16 RGB_Code);		  
void LCD_Scan_Dir(u8 dir);							                                //������ɨ�跽��
void LCD_Display_Dir(u8 dir);						                                //������Ļ��ʾ����
void LCD_Set_Window(u16 sx,u16 sy,u16 width,u16 height);                //���ô���	


#endif   //__LCD_ENABLE

#endif
