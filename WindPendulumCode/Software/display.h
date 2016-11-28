#ifndef __DISPLAY_H_
#define __DISPLAY_H_
#include "Sys.h"
#include "lcd_drive.h"
#include "LCD_OWN.h"
#include "Fonts.h"
#if __DISPLAY_ENABLE

extern u8 flag;

void First_Show(void);
void Second_Show(void);
void MEMU_Show(void);
void BF1_Show(void);
void BF2_Show(void);
void BF3_Show(void);
void BF4_Show(void);
void IF1_Show(void);
void IF2_Show(void);
void IF3_Show(void);
void IF4_Show(void);
void Temperate_Show(int _Temperate);



#endif  //__DISPLAY_ENABLE
#endif  //__DISPLAY_H
