#ifndef __RUN_H_
#define __RUN_H_
#include "sys.h"
#if __RUN_ENABLE

#define _gan 88
#define BF3_DLong 15


void BF1_Init(void);
void BF1_End(void);
void Base1(void);

void BF2_Init(void);
void BF2_End(void);
void Base2(void);

void BF3_Init(void);
void BF3_End(void);
void Base3(void);

void BF4_Init(void);
void BF4_End(void);
void Base4(void);

void IF1_Init(void);
void IF1_End(void);
void Improve(void);

void Mode_BF1(void);
void Mode_BF2(void);
void Mode_BF3(void);
void Mode_BF4(void);

void Mode_IF1(void);
void Mode_IF2(void);

void MPU6050_data(void);
void Up_Mode(void);
void Err_Deal(void);

void Input2(void);
void Input3(void);
void Input4(void);

#endif
#endif
