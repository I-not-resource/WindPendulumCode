#ifndef __PWM_H_
#define __PWM_H_
#include "Sys.h"
#if __PWM_ENABLE

#define KEY0 		PEin(4)   	//PE4
#define KEY1 		PEin(3)		  //PE3 
#define KEY2 		PEin(2)		  //P32
#define WK_UP 	PAin(0)		  //PA0



void PWMx4_Init(void);
void Control_Init(void);



#endif   //__PWM_ENABLE

#endif   //__PWM_H_
