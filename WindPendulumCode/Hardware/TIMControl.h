#ifndef __TIMCONTROL_H_
#define __TIMCONTROL_H_
#include "Sys.h"
#if __TIM_ENABLE
#define T5ARR 1000
#define T5PSC 42
#define T7ARR 5000
#define T7PSC 84
#define T9ARR 4000
#define T9PSC 840
#define T11ARR 4000
#define T11PSC 840

void TIM5Line_Init(void);
void TIM9Line_Init(void);
void TIM7Line_Init(void);
void TIM11Line_Init(void);


#endif  //__TIM_ENABLE
#endif  //__TIMCONTROL_H_
