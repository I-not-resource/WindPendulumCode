#ifndef __Ort_encoding_H__
#define __Ort_encoding_H__
#include "sys.h"
#if     __ORT_ENABLE

#define ENCODER_TIM_PERIOD 65535
#define MAX_COUNT          4000


void ORT0_ENC_Init(void);
void ORT1_ENC_Init(void);

s16 ORT0_ENC_GetData(void);
s16 ORT1_ENC_GetData(void);

#endif  //__ORT_ENABLE

#endif  //__Ort_encoding_H__
