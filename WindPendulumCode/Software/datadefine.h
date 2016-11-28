#ifndef __DATADEFINE_H_
#define __DATADEFINE_H_
#include "Sys.h"

#define PID_ERR    200

#define PID_LEVEL  100
#define X           0
#define Y           1

#define MOTOR_Y_P TIM4->CCR2 
#define MOTOR_Y_N TIM4->CCR4
#define MOTOR_X_P TIM4->CCR1 
#define MOTOR_X_N TIM4->CCR3


extern u8 Re_buf[11],temp_buf[11],counter;
extern u8 sign;
extern float a[3],w[3],angle[3],T;
extern u8 flag_Err;
extern float Err_x,Err_y,Err_wx,Err_wy;                       //MPU6050ŒÛ≤Óº∆À„
extern u8 RUN_Mode;


#endif  


