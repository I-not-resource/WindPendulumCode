#include "datadefine.h"


///////////////////////////////////////////////////
//MPU6050
u8 Re_buf[11],temp_buf[11],counter=0;
u8 sign;
float a[3],w[3],angle[3],T;
u8 flag_Err=0;
float Err_x=0,Err_y=0,Err_wx=0,Err_wy=0;                       //MPU6050ŒÛ≤Óº∆À„

u8 RUN_Mode = 0;




