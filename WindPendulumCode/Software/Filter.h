#ifndef _Filter_h_
#define _Filter_h_

extern float err_x;
extern float Angle;
extern float _Angle_ax;

void Err_Gry(void);
void Kalman_Filter(float Accel,float Gyro);
void Angle_Calcu(void) ;
float Read_ACCEL(void);
float Read_Gry(void);
#endif
