#ifndef __MPU9150_H
#define __MPU9150_H

#include "stm32f4xx.h"

#define MPUADDR 0x68

extern u8 Acc_CALIBRATED;
extern u8 Gyro_CALIBRATED;

void Init_Mpu9150_Sensor_Att(void);
void init_mpu9150(void);
void Calibra_Mag(void);
void Calibra_GyrAcc(void);
void Calibra_Att(void);

#endif
