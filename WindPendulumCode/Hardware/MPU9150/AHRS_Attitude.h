#ifndef __AHRS_Attitude_H
#define __AHRS_Attitude_H

#include "stm32f4xx.h"

typedef struct
{
	float X;
	float Y;
	float Z;
} Vector_3f;

typedef struct
{
	double X;
	double Y;
	double Z;
} Vector_3d;

extern float Gyro_Xout_Offset, Gyro_Yout_Offset, Gyro_Zout_Offset;
extern float Accel_Xout_Offset, Accel_Yout_Offset, Accel_Zout_Offset;
extern float Att_Roll_Offset, Att_Pitch_Offset, Att_Yaw_Offset;

extern Vector_3f Gyro_ADC;
extern Vector_3f Acc_ADC;
extern Vector_3f Mag_ADC;
extern Vector_3f Imu;

/*滤波后的数据*/
extern Vector_3f Acc_lpf, Mag_lpf;

extern Vector_3f Last_Gyro_ADC;

#define MPU9150_Addr            0x68
#define P_M_1                   0x6B
#define Accel_FILTER_98HZ		      98
#define Accel_Xout_H		        0x3B
#define Gyro_Xout_H		          0x43
#define Bypass_Enable_Cfg		    0x37
#define User_Ctrl               0x6A
#define Compass_Addr            0x0C
#define Compass_CNTL            0x0A
#define Compass_ST1             0x02
#define	Compass_HXL             0x03

/*
  是否启动姿态、陀螺仪、加速度校准。
  1：启动
	0：禁止
	void get_Attitude_bias(void);
  int get_gyro_bias(void);
  int get_accel_bias(void);
*/
//#define ATTITUDE_BIAS 0
//#define GYRO_BIAS     0
//#define ACCEL_BIAS    0

extern float Gd32_Pitch, Gd32_Roll, Gd32_Yaw;
extern float Pitch, Roll, Yaw;
extern float halfT;
extern float init_ax, init_ay, init_az, init_gx, init_gy, init_gz, init_mx, init_my, init_mz;
extern float MXgain;
extern float MYgain;
extern float MZgain;
extern float MXoffset;
extern float MYoffset;
extern float MZoffset;
extern float heading;
extern float MPU6050_GYRO_LAST_X, MPU6050_GYRO_LAST_Y, MPU6050_GYRO_LAST_Z;
extern float MPU6050_ACC_LAST_X, MPU6050_ACC_LAST_Y, MPU6050_ACC_LAST_Z;


void get_mpu9150_data(void);
Vector_3f get_mpugyro_data(void);
Vector_3f get_mpuacc_data(void);
void init_quaternion(void);
void AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
float invSqrt(float x);
void Get_Attitude(void);
void get_Attitude_bias(void);
int get_gyro_bias(void);
int get_accel_bias(void);
void get_compass_bias(void);
void compass_calibration(void);
void Read_MPU9150_Mag(void);
int Init_MPU9150_Mag(void);

#endif
