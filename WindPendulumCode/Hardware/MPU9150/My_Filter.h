#ifndef __MY_FILTER_H_
#define __MY_FILTER_H_


#include "AHRS_Attitude.h"

#define AHRS_LOOP_TIME					2000	//单位为uS
#define ACC_LPF_CUT 30.0f		//加速度低通滤波器截止频率30Hz
#define MAG_LPF_CUT 10.0f		//磁力计低通滤波器截止频率10Hz
#define GYRO_CF_TAU 1.5f
#define MAG_CF_TAU 	1.0f

#define ACC_1G 			4096//8192		//由加速度计的量程确定

#define FILTER_NUM 	20

typedef struct
{
	Vector_3f last; //上一时刻的最优结果
	Vector_3f mid;  //当前时刻的预测结果
	Vector_3f now;  //当前时刻的最优结果
	Vector_3f P_mid;  //当前时刻预测结果的协方差
	Vector_3f P_now;  //当前时刻最优结果的协方差
	Vector_3f P_last; //上一时刻最优结果的协方差
	Vector_3f kg;     //卡尔曼增益
	Vector_3f Kalman_Q;  //系统噪声
	Vector_3f R;       //测量噪声
}Kalman1nd;	

typedef struct
{
	float b0;
	float a1;
	float a2;
	Vector_3f preout;
	Vector_3f lastout;
}LPF2ndData_t;	

typedef struct
{
	float ax;
	float ay;
	float az;
	
	float bx;
	float by;
	float bz;
	
	float cx;
	float cy;
	float cz;
}Matrix3f;	

typedef struct
{
	float q0;
	float q1;
	float q2;	
	float q3;
}Quater;	

extern float acc_lpf;	
extern float mag_lpf;
extern float gyro_cf;		
extern float mag_cf;

extern LPF2ndData_t Acc_lpf_2nd;
extern Vector_3f angle;
extern Vector_3f Acc_AVG;

extern Vector_3f Acc_lpf, Mag_lpf; 


/*----------------------滤波器初始化-------------------------*/
void My_Filter_Init(void);

/*----------------------一阶低通滤波器系数计算-------------------------*/
float LPF_1st_Factor_Cal(float deltaT, float Fcut);

/*----------------------一阶低通滤波器------------------------*/
Vector_3f LPF_1st(Vector_3f oldData, Vector_3f newData, float lpf_factor);

/*----------------------二阶低通滤波器系数计算-------------------------*/
void LPF_2nd_Factor_Cal(LPF2ndData_t* lpf_data);

/*----------------------二阶低通滤波器------------------------*/
Vector_3f LPF_2nd(LPF2ndData_t* lpf_2nd, Vector_3f newData);

/*----------------------互补滤波器系数计算-------------------------*/
float CF_Factor_Cal(float deltaT, float tau);

/*----------------------一阶互补滤波器-----------------------------*/
Vector_3f CF_1st(Vector_3f gyroData, Vector_3f accData, float cf_factor);

/*----------------------获取本向量平方-----------------------------*/
Vector_3f length_squared(Vector_3f Acc_lpf);

Vector_3f KalmanFilter(Vector_3f accData);

//窗口滑动滤波
void Prepare_Data(Vector_3f *acc_in,Vector_3f *acc_out);

//计算飞行器姿态
void GetAttitude(void);

//余弦矩阵更新姿态
void DCM_CF(Vector_3f gyro,Vector_3f acc, Vector_3f mag, float deltaT);

/*----------------------欧拉角转余弦矩阵-----------------------------*/
Matrix3f Dcm_From_euler(Vector_3f euler);

void Get_rollpitch(Vector_3f *angle, Vector_3f Vector_G);

void Get_yaw(Vector_3f *angle, Vector_3f Vector_M);

/*获取本次与上次的时间差*/
float getDeltaT(uint32_t currentT);

//欧拉角->四元数
Quater Euler2Quater(double roll, double pitch, double yaw);

//四元数->欧拉角
Vector_3f Quater2Euler(float q0, float q1, float q2, float q3);
#endif
