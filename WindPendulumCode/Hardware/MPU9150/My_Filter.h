#ifndef __MY_FILTER_H_
#define __MY_FILTER_H_


#include "AHRS_Attitude.h"

#define AHRS_LOOP_TIME					2000	//��λΪuS
#define ACC_LPF_CUT 30.0f		//���ٶȵ�ͨ�˲�����ֹƵ��30Hz
#define MAG_LPF_CUT 10.0f		//�����Ƶ�ͨ�˲�����ֹƵ��10Hz
#define GYRO_CF_TAU 1.5f
#define MAG_CF_TAU 	1.0f

#define ACC_1G 			4096//8192		//�ɼ��ٶȼƵ�����ȷ��

#define FILTER_NUM 	20

typedef struct
{
	Vector_3f last; //��һʱ�̵����Ž��
	Vector_3f mid;  //��ǰʱ�̵�Ԥ����
	Vector_3f now;  //��ǰʱ�̵����Ž��
	Vector_3f P_mid;  //��ǰʱ��Ԥ������Э����
	Vector_3f P_now;  //��ǰʱ�����Ž����Э����
	Vector_3f P_last; //��һʱ�����Ž����Э����
	Vector_3f kg;     //����������
	Vector_3f Kalman_Q;  //ϵͳ����
	Vector_3f R;       //��������
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


/*----------------------�˲�����ʼ��-------------------------*/
void My_Filter_Init(void);

/*----------------------һ�׵�ͨ�˲���ϵ������-------------------------*/
float LPF_1st_Factor_Cal(float deltaT, float Fcut);

/*----------------------һ�׵�ͨ�˲���------------------------*/
Vector_3f LPF_1st(Vector_3f oldData, Vector_3f newData, float lpf_factor);

/*----------------------���׵�ͨ�˲���ϵ������-------------------------*/
void LPF_2nd_Factor_Cal(LPF2ndData_t* lpf_data);

/*----------------------���׵�ͨ�˲���------------------------*/
Vector_3f LPF_2nd(LPF2ndData_t* lpf_2nd, Vector_3f newData);

/*----------------------�����˲���ϵ������-------------------------*/
float CF_Factor_Cal(float deltaT, float tau);

/*----------------------һ�׻����˲���-----------------------------*/
Vector_3f CF_1st(Vector_3f gyroData, Vector_3f accData, float cf_factor);

/*----------------------��ȡ������ƽ��-----------------------------*/
Vector_3f length_squared(Vector_3f Acc_lpf);

Vector_3f KalmanFilter(Vector_3f accData);

//���ڻ����˲�
void Prepare_Data(Vector_3f *acc_in,Vector_3f *acc_out);

//�����������̬
void GetAttitude(void);

//���Ҿ��������̬
void DCM_CF(Vector_3f gyro,Vector_3f acc, Vector_3f mag, float deltaT);

/*----------------------ŷ����ת���Ҿ���-----------------------------*/
Matrix3f Dcm_From_euler(Vector_3f euler);

void Get_rollpitch(Vector_3f *angle, Vector_3f Vector_G);

void Get_yaw(Vector_3f *angle, Vector_3f Vector_M);

/*��ȡ�������ϴε�ʱ���*/
float getDeltaT(uint32_t currentT);

//ŷ����->��Ԫ��
Quater Euler2Quater(double roll, double pitch, double yaw);

//��Ԫ��->ŷ����
Vector_3f Quater2Euler(float q0, float q1, float q2, float q3);
#endif
