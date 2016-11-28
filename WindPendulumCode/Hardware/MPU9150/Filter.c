#include "My_Filter.h"
//#include "mymath.h"
#include "Sys.h"
#include <math.h>

//----------------------------------------------------------------------------------------------------
// Definitions

#define Kp 2.0f                        // proportional gain governs rate of convergence to accelerometer/magnetometer
#define Ki 0.005f                // integral gain governs rate of convergence of gyroscope biases
#define halfT 0.5f                // half the sample period

//---------------------------------------------------------------------------------------------------
// Variable definitions


//====================================================================================================
// Function
//====================================================================================================

void AHRSup(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) 
{
	float norm;
	float hx, hy, hz, bx, bz;
	float vx, vy, vz, wx, wy, wz;
	float ex, ey, ez;
	
	static float q0 = 1, q1 = 0, q2 = 0, q3 = 0;        // quaternion elements representing the estimated orientation
	static float exInt = 0, eyInt = 0, ezInt = 0;        // scaled integral error

	// auxiliary variables to reduce number of repeated operations
	float q0q0 = q0*q0;
	float q0q1 = q0*q1;
	float q0q2 = q0*q2;
	float q0q3 = q0*q3;
	float q1q1 = q1*q1;
	float q1q2 = q1*q2;
	float q1q3 = q1*q3;
	float q2q2 = q2*q2;   
	float q2q3 = q2*q3;
	float q3q3 = q3*q3;          
	
	// normalise the measurements
	norm = sqrt(ax*ax + ay*ay + az*az);       
	ax = ax / norm;
	ay = ay / norm;
	az = az / norm;
	norm = sqrt(mx*mx + my*my + mz*mz);          
	mx = mx / norm;
	my = my / norm;
	mz = mz / norm;         
	
	// compute reference direction of flux
	hx = 2*mx*(0.5f - q2q2 - q3q3) + 2*my*(q1q2 - q0q3) + 2*mz*(q1q3 + q0q2);
	hy = 2*mx*(q1q2 + q0q3) + 2*my*(0.5f - q1q1 - q3q3) + 2*mz*(q2q3 - q0q1);
	hz = 2*mx*(q1q3 - q0q2) + 2*my*(q2q3 + q0q1) + 2*mz*(0.5f - q1q1 - q2q2);         

	bx = sqrt((hx*hx) + (hy*hy));
	bz = hz;        

	// estimated direction of gravity and flux (v and w)
	vx = 2*(q1q3 - q0q2);
	vy = 2*(q0q1 + q2q3);
	vz = q0q0 - q1q1 - q2q2 + q3q3;

	wx = 2*bx*(0.5f - q2q2 - q3q3) + 2*bz*(q1q3 - q0q2);
	wy = 2*bx*(q1q2 - q0q3) + 2*bz*(q0q1 + q2q3);
	wz = 2*bx*(q0q2 + q1q3) + 2*bz*(0.5f - q1q1 - q2q2);  

	// error is sum of cross product between reference direction of fields and direction measured by sensors
	ex = (ay*vz - az*vy) + (my*wz - mz*wy);
	ey = (az*vx - ax*vz) + (mz*wx - mx*wz);
	ez = (ax*vy - ay*vx) + (mx*wy - my*wx);
	
	// integral error scaled integral gain
	exInt = exInt + ex*Ki;
	eyInt = eyInt + ey*Ki;
	ezInt = ezInt + ez*Ki;
	
	// adjusted gyroscope measurements
	gx = gx + Kp*ex + exInt;
	gy = gy + Kp*ey + eyInt;
	gz = gz + Kp*ez + ezInt;
	
	// integrate quaternion rate and normalise
	q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
	q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
	q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
	q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;  
	
	// normalise quaternion
	norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
	q0 = q0 / norm;
	q1 = q1 / norm;
	q2 = q2 / norm;
	q3 = q3 / norm;
	
	angle.Z   = (-atan2(2*q1*q2 - 2*q0*q3, -2 * q1 * q1 - 2 * q3 * q3 + 1) * 57.295780) - Att_Yaw_Offset;  //偏航角，绕z轴转动	
    if(Yaw < 0 ){Yaw = Yaw + 360;}
	if(Yaw > 360 ){Yaw = Yaw - 360;}
	angle.Y =  (-asin(2*q2*q3 + 2*q0*q1) * 57.295780) - Att_Pitch_Offset; //俯仰角，绕x轴转动	 
    angle.X  = (-atan2(-2*q0*q2 + 2*q1*q3, -2 * q1 * q1 - 2 * q2* q2 + 1) * 57.295780) - Att_Roll_Offset; //滚动角，绕y轴转动/**/
	
}

//====================================================================================================
// END OF CODE
//====================================================================================================





//------------------------------------------------------------------------------------------//
Kalman1nd Kalman;

float acc_lpf;	
float mag_lpf;
float gyro_cf;		
float mag_cf;

Vector_3f Gyro, Acc, Acc_lpf, Mag, Mag_lpf; 
Vector_3f angle;
Vector_3f Acc_AVG;

LPF2ndData_t Acc_lpf_2nd;

void My_Filter_Init(void)
{
	//加速度一阶低通滤波器系数计算
	acc_lpf = LPF_1st_Factor_Cal(AHRS_LOOP_TIME * 1e-6, ACC_LPF_CUT);
	
	//加速度二阶低通滤波器系数计算
	LPF_2nd_Factor_Cal(&Acc_lpf_2nd);
	
	//磁力计一阶低通滤波器系数计算
	mag_lpf = LPF_1st_Factor_Cal(AHRS_LOOP_TIME * 5e-6, MAG_LPF_CUT);	
	
	//互补滤波器系数计算
	gyro_cf = CF_Factor_Cal(AHRS_LOOP_TIME * 1e-6, GYRO_CF_TAU);
	mag_cf = CF_Factor_Cal(AHRS_LOOP_TIME * 1e-6, MAG_CF_TAU);
	
	//一阶卡尔曼初始化
	Kalman.Kalman_Q.X = 1.0;  //系统噪声
	Kalman.Kalman_Q.Y = 1.0;  //系统噪声
	Kalman.Kalman_Q.Z = 1.0;  //系统噪声
	
	Kalman.R.X = 500.0;       //测量噪声
	Kalman.R.Y = 500.0;       //测量噪声
	Kalman.R.Z = 500.0;       //测量噪声
}

/*----------------------一阶低通滤波器系数计算-------------------------*/
float LPF_1st_Factor_Cal(float deltaT, float Fcut)
{
	return deltaT / (deltaT + 1 / (2 * 3.14 * Fcut));
}

/*----------------------一阶低通滤波器------------------------*/
Vector_3f LPF_1st(Vector_3f oldData, Vector_3f newData, float lpf_factor)
{
	Vector_3f lpf_1nd_data;
	lpf_1nd_data.X = oldData.X * (1 - lpf_factor) + newData.X * lpf_factor;
	lpf_1nd_data.Y = oldData.Y * (1 - lpf_factor) + newData.Y * lpf_factor;
	lpf_1nd_data.Z = oldData.Z * (1 - lpf_factor) + newData.Z * lpf_factor;
	return lpf_1nd_data;
}

/*----------------------二阶低通滤波器系数计算-------------------------*/
void LPF_2nd_Factor_Cal(LPF2ndData_t* lpf_data)
{
	//截止频率:30Hz 采样频率:500Hz
	lpf_data->b0 = 0.1883633f;
	lpf_data->a1 = 1.023694f;
	lpf_data->a2 = 0.2120577f;
}

/*----------------------二阶低通滤波器------------------------*/
Vector_3f LPF_2nd(LPF2ndData_t* lpf_2nd, Vector_3f newData)
{
	Vector_3f lpf_2nd_data;
	
	lpf_2nd_data.X = newData.X * (lpf_2nd->b0) + (lpf_2nd->lastout.X) * (lpf_2nd->a1) - (lpf_2nd->preout.X) * (lpf_2nd->a2);
	lpf_2nd_data.Y = newData.Y * (lpf_2nd->b0) + (lpf_2nd->lastout.Y) * (lpf_2nd->a1) - (lpf_2nd->preout.Y) * (lpf_2nd->a2);
	lpf_2nd_data.Z = newData.Z * (lpf_2nd->b0) + (lpf_2nd->lastout.Z) * (lpf_2nd->a1) - (lpf_2nd->preout.Z) * (lpf_2nd->a2);
	
	lpf_2nd->preout = lpf_2nd->lastout;
	lpf_2nd->lastout = lpf_2nd_data;
	
	return lpf_2nd_data;
}

/*----------------------互补滤波器系数计算-------------------------*/
float CF_Factor_Cal(float deltaT, float tau)
{
	return tau / (deltaT + tau);
}

/*----------------------一阶互补滤波器-----------------------------*/
Vector_3f CF_1st(Vector_3f gyroData, Vector_3f accData, float cf_factor)
{ 
	Vector_3f cf_1st_data;
	cf_1st_data.X = (gyroData.X * cf_factor + accData.X *(1 - cf_factor));
	cf_1st_data.Y = (gyroData.Y * cf_factor + accData.Y *(1 - cf_factor));
	cf_1st_data.Z = (gyroData.Z * cf_factor + accData.Z *(1 - cf_factor));
	
	return cf_1st_data;	
}

/*----------------------获取本向量平方-----------------------------*/
Vector_3f length_squared(Vector_3f Acc_lpf) 
{
	Vector_3f Squa_Acc_lpf;
	
	Squa_Acc_lpf.X = ((Acc_lpf.X) * (Acc_lpf.X));
	Squa_Acc_lpf.Y = ((Acc_lpf.Y) * (Acc_lpf.Y));
	Squa_Acc_lpf.Z = ((Acc_lpf.Z) * (Acc_lpf.Z));
	
	return Squa_Acc_lpf;
}

Vector_3f KalmanFilter(Vector_3f accData)
{
    Kalman.mid.X = Kalman.last.X;                //x(k|k-1) = AX(k-1|k-1)+BU(k)
    Kalman.P_mid.X = Kalman.last.X+Kalman.Kalman_Q.X;       //p(k|k-1) = Ap(k-1|k-1)A'+Q
    Kalman.kg.X = Kalman.P_mid.X/(Kalman.P_mid.X+Kalman.R.X);          //kg(k) = p(k|k-1)H'/(Hp(k|k-1)'+R)
    Kalman.now.X=Kalman.mid.X+Kalman.kg.X*(accData.X-Kalman.mid.X);    //x(k|k) = X(k|k-1)+kg(k)(Z(k)-HX(k|k-1))
    Kalman.P_now.X=(1.0f-Kalman.kg.X)*Kalman.P_mid.X;         //p(k|k) = (I-kg(k)H)P(k|k-1)
    Kalman.P_last.X=Kalman.P_now.X;                  //状态更新
    Kalman.last.X=Kalman.now.X;
	
		Kalman.mid.Y = Kalman.last.Y;                //x(k|k-1) = AX(k-1|k-1)+BU(k)
    Kalman.P_mid.Y = Kalman.last.Y+Kalman.Kalman_Q.Y;       //p(k|k-1) = Ap(k-1|k-1)A'+Q
    Kalman.kg.Y = Kalman.P_mid.Y/(Kalman.P_mid.Y+Kalman.R.Y);          //kg(k) = p(k|k-1)H'/(Hp(k|k-1)'+R)
    Kalman.now.Y=Kalman.mid.Y+Kalman.kg.Y*(accData.Y-Kalman.mid.Y);    //x(k|k) = X(k|k-1)+kg(k)(Z(k)-HX(k|k-1))
    Kalman.P_now.Y=(1.0f-Kalman.kg.Y)*Kalman.P_mid.Y;         //p(k|k) = (I-kg(k)H)P(k|k-1)
    Kalman.P_last.Y=Kalman.P_now.Y;                  //状态更新
    Kalman.last.Y=Kalman.now.Y;
	
		Kalman.mid.Z = Kalman.last.Z;                //x(k|k-1) = AX(k-1|k-1)+BU(k)
    Kalman.P_mid.Z = Kalman.last.Z+Kalman.Kalman_Q.Z;       //p(k|k-1) = Ap(k-1|k-1)A'+Q
    Kalman.kg.Z = Kalman.P_mid.Z/(Kalman.P_mid.Z+Kalman.R.Z);          //kg(k) = p(k|k-1)H'/(Hp(k|k-1)'+R)
    Kalman.now.Z=Kalman.mid.Z+Kalman.kg.Z*(accData.Z-Kalman.mid.Z);    //x(k|k) = X(k|k-1)+kg(k)(Z(k)-HX(k|k-1))
    Kalman.P_now.Z=(1.0f-Kalman.kg.Z)*Kalman.P_mid.Z;         //p(k|k) = (I-kg(k)H)P(k|k-1)
    Kalman.P_last.Z=Kalman.P_now.Z;                  //状态更新
    Kalman.last.Z=Kalman.now.Z;
		
    return Kalman.now;
}

//窗口滑动滤波
void Prepare_Data(Vector_3f *acc_in,Vector_3f *acc_out)
{
	static uint8_t 	filter_cnt=0;
	static int16_t	ACC_X_BUF[FILTER_NUM],ACC_Y_BUF[FILTER_NUM],ACC_Z_BUF[FILTER_NUM];
	int32_t temp1=0,temp2=0,temp3=0;
	uint8_t i;

	ACC_X_BUF[filter_cnt] = acc_in->X;
	ACC_Y_BUF[filter_cnt] = acc_in->Y;
	ACC_Z_BUF[filter_cnt] = acc_in->Z;
	for(i=0;i<FILTER_NUM;i++)
	{
		temp1 += ACC_X_BUF[i];
		temp2 += ACC_Y_BUF[i];
		temp3 += ACC_Z_BUF[i];
	}
	acc_out->X = temp1 / FILTER_NUM;
	acc_out->Y = temp2 / FILTER_NUM;
	acc_out->Z = temp3 / FILTER_NUM;
	filter_cnt++;
	if(filter_cnt==FILTER_NUM)	filter_cnt=0;
}

//计算飞行器姿态
//void GetAttitude(void)
//{
//	float deltaT;
//	Vector_3f Acc_temp;
//	
//	//加速度数据二阶低通滤波
//	Acc_lpf = LPF_2nd(&Acc_lpf_2nd, Acc_ADC);
//	
//	Prepare_Data(&Acc_ADC, &Acc_temp);
//	Acc_AVG.X = 0.9f*Acc_temp.X + 0.1f*Acc_ADC.X;
//	Acc_AVG.Y = 0.9f*Acc_temp.Y + 0.1f*Acc_ADC.Y;
//	Acc_AVG.Z = 0.9f*Acc_temp.Z + 0.1f*Acc_ADC.Z;
//	//Acc_lpf = KalmanFilter(Acc_ADC);
//	
//	//磁力计数据一阶低通滤波
//	Mag_lpf = LPF_1st(Mag_lpf, Mag_ADC, mag_lpf);		
//	
//	deltaT = getDeltaT(GetSysTime_us());
//	
//	//加速度和陀螺仪互补滤波，计算得到飞行器姿态
//	DCM_CF(Gyro_ADC,Acc_AVG,Mag_ADC,deltaT);
//	//DCM_CF(Gyro_ADC,Acc_lpf,Mag_ADC,deltaT);
//	
//	//AHRSup(Gyro_ADC.X, Gyro_ADC.Y, Gyro_ADC.Z, Acc_lpf.X, Acc_lpf.Y, Acc_lpf.Z, Mag_lpf.X, Mag_lpf.Y, Mag_lpf.Z);
//}

//余弦矩阵更新姿态
void DCM_CF(Vector_3f gyro,Vector_3f acc, Vector_3f mag, float deltaT)
{
	static Vector_3f deltaGyroAngle, LastGyro;
	static Vector_3f Vector_G = {0,0,ACC_1G}, Vector_M = {1000,0,0};
	
	Vector_3f num;
	
	Matrix3f dcm;
	
	//计算陀螺仪角度变化，二阶龙格库塔积分	
	deltaGyroAngle.X = (gyro.X + LastGyro.X) * 0.5f * deltaT;
	deltaGyroAngle.Y = (gyro.Y + LastGyro.Y) * 0.5f * deltaT;
	deltaGyroAngle.Z = (gyro.Z + LastGyro.Z) * 0.5f * deltaT;
	
	LastGyro = gyro;
	
	//计算表示单次旋转的余弦矩阵
	dcm = Dcm_From_euler(deltaGyroAngle);
	
	
	//利用余弦矩阵更新重力向量在机体坐标系的投影
	num.X = dcm.ax * Vector_G.X + dcm.ay * Vector_G.Y + dcm.az * Vector_G.Z;
	num.Y = dcm.bx * Vector_G.X + dcm.by * Vector_G.Y + dcm.bz * Vector_G.Z;
	num.Z = dcm.cx * Vector_G.X + dcm.cy * Vector_G.Y + dcm.cz * Vector_G.Z;
	Vector_G.X = num.X;
	Vector_G.Y = num.Y;
	Vector_G.Z = num.Z;
	
	//利用余弦矩阵更新地磁向量在机体坐标系的投影
	num.X = dcm.ax * Vector_M.X + dcm.ay * Vector_M.Y + dcm.az * Vector_M.Z;
	num.Y = dcm.bx * Vector_M.X + dcm.by * Vector_M.Y + dcm.bz * Vector_M.Z;
	num.Z = dcm.cx * Vector_M.X + dcm.cy * Vector_M.Y + dcm.cz * Vector_M.Z;
	Vector_M.X = num.X;
	Vector_M.Y = num.Y;
	Vector_M.Z = num.Z;
	
	//互补滤波，使用加速度和磁感强度测量值矫正角速度积分漂移
	Vector_G = CF_1st(Vector_G, acc, gyro_cf);
	Vector_M = CF_1st(Vector_M, mag, mag_cf);
	
	//计算飞行器的ROLL和PITCH
	Get_rollpitch(&angle, Vector_G);	
	
	//计算飞行器的YAW
	Get_yaw(&angle, Vector_M);
	//angle.Z = atan2(-Mag_lpf.X, Mag_lpf.Y)*57.3f + 180;
}

Matrix3f Dcm_From_euler(Vector_3f euler)
{
	Matrix3f Dcm;
	float sinx = sinf(euler.X);
	float cosx = cosf(euler.X);
	float siny = sinf(euler.Y);
	float cosy = cosf(euler.Y);
	float sinz = sinf(euler.Z);
	float cosz = cosf(euler.Z);

	Dcm.ax = cosy * cosz;
	Dcm.ay = (sinx * siny * cosz) + (cosx * sinz);
	Dcm.az = -(cosx * siny * cosz) + (sinx * sinz);
	Dcm.bx = -cosy * sinz;
	Dcm.by = -(sinx * siny * sinz) + (cosx * cosz);
	Dcm.bz = (cosx * siny * sinz) + (sinx * cosz);
	Dcm.cx = siny;
	Dcm.cy = -sinx * cosy;
	Dcm.cz = cosx * cosy;
	
//	Dcm.ax = 1;
//	Dcm.ay = 2;
//	Dcm.az = 3;
//	Dcm.bx = 4;
//	Dcm.by = 5;
//	Dcm.bz = 6;
//	Dcm.cx = 7;
//	Dcm.cy = 8;
//	Dcm.cz = 9;
	
	return Dcm;
}

//void Get_rollpitch(Vector_3f *angle, Vector_3f Vector_G)
//{
//	angle->X = degrees(atan2f(Vector_G.Y,sqrtf(Vector_G.X * Vector_G.X + Vector_G.Z* Vector_G.Z)));
//	angle->Y = degrees(atan2f(-Vector_G.X, sqrtf(Vector_G.Y * Vector_G.Y + Vector_G.Z* Vector_G.Z)));	
//}

//void Get_yaw(Vector_3f *angle, Vector_3f Vector_M)
//{
//	angle->Z = degrees(atan2f(Vector_M.Y,Vector_M.X));
//}

//void Get_yaw_F_acc_mag(Vector_3f *angle, Vector_3f Mag)
//{
//	
//}

float getDeltaT(uint32_t currentT)
{
	static uint32_t previousT;
	float	deltaT = (currentT - previousT) * 1e-6;	
	previousT = currentT;
	
	return deltaT;
}

//欧拉角->四元数
Quater Euler2Quater(double roll, double pitch, double yaw)
{
	Quater Quaternions;
	
	double sinrol = sin((double)(roll)*0.5f);
	double cosrol = cos((double)(roll)*0.5f);
	
	double sinpit = sin((double)(pitch)*0.5f);
	double cospit = cos((double)(pitch)*0.5f);
	
	double sinyaw = sin((double)(yaw)*0.5f);
	double cosyaw = cos((double)(yaw)*0.5f);

	Quaternions.q0 = cosyaw*cospit*cosrol + sinyaw*sinpit*sinrol;
	Quaternions.q1 = cosyaw*cospit*sinrol - sinyaw*sinpit*cosrol;
	Quaternions.q2 = cosyaw*sinpit*cosrol + sinyaw*cospit*sinrol;
	Quaternions.q3 = sinyaw*cospit*cosrol - cosyaw*sinpit*sinrol;
	
	return Quaternions;
}

//void from_euler(float roll, float pitch, float yaw) 
//{
//	double cosPhi_2 = cos(double(roll) / 2.0);
//	double sinPhi_2 = sin(double(roll) / 2.0);
//	double cosTheta_2 = cos(double(pitch) / 2.0);
//	double sinTheta_2 = sin(double(pitch) / 2.0);
//	double cosPsi_2 = cos(double(yaw) / 2.0);
//	double sinPsi_2 = sin(double(yaw) / 2.0);
//	data[0] = cosPhi_2 * cosTheta_2 * cosPsi_2 + sinPhi_2 * sinTheta_2 * sinPsi_2;
//	data[1] = sinPhi_2 * cosTheta_2 * cosPsi_2 - cosPhi_2 * sinTheta_2 * sinPsi_2;
//	data[2] = cosPhi_2 * sinTheta_2 * cosPsi_2 + sinPhi_2 * cosTheta_2 * sinPsi_2;
//	data[3] = cosPhi_2 * cosTheta_2 * sinPsi_2 - sinPhi_2 * sinTheta_2 * cosPsi_2;
//}

//四元数->欧拉角
Vector_3f Quater2Euler(float q0, float q1, float q2, float q3)
{
	Vector_3f Euler;
	float norm;
	
	// normalise quaternion
  norm = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
  q0 = q0 * norm;       //w
  q1 = q1 * norm;       //x
  q2 = q2 * norm;       //y
  q3 = q3 * norm;       //z
	
	Euler.X = asin(2*q2*q3 + 2*q0*q1) * 57.295780; //俯仰角，绕x轴转动	 
	Euler.Y = -atan2(-2*q0*q2 + 2*q1*q3, -2 * q1 * q1 - 2 * q2* q2 + 1) * 57.295780; //滚动角，绕y轴转动
	Euler.Z = -atan2(2*q1*q2 - 2*q0*q3, -2 * q1 * q1 - 2 * q3 * q3 + 1) * 57.295780;  //偏航角，绕z轴转动	
	return Euler;
}

