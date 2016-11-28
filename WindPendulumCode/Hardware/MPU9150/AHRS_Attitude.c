#include "AHRS_Attitude.h"
#include "Sys.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "stm32i2c.h"
#include "math.h"
#include <stdio.h>
#include "mpu9150.h"
#include "Filter.h"

//���岻ͬ������Χ�Ŀ̶�����
#define Gyro_250_Scale_Factor   131.0f
#define Gyro_500_Scale_Factor   65.5f
#define Gyro_1000_Scale_Factor  32.8f
#define Gyro_2000_Scale_Factor  16.4f
#define Accel_2_Scale_Factor    16384.0f
#define Accel_4_Scale_Factor    8192.0f
#define Accel_8_Scale_Factor    4096.0f
#define Accel_16_Scale_Factor   2048.0f


//#define Accel_Xout_Offset		-130
//#define Accel_Yout_Offset		 96
//#define Accel_Zout_Offset		 460
//#define FILTER_NUM	            20

#define Kp 2.5f     //proportional gain governs rate of convergence to accelerometer/magnetometer
#define Ki 0.005f   //integral gain governs rate of convergence of gyroscope biases
//#define halfT 0.00125f  //half the sample period,halfT��Ҫ���ݾ�����̬����������������T����̬�������ڣ�T*���ٶ�=΢�ֽǶ�
//float Yaw;
//#define q30  1073741824.0f
//float q0=1.0f,q1=0.0f,q2=0.0f,q3=0.0f;
float q0, q1, q2, q3;
float exInt = 0, eyInt = 0, ezInt = 0;        // scaled integral error
float init_ax, init_ay, init_az, init_gx, init_gy, init_gz, init_mx, init_my, init_mz;

/*У׼����������ǳ�ֵ*/
float gyrox_bias_init = 0;
/*У׼�������������ʱ���Ư��*/
float gyrox_bias_time = 0;
/*����׼У׼������������ƫ��*///gyrox_bias_other = gyrox_bias_init + (float)cnt_6_second*0.1f;
float gyrox_bias_other = 0;

#if ATTITUDE_BIAS
  /*��ҪУ׼�����ʼֵΪ0*/
  float Att_Roll_Offset, Att_Pitch_Offset, Att_Yaw_Offset = 0;
#else
  float Att_Roll_Offset = -0.006477, Att_Pitch_Offset = 0.291226, Att_Yaw_Offset = 0;
	//float Att_Roll_Offset, Att_Pitch_Offset, Att_Yaw_Offset = 0;
#endif

#if GYRO_BIAS
  /*��ҪУ׼�����ʼֵΪ0*/
	float Gyro_Xout_Offset, Gyro_Yout_Offset, Gyro_Zout_Offset;
#else
	float Gyro_Xout_Offset = 37.969398, Gyro_Yout_Offset = 52.193401, Gyro_Zout_Offset = 60.336399;
#endif

#if ACCEL_BIAS
  /*��ҪУ׼�����ʼֵΪ0*/
	float Accel_Xout_Offset, Accel_Yout_Offset, Accel_Zout_Offset;
#else
  float Accel_Xout_Offset = -474.707397, Accel_Yout_Offset = 206.795197, Accel_Zout_Offset = 78.990234;
#endif

float Pitch, Roll, Yaw;
float halfT;
float maxMagX = 0;
float minMagX = 0;
float maxMagY = 0;
float minMagY = 0;
float maxMagZ = 0;
float minMagZ = 0;
float MXgain = 0;
float MYgain = 0;
float MZgain = 0;
float MXoffset = 0;
float MYoffset = 0;
float MZoffset = 0;
float heading;

float MPU6050_GYRO_LAST_X, MPU6050_GYRO_LAST_Y, MPU6050_GYRO_LAST_Z;
float MPU6050_ACC_LAST_X, MPU6050_ACC_LAST_Y, MPU6050_ACC_LAST_Z;

Vector_3f Gyro_ADC;
Vector_3f Acc_ADC;
Vector_3f Mag_ADC;
Vector_3f Imu;

///*�˲��������*/
//Vector_3f Acc_lpf;

Vector_3f Last_Gyro_ADC;
/*******************************************************************************
* Function Name  : get_mpu9150_data
* Description    : ��ȡmpu9150�ļ��ٶȼ� ������ ���������ݲ���У׼���˲�.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void get_mpu9150_data(void)
{
   signed short int gyro[3], accel[3], mag[3]; 
   unsigned char tmp[7], data_write[14];
	static signed int gyro_x=0; //����У׼��Ʈ
	static unsigned short count = 0;
	Last_Gyro_ADC.X = Gyro_ADC.X;
	Last_Gyro_ADC.Y = Gyro_ADC.Y;
	Last_Gyro_ADC.Z = Gyro_ADC.Z;
	
//	Accel_Xout_Offset = 0;
//	Accel_Yout_Offset = 0;
//	Accel_Zout_Offset = 0;
////	
//	Gyro_Xout_Offset = 0;
//	Gyro_Yout_Offset = 0;
//	Gyro_Zout_Offset = 0;
//	
//	MXoffset = 0;
//	MYoffset = 0;
//	MZoffset = 0;
//	
//	MXgain = 0;
//	MYgain = 0;
//	MZgain = 0;
//	gyrox_bias_other = gyrox_bias_init + (float)cnt_6_second*0.078f;
	gyrox_bias_other = 0;
	if (gyrox_bias_other >= 50)
	{
		gyrox_bias_other = 50;
	}
	if (gyrox_bias_other <= -50)
	{
		gyrox_bias_other = -50;
	}
	
   if(!i2cread(MPU9150_Addr, Accel_Xout_H, 14, data_write))
   {
			accel[0]=(((signed short int)data_write[0])<<8) | data_write[1];
			accel[1]=(((signed short int)data_write[2])<<8) | data_write[3];
			accel[2]=(((signed short int)data_write[4])<<8) | data_write[5];
			gyro[0] =(((signed short int)data_write[8])<<8) | data_write[9];
			gyro[1] =(((signed short int)data_write[10])<<8) | data_write[11];
			gyro[2] =(((signed short int)data_write[12])<<8) | data_write[13];
		 
		 if (Gyro_CALIBRATED == 1)
		 {
//			  LED4_Orange = 1;
			  gyro_x  += gyro[0];
				//gyro_y	+= gyro[1];
				//gyro_z	+= gyro[2];
				count++;
		 }
		  //Gyro_Xout_Offset = -125.315698f;
		  //Gyro_Xout_Offset = 0;
			init_ax=(float)(accel[0]) - Accel_Xout_Offset;   	  
			init_ay=(float)(accel[1]) - Accel_Yout_Offset;
			init_az=(float)(accel[2]) - Accel_Zout_Offset;  		 
			init_gx=(((float)gyro[0]) - Gyro_Xout_Offset + gyrox_bias_other) * 0.000266f; //0.0010642f;//   //��λת���ɣ�����/s��0.000266=1/(Gyro_500_Scale_Factor * 57.295780)
			init_gy=(((float)gyro[1]) - Gyro_Yout_Offset) * 0.000266f; //0.0010642f;//
			init_gz=(((float)gyro[2]) - Gyro_Zout_Offset) * 0.000266f; //0.0010642f;//
		 
//	 	 init_gx=(((float)gyro[0]) - Gyro_Xout_Offset) * 0.174532f;    //��λת���ɣ�����/s��0.000266=1/(Gyro_500_Scale_Factor * 57.295780)
//			init_gy=(((float)gyro[1]) - Gyro_Yout_Offset) * 0.174532f;
//			init_gz=(((float)gyro[2]) - Gyro_Zout_Offset) * 0.174532f;
			
			Acc_ADC.X  = init_ax;
		  Acc_ADC.Y  = init_ay;
		  Acc_ADC.Z  = init_az;
			Gyro_ADC.X = init_gx;
		  Gyro_ADC.Y = init_gy;
		  Gyro_ADC.Z = init_gz;
			
      tmp[6]=0x00;
      data_write[6]=0x01;
      i2cread(Compass_Addr, Compass_ST1, 1, tmp+6);
      if(tmp[6] == 1)
      {
        i2cread(Compass_Addr, Compass_HXL, 6, tmp);//��ȡcompass
				mag[0] = (((signed short int)tmp[1]) << 8) | tmp[0];
        mag[1] = (((signed short int)tmp[3]) << 8) | tmp[2];
        mag[2] = (((signed short int)tmp[5]) << 8) | tmp[4];

//				mag[0] = ((long)mag[0] * mag_sens_adj_val[0]) >> 8;  //�����ȵ���
//        mag[1] = ((long)mag[1] * mag_sens_adj_val[1]) >> 8;
//        mag[2] = ((long)mag[2] * mag_sens_adj_val[2]) >> 8;
				//����mag
				init_mx =(float)mag[1] * MXgain + MXoffset;		//ת��������				
        init_my =(float)mag[0] * MYgain + MYoffset;
        init_mz =(float)-mag[2] * MZgain + MZoffset;
				
//			init_mx =(float)mag[1];		//ת��������		
//			init_my =(float)mag[0];
//			init_mz =(float)-mag[2];
				
				Mag_ADC.X = init_mx;
				Mag_ADC.Y = init_my;
				Mag_ADC.Z = init_mz;
				
				i2cwrite(Compass_Addr, Compass_CNTL, 1, data_write+6);	 //����compass��single measurement mode
	  }
			
		if (count >= 5000)
		{
			Gyro_CALIBRATED = 0;
			Gyro_Xout_Offset = (float)gyro_x / count;
			//Gyro_Yout_Offset = (float)gyro_y / count;
			//Gyro_Zout_Offset = (float)gyro_z / count;
			get_gyro_bias();
//			Write_Gyr_Acc_Att_Para();
			count = 0;
			gyro_x = 0;
//			LED4_Orange = 0; 
		}
		if (Acc_CALIBRATED == 1)
		{
			Acc_CALIBRATED = 0;
			get_accel_bias();
//			Write_Gyr_Acc_Att_Para();
		}
//		
//		if (Gyro_CALIBRATED == 1)
//		{
//			Gyro_CALIBRATED = 0;
//			get_gyro_bias();
//			Write_Gyr_Acc_Att_Para();
//		}
//	  printf("mx=%f, my=%f, mz=%f \n\r", init_mx, init_my, init_mz); 
   }
}
/*******************************************************************************
* Function Name  : get_mpu6x_data
* Description    : ��ȡmpu9150�� ������ 
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
Vector_3f get_mpugyro_data(void)
{
   signed short int gyro[3]; 
   unsigned char data_write[6];
	 Vector_3f Gyro_3f;
	
//	 gyrox_bias_other = gyrox_bias_init + (float)cnt_6_second*0.078f;
	gyrox_bias_other = 0;
	if (gyrox_bias_other >= 50)
	{
		gyrox_bias_other = 50;
	}
	if (gyrox_bias_other <= -50)
	{
		gyrox_bias_other = -50;
	}
	
//	Gyro_Xout_Offset = 0;
//	Gyro_Yout_Offset = 0;
//	Gyro_Zout_Offset = 0;
	
   if(!i2cread(MPU9150_Addr, Gyro_Xout_H, 6, data_write))
   {
		 gyro[0] = ((((signed short int)data_write[0])<<8) | data_write[1]);
	   gyro[1] = ((((signed short int)data_write[2])<<8) | data_write[3]);
	   gyro[2] = ((((signed short int)data_write[4])<<8) | data_write[5]);
		 //Gyro_Xout_Offset = -125.315698f;
		 //Gyro_Xout_Offset = 0;
		 //init_gx = ((float)gyro[0]);
		 init_gx=(((float)gyro[0]) - Gyro_Xout_Offset + gyrox_bias_other); //  + 10.0f
		 init_gy=(((float)gyro[1]) - Gyro_Yout_Offset);
		 init_gz=(((float)gyro[2]) - Gyro_Zout_Offset);
		 Gyro_3f.X = init_gx;
		 Gyro_3f.Y = init_gy;
		 Gyro_3f.Z = init_gz;
	   //printf("gx=%f, gy=%f, gz=%f \n\r", Gyro_3f.X, Gyro_3f.Y, Gyro_3f.Z); 
   }
	 return Gyro_3f;
}

/*******************************************************************************
* Function Name  : get_mpuacc_data
* Description    : ��ȡmpu9150�� ���ٶ�
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
Vector_3f get_mpuacc_data(void)
{
   signed short int acc[3]; 
   unsigned char data_write[6];
	 Vector_3f Acc_3f;
	
//	Accel_Xout_Offset = 0;
//	Accel_Yout_Offset = 0;
//	Accel_Zout_Offset = 0;
	
   if(!i2cread(MPU9150_Addr, Accel_Xout_H, 6, data_write))
   {
		 acc[0] = ((((signed short int)data_write[0])<<8) | data_write[1]);
	   acc[1] = ((((signed short int)data_write[2])<<8) | data_write[3]);
	   acc[2] = ((((signed short int)data_write[4])<<8) | data_write[5]);
		 //Gyro_Xout_Offset = -125.315698f;
		 //Gyro_Xout_Offset = 0;
		 //init_gx = ((float)gyro[0]);
		 init_ax=(float)(acc[0]) - Accel_Xout_Offset;   	  
		 init_ay=(float)(acc[1]) - Accel_Yout_Offset;
		 init_az=(float)(acc[2]) - Accel_Zout_Offset;  
		 Acc_3f.X = init_ax;
		 Acc_3f.Y = init_ay;
		 Acc_3f.Z = init_az;
	   //printf("ax=%f, ay=%f, az=%f \n\r", Acc_3f.X, Acc_3f.Y, Acc_3f.Z); 
   }
	 return Acc_3f;
}

/*******************************************************************************
* Function Name  : init_quaternion
* Description    : �����ʼ����Ԫ��q0 q1 q2 q3.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void init_quaternion(void)
{ 
  signed short int accel[3], mag[3];
  float init_Yaw, init_Pitch, init_Roll;
  unsigned char tmp[7], data_write[7];

  if(!i2cread(MPU9150_Addr, Accel_Xout_H, 6, data_write))
  {
	  accel[0]=((((signed short int)data_write[0])<<8) | data_write[1]) + Accel_Xout_Offset;
	  accel[1]=((((signed short int)data_write[2])<<8) | data_write[3]) + Accel_Yout_Offset;
	  accel[2]=((((signed short int)data_write[4])<<8) | data_write[5]) + Accel_Zout_Offset;
	  	    
	  init_ax=((float)accel[0]) / Accel_8_Scale_Factor;	   //��λת�����������ٶȵĵ�λ��g
	  init_ay=((float)accel[1]) / Accel_8_Scale_Factor;
     init_az=((float)accel[2]) / Accel_8_Scale_Factor;
	  printf("ax=%f, ay=%f, az=%f  ", init_ax, init_ay, init_az);

    tmp[6]=0x00;
    data_write[6]=0x01;
	  i2cwrite(Compass_Addr, Compass_CNTL, 1, data_write+6);	 //����compass��single measurement mode
	  Delay_ms(10);  //wait data ready
		i2cread(Compass_Addr, Compass_ST1, 1, tmp+6);
		if(tmp[6] == 1)
		{
			i2cread(Compass_Addr, Compass_HXL, 6, tmp);
			mag[0] = (((signed short int)tmp[1]) << 8) | tmp[0];
			mag[1] = (((signed short int)tmp[3]) << 8) | tmp[2];
			mag[2] = (((signed short int)tmp[5]) << 8) | tmp[4];

//			mag[0] = ((long)mag[0] * mag_sens_adj_val[0]) >> 8;  //�����ȵ���
//			mag[1] = ((long)mag[1] * mag_sens_adj_val[1]) >> 8;
//			mag[2] = ((long)mag[2] * mag_sens_adj_val[2]) >> 8;

			init_mx =(float)mag[1] * MXgain + MXoffset;		//ת��������				
			init_my =(float)mag[0] * MYgain + MYoffset;
			init_mz =(float)-mag[2] * MZgain + MZoffset;
			i2cwrite(Compass_Addr, Compass_CNTL, 1, data_write+6);	 //����compass��single measurement mode
			printf("mx=%f, my=%f, mz=%f \n\r", init_mx, init_my, init_mz);
    }

		//������y��Ϊǰ������    
		init_Roll = -atan2(init_ax, init_az);    //����ĵ�λ�ǻ��ȣ�����Ҫ�۲���Ӧ����57.295780ת��Ϊ�Ƕ�
		init_Pitch=  asin(init_ay);              //init_Pitch = asin(ay / 1);      
		init_Yaw  =  atan2(init_mx*cos(init_Roll) + init_my*sin(init_Roll)*sin(init_Pitch) + init_mz*sin(init_Roll)*cos(init_Pitch),
											 init_my*cos(init_Pitch) - init_mz*sin(init_Pitch));//������atan2(my, mx)�����е�init_Roll��init_Pitch�ǻ���				            
		//����ʼ��ŷ����ת���ɳ�ʼ����Ԫ����ע��sin(a)��λ�õĲ�ͬ������ȷ����xyz��ת����Pitch����Roll����Yaw������ZXY˳����ת,Qzyx=Qz*Qy*Qx�����е�init_YawRollPtich�ǽǶ�        
		q0 = cos(0.5f*init_Roll)*cos(0.5f*init_Pitch)*cos(0.5f*init_Yaw) - sin(0.5f*init_Roll)*sin(0.5f*init_Pitch)*sin(0.5f*init_Yaw);  //w
		q1 = cos(0.5f*init_Roll)*sin(0.5f*init_Pitch)*cos(0.5f*init_Yaw) - sin(0.5f*init_Roll)*cos(0.5f*init_Pitch)*sin(0.5f*init_Yaw);  //x   ��x����ת��pitch
		q2 = sin(0.5f*init_Roll)*cos(0.5f*init_Pitch)*cos(0.5f*init_Yaw) + cos(0.5f*init_Roll)*sin(0.5f*init_Pitch)*sin(0.5f*init_Yaw);  //y   ��y����ת��roll
		q3 = cos(0.5f*init_Roll)*cos(0.5f*init_Pitch)*sin(0.5f*init_Yaw) + sin(0.5f*init_Roll)*sin(0.5f*init_Pitch)*cos(0.5f*init_Yaw);  //z   ��z����ת��Yaw

		init_Roll  = init_Roll * 57.295780f;	 //����ת�Ƕ�
		init_Pitch = init_Pitch * 57.295780f;
		init_Yaw   = init_Yaw * 57.295780f;
		if(init_Yaw < 0){init_Yaw = init_Yaw + 360;}      //��Yaw�ķ�Χת��0-360
		if(init_Yaw > 360){init_Yaw = init_Yaw - 360;}
		heading    = init_Yaw; 	    
		printf("�ɳ�ʼ����Ԫ���õ���Yaw=%f, Pitch=%f, Roll=%f \n\r", init_Yaw, init_Pitch, init_Roll);
  }
}

/***************************************************************************************************************************************
* Function Name  : AHRSupdate
* Description    : accel gyro mag���ں��㷨��Դ��S.O.H. Madgwick
* Input          : None
* Output         : None
* Return         : None
// q0 q1 q2 q3��Ҫ��ʼ�����ܴ��뵽����ĳ����У�����ֱ��ʹ��1 0 0 0��������ļ��㣬��������Ϊ��
// 1.����У׼accle gyro mag��
// 2.����init_quaternion������1��accle��xyz�����ݣ������ù�ʽ�������ʼ��ŷ���ǣ�
//   ����ACCEL_1G=9.81����λ����m/s2����init_Yaw�����ô����Ƽ��������
// 3.�����Լ��Ĳ������ڣ�������halfT��halfT=��������/2����������Ϊִ��1��AHRSupdate���õ�ʱ�䣻
// 4.��2�м������ŷ����ת��Ϊ��ʼ������Ԫ��q0 q1 q2 q3���ںϼ��ٶȼƣ������ǣ�������º��ŷ����pitch��roll��Ȼ��ʹ��pitch roll�ʹ����Ƶ����ݽ��л����˲��ںϵõ�Yaw������ʹ�ã�����ŷ��������㣻
// 5.��ֱ��ʹ����Ԫ����
// 6.�ظ�4�����ɸ�����̬;

//�ܵ���˵�������������ǣ����ٶȼ�������������Pitch��Roll��������������������Yaw;
//���³����У�gx, gy, gz��λΪ����/s��ax, ay, azΪ���ٶȼ������ԭʼ16��������, mx, my, mzΪ�����������ԭʼ16�������ݣ�
//ǰ������mpu9150�ļ��ٶȼƺ������ǵ�x��Ϊǰ������;
//���³�����õĲο�����Ϊ��mpu9150�ļ��ٶȼƺ���������ָ��xyz����Ϊ������

//������Ϊ����500��/s��ǰ���£������ǵ���������65.5LSB/��/s�����԰������������ʮ���������ݳ���65.5���ǽ��ٶȣ���λ�ǡ�/s��
//Ȼ���ٳ���57.3�ͱ�ɻ�����;(1����=180/pi=57.3��)

//ŷ���ǵ�λΪ����radian������57.3�Ժ�ת��Ϊ�Ƕ�,0<yaw<360, -90<pitch<+90, -180<roll<180
***************************************************************************************************************************************/
float Gd32_Pitch = 0.0f, Gd32_Roll = 0.0f, Gd32_Yaw = 0.0f;
void AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) 
{
   float norm;
   float hx, hy, hz, bz, by;
   float vx, vy, vz, wx, wy, wz;
   float ex, ey, ez;
	 float halft;

   //auxiliary variables to reduce number of repeated operations,
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
                 
   //normalise the measurements
   norm = invSqrt(ax*ax + ay*ay + az*az);       
   ax = ax * norm;
   ay = ay * norm;
   az = az * norm;
   norm = invSqrt(mx*mx + my*my + mz*mz);          
   mx = mx * norm;
   my = my * norm;
   mz = mz * norm;         
        
   //compute reference direction of flux
   hx = 2*mx*(0.5f - q2q2 - q3q3) + 2*my*(q1q2 - q0q3) + 2*mz*(q1q3 + q0q2);
   hy = 2*mx*(q1q2 + q0q3) + 2*my*(0.5f - q1q1 - q3q3) + 2*mz*(q2q3 - q0q1);
   hz = 2*mx*(q1q3 - q0q2) + 2*my*(q2q3 + q0q1) + 2*mz*(0.5f - q1q1 - q2q2);
	 
   by = sqrtf((hx*hx) + (hy*hy));
   bz = hz;        
    
   // estimated direction of gravity and flux (v and w)
   vx = 2*(q1q3 - q0q2);
   vy = 2*(q0q1 + q2q3);
   vz = q0q0 - q1q1 - q2q2 + q3q3;

   wx = 2*by*(q1q2 + q0q3) + 2*bz*(q1q3 - q0q2);
   wy = 2*by*(0.5f - q1q1 - q3q3) + 2*bz*(q0q1 + q2q3);
   wz = 2*by*(q2q3 - q0q1) + 2*bz*(0.5f - q1q1 - q2q2);
           
   // error is sum of cross product between reference direction of fields and direction measured by sensors
   ex = ((ay*vz - az*vy) + (my*wz - mz*wy)) / 2;//   ( / 2) by wong
   ey = ((az*vx - ax*vz) + (mz*wx - mx*wz)) / 2;
   ez = ((ax*vy - ay*vx) + (mx*wy - my*wx)) / 2;
   
//   halft=GET_NOWTIME();		//??????????????
   
   if(ex != 0.0f && ey != 0.0f && ez != 0.0f)      //???????,?????
   {
      // integral error scaled integral gain
      exInt = exInt + ex*Ki * halft;			   //?????????
      eyInt = eyInt + ey*Ki * halft;
      ezInt = ezInt + ez*Ki * halft;
      // adjusted gyroscope measurements
      gx = gx + Kp*ex + exInt;
      gy = gy + Kp*ey + eyInt;
      gz = gz + Kp*ez + ezInt;
   }         

   // integrate quaternion rate and normalise,???????
   q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halft;
   q1 = q1 + (q0*gx + q2*gz - q3*gy)*halft;
   q2 = q2 + (q0*gy - q1*gz + q3*gx)*halft;
   q3 = q3 + (q0*gz + q1*gy - q2*gx)*halft;  
        
   // normalise quaternion
   norm = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
   q0 = q0 * norm;       //w
   q1 = q1 * norm;       //x
   q2 = q2 * norm;       //y
   q3 = q3 * norm;       //z

/*Y��ָ������������Ԫ�������Pitch  Roll  Yaw��ֻ������ҪPID����ʱ�Ž���Ԫ��ת��Ϊŷ����
����57.295780��Ϊ�˽�����ת��Ϊ�Ƕ�*/
	Yaw   = -atan2(2*q1*q2 - 2*q0*q3, -2 * q1 * q1 - 2 * q3 * q3 + 1) * 57.295780;  //ƫ���ǣ���z��ת��	
    //if(Yaw < 0 ){Yaw = Yaw + 360;}
	//if(Yaw > 360 ){Yaw = Yaw - 360;}
	Pitch =  asin(2*q2*q3 + 2*q0*q1) * 57.295780; //�����ǣ���x��ת��	 
    Roll  = -atan2(-2*q0*q2 + 2*q1*q3, -2 * q1 * q1 - 2 * q2* q2 + 1) * 57.295780; //�����ǣ���y��ת��
	
	Imu.X = Roll;//asin(2*(q0*q2-q1*q3 ))* 57.2957795f;
	Imu.Y = Pitch;//asin(2*(q0*q1+q2*q3 ))* 57.2957795f;
	Imu.Z = Yaw;
//	printf("halfT=%f  \n\r", halfT);
//  printf("Yaw=%f, Pitch=%f, Roll=%f \n\r", Yaw, Pitch, Roll);
}

/*******************************************************************************
��̬����-������Ԫ��
*******************************************************************************/
void Get_Attitude(void)
{
   AHRSupdate(init_gx, init_gy, init_gz, init_ax, init_ay, init_az, init_mx, init_my, init_mz);
	 //AHRSupdate(init_gx, init_gy, init_gz, Acc_lpf.X, Acc_lpf.Y, Acc_lpf.Z, Mag_lpf.X, Mag_lpf.Y, Mag_lpf.Z);
	//AHRSupdate(init_gx, init_gy, init_gz, Acc_lpf.X, Acc_lpf.Y, Acc_lpf.Z, init_mx, init_my, init_mz);
}

/*******************************************************************************
�Լ�����̬������ʱ��������У׼�ֶ�����У����̬��ƫ
*******************************************************************************/
void get_Attitude_bias(void)
{
	unsigned short int i;
	#if ATTITUDE_BIAS
	double Att_x = 0, Att_y = 0;// Att_z = 0;
	unsigned short count=0;
	printf("\r\n Start Attitude_bias...\r\n");
	#endif
	//float Att_Roll_Offset, Att_Pitch_Offset, Att_yaw_Offset;
	/*�Լ�����̬����ʱ�������ʱ*/
	for (i = 7000; i >0; i--)
	{
		get_mpu9150_data();		 //��ȡ����������
		Get_Attitude();			 	 //��̬����
	}
	#if ATTITUDE_BIAS
	for (i = 5000; i >0; i--)
	{
		get_mpu9150_data();		 //��ȡ����������
		Get_Attitude();			 	 //��̬����
		Att_x += Roll;
		Att_y += Pitch;
		//Att_z += Yaw;
		count++;
	}
  Att_Roll_Offset = (float)Att_x/count;
	Att_Pitch_Offset = (float)Att_y/count;
	//Att_Yaw_Offset = (float)Att_z/count;
	printf("\r\n Att_Roll_Offset=%f, Att_Pitch_Offset=%f, Att_Yaw_Offset=%f \n\r",
           Att_Roll_Offset, Att_Pitch_Offset, Att_Yaw_Offset );
	#endif
}

/*******************************************************************************
У׼��������ƫ	
*******************************************************************************/
int get_gyro_bias(void)
{
  unsigned short int i;
  signed short int gyro[3];
  signed int gyro_y=0, gyro_z=0;
  unsigned short count=0;
  unsigned char data_write[6];
	printf("\r\n Start gyro_bias...\r\n");
  for(i=0;i<5000;i++)
  {
     if(!i2cread(MPU9150_Addr, Gyro_Xout_H, 6, data_write))
	 {
	 gyro[0] = ((((signed short int)data_write[0])<<8) | data_write[1]);
	 gyro[1] = ((((signed short int)data_write[2])<<8) | data_write[3]);
	 gyro[2] = ((((signed short int)data_write[4])<<8) | data_write[5]);
	 //gyro_x += gyro[0];
	 gyro_y	+= gyro[1];
	 gyro_z	+= gyro[2];
	 count++;
	 }
  }

  //Gyro_Xout_Offset = (float)gyro_x / count;
  Gyro_Yout_Offset = (float)gyro_y / count;
  Gyro_Zout_Offset = (float)gyro_z / count;
//  printf("gyro_x=%d, gyro_y=%d, gyro_z=%d, Gyro_Xout_Offset=%f, Gyro_Yout_Offset=%f, Gyro_Zout_Offset=%f, count=%d \n\r",
//          gyro_x, gyro_y, gyro_z, Gyro_Xout_Offset, Gyro_Yout_Offset, Gyro_Zout_Offset, count);
	printf(" Gyro_Xout_Offset=%f, Gyro_Yout_Offset=%f, Gyro_Zout_Offset=%f, count=%d \n\r",
           Gyro_Xout_Offset, Gyro_Yout_Offset, Gyro_Zout_Offset, count);
	/*
	count = 0;
	gyro_x = 0;
	for(i=0;i<5000;i++)
  {
    if(!i2cread(MPU9150_Addr, Gyro_Xout_H, 6, data_write))
	 {
	 gyro[0] = ((((signed short int)data_write[0])<<8) | data_write[1]);
	 gyro_x += gyro[0];
	 count++;
	 }
  }
	//Gyro_X_Offset = (float)gyro_x / count;
	//Gyro_Xout_Offset = (Gyro_Xout_Offset + Gyro_X_Offset)/2;
	
	//gyrox_bias_init
	count = 0;
	gyro_x = 0;
	for(i=0;i<200;i++)
  {
     if(!i2cread(MPU9150_Addr, Gyro_Xout_H, 6, data_write))
		 {
			 gyro[0] = ((((signed short int)data_write[0])<<8) | data_write[1]);
			 gyro[0] = gyro[0] - Gyro_X_Offset;
			 gyro_x += gyro[0];
			 count++;
		 }
  }
	gyrox_bias_init = (float)gyro_x / count;
	gyrox_bias_init = -(gyrox_bias_init);//+15;
	cnt_6_second = 0;
*/
  return 0;
}

/*******************************************************************************
У׼���ٶ���ƫ	
*******************************************************************************/
int get_accel_bias(void)
{
  unsigned short int i;
	float num = 0;
  signed short int accel[3];
  signed int accel_x=0, accel_y=0, accel_z=0;
  unsigned short count=0;
  unsigned char data_write[6];
	printf("\r\n Start accel_bias...\r\n");
  for(i=0;i<5000;i++)
  {
		if(!i2cread(MPU9150_Addr, Accel_Xout_H, 6, data_write))
		{
			accel[0]=((((signed short int)data_write[0])<<8) | data_write[1]);// + Accel_Xout_Offset;
			accel[1]=((((signed short int)data_write[2])<<8) | data_write[3]);// + Accel_Yout_Offset;
			accel[2]=((((signed short int)data_write[4])<<8) | data_write[5]);// + Accel_Zout_Offset;
			 
			accel_x += accel[0];
			accel_y	+= accel[1];
			accel_z	+= accel[2];
			count++;
		}
  }
  Accel_Xout_Offset = (float)accel_x / count;
  Accel_Yout_Offset = (float)accel_y / count;
	num = ((float)accel_z / count);
  Accel_Zout_Offset = num - Accel_8_Scale_Factor;
//  printf("accel_x=%d, accel_y=%d, accel_z=%d, Accel_Xout_Offset=%f, Accel_Yout_Offset=%f, Accel_Zout_Offset=%f, count=%d \n\r",
//          accel_x, accel_y, accel_z, Accel_Xout_Offset, Accel_Yout_Offset, Accel_Zout_Offset, count);
  printf(" Accel_Xout_Offset=%f, Accel_Yout_Offset=%f, Accel_Zout_Offset=%f, count=%d \n\r",
           Accel_Xout_Offset, Accel_Yout_Offset, Accel_Zout_Offset, count);

  return 0;
}

/*******************************************************************************
�õ�mag��Xmax��Xmin��Ymax��Ymin��Zmax��Zmin	
*******************************************************************************/
void get_compass_bias(void)
{
  Read_MPU9150_Mag();

  if(init_mx > maxMagX)
  maxMagX = init_mx;
  if(init_mx < minMagX)
  minMagX = init_mx;

  if(init_my > maxMagY)
  maxMagY = init_my;
  if(init_my < minMagY)
  minMagY = init_my;

  if(init_mz > maxMagZ)
  maxMagZ = init_mz;
  if(init_mz < minMagZ)
  minMagZ = init_mz;
  printf("maxMagX=%f, minMagX=%f, maxMagY=%f, minMagY=%f, maxMagZ=%f, minMagZ=%f \n\r", 
          maxMagX, minMagX, maxMagY, minMagY, maxMagZ, minMagZ);  
}
/*******************************************************************************
�ռ�У׼compass	
*******************************************************************************/
void compass_calibration(void)
{ //���������Ӧ�����������Ϊ1
  if(((maxMagX - minMagX) >= (maxMagY - minMagY)) && ((maxMagX - minMagX) >= (maxMagZ - minMagZ)))
  {
    MXgain = 1.0;
	MYgain = (maxMagX - minMagX) / (maxMagY - minMagY);
	MZgain = (maxMagX - minMagX) / (maxMagZ - minMagZ);
	MXoffset = -0.5 * (maxMagX + minMagX);
	MYoffset = -0.5 * MYgain * (maxMagY + minMagY);
	MZoffset = -0.5 * MZgain * (maxMagZ + minMagZ);	 
  }
  if(((maxMagY - minMagY) > (maxMagX - minMagX)) && ((maxMagY - minMagY) >= (maxMagZ - minMagZ)))
  {
    MXgain = (maxMagY - minMagY) / (maxMagX - minMagX);
	MYgain = 1.0;
	MZgain = (maxMagY - minMagY) / (maxMagZ - minMagZ);
	MXoffset = -0.5 * MXgain * (maxMagX + minMagX);
	MYoffset = -0.5 * (maxMagY + minMagY);
	MZoffset = -0.5 * MZgain * (maxMagZ + minMagZ);    
  }
  if(((maxMagZ - minMagZ) > (maxMagX - minMagX)) && ((maxMagZ - minMagZ) > (maxMagY - minMagY)))
  {
    MXgain = (maxMagZ - minMagZ) / (maxMagX - minMagX);
	MYgain = (maxMagZ - minMagZ) / (maxMagY - minMagY);
	MZgain = 1.0;
	MXoffset = -0.5 * MXgain * (maxMagX + minMagX);
	MYoffset = -0.5 * MYgain * (maxMagY + minMagY);
	MZoffset = -0.5 * (maxMagZ + minMagZ);    
  }
  printf("MXgain=%f, MYgain=%f, MZgain=%f, MXoffset=%f, MYoffset=%f, MZoffset=%f \n\r", 
          MXgain, MYgain, MZgain, MXoffset, MYoffset, MZoffset);         
}
/*******************************************************************************
��ȡcompass���ݣ�����compassУ׼	
*******************************************************************************/
void Read_MPU9150_Mag(void)
{
  signed short int mag[3];
  unsigned char tmp[7], data_write[1];
  
  tmp[6]=0x00;
  data_write[0]=0x01;
  i2cread(Compass_Addr, Compass_ST1, 1, tmp+6);
  if(tmp[6] == 1)
  {
    i2cread(Compass_Addr, Compass_HXL, 6, tmp);
	mag[0] = (((signed short int)tmp[1]) << 8) | tmp[0];
    mag[1] = (((signed short int)tmp[3]) << 8) | tmp[2];
    mag[2] = (((signed short int)tmp[5]) << 8) | tmp[4];

//	mag[0] = ((long)mag[0] * mag_sens_adj_val[0]) >> 8;  //�����ȵ���
//    mag[1] = ((long)mag[1] * mag_sens_adj_val[1]) >> 8;
//    mag[2] = ((long)mag[2] * mag_sens_adj_val[2]) >> 8;

	init_mx =(float)mag[1];		//ת��������				
    init_my =(float)mag[0];
    init_mz =(float)-mag[2];
	i2cwrite(Compass_Addr, Compass_CNTL, 1, data_write);	 //����compass��single measurement mode
  }  
}

/*******************************************************************************
��ȡcompass���ݣ��ڳ�ʼ��mpu9150���ȶ�����mag�����ݣ���Ϊǰ���ζ�ȡ��mag������
����оƬbug	
*******************************************************************************/
int Init_MPU9150_Mag(void)
{
  unsigned char data_write[3];
  
  data_write[0]=0x02;       
  data_write[1]=0x00;
  data_write[2]=0x01;
  
  i2cwrite(MPU9150_Addr, Bypass_Enable_Cfg, 1, data_write);	 //����bypass
  Delay_ms(10);                     
  i2cwrite(MPU9150_Addr, User_Ctrl, 1, data_write+1);	 //�ر�MPU9150��I2C_MASTERģʽ������Ҫ�����
  Delay_ms(10);
  i2cwrite(Compass_Addr, Compass_CNTL, 1, data_write+2);	 //����compass��single measurement mode

  return 0;  
}

/*******************************************************************************
���ټ��� 1/Sqrt(x)��Դ������3��һ�δ��룬�����0x5f3759df���������Ĵ����4�� 	
*******************************************************************************/
float invSqrt(float x) 
{
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}
