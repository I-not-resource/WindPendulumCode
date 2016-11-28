#include "mpu9150.h"
#include "AHRS_Attitude.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "IIC.h"
#include "usart.h"
#include <stdio.h>
#include "Sys.h"
#include "stm32i2c.h"

//#define DEFAULT_MPU_HZ  1000

u8 Acc_CALIBRATED = 0;
u8 Gyro_CALIBRATED = 0;


void Init_Mpu9150_Sensor_Att(void)
{
	init_mpu9150();	//初始化mpu9150
	
	Calibra_GyrAcc();
	
	init_quaternion();        //得到初始化四元数
	
//	get_Attitude_bias();
	
		Calibra_Att();
	//Delay_ms(3000);		  //延迟3s等待系统稳定,等待接收机与遥控连接，等待是否启动compass校准
	Calibra_Mag();		//磁力计校
}

u8 Test_mpu9150(void)
{
	u8 res;
	IIC_SimulationConfig();//初始化IIC总线
	//IIC_Init();
	res=i2cRead_Byte(MPUADDR, 0X75);
	printf("mpu6050 chip id is 0x%x",res);
	if(res==MPUADDR)//器件ID正确
	{
		return 1;
 	}
	return 0;
}


void init_mpu9150(void)
{
  int result;
	int count = 1;
  unsigned char data_write[1];

	Test_mpu9150();
  result = mpu_init();
	while(result)
	{
		printf("\r\n mpu initialization error......\n\r ");
		count++;
		result = mpu_init();
		Delay_ms(50);
	}
  if(!result)
  {
	  //mpu_init();
		if (count != 1)
			printf("\r\n mpu initialization complete %d time ......\n\r ",count);
		else
			printf("\r\n mpu initialization complete  ......\n\r ");
		count = 1;
	  //开启加速度计、陀螺仪、磁力计，使用电子罗盘，要加入INV_XYZ_COMPASS
	  if(!mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS))
	  {
	  	 printf("\r\n mpu_set_sensor complete ......\n\r");
	  }
	  else
	  {
	  	 printf("\r\n mpu_set_sensor error ......\n\r");
			 while(mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS)){
				Delay_ms(50);
				 count++;
			 }
			 printf("\r\n mpu_set_sensor complete %d time ......\n\r ",count);
			 count = 1;
	  }

	  //设置MPU9150的时钟源为GX的PLL
	  data_write[0]=0x01;				//GX_PLL:0x01
	  if(!i2cwrite(MPU9150_Addr, P_M_1, 1, data_write))
	  {
	  	 printf("\r\n set_mpu9150_ClockSource complete ......\n\r");
	  }
	  else
	  {
	  	 printf("\r\n set_mpu9150_ClockSource error ......\n\r");
			 while(i2cwrite(MPU9150_Addr, P_M_1, 1, data_write)){
			 Delay_ms(50);
				count++;
			 }
		 	 printf("\r\n set_mpu9150_ClockSource complete %d time ......\n\r ",count);
			 count = 1;
	  }

		//by wong// 设置为±2000°
	  //设置陀螺仪测量范：+-500度/s
	  if(!mpu_set_gyro_fsr(500))
	  {
	  	 printf("\r\n mpu_set_gyro_fsr complete ......\n\r");
	  }
	  else
	  {
	  	 printf("\r\n mpu_set_gyro_fsr error ......\n\r");
			 while(mpu_set_gyro_fsr(500)){
				Delay_ms(50);
				count++;
			 }
			 printf("\r\n mpu_set_gyro_fsr complete %d time ......\n\r ",count);
			 count = 1;
	  }

		//////by wong// 设置为±8G
	  //设置加速度计测量范围：+-4G
	  if(!mpu_set_accel_fsr(8))
	  {
	  	 printf("\r\n mpu_set_accel_fsr complete ......\n\r");
	  }
	  else
	  {
	  	 printf("\r\n mpu_set_accel_fsr error ......\n\r");
			 while(mpu_set_accel_fsr(4)){
				Delay_ms(50);
				count++;
		 	 }
		 	 printf("\r\n mpu_set_accel_fsr complete %d time ......\n\r ",count);
			 count = 1;
	  }

	  //设置加速度计的低通滤波器，防震动，由于朗宇X2212 kv980电机电压11.1v下转速为120r/s，因此DLPF选98hz，后面可考虑加权平均滤波
	  if(!mpu_set_lpf(Accel_FILTER_98HZ))
	  {
	  	 printf("\r\n mpu_set_lpf complete ......\n\r");
	  }
	  else
	  {
	  	 printf("\r\n mpu_set_lpf error ......\n\r");
		 	 while(mpu_set_lpf(Accel_FILTER_98HZ)){
				Delay_ms(50);
				count++;
		 	 }
		 	 printf("\r\n mpu_set_lpf complete %d time ......\n\r ",count);
			 count = 1;
	  }

	  //设置采样率1kHz
	  if(!mpu_set_sample_rate(DEFAULT_MPU_HZ))
	  {
	  	 printf("\r\n mpu_set_sample_rate complete ......\n\r");
	  }
	  else
	  {
	  	 printf("\r\n mpu_set_sample_rate error ......\n\r");
		 	 while(mpu_set_sample_rate(DEFAULT_MPU_HZ)){
				Delay_ms(50);
				count++;
		 	 }
		 	 printf("\r\n mpu_set_sample_rate complete %d time ......\n\r ",count);
			 count = 1;
	  }
		
		#if ACCEL_BIAS
		//通过5000次静置加权平均求出加速度零偏OFFSET
	  if(!get_accel_bias())
	  {
	  	 printf("\r\n get_accel_bias complete ......\n\r");
	  }
	  else
	  {
	  	 printf("\r\n get_accel_bias error ......\n\r");
		 	 while(get_accel_bias()){
				Delay_ms(50);
				count++;
		 	 }
		 	 printf("\r\n get_accel_bias complete %d time ......\n\r ",count);
			 count = 1;
	  }
		#endif
		
		#if GYRO_BIAS
	  //通过5000次静置加权平均求出陀螺仪零偏OFFSET
	  if(!get_gyro_bias())
	  {
	  	 printf("\r\n get_gyro_bias complete ......\n\r");
	  }
	  else
	  {
	  	 printf("\r\n get_gyro_bias error ......\n\r");
		 	 while(get_gyro_bias()){
				Delay_ms(50);
				count++;
		 	 }
		 	 printf("\r\n get_gyro_bias complete %d time ......\n\r ",count);
			 count = 1;
	  }
		#endif

	  //先读几次mag数据，因为前几次mag数据有错误，芯片bug
	  if(!Init_MPU9150_Mag())
	  {
	  	 printf("\r\n Init_MPU9150_Mag complete ......\n\r");
	  }
	  else
	  {
	  	 printf("\r\n Init_MPU9150_Mag error ......\n\r");
		 	 while(Init_MPU9150_Mag()){
				Delay_ms(50);
				 count++;
		 	 }
		 	 printf("\r\n Init_MPU9150_Mag complete %d time ......\n\r ",count);
			 count = 1;
	  }
  }
  else
  {
      printf("\r\n mpu initialization error......\n\r ");
	  while(1);
  }
  printf("\r\n Start Calculating .....\n\r");
}

//void Calibra_Mag(void)
//{
//	/*
//		校正磁力计：若PA0为高电平，则校正磁力计，并且在PA0变为低电平时将校正参数写入flash.
//								若PA0为低电平，则读flash，将校正参数读出
//	*/
//	if((PWMInCh7 < 1050) && (PWMInCh7 != 0)) //启动校准compass
//  {	
//		Not_Touch(2);
//		while(!((PWMInCh7 > 1900) && (PWMInCh7 != 0) ))//
//		{
//			get_compass_bias();     
//		}
//		compass_calibration();
//		//Write_Mag_Para();
//		Write_MAG_Para();
//		printf("Ch1=%dus Ch2=%dus Ch3=%dus Ch4=%dus Ch5=%dus Ch6=%dus, Ch7=%dus\r\n",
//					PWMInCh1, PWMInCh2, PWMInCh3, PWMInCh4, PWMInCh5, PWMInCh6, PWMInCh7);//打印总的高点平时间
//		printf("Write_Mag : MXgain=%f, MYgain=%f, MZgain=%f, MXoffset=%f, MYoffset=%f, MZoffset=%f \n\r", 
//            MXgain, MYgain, MZgain, MXoffset, MYoffset, MZoffset);
//	}
//	else//不需要再次校准磁力计时，调用上次校准时的数据
//  { 
//    //Read_Mag_Para();
//		Read_MAG_Para();
//		printf("Read_Mag : MXgain=%f, MYgain=%f, MZgain=%f, MXoffset=%f, MYoffset=%f, MZoffset=%f \n\r", 
//            MXgain, MYgain, MZgain, MXoffset, MYoffset, MZoffset);
//  }
//	Not_Touch(1);
//	/*校正结束*/
//}

//void Calibra_GyrAcc(void)
//{
//	int count = 1;
//	
//	while((PWMInCh5 > 2000) && (PWMInCh5 < 2050) )//准备进入校准
//	{
//		Bias_Note(0);
//		printf("\r\n waiting for Gry,Acc,Att Bras. Push CH5 low to continue......\n\r ");
//		Delay_ms(500);
//	}
//	
//	if((PWMInCh5 < 1050) && (PWMInCh5 != 0)) //启动校准六轴及姿态
//  {	
//		Bias_Note(2);
//		
//		//通过5000次静置加权平均求出加速度零偏OFFSET
//	  if(!get_accel_bias())
//	  {
//	  	 printf("\r\n get_accel_bias complete ......\n\r");
//	  }
//	  else
//	  {
//	  	 printf("\r\n get_accel_bias error ......\n\r");
//		 	 while(get_accel_bias()){
//				Delay_ms(50);
//				count++;
//		 	 }
//		 	 printf("\r\n get_accel_bias complete %d time ......\n\r ",count);
//			 count = 1;
//	  }
//		
//	  //通过5000次静置加权平均求出陀螺仪零偏OFFSET
//	  if(!get_gyro_bias())
//	  {
//	  	 printf("\r\n get_gyro_bias complete ......\n\r");
//	  }
//	  else
//	  {
//	  	 printf("\r\n get_gyro_bias error ......\n\r");
//		 	 while(get_gyro_bias()){
//				Delay_ms(50);
//				count++;
//		 	 }
//		 	 printf("\r\n get_gyro_bias complete %d time ......\n\r ",count);
//			 count = 1;
//	  }
//	}
//	else
//	{
//		Read_Gyr_Acc_Att_Para();
//	}
//}

//void Calibra_Att(void)
//{
//	unsigned short int i;
//	double Att_x = 0, Att_y = 0;// Att_z = 0;
//	unsigned short count=0;
//	printf("\r\n Start Attitude_bias...\r\n");
//	
//	Att_Roll_Offset = 0;
//	Att_Pitch_Offset = 0;
//	
//	for (i = 5000; i >0; i--)
//	{
//		get_mpu9150_data();		 //读取传感器数据
//		Get_Attitude();			 	 //姿态解算
//	}
//	
//	for (i = 5000; i >0; i--)
//	{
//		get_mpu9150_data();		 //读取传感器数据
//		Get_Attitude();			 	 //姿态解算
//		Att_x += Roll;
//		Att_y += Pitch;
//		//Att_z += Yaw;
//		count++;
//	}
//  Att_Roll_Offset = (float)Att_x/count;
//	Att_Pitch_Offset = (float)Att_y/count;
//	//Att_Yaw_Offset = (float)Att_z/count;
//	printf("\r\n Att_Roll_Offset=%f, Att_Pitch_Offset=%f, Att_Yaw_Offset=%f \n\r",
//           Att_Roll_Offset, Att_Pitch_Offset, Att_Yaw_Offset );
//	Write_Gyr_Acc_Att_Para();
//	Bias_Note(5);
//}

