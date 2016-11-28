#include "sys.h"        
#include "Filter.h"
#include "mpu6050.h"
#include "math.h"
#include "LCD_OWN.h"
#include "stdio.h"

/****************误差计算*********************/
float err_x = 0;
void Err_Gry(void)
{
  u8 i=100;
	short _err_x,_err_y,_err_z;
    while(i--)
		{
		MPU_Get_Gyroscope(&_err_x,&_err_y,&_err_z);
		Delay_ms(10);
		err_x += _err_x;
		}
	  err_x /= 100.0;
}

/****************角度速度转换角度计算*********************/
//float Read_Gry(void)
// {
//    static float angleG=0;
//    float read_gyro_y;
//    float Angle_gyro; 
//    short x,y,z;		
//                                        //角速度量程见配置   本处使用2000 deg/s。scal系数为16.4 LSB

//         MPU_Get_Gyroscope(&x,&y,&z); //静止时角速度Y轴输出            //Gyro_y_offset计算方法gyro静止时候N多个数据的算术均值        
//         Angle_gyro= - (x-err_x)/16.4 ; //去除零点偏移，计算角速度值,负号为方向处理
//         angleG=angleG+Angle_gyro*0.036;	 //角速度积分
////	     sprintf(disp,"X:%d-%.2f",GetData(GYRO_XOUT_H),read_gyro_y); //不知为什么去掉这个不行。。。
////	     OLED_DIS_8x16(0,2,disp);                          
//        return angleG;
// }
/****************加速度转换角度计算***********************/
//float Read_ACCEL(void)
//{
// //------加速度--------------------------
//
//        //范围为2g时，换算关系：16384 LSB/g
//         //角度较小时，x=sinx得到角度（弧度）, deg = rad*180/3.14
//         //因为x>=sinx,故乘以1.3适当放大
//		float Angle_ax,Accel_x;
//
//        Accel_x  = GetData(ACCEL_XOUT_H);          //读取X轴加速度
//        Angle_ax = Accel_x/16384;   //去除零点偏移,计算得到角度（弧度）
//		if(Angle_ax>=1 )
//			return Angle_ax=90;
//		if(Angle_ax<=-1)
//			return Angle_ax=-90;
//		else
//        Angle_ax = asin(Angle_ax)*180/3.14;     //弧度转换为度,		
//		return Angle_ax;
//
//}
//******角度参数************

float Gyro_y;        //Y轴陀螺仪数据暂存
float Accel_x,Accel_z,Accel_y;             //各轴角度
short Angle_ax,Angle_az,Angle_ay;         //各轴角加速度
float _Angle_ax,_Angle_az,_Angle_ay;      //各轴角加速度——暂存
short Angle_gx,Angle_gz,Angle_gy;         //各轴重力加速度
float _Angle_gx,_Angle_gz,_Angle_gy;      //各轴重力加速度——暂存
float Angle = 0;
uchar value;                              //角度正负极性标记 
   	
 //******卡尔曼参数************
                 
float   Q_angle=0.001;  
 float   Q_gyro=0.003;
 float   R_angle=0.5;
 float   dt=0.001;                          //dt为kalman滤波器采样时间;
char    C_0 = 1;
 float   Q_bias, Angle_err;
 float   PCt_0, PCt_1, E;
 float   K_0, K_1, t_0, t_1;
 float   Pdot[4] ={0,0,0,0};
 float   PP[2][2] = { { 1, 0 },{ 0, 1 } };

 //*********************************************************
 // 卡尔曼滤波
//*********************************************************                
 //在程序中利用Angle+=(Gyro - Q_bias) * dt计算出陀螺仪积分出的角度，其中Q_bias是陀螺仪偏差。
//此时利用陀螺仪积分求出的Angle相当于系统的估计值，得到系统的观测方程；而加速度计检测的角度Accel相当于系统中的测量值，得到系统状态方程。
//程序中Q_angle和Q_gyro分别表示系统对加速度计及陀螺仪的信任度。根据Pdot = A*P + P*A' + Q_angle计算出先验估计协方差的微分，用于将当前估计值进行线性化处理。其中A为雅克比矩阵。  
 //随后计算系统预测角度的协方差矩阵P。计算估计值Accel与预测值Angle间的误差Angle_err。
//计算卡尔曼增益K_0,K_1，K_0用于最优估计值，K_1用于计算最优估计值的偏差并更新协方差矩阵P。
//通过卡尔曼增益计算出最优估计值Angle及预测值偏差Q_bias，此时得到最优角度值Angle及角速度值。


//Kalman滤波，20MHz的处理时间约0.77ms；                        

void Kalman_Filter(float Accel,float Gyro)                  
 {
         Angle+=(Gyro - Q_bias) * dt; //先验估计

        
         Pdot[0]=Q_angle - PP[0][1] - PP[1][0]; // Pk-先验估计误差协方差的微分

         Pdot[1]=- PP[1][1];
         Pdot[2]=- PP[1][1];
         Pdot[3]=Q_gyro;
         
         PP[0][0] += Pdot[0] * dt;   // Pk-先验估计误差协方差微分的积分
         PP[0][1] += Pdot[1] * dt;   // =先验估计误差协方差
         PP[1][0] += Pdot[2] * dt;
         PP[1][1] += Pdot[3] * dt;
                 
         Angle_err = Accel - Angle;        //zk-先验估计
        
         PCt_0 = C_0 * PP[0][0];
         PCt_1 = C_0 * PP[1][0];
         
         E = R_angle + C_0 * PCt_0;
         
         K_0 = PCt_0 / E;
         K_1 = PCt_1 / E;
         
         t_0 = PCt_0;
         t_1 = C_0 * PP[0][1];

         PP[0][0] -= K_0 * t_0;                 //后验估计误差协方差
        PP[0][1] -= K_0 * t_1;
         PP[1][0] -= K_1 * t_0;
         PP[1][1] -= K_1 * t_1;
                 
         Angle        += K_0 * Angle_err;         //后验估计
        Q_bias        += K_1 * Angle_err;         //后验估计
        Gyro_y   = Gyro - Q_bias;         //输出值(后验估计)的微分=角速度

}



 //*********************************************************
 // 倾角计算（卡尔曼融合）
//*********************************************************

void Angle_Calcu(void)         
 {
         //------加速度--------------------------

        //范围为2g时，换算关系：16384 LSB/g
         //角度较小时，x=sinx得到角度（弧度）, deg = rad*180/3.14
         //因为x>=sinx,故乘以1.3适当放大
		u8 a[20];
    MPU_Get_Accelerometer(&Angle_gx,&Angle_gy,&Angle_gz);
    _Angle_gy = Angle_gy /16384.0; 
		_Angle_gz = Angle_gz /16384.0;                                    //去除零点偏移,计算得到角度（弧度）
		Accel_x = atan2(_Angle_gy,_Angle_gz)*180/3.14;      //弧度转换为度,		


     //-------角速度-------------------------

        //范围为2000deg/s时，换算关系：16.4 LSB/(deg/s)
         MPU_Get_Gyroscope(&Angle_ax,&Angle_ay,&Angle_az); //静止时角速度Y轴输出            //Gyro_y_offset计算方法gyro静止时候N多个数据的算术均值        
         _Angle_ax= - (Angle_ax-err_x)/16.4 ; //去除零点偏移，计算角速度值,负号为方向处理
        //Angle_gy = Angle_gy + Gyro_y*0.01;  //角速度积分得到倾斜角度.        

         
//         //-------卡尔曼滤波融合-----------------------

        Kalman_Filter(Accel_x,_Angle_ax);       //卡尔曼滤波计算倾角


//        /*//-------互补滤波-----------------------

//        //补偿原理是取当前倾角和加速度获得倾角差值进行放大，然后与
//    //陀螺仪角速度叠加后再积分，从而使倾角最跟踪为加速度获得的角度
//        //0.5为放大倍数，可调节补偿度；0.01为系统周期10ms        
//                 
//         Angle = Angle + (((Angle_ax-Angle)*0.5 + Gyro_y)*0.01);*/
//                                                                                                                           
 } 
