#include "sys.h"        
#include "Filter.h"
#include "mpu6050.h"
#include "math.h"
#include "LCD_OWN.h"
#include "stdio.h"

/****************������*********************/
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

/****************�Ƕ��ٶ�ת���Ƕȼ���*********************/
//float Read_Gry(void)
// {
//    static float angleG=0;
//    float read_gyro_y;
//    float Angle_gyro; 
//    short x,y,z;		
//                                        //���ٶ����̼�����   ����ʹ��2000 deg/s��scalϵ��Ϊ16.4 LSB

//         MPU_Get_Gyroscope(&x,&y,&z); //��ֹʱ���ٶ�Y�����            //Gyro_y_offset���㷽��gyro��ֹʱ��N������ݵ�������ֵ        
//         Angle_gyro= - (x-err_x)/16.4 ; //ȥ�����ƫ�ƣ�������ٶ�ֵ,����Ϊ������
//         angleG=angleG+Angle_gyro*0.036;	 //���ٶȻ���
////	     sprintf(disp,"X:%d-%.2f",GetData(GYRO_XOUT_H),read_gyro_y); //��֪Ϊʲôȥ��������С�����
////	     OLED_DIS_8x16(0,2,disp);                          
//        return angleG;
// }
/****************���ٶ�ת���Ƕȼ���***********************/
//float Read_ACCEL(void)
//{
// //------���ٶ�--------------------------
//
//        //��ΧΪ2gʱ�������ϵ��16384 LSB/g
//         //�ǶȽ�Сʱ��x=sinx�õ��Ƕȣ����ȣ�, deg = rad*180/3.14
//         //��Ϊx>=sinx,�ʳ���1.3�ʵ��Ŵ�
//		float Angle_ax,Accel_x;
//
//        Accel_x  = GetData(ACCEL_XOUT_H);          //��ȡX����ٶ�
//        Angle_ax = Accel_x/16384;   //ȥ�����ƫ��,����õ��Ƕȣ����ȣ�
//		if(Angle_ax>=1 )
//			return Angle_ax=90;
//		if(Angle_ax<=-1)
//			return Angle_ax=-90;
//		else
//        Angle_ax = asin(Angle_ax)*180/3.14;     //����ת��Ϊ��,		
//		return Angle_ax;
//
//}
//******�ǶȲ���************

float Gyro_y;        //Y�������������ݴ�
float Accel_x,Accel_z,Accel_y;             //����Ƕ�
short Angle_ax,Angle_az,Angle_ay;         //����Ǽ��ٶ�
float _Angle_ax,_Angle_az,_Angle_ay;      //����Ǽ��ٶȡ����ݴ�
short Angle_gx,Angle_gz,Angle_gy;         //�����������ٶ�
float _Angle_gx,_Angle_gz,_Angle_gy;      //�����������ٶȡ����ݴ�
float Angle = 0;
uchar value;                              //�Ƕ��������Ա�� 
   	
 //******����������************
                 
float   Q_angle=0.001;  
 float   Q_gyro=0.003;
 float   R_angle=0.5;
 float   dt=0.001;                          //dtΪkalman�˲�������ʱ��;
char    C_0 = 1;
 float   Q_bias, Angle_err;
 float   PCt_0, PCt_1, E;
 float   K_0, K_1, t_0, t_1;
 float   Pdot[4] ={0,0,0,0};
 float   PP[2][2] = { { 1, 0 },{ 0, 1 } };

 //*********************************************************
 // �������˲�
//*********************************************************                
 //�ڳ���������Angle+=(Gyro - Q_bias) * dt����������ǻ��ֳ��ĽǶȣ�����Q_bias��������ƫ�
//��ʱ���������ǻ��������Angle�൱��ϵͳ�Ĺ���ֵ���õ�ϵͳ�Ĺ۲ⷽ�̣������ٶȼƼ��ĽǶ�Accel�൱��ϵͳ�еĲ���ֵ���õ�ϵͳ״̬���̡�
//������Q_angle��Q_gyro�ֱ��ʾϵͳ�Լ��ٶȼƼ������ǵ����ζȡ�����Pdot = A*P + P*A' + Q_angle������������Э�����΢�֣����ڽ���ǰ����ֵ�������Ի���������AΪ�ſ˱Ⱦ���  
 //������ϵͳԤ��Ƕȵ�Э�������P���������ֵAccel��Ԥ��ֵAngle������Angle_err��
//���㿨��������K_0,K_1��K_0�������Ź���ֵ��K_1���ڼ������Ź���ֵ��ƫ�����Э�������P��
//ͨ�������������������Ź���ֵAngle��Ԥ��ֵƫ��Q_bias����ʱ�õ����ŽǶ�ֵAngle�����ٶ�ֵ��


//Kalman�˲���20MHz�Ĵ���ʱ��Լ0.77ms��                        

void Kalman_Filter(float Accel,float Gyro)                  
 {
         Angle+=(Gyro - Q_bias) * dt; //�������

        
         Pdot[0]=Q_angle - PP[0][1] - PP[1][0]; // Pk-����������Э�����΢��

         Pdot[1]=- PP[1][1];
         Pdot[2]=- PP[1][1];
         Pdot[3]=Q_gyro;
         
         PP[0][0] += Pdot[0] * dt;   // Pk-����������Э����΢�ֵĻ���
         PP[0][1] += Pdot[1] * dt;   // =����������Э����
         PP[1][0] += Pdot[2] * dt;
         PP[1][1] += Pdot[3] * dt;
                 
         Angle_err = Accel - Angle;        //zk-�������
        
         PCt_0 = C_0 * PP[0][0];
         PCt_1 = C_0 * PP[1][0];
         
         E = R_angle + C_0 * PCt_0;
         
         K_0 = PCt_0 / E;
         K_1 = PCt_1 / E;
         
         t_0 = PCt_0;
         t_1 = C_0 * PP[0][1];

         PP[0][0] -= K_0 * t_0;                 //����������Э����
        PP[0][1] -= K_0 * t_1;
         PP[1][0] -= K_1 * t_0;
         PP[1][1] -= K_1 * t_1;
                 
         Angle        += K_0 * Angle_err;         //�������
        Q_bias        += K_1 * Angle_err;         //�������
        Gyro_y   = Gyro - Q_bias;         //���ֵ(�������)��΢��=���ٶ�

}



 //*********************************************************
 // ��Ǽ��㣨�������ںϣ�
//*********************************************************

void Angle_Calcu(void)         
 {
         //------���ٶ�--------------------------

        //��ΧΪ2gʱ�������ϵ��16384 LSB/g
         //�ǶȽ�Сʱ��x=sinx�õ��Ƕȣ����ȣ�, deg = rad*180/3.14
         //��Ϊx>=sinx,�ʳ���1.3�ʵ��Ŵ�
		u8 a[20];
    MPU_Get_Accelerometer(&Angle_gx,&Angle_gy,&Angle_gz);
    _Angle_gy = Angle_gy /16384.0; 
		_Angle_gz = Angle_gz /16384.0;                                    //ȥ�����ƫ��,����õ��Ƕȣ����ȣ�
		Accel_x = atan2(_Angle_gy,_Angle_gz)*180/3.14;      //����ת��Ϊ��,		


     //-------���ٶ�-------------------------

        //��ΧΪ2000deg/sʱ�������ϵ��16.4 LSB/(deg/s)
         MPU_Get_Gyroscope(&Angle_ax,&Angle_ay,&Angle_az); //��ֹʱ���ٶ�Y�����            //Gyro_y_offset���㷽��gyro��ֹʱ��N������ݵ�������ֵ        
         _Angle_ax= - (Angle_ax-err_x)/16.4 ; //ȥ�����ƫ�ƣ�������ٶ�ֵ,����Ϊ������
        //Angle_gy = Angle_gy + Gyro_y*0.01;  //���ٶȻ��ֵõ���б�Ƕ�.        

         
//         //-------�������˲��ں�-----------------------

        Kalman_Filter(Accel_x,_Angle_ax);       //�������˲��������


//        /*//-------�����˲�-----------------------

//        //����ԭ����ȡ��ǰ��Ǻͼ��ٶȻ����ǲ�ֵ���зŴ�Ȼ����
//    //�����ǽ��ٶȵ��Ӻ��ٻ��֣��Ӷ�ʹ��������Ϊ���ٶȻ�õĽǶ�
//        //0.5Ϊ�Ŵ������ɵ��ڲ����ȣ�0.01Ϊϵͳ����10ms        
//                 
//         Angle = Angle + (((Angle_ax-Angle)*0.5 + Gyro_y)*0.01);*/
//                                                                                                                           
 } 
