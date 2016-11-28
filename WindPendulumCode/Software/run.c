#include "run.h"
#include "includes.h"
#include "string.h"
#if __RUN_ENABLE
u16 Speed1,Speed2,Speed3,Speed4;  //设置风机的速度
char length,Angle0,Anglex,Angley;                //设备摆动的线长、角度
u16 direction;                    //设置设备的方向
u8 statue;
float Xita = 0;
u8 Length = 0;    //定义半长度
float Beita,Forward = 0;  //定义方向
u8 R=0;                 //画圆半径
u8 _i = 0;
/***************************************************************************
函数名称：
函数功能：
函数备注：
***************************************************************************/
void BF1_Init(void)
{
	TIM_ITConfig(TIM7,TIM_IT_Update,DISABLE);
  PID_BP1Init();
	USART6_Config() ;
	Err_Deal();
	PWMx4_Init();
	while(PAin(0) == 0){;}
	 MOTOR_X_P = 999;
	 Delay_ms(250);
	
}

/*------------------------------------------------------------------------*/
void Mode_BF1(void)
{
	while ((angle[0] < 16) && (angle[0] > -16))
   {
		MPU6050_data();
    PID_del2(0,Y);
		Wind_drive_YStop();
    Up_Mode();
   }
	 	if((angle[0] > 15.0f) || (angle[0] < -15.0f) )
		BEEP = 0;
		PID_del1(17,X);
		Wind_drive_XGoal();
}


void BF1_End(void)
{
	 MOTOR_Y_P = 0; 
	 MOTOR_Y_N = 0;
	 MOTOR_X_P = 0;
	 MOTOR_X_N = 0;
	 BEEP = 1;
	TIM7Line_Init();
	
}

void Base1(void)
{
	BF1_Init();
	while(PAin(0) == 0)
		Mode_BF1();
	BF1_End();
}

/***************************************************************************
函数名称：
函数功能：
函数备注：
***************************************************************************/
void BF2_Init(void)
{
	Input2();
	Length /= 2;
	TIM_ITConfig(TIM7,TIM_IT_Update,DISABLE);
  PID_BP2Init();
	USART6_Config() ;
	Err_Deal();
	PWMx4_Init();
	Xita = atan2(Length,_gan) * 180 / 3.14;
	while(PAin(0) == 0){;}
	 MOTOR_X_P = 999;
	 Delay_ms(250);
	 while ((angle[0] < (Xita/1.5f)) && (angle[0] > -Xita/1.5f))
   {
		MPU6050_data();

    PID_del2(0,Y);
		Wind_drive_YStop();
		PID_del1(80,X);
		Wind_drive_XGoal();
   }
   while ((angle[0] < Xita) && (angle[0] > -Xita))
   {
		MPU6050_data();
		if(angle[0] > Xita)
			pid[X].Kp       = 55;
		else
			pid[X].Kp       = 85;
    Up_Mode();
    PID_del2(0,Y);
		Wind_drive_YStop();
  	if((angle[0] > Xita) || (angle[0] < -Xita) )
		BEEP = 0;
   }
}

/*------------------------------------------------------------------------*/
void Mode_BF2(void)
{
		MPU6050_data();
    PID_del2(0,Y);
		Wind_drive_YStop();
		PID_del1(Xita,X);
		Wind_drive_XGoal();
}

void BF2_End(void)
{
	 MOTOR_Y_P = 0; 
	 MOTOR_Y_N = 0;
	 MOTOR_X_P = 0;
	 MOTOR_X_N = 0;
		TIM7Line_Init();
}

void Base2(void)
{
	BF2_Init();
	while(PAin(0) == 0)
		Mode_BF2();
	BF2_End();
}

void Input2(void)
{
  u8 wenzi[20];
	Length = 30;
	while(MEMU_SURE == 0)
	{
		if(MoveLive > 55)
		{
	    Length+=1;
			MoveLive=50;
	  }
		else if(MoveLive < 35)
		{
	    Length-=1;
			MoveLive=50;
	  }
		else if(MoveLiveB > 55)
		{
	    Length+=10;
			MoveLiveB=50;
	  }
		else if(MoveLiveB < 45)
		{
	    Length-=10;
			MoveLiveB=50;
	  }
    sprintf(wenzi,"%d",Length);
		LCD_ShowWord(204,224,24,"    ",0);
		LCD_ShowWord(204,224,24,wenzi,0);
	}
}
/***************************************************************************
函数名称：
函数功能：
函数备注：
***************************************************************************/
void BF3_Init(void)
{
	Input3();
//  Forward = 120;
	TIM_ITConfig(TIM7,TIM_IT_Update,DISABLE);
  PID_BP3Init();
	USART6_Config() ;
	Err_Deal();
	PWMx4_Init();
	if(Forward < 0)
		Forward += 180;
	Xita  = atan2(BF3_DLong,_gan) * 180 / 3.14;
	Beita = Forward/180*3.14;
		
	while(PAin(0) == 0){;}
		MOTOR_X_P = 999;
		Delay_ms(250);
}

void Mode_BF3(void)
{
	float _y_goal,x0;
	if(Forward  == 0||Forward ==180)
	{
		MPU6050_data();
		PID_del1(Xita,Y);
		Wind_drive_YGoal();
	}
	else if(Forward == 90)
	{
	  MPU6050_data();
		PID_del1(Xita,X);
		Wind_drive_XGoal();
	}
	else
	{
	  MPU6050_data();
		PID_del1(Xita,X);
		Wind_drive_XGoal();
		PID_del1(_y_goal,Y);
		if((Forward > 0) &&(Forward <90))
			Wind_drive_YGoalBF3_0();
		else if((Forward > 90) &&(Forward < 180))
			Wind_drive_YGoalBF3_1();
	}
		if(((angle[1] > _y_goal) && (angle[0] > Xita)) ||
	      ((angle[1] < -_y_goal)&& (angle[0] < Xita)))
		BEEP = 0;
}
void BF3_End(void)
{
	 MOTOR_Y_P = 0; 
	 MOTOR_Y_N = 0;
	 MOTOR_X_P = 0;
	 MOTOR_X_N = 0;
	 TIM7Line_Init();
}

void Base3(void)
{
	BF3_Init();
	while(PAin(0) == 0)
		Mode_BF3();
	BF3_End();
}

void Input3(void)
{
  u8 wenzi[20];
	Forward = 0;
	while(MEMU_SURE == 0)
	{
		if(MoveLive > 55)
		{
	    Forward+=1;
			MoveLive=50;
	  }
		else if(MoveLive < 45)
		{
	    Forward-=1;
			MoveLive=50;
	  }
		else if(MoveLiveB > 55)
		{
	    Forward+=10;
			MoveLiveB=50;
	  }
		else if(MoveLiveB < 45)
		{
	    Forward-=10;
			MoveLiveB=50;
	  }
    sprintf(wenzi,"%.2f",Forward);
		LCD_ShowWord(228,224,24,"    ",0);
		LCD_ShowWord(228,224,24,wenzi,0);
		Delay_ms(200);
	}
}

/***************************************************************************
函数名称：
函数功能：
函数备注：
***************************************************************************/
void BF4_Init(void)
{
	TIM_ITConfig(TIM7,TIM_IT_Update,DISABLE);
  PID_BP4Init();
	USART6_Config() ;
	Err_Deal();
	PWMx4_Init();
	while(PAin(0) == 0){;}
	 Delay_ms(400);
	
}

void Mode_BF4(void)
{
	  MPU6050_data();
    PID_del2(0,Y);
    Wind_drive_YStop();
		PID_del2(0,X);
		Wind_drive_XStop();
		if(((w[0] > -0.5f) &&(w[0] <0.5f))
			&&((w[1] > -0.5f) && (w[1] <0.5f))&&(angle[0] > -0.5f) &&(angle[0] <0.5f)
			&&((angle[1] > -0.5f) && (angle[1] <0.5f)))
		BEEP = 0;
}

void BF4_End(void)
{
	 MOTOR_Y_P = 0; 
	 MOTOR_Y_N = 0;
	 MOTOR_X_P = 0;
	 MOTOR_X_N = 0;
	 TIM7Line_Init();
}

void Base4(void)
{
	BF4_Init();
	while(PAin(0) == 0)
		Mode_BF4();
	BF4_End();
}


/***************************************************************************
函数名称：
函数功能：
函数备注：
***************************************************************************/
void IF1_Init(void)
{
	Input4();
	TIM_ITConfig(TIM7,TIM_IT_Update,DISABLE);
  PID_IF1Init();
	USART6_Config() ;
	Err_Deal();
	PWMx4_Init();
	while(PAin(0) == 0){;}
	 Delay_ms(400);
//	   R = R-((R-15)/5+2);
		 Xita = atan2(R,_gan) * 180 / 3.14;
	  while ((angle[0] < (Xita-2.0f)) && (angle[0] > -Xita+2.0f))
   {
		MPU6050_data();

    PID_del1(80,Y);
		Wind_drive_YGoal();
		PID_del1(80,X);
		Wind_drive_XGoal();

   }
}
void Mode_IF1(void)
{
	float x0_0,x0_1,x1_0,x1_1,xitaX,xitaY;

  MPU6050_data();
//			x0_0  = atan2((tan(angle[1])),(tan(angle[0])));
//			x0_1  = cos(x0_0);
//			xitaX = atan(x0_1) * (R+5)/_gan * 180 / 3.14;
//	  	if(xitaX < 0)
//				xitaX = -xitaX;
//							PID_del1(xitaX,X);
//							Wind_drive_XGoal2();
//			x1_0  = atan2((tan(angle[1])),(tan(angle[0])));
//			x1_1  = sin(x1_0);
//			xitaY = atan(x1_1) * (R+5)/_gan * 180 / 3.14;
//			if(xitaY < 0)
//				xitaY = -xitaY;
//	    PID_del1(xitaY,Y);
//      Wind_drive_YGoal();


     PID_del1(Xita,X);
	  Wind_drive_XGoal();
		
		PID_del1(Xita,Y);
    Wind_drive_YGoal();
		
		
		if(((angle[0] > -xitaX) &&(angle[0] <xitaX))
			&&((angle[1] > -xitaY) && (angle[1] <xitaY)))
		BEEP = 0;
}

void IF1_End(void)
{
	 MOTOR_Y_P = 0; 
	 MOTOR_Y_N = 0;
	 MOTOR_X_P = 0;
	 MOTOR_X_N = 0;
	 TIM7Line_Init();
}

void Improve(void)
{
	IF1_Init();
	while(PAin(0) == 0)
		Mode_IF1();
	IF1_End();
}

void Input4(void)
{
  u8 wenzi[20];
	R = 15;
	while(MEMU_SURE == 0)
	{
		if(MoveLive > 55)
		{
	    R+=1;
			MoveLive=50;
	  }
		else if(MoveLive < 45)
		{
	    R-=1;
			MoveLive=50;
	  }
		else if(MoveLiveB > 55)
		{
	    R+=10;
			MoveLiveB=50;
	  }
		else if(MoveLiveB < 45)
		{
	    R-=10;
			MoveLiveB=50;
	  }
    sprintf(wenzi,"%d",R);
		LCD_ShowWord(228,224,24,"    ",0);
		LCD_ShowWord(228,224,24,wenzi,0);
		Delay_ms(100);
	}
}
/***************************************************************************
函数名称：
函数功能：
函数备注：
***************************************************************************/
//void Mode_IF2(void)
//{
//  statue    = 1;
//  Speed1    = 999;
//	Speed2    = 999;
//	Speed3    = 999;
//	Speed4    = 999;
//	Angle0    = 0;
//	length    = 0;
//	while(1);         //等待陀螺仪初始化
//	statue    = 0;
//	
//	while(1)
//	{
//    while(1)        //设置半径
//		{
//	
//	  }
//		while(1)        //执行圆周运动，具抗扰动能力
//		{
//	
//	  }
//	}
//}








void Err_Deal(void)
{
	u8 i;
	 for(i=0;i<10;i++)
	 {
      MPU6050_data();
			Err_x  += angle[0];
			Err_y  += angle[1];
			Err_wx += w[0];
			Err_wy += w[1];
			Delay_ms(10);
	 }
	 Err_y  /= 10;
	 Err_x  /= 10;
	 Err_wx /= 10;
	 Err_wy /= 10;
	 flag_Err = 1;
}



/***************************************************************************
函数名称：
函数功能：
函数备注：
***************************************************************************/
void Up_Mode(void)
{
				MPU6050_data();
	      PID_del1(80,X);
	      Wind_drive_XGoal();
//    if((w[0] <= 1) )
//		{
//			TIM_SetCompare3(TIM4,999);
//			TIM_SetCompare1(TIM4,100);
//		}
//		 else if((w[0] > 1))
//		{
//			TIM_SetCompare3(TIM4,100);
//			TIM_SetCompare1(TIM4,999);
//		}
			
}

/***************************************************************************
函数名称：
函数功能：
函数备注：
***************************************************************************/
void MPU6050_data(void)
{
	unsigned char Temp[11];
     while(sign)
      {  
         memcpy(Temp,Re_buf,11);
         sign=0;
         if(Re_buf[0]==0x55)       //检查帧头
         {  
            switch(Re_buf[1])
            {
               case 0x51: //标识这个包是加速度包
                  a[0] = ((short)(Temp[3]<<8 | Temp[2]))/32768.0*16;      //X轴加速度
                  a[1] = ((short)(Temp[5]<<8 | Temp[4]))/32768.0*16;      //Y轴加速度
                  a[2] = ((short)(Temp[7]<<8 | Temp[6]))/32768.0*16;      //Z轴加速度
                  T    = ((short)(Temp[9]<<8 | Temp[8]))/340.0+36.25;      //温度
                  break;
               case 0x52: //标识这个包是角速度包
                  w[0] = ((short)(Temp[3]<<8| Temp[2]))/32768.0*2000;      //X轴角速度
                  w[1] = ((short)(Temp[5]<<8| Temp[4]))/32768.0*2000;      //Y轴角速度
                  w[2] = ((short)(Temp[7]<<8| Temp[6]))/32768.0*2000;      //Z轴角速度
									 if(flag_Err)
									{
									  w[0] -= Err_wx;
									  w[1] -= Err_wy;
									}
                  T    = ((short)(Temp[9]<<8| Temp[8]))/340.0+36.25;       //温度
                  break;
               case 0x53: //标识这个包是角度包
                  angle[0] = ((short)(Temp[3]<<8| Temp[2]))/32768.0*180;   //X轴滚转角（x 轴）
                  angle[1] = ((short)(Temp[5]<<8| Temp[4]))/32768.0*180;   //Y轴俯仰角（y 轴）
                  angle[2] = ((short)(Temp[7]<<8| Temp[6]))/32768.0*180;   //Z轴偏航角（z 轴）
                  T        = ((short)(Temp[9]<<8| Temp[8]))/340.0+36.25;   //温度

                  //printf("X轴角度：%.2f   Y轴角度：%.2f   Z轴角度：%.2f\r\n",angle[0],angle[1],angle[2]);
                  if(flag_Err)
									{
									  angle[0] -= Err_x;
									  angle[1] -= Err_y;
									}
                  break;
               default:  break;
            }
					}
			 }
}

#endif

