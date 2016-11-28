#include "PID.h"
#include "run.h"
#if __PID_ENABLE

u8 _fangxiang,Max;
int fi;
float a0,a1,aa,Xita_ERR[2]={0,0};
/***************************************************************************
函数名称：PID_Init
函数功能：PID控制器初始化
函数备注：
***************************************************************************/
void PID_BP1Init(void)
{
	pid[X].Sets     = 0;
	pid[X].Actuals  = 0;
	pid[X].err      = 0;
	pid[X].err_last = 0;
	pid[X].voltage  = 0;
	pid[X].integral = 0;
	pid[X].Kp       = 800;
	pid[X].Ki       = 0;
	pid[X].Kd       = 0;
	
  pid[Y].Sets     = 0;
	pid[Y].Actuals  = 0;
	pid[Y].err      = 0;
	pid[Y].err_last = 0;
	pid[Y].voltage  = 0;
	pid[Y].integral = 0;
	pid[Y].Kp       = 300;
	pid[Y].Ki       = 0;
	pid[Y].Kd       = 150;
}

/***************************************************************************
函数名称：PID_Init
函数功能：PID控制器初始化
函数备注：
***************************************************************************/
void PID_BP2Init(void)
{
	pid[X].Sets     = 0;
	pid[X].Actuals  = 0;
	pid[X].err      = 0;
	pid[X].err_last = 0;
	pid[X].voltage  = 0;
	pid[X].integral = 0;
	pid[X].Kp       = 80;
	pid[X].Ki       = 150;
	pid[X].Kd       = 150;
	
  pid[Y].Sets     = 0;
	pid[Y].Actuals  = 0;
	pid[Y].err      = 0;
	pid[Y].err_last = 0;
	pid[Y].voltage  = 0;
	pid[Y].integral = 0;
	pid[Y].Kp       = 280;
	pid[Y].Ki       = 0;
	pid[Y].Kd       = 300;
}

/***************************************************************************
函数名称：PID_Init
函数功能：PID控制器初始化
函数备注：
***************************************************************************/
void PID_BP3Init(void)
{
	pid[X].Sets     = 0;
	pid[X].Actuals  = 0;
	pid[X].err      = 0;
	pid[X].err_last = 0;
	pid[X].voltage  = 0;
	pid[X].integral = 0;
	pid[X].Kp       = 280;
	pid[X].Ki       = 0;
	pid[X].Kd       = 800;
	
  pid[Y].Sets     = 0;
	pid[Y].Actuals  = 0;
	pid[Y].err      = 0;
	pid[Y].err_last = 0;
	pid[Y].voltage  = 0;
	pid[Y].integral = 0;
	pid[Y].Kp       = 280;
	pid[Y].Ki       = 0;
	pid[Y].Kd       = 800;
}

/***************************************************************************
函数名称：PID_Init
函数功能：PID控制器初始化
函数备注：
***************************************************************************/
void PID_BP4Init(void)
{
	pid[X].Sets     = 0;
	pid[X].Actuals  = 0;
	pid[X].err      = 0;
	pid[X].err_last = 0;
	pid[X].voltage  = 0;
	pid[X].integral = 0;
	pid[X].Kp       = 300;
	pid[X].Ki       = 0;
	pid[X].Kd       = 480;
	
  pid[Y].Sets     = 0;
	pid[Y].Actuals  = 0;
	pid[Y].err      = 0;
	pid[Y].err_last = 0;
	pid[Y].voltage  = 0;
	pid[Y].integral = 0;
	pid[Y].Kp       = 300;
	pid[Y].Ki       = 0;
	pid[Y].Kd       = 480;
}

/***************************************************************************
函数名称：PID_Init
函数功能：PID控制器初始化
函数备注：
***************************************************************************/
void PID_IF1Init(void)
{
	pid[X].Sets     = 0;
	pid[X].Actuals  = 0;
	pid[X].err      = 0;
	pid[X].err_last = 0;
	pid[X].voltage  = 0;
	pid[X].integral = 0;
	pid[X].Kp       = 400;
	pid[X].Ki       = 0;
	pid[X].Kd       = 0;
	
  pid[Y].Sets     = 0;
	pid[Y].Actuals  = 0;
	pid[Y].err      = 0;
	pid[Y].err_last = 0;
	pid[Y].voltage  = 0;
	pid[Y].integral = 0;
	pid[Y].Kp       = 400;
	pid[Y].Ki       = 0;
	pid[Y].Kd       = 0;
}

/***************************************************************************
函数名称：PID_del1
函数功能：PID数据处理函数
函数备注：_data，设置目标数据,用于处理Goal
***************************************************************************/
u16 PID_del1(float _data,u8 _num)
{
	MPU6050_data();
		 pid[_num].Actuals = angle[_num];
		 _fangxiang = 0;
		 if(pid[_num].Actuals < 0)
		 {
			 pid[_num].Actuals = -pid[_num].Actuals;
			 _fangxiang = 1;
		 }
	pid[_num].Sets = _data;                           //设置目标值
/*------------------------------------------------------*/	
  if(Max == 0 && (angle[_num] * w[_num] > 0))
		Max = 1;
  if(Max == 1)
	{
	if(angle[_num] < 0)
		a0 = -angle[_num];
	else
		a0 = angle[_num];
		aa = a0 - a1;
		a1 = a0;
   }
	if(aa < 0)	
	{
		Xita_ERR[_num] = pid[_num].Sets - a0;
		pid[_num].integral += pid[_num].err;                //对误差进行积分
		Max = 0;
	}
/*------------------------------------------------------*/		
	pid[_num].err = pid[_num].Sets - pid[_num].Actuals;           //误差系数
	pid[_num].voltage = (pid[_num].Kp * pid[_num].err                    //比例项
	                   + pid[_num].Ki * pid[_num].integral          //积分项
							+ pid[_num].Kd * (pid[_num].err - pid[_num].err_last));  //微分项
	
	pid[_num].err_last = pid[_num].err;                           //记录成上一个误差
	pid[_num].Actuals  = pid[_num].voltage;                       //调整输出
		
	return pid[_num].Actuals;
}
/***************************************************************************
函数名称：PID_del2
函数功能：PID数据处理函数
函数备注：_data，设置目标数据,用于处理Stop
***************************************************************************/
u16 PID_del2(float _data,u8 _num)
{
	 MPU6050_data();
			
			
			pid[_num].Actuals = angle[_num];
	pid[_num].Sets = _data;                           //设置目标值
	pid[_num].err = pid[_num].Sets - pid[_num].Actuals;           //误差系数
	
	pid[_num].integral += pid[_num].err;                //对误差进行积分
	
//////////////////////////////////	
//	if(pid[_num].integral>600.0) pid[_num].integral=600.0;/*死区限制*/
//	if(pid[_num].integral<-600.0) pid[_num].integral=-600.0;
//	pid[_num].integral=pid[_num].integral*0.5;

//if((w[0]>0) && (angle[0]>20))
//{	
//    pid[_num].integral += pid[_num].err ;
//}

//else if ((w[0]<0) && (angle[0]<-20))
//{
//     pid[_num].integral += -pid[_num].err ;
//}
///////////////////////////////

	pid[_num].voltage = (pid[_num].Kp * pid[_num].err                     //比例项
//	            + pid[_num].Ki * pid[_num].integral          //积分项
							+ pid[_num].Kd * (pid[_num].err - pid[_num].err_last));
	
	pid[_num].err_last = pid[_num].err;                           //记录成上一个误差
	pid[_num].Actuals  = pid[_num].voltage;                       //调整输出
		
	return pid[_num].Actuals;
}

/***************************************************************************
函数名称：Wind_drive_Stop
函数功能：制动处理
函数备注：
***************************************************************************/
void Wind_drive_YStop(void)
{

		if(pid[Y].voltage > 0)
		{
	//		if(pid[Y].voltage < Lost[2])
	//			pid[Y].voltage = Lost[2];
			if(pid[Y].voltage > 999)
				pid[Y].voltage = 999;
				
			MOTOR_Y_N = (u16)pid[Y].voltage;
			MOTOR_Y_P = 0;
		}
		else if(pid[Y].voltage < 0)
		{
			pid[Y].voltage = -pid[Y].voltage;
	//	  if(pid[Y].voltage < Lost[1])
	//		  pid[Y].voltage = Lost[1];
			if(pid[Y].voltage > 999)
				pid[Y].voltage = 999;
				
			MOTOR_Y_N = 0;
			MOTOR_Y_P = (u16)pid[Y].voltage;
		}
	else
	{
		MOTOR_Y_N = 0;
		MOTOR_Y_P = 0;
	}	
}
/////////////////////////////////////////
void Wind_drive_XStop(void)
{

  if(angle[1] * w[1] < 0)
	{
		if(pid[X].voltage > 0)
		{
	//		if(pid[Y].voltage < Lost[2])
	//			pid[Y].voltage = Lost[2];
			if(pid[X].voltage > 999)
				pid[X].voltage = 999;
				
			MOTOR_X_N = (u16)pid[X].voltage;
			MOTOR_X_P = 0;
		}
		else if(pid[X].voltage < 0)
		{
			pid[X].voltage = -pid[X].voltage;
	//	  if(pid[Y].voltage < Lost[1])
	//		  pid[Y].voltage = Lost[1];
			if(pid[X].voltage > 999)
				pid[X].voltage = 999;
				
			MOTOR_X_N = 0;
			MOTOR_X_P = (u16)pid[X].voltage;
		}
	}

	else
	{
		MOTOR_X_N = 0;
		MOTOR_X_P = 0;
	}	
}
/***************************************************************************
函数名称：Wind_drive_Goal
函数功能：偏移处理
函数备注：
***************************************************************************/
void Wind_drive_YGoal(void)
{
		if(angle[1] * w[1] < 0)
	{
//		if(pid[Y].err > 0)
//			_fangxiang = ~_fangxiang;
  if(_fangxiang == 1)
	{
//		if(pid[Y].voltage < Lost[2])
//			pid[Y].voltage = Lost[2];
		if(pid[Y].voltage > 999)
			pid[Y].voltage = 999;
			
		MOTOR_Y_P = (u16)pid[Y].voltage;
		MOTOR_Y_N = 0;
	}
	else if(_fangxiang == 0)
	{
//		pid[X].voltage = -pid[X].voltage;
//	  if(pid[Y].voltage < Lost[1])
//		  pid[Y].voltage = Lost[1];
		if(pid[Y].voltage > 999)
			pid[Y].voltage = 999;
			
		MOTOR_Y_P = 0;
		MOTOR_Y_N = (u16)pid[Y].voltage;
	}
  }
	else
	{
		MOTOR_Y_N = 0;
		MOTOR_Y_P = 0;
	}
		
}

/***************************************************************************
函数名称：Wind_drive_Goal
函数功能：偏移处理
函数备注：
***************************************************************************/
void Wind_drive_YGoal2(void)
{
//		if(angle[1] * w[1] < 0)
//	{
		if(w[1] > 0)
			_fangxiang = ~_fangxiang;
  if(_fangxiang == 1)
	{
//		if(pid[Y].voltage < Lost[2])
//			pid[Y].voltage = Lost[2];
		if(pid[Y].voltage > 999)
			pid[Y].voltage = 999;
			
		MOTOR_Y_P = (u16)pid[Y].voltage;
		MOTOR_Y_N = 0;
	}
	else if(_fangxiang == 0)
	{
//		pid[X].voltage = -pid[X].voltage;
//	  if(pid[Y].voltage < Lost[1])
//		  pid[Y].voltage = Lost[1];
		if(pid[Y].voltage > 999)
			pid[Y].voltage = 999;
			
		MOTOR_Y_P = 0;
		MOTOR_Y_N = (u16)pid[Y].voltage;
//	}
  }
	else
	{
		MOTOR_Y_N = 0;
		MOTOR_Y_P = 0;
	}
		
}
/***************************************************************************
函数名称：Wind_drive_Goal
函数功能：偏移处理
函数备注：
***************************************************************************/
void Wind_drive_YGoalBF3_0(void)
{
  if(angle[0] > 0)
	{
		if(pid[Y].voltage > 999)
			pid[Y].voltage = 999;
			
		MOTOR_Y_P = (u16)pid[Y].voltage;
		MOTOR_Y_N = 0;
	}
	else if(angle[0] < 0)
	{
		if(pid[Y].voltage > 999)
			pid[Y].voltage = 999;
			
		MOTOR_Y_P = 0;
		MOTOR_Y_N = (u16)pid[Y].voltage;
  }
	else
	{
		MOTOR_Y_N = 0;
		MOTOR_Y_P = 0;
	}
		
}
/***************************************************************************
函数名称：Wind_drive_Goal
函数功能：偏移处理
函数备注：
***************************************************************************/
void Wind_drive_YGoalBF3_1(void)
{
  if(angle[0] < 0)
	{
		if(pid[Y].voltage > 999)
			pid[Y].voltage = 999;
			
		MOTOR_Y_P = (u16)pid[Y].voltage;
		MOTOR_Y_N = 0;
	}
	else if(angle[0] > 0)
	{
		if(pid[Y].voltage > 999)
			pid[Y].voltage = 999;
			
		MOTOR_Y_P = 0;
		MOTOR_Y_N = (u16)pid[Y].voltage;
  }
	else
	{
		MOTOR_Y_N = 0;
		MOTOR_Y_P = 0;
	}
		
}
/***************************************************************************
函数名称：Wind_drive_Goal
函数功能：偏移处理
函数备注：
***************************************************************************/
void Wind_drive_XGoal(void)
{
	if(angle[0] * w[0] < 0)
	{
//		if(w[0] < 0)
//			_fangxiang = ~_fangxiang;
  if(_fangxiang == 1)
	{
//		if(pid[Y].voltage < Lost[2])
//			pid[Y].voltage = Lost[2];
		if(pid[X].voltage > 999)
			pid[X].voltage = 999;
			
		MOTOR_X_P = (u16)pid[X].voltage;
		MOTOR_X_N = 0;
	}
	else if(_fangxiang == 0)
	{
//		pid[X].voltage = -pid[X].voltage;
//	  if(pid[Y].voltage < Lost[1])
//		  pid[Y].voltage = Lost[1];
		if(pid[X].voltage > 999)
			pid[X].voltage = 999;
			
		MOTOR_X_P = 0;
		MOTOR_X_N = (u16)pid[X].voltage;
	}
  }
	else
	{
		MOTOR_X_N = 100;
		MOTOR_X_P = 100;
	}
}

/***************************************************************************
函数名称：Wind_drive_Goal
函数功能：偏移处理
函数备注：
***************************************************************************/
void Wind_drive_XGoal2(void)
{
//	if(angle[0] * w[0] < 0)
//	{
		if(w[0] > 0)
			_fangxiang = ~_fangxiang;
  if(_fangxiang == 1)
	{
//		if(pid[Y].voltage < Lost[2])
//			pid[Y].voltage = Lost[2];
		if(pid[X].voltage > 999)
			pid[X].voltage = 999;
			
		MOTOR_X_P = (u16)pid[X].voltage;
		MOTOR_X_N = 0;
	}
	else if(_fangxiang == 0)
	{
//		pid[X].voltage = -pid[X].voltage;
//	  if(pid[Y].voltage < Lost[1])
//		  pid[Y].voltage = Lost[1];
		if(pid[X].voltage > 999)
			pid[X].voltage = 999;
			
		MOTOR_X_P = 0;
		MOTOR_X_N = (u16)pid[X].voltage;
//	}
  }
	else
	{
		MOTOR_X_N = 100;
		MOTOR_X_P = 100;
	}

		
}




#endif   // __PID_ENABLE

