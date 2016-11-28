#include "IIC.h"
#if __IIC_ENABLE
/***************************************************************************
函数名称：IIC_SimulationConfig
函数功能：模拟IIC通信
函数备注：
***************************************************************************/
void IIC_SimulationConfig(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
	
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_4 | GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	IIC_SCL = 1;
	IIC_SDA = 1;
}

/***************************************************************************
函数名称：IIC_Start
函数功能：IIC起始信号
函数备注：
***************************************************************************/
void IIC_Start(void)
{
	SDA_OUT();     //sda线输出
	IIC_SDA=1;	  	  
	IIC_SCL=1;
	Delay_us(4);
 	IIC_SDA=0;     //START:when CLK is high,DATA change form high to low 
	Delay_us(4);
	IIC_SCL=0;     //钳住I2C总线，准备发送或接收数据 
}	 

/***************************************************************************
函数名称：IIC_Stop
函数功能：IIC停止信号
函数备注：
***************************************************************************/
void IIC_Stop(void)
{
	SDA_OUT();//sda线输出
	IIC_SCL=0;
	IIC_SDA=0;//STOP:when CLK is high DATA change form low to high
 	Delay_us(4);
	IIC_SCL=1; 
	IIC_SDA=1;//发送I2C总线结束信号
	Delay_us(4);							   	
}

/***************************************************************************
函数名称：IIC_Wait_Ack
函数功能：等待应答信号
函数备注： 0，接收应答成功/1，接收应答失败
***************************************************************************/
u8 IIC_Wait_Ack(void)
{
	u8 ucErrTime=0;
	SDA_IN();      //SDA设置为输入  
	IIC_SDA=1;
	Delay_us(1);
	IIC_SCL=1;
	Delay_us(1);
	
	while(READ_SDA)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			IIC_Stop();
			return 1;
		}
	}
	IIC_SCL=0;//时钟输出0 	   
	return 0;  
}

/***************************************************************************
函数名称：IIC_Ack
函数功能：产生ACK应答
函数备注：
***************************************************************************/
void IIC_Ack(void)
{
	IIC_SCL=0;
	SDA_OUT();
	IIC_SDA=0;
	Delay_us(2);
	IIC_SCL=1;
	Delay_us(2);
	IIC_SCL=0;
}

/***************************************************************************
函数名称：IIC_NAck
函数功能：不产生ACK应答	
函数备注：
***************************************************************************/
void IIC_NAck(void)
{
	IIC_SCL=0;
	SDA_OUT();
	IIC_SDA=1;
	Delay_us(2);
	IIC_SCL=1;
	Delay_us(2);
	IIC_SCL=0;
}	

/***************************************************************************
函数名称：IIC_Send_Byte
函数功能：IIC发送一个字节
函数备注：txd：发送的数据，返回从机有无应答/1，有应答/0，无应答	
***************************************************************************/
void IIC_Send_Byte(u8 txd)
{                        
    u8 t;   
	SDA_OUT(); 	    
    IIC_SCL=0;//拉低时钟开始数据传输
    for(t=0;t<8;t++)
    {              
        IIC_SDA=(txd&0x80)>>7;
        txd<<=1; 	  
		Delay_us(2);   //对TEA5767这三个延时都是必须的
		IIC_SCL=1;
		Delay_us(2); 
		IIC_SCL=0;	
    }	 
}

/***************************************************************************
函数名称：IIC_Read_Byte
函数功能：读1个字节,ack=1时，发送ACK，ack=0，发送nACK 
函数备注：
***************************************************************************/
u8 IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_IN();//SDA设置为输入
    for(i=0;i<8;i++ )
	{
        IIC_SCL=0; 
        Delay_us(2);
		IIC_SCL=1;
        receive<<=1;
        if(READ_SDA)receive++;   
		Delay_us(1); 
    }					 
    if (!ack)
        IIC_NAck();//发送nACK
    else
        IIC_Ack(); //发送ACK   
    return receive;
}

#endif  //__IIC_ENABLE

