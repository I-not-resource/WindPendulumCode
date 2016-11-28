#include "IIC.h"
#if __IIC_ENABLE
/***************************************************************************
�������ƣ�IIC_SimulationConfig
�������ܣ�ģ��IICͨ��
������ע��
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
�������ƣ�IIC_Start
�������ܣ�IIC��ʼ�ź�
������ע��
***************************************************************************/
void IIC_Start(void)
{
	SDA_OUT();     //sda�����
	IIC_SDA=1;	  	  
	IIC_SCL=1;
	Delay_us(4);
 	IIC_SDA=0;     //START:when CLK is high,DATA change form high to low 
	Delay_us(4);
	IIC_SCL=0;     //ǯסI2C���ߣ�׼�����ͻ�������� 
}	 

/***************************************************************************
�������ƣ�IIC_Stop
�������ܣ�IICֹͣ�ź�
������ע��
***************************************************************************/
void IIC_Stop(void)
{
	SDA_OUT();//sda�����
	IIC_SCL=0;
	IIC_SDA=0;//STOP:when CLK is high DATA change form low to high
 	Delay_us(4);
	IIC_SCL=1; 
	IIC_SDA=1;//����I2C���߽����ź�
	Delay_us(4);							   	
}

/***************************************************************************
�������ƣ�IIC_Wait_Ack
�������ܣ��ȴ�Ӧ���ź�
������ע�� 0������Ӧ��ɹ�/1������Ӧ��ʧ��
***************************************************************************/
u8 IIC_Wait_Ack(void)
{
	u8 ucErrTime=0;
	SDA_IN();      //SDA����Ϊ����  
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
	IIC_SCL=0;//ʱ�����0 	   
	return 0;  
}

/***************************************************************************
�������ƣ�IIC_Ack
�������ܣ�����ACKӦ��
������ע��
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
�������ƣ�IIC_NAck
�������ܣ�������ACKӦ��	
������ע��
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
�������ƣ�IIC_Send_Byte
�������ܣ�IIC����һ���ֽ�
������ע��txd�����͵����ݣ����شӻ�����Ӧ��/1����Ӧ��/0����Ӧ��	
***************************************************************************/
void IIC_Send_Byte(u8 txd)
{                        
    u8 t;   
	SDA_OUT(); 	    
    IIC_SCL=0;//����ʱ�ӿ�ʼ���ݴ���
    for(t=0;t<8;t++)
    {              
        IIC_SDA=(txd&0x80)>>7;
        txd<<=1; 	  
		Delay_us(2);   //��TEA5767��������ʱ���Ǳ����
		IIC_SCL=1;
		Delay_us(2); 
		IIC_SCL=0;	
    }	 
}

/***************************************************************************
�������ƣ�IIC_Read_Byte
�������ܣ���1���ֽ�,ack=1ʱ������ACK��ack=0������nACK 
������ע��
***************************************************************************/
u8 IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_IN();//SDA����Ϊ����
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
        IIC_NAck();//����nACK
    else
        IIC_Ack(); //����ACK   
    return receive;
}

#endif  //__IIC_ENABLE

