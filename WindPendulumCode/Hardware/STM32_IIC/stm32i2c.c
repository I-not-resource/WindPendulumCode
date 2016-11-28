#include "stm32i2c.h"

#include "delay.h"
#include "usart.h"
#include "myiic.h"

//#define SCL_H         GPIOB->BSRRH = GPIO_Pin_8 /* GPIO_SetBits(GPIOB , GPIO_Pin_10)   */
//#define SCL_L         GPIOB->BSRRL  = GPIO_Pin_8 /* GPIO_ResetBits(GPIOB , GPIO_Pin_10) */

//#define SDA_H         GPIOB->BSRRH = GPIO_Pin_9 /* GPIO_SetBits(GPIOB , GPIO_Pin_11)   */
//#define SDA_L         GPIOB->BSRRL  = GPIO_Pin_9 /* GPIO_ResetBits(GPIOB , GPIO_Pin_11) */

//#define SCL_read      GPIOB->IDR  & GPIO_Pin_8 /* GPIO_ReadInputDataBit(GPIOB , GPIO_Pin_10) */
//#define SDA_read      GPIOB->IDR  & GPIO_Pin_9 /* GPIO_ReadInputDataBit(GPIOB , GPIO_Pin_11) */

#define SCL_H         GPIOC->BSRRH = GPIO_Pin_8 /* GPIO_SetBits(GPIOB , GPIO_Pin_10)   */
#define SCL_L         GPIOC->BSRRL  = GPIO_Pin_8 /* GPIO_ResetBits(GPIOB , GPIO_Pin_10) */

#define SDA_H         GPIOC->BSRRH = GPIO_Pin_9 /* GPIO_SetBits(GPIOB , GPIO_Pin_11)   */
#define SDA_L         GPIOC->BSRRL  = GPIO_Pin_9 /* GPIO_ResetBits(GPIOB , GPIO_Pin_11) */

#define SCL_read      GPIOC->IDR  & GPIO_Pin_8 /* GPIO_ReadInputDataBit(GPIOB , GPIO_Pin_10) */
#define SDA_read      GPIOC->IDR  & GPIO_Pin_9 /* GPIO_ReadInputDataBit(GPIOB , GPIO_Pin_11) */

void IIC_INPUT_TEST(void)
{ 
	if (SCL_read)
	{
		printf("It's the SCL_H\n");
	}
	else
	{
		printf("It's  the SCL_L\n");
	}
	
	if (SDA_read)
	{
		printf("It's the SDA_H\n");
	}
	else
	{
		printf("It's the SDA_L\n");
	}
}

/*
static void I2C_delay(void)
{
    volatile int i = 50;	  //ԭΪi=7��ʹ��GD32оƬ���ӳ�������1����������ʱ��72Mʱi=14��108Mʱi=20
    while (i)
        i--;
}

static bool I2C_Start(void)
{
    SDA_H;
    SCL_H;
    I2C_delay();
    if (!SDA_read)
        return false;
    SDA_L;
    I2C_delay();
//    if (SDA_read)
//        return false;
    SCL_L;
    I2C_delay();
    return true;
}

static void I2C_Stop(void)
{
    IIC_Stop();
}

static void I2C_Ack(void)
{
    IIC_Ack();
}

static void I2C_NoAck(void)
{
    IIC_NAck();
}

static bool I2C_WaitAck(void)
{ 
		u8 num = 0;
		num = IIC_Wait_Ack();
		num = !num;
		return num;
}

static void I2C_SendByte(uint8_t byte)
{
    IIC_Send_Byte(byte);
}

static uint8_t I2C_ReceiveByte(unsigned char ack)
{
    uint8_t byte = 0;
		byte = IIC_Read_Byte(ack);
    return byte;
}
*/
/*PB8----I2C2_SCL    PB9----I2C2_SDA*/
//---------------�ú���ӦΪ�ײ����һ�㺯����Ӧ�õ���myiic����ĺ������г�ʼ��IO,������ֲ-----------//
void i2cInit(void)
{
//	RCC->AHB1ENR|=1<<2;    //ʹ��PORTCʱ��	   	  
//	GPIO_Set(GPIOC,PIN8|PIN9,GPIO_MODE_OUT,GPIO_OTYPE_PP,GPIO_SPEED_50M,GPIO_PUPD_PU);//PB8/PB9���� 
//	SCL_H;
//	SDA_H;
	IIC_Init();
}

bool i2cWriteBuffer(uint8_t addr, uint8_t reg, uint8_t len, uint8_t * data)
{
//    int i;
		u8 num = 0;
		num = MPU9150_Write_Len(addr, reg, len, data);
		num = !num;
		return num;
}
/////////////////////////////////////////////////////////////////////////////////
int8_t i2cwrite(uint8_t addr, uint8_t reg, uint8_t len, uint8_t * data)
{
	if(i2cWriteBuffer(addr,reg,len,data))
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
	//return FALSE;
}
int8_t i2cread(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
	if(!MPU9150_Read_Len(addr,reg,len,buf))
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}
//////////////////////////////////////////////////////////////////////////////////
bool i2cWrite(uint8_t addr, uint8_t reg, uint8_t data)
{
		u8 num = 0;
		num = MPU9150_Write_Byte(addr, reg, data);
		num = !num;
		return num;
}

bool i2cRead(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
		u8 num = 0;
	  num = MPU9150_Read_Len(addr, reg, len, buf);
		num = !num;
		return num;
}

uint16_t i2cGetErrorCounter(void)
{
    // TODO maybe fix this, but since this is test code, doesn't matter.
    return 0;
}

//IIC��һ���ֽ� 
//reg:�Ĵ�����ַ 
//����ֵ:����������
u8 i2cRead_Byte(uint8_t addr, uint8_t reg)
{
	u8 res;
  res = MPU9150_Read_Byte(addr, reg);
	return res;		
}

//IICдһ���ֽ� 
//reg:�Ĵ�����ַ
//data:����
//����ֵ:0,����
//    ����,�������
u8 MPU9150_Write_Byte(u8 ADDR,u8 reg,u8 data) 				 
{ 
    IIC_Start(); 
	IIC_Send_Byte((ADDR<<1)|0);//����������ַ+д����	
	if(IIC_Wait_Ack())	//�ȴ�Ӧ��
	{
		IIC_Stop();		 
		return 1;		
	}
    IIC_Send_Byte(reg);	//д�Ĵ�����ַ
    IIC_Wait_Ack();		//�ȴ�Ӧ�� 
	IIC_Send_Byte(data);//��������
	if(IIC_Wait_Ack())	//�ȴ�ACK
	{
		IIC_Stop();	 
		return 1;		 
	}		 
    IIC_Stop();	 
	return 0;
}
//IIC��һ���ֽ� 
//reg:�Ĵ�����ַ 
//����ֵ:����������
u8 MPU9150_Read_Byte(u8 ADDR,u8 reg)
{
	u8 res;
    IIC_Start(); 
	IIC_Send_Byte((ADDR<<1)|0);//����������ַ+д����	
	IIC_Wait_Ack();		//�ȴ�Ӧ�� 
    IIC_Send_Byte(reg);	//д�Ĵ�����ַ
    IIC_Wait_Ack();		//�ȴ�Ӧ��
    IIC_Start();
	IIC_Send_Byte((ADDR<<1)|1);//����������ַ+������	
    IIC_Wait_Ack();		//�ȴ�Ӧ�� 
	res=IIC_Read_Byte(0);//��ȡ����,����nACK 
    IIC_Stop();			//����һ��ֹͣ���� 
	return res;		
}

//IIC����д
//addr:������ַ 
//reg:�Ĵ�����ַ
//len:д�볤��
//buf:������
//����ֵ:0,����
//    ����,�������
u8 MPU9150_Write_Len(u8 addr,u8 reg,u8 len,u8 *buf)
{
	u8 i; 
    IIC_Start(); 
	IIC_Send_Byte((addr<<1)|0);//����������ַ+д����	
	if(IIC_Wait_Ack())	//�ȴ�Ӧ��
	{
		IIC_Stop();		 
		return 1;		
	}
    IIC_Send_Byte(reg);	//д�Ĵ�����ַ
    IIC_Wait_Ack();		//�ȴ�Ӧ��
	for(i=0;i<len;i++)
	{
		IIC_Send_Byte(buf[i]);	//��������
		if(IIC_Wait_Ack())		//�ȴ�ACK
		{
			IIC_Stop();	 
			return 1;		 
		}		
	}    
    IIC_Stop();	 
	return 0;	
} 
//IIC������
//addr:������ַ
//reg:Ҫ��ȡ�ļĴ�����ַ
//len:Ҫ��ȡ�ĳ���
//buf:��ȡ�������ݴ洢��
//����ֵ:0,����
//    ����,�������
u8 MPU9150_Read_Len(u8 addr,u8 reg,u8 len,u8 *buf)
{ 
 	IIC_Start(); 
	IIC_Send_Byte((addr<<1)|0);//����������ַ+д����	
	if(IIC_Wait_Ack())	//�ȴ�Ӧ��
	{
		IIC_Stop();		 
		return 1;		
	}
    IIC_Send_Byte(reg);	//д�Ĵ�����ַ
    IIC_Wait_Ack();		//�ȴ�Ӧ��
    IIC_Start();
	IIC_Send_Byte((addr<<1)|1);//����������ַ+������	
    IIC_Wait_Ack();		//�ȴ�Ӧ�� 
	while(len)
	{
		if(len==1)*buf=IIC_Read_Byte(0);//������,����nACK 
		else *buf=IIC_Read_Byte(1);		//������,����ACK  
		len--;
		buf++; 
	}    
    IIC_Stop();	//����һ��ֹͣ���� 
	return 0;
}

