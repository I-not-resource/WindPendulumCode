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
    volatile int i = 50;	  //原为i=7，使用GD32芯片，延迟需增加1倍，以满足时序，72M时i=14，108M时i=20
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
//---------------该函数应为底层的上一层函数，应该调用myiic里面的函数进行初始化IO,方便移植-----------//
void i2cInit(void)
{
//	RCC->AHB1ENR|=1<<2;    //使能PORTC时钟	   	  
//	GPIO_Set(GPIOC,PIN8|PIN9,GPIO_MODE_OUT,GPIO_OTYPE_PP,GPIO_SPEED_50M,GPIO_PUPD_PU);//PB8/PB9设置 
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

//IIC读一个字节 
//reg:寄存器地址 
//返回值:读到的数据
u8 i2cRead_Byte(uint8_t addr, uint8_t reg)
{
	u8 res;
  res = MPU9150_Read_Byte(addr, reg);
	return res;		
}

//IIC写一个字节 
//reg:寄存器地址
//data:数据
//返回值:0,正常
//    其他,错误代码
u8 MPU9150_Write_Byte(u8 ADDR,u8 reg,u8 data) 				 
{ 
    IIC_Start(); 
	IIC_Send_Byte((ADDR<<1)|0);//发送器件地址+写命令	
	if(IIC_Wait_Ack())	//等待应答
	{
		IIC_Stop();		 
		return 1;		
	}
    IIC_Send_Byte(reg);	//写寄存器地址
    IIC_Wait_Ack();		//等待应答 
	IIC_Send_Byte(data);//发送数据
	if(IIC_Wait_Ack())	//等待ACK
	{
		IIC_Stop();	 
		return 1;		 
	}		 
    IIC_Stop();	 
	return 0;
}
//IIC读一个字节 
//reg:寄存器地址 
//返回值:读到的数据
u8 MPU9150_Read_Byte(u8 ADDR,u8 reg)
{
	u8 res;
    IIC_Start(); 
	IIC_Send_Byte((ADDR<<1)|0);//发送器件地址+写命令	
	IIC_Wait_Ack();		//等待应答 
    IIC_Send_Byte(reg);	//写寄存器地址
    IIC_Wait_Ack();		//等待应答
    IIC_Start();
	IIC_Send_Byte((ADDR<<1)|1);//发送器件地址+读命令	
    IIC_Wait_Ack();		//等待应答 
	res=IIC_Read_Byte(0);//读取数据,发送nACK 
    IIC_Stop();			//产生一个停止条件 
	return res;		
}

//IIC连续写
//addr:器件地址 
//reg:寄存器地址
//len:写入长度
//buf:数据区
//返回值:0,正常
//    其他,错误代码
u8 MPU9150_Write_Len(u8 addr,u8 reg,u8 len,u8 *buf)
{
	u8 i; 
    IIC_Start(); 
	IIC_Send_Byte((addr<<1)|0);//发送器件地址+写命令	
	if(IIC_Wait_Ack())	//等待应答
	{
		IIC_Stop();		 
		return 1;		
	}
    IIC_Send_Byte(reg);	//写寄存器地址
    IIC_Wait_Ack();		//等待应答
	for(i=0;i<len;i++)
	{
		IIC_Send_Byte(buf[i]);	//发送数据
		if(IIC_Wait_Ack())		//等待ACK
		{
			IIC_Stop();	 
			return 1;		 
		}		
	}    
    IIC_Stop();	 
	return 0;	
} 
//IIC连续读
//addr:器件地址
//reg:要读取的寄存器地址
//len:要读取的长度
//buf:读取到的数据存储区
//返回值:0,正常
//    其他,错误代码
u8 MPU9150_Read_Len(u8 addr,u8 reg,u8 len,u8 *buf)
{ 
 	IIC_Start(); 
	IIC_Send_Byte((addr<<1)|0);//发送器件地址+写命令	
	if(IIC_Wait_Ack())	//等待应答
	{
		IIC_Stop();		 
		return 1;		
	}
    IIC_Send_Byte(reg);	//写寄存器地址
    IIC_Wait_Ack();		//等待应答
    IIC_Start();
	IIC_Send_Byte((addr<<1)|1);//发送器件地址+读命令	
    IIC_Wait_Ack();		//等待应答 
	while(len)
	{
		if(len==1)*buf=IIC_Read_Byte(0);//读数据,发送nACK 
		else *buf=IIC_Read_Byte(1);		//读数据,发送ACK  
		len--;
		buf++; 
	}    
    IIC_Stop();	//产生一个停止条件 
	return 0;
}

