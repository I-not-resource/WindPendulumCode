#ifndef __IIC_H_
#define __IIC_H_
#include "sys.h"
#if __IIC_ENABLE


#define I2CSIDE_ADDRESS 0xd0   //MPU6050地址
#define IIC_SCL   PAout(4)
#define IIC_SDA   PAout(5)
#define READ_SDA  PAin(5)

#define SDA_IN()  {GPIOA->MODER&=~(3<<(5*2));GPIOA->MODER|=0<<(5*2);}	//PA5输入模式
#define SDA_OUT() {GPIOA->MODER&=~(3<<(5*2));GPIOA->MODER|=1<<(5*2);} //PA5输出模式

//IIC所有操作函数
void IIC_SimulationConfig(void);                //初始化IIC的IO口				 
void IIC_Start(void);               //发送IIC开始信号
void IIC_Stop(void);                //发送IIC停止信号
void IIC_Send_Byte(u8 txd);         //IIC发送一个字节
u8 IIC_Read_Byte(unsigned char ack);//IIC读取一个字节
u8 IIC_Wait_Ack(void);              //IIC等待ACK信号
void IIC_Ack(void);	                //IIC发送ACK信号
void IIC_NAck(void);                //IIC不发送ACK信号

void IIC_Write_One_Byte(u8 daddr,u8 addr,u8 data);
u8 IIC_Read_One_Byte(u8 daddr,u8 addr);	
#endif  //__IIC_ENABLE
#endif  //__IIC_H_
