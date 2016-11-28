#ifndef __STM32_I2C_H
#define __STM32_I2C_H

#include "stm32f4xx.h"

//#include "mpu6050.h"
//#define BYTE0(dwTemp)       (*(char *)(&dwTemp))
//#define BYTE1(dwTemp)       (*((char *)(&dwTemp) + 1))
//#define BYTE2(dwTemp)       (*((char *)(&dwTemp) + 2))
//#define BYTE3(dwTemp)       (*((char *)(&dwTemp) + 3))

#define true 1
#define false 0 
#define bool  uint8_t

#define TRUE  0
#define FALSE -1

////0表示写
//#define	I2C_Direction_Transmitter   0
////１表示读
//#define	I2C_Direction_Receiver      1	
static void I2C_delay(void);
static bool I2C_Start(void);
static void I2C_Stop(void);
static void I2C_Ack(void);
static void I2C_NoAck(void);
static bool I2C_WaitAck(void);
static void I2C_SendByte(uint8_t byte);
static uint8_t I2C_ReceiveByte(unsigned char ack);
void i2cInit(void);
uint16_t i2cGetErrorCounter(void);
/*====================================================================================================*/
/*====================================================================================================*/
void IIC_INPUT_TEST(void);
bool i2cWriteBuffer(uint8_t addr_, uint8_t reg_, uint8_t len_, uint8_t *data);
bool i2cWrite(uint8_t addr_, uint8_t reg_, uint8_t data);
bool i2cRead(uint8_t addr_, uint8_t reg_, uint8_t len, uint8_t* buf);
void i2cInit(void);
uint16_t i2cGetErrorCounter(void);
static void i2cUnstick(void);

int8_t i2cwrite(uint8_t addr, uint8_t reg, uint8_t len, uint8_t * data);
int8_t i2cread(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf);
/*====================================================================================================*/
/*====================================================================================================*/
u8 i2cRead_Byte(uint8_t addr, uint8_t reg);
u8 MPU9150_Write_Byte(u8 ADDR,u8 reg,u8 data);
u8 MPU9150_Read_Byte(u8 ADDR,u8 reg);
u8 MPU9150_Write_Len(u8 addr,u8 reg,u8 len,u8 *buf);
u8 MPU9150_Read_Len(u8 addr,u8 reg,u8 len,u8 *buf);

#endif


