#ifndef __24L01_H_
#define __24L01_H_
#include "sys.h"
#include "spi.h"
#if __24L01_ENABLE

#define TX_ADR_WIDTH      5   		
#define RX_ADR_WIDTH      5 
#define TX_PAYLOAD_WIDTH  6
#define RX_PAYLOAD_WIDTH  6


#define MAX_TX  		0x10  //达到最大发送次数中断
#define TX_OK   		0x20  //TX发送完成中断
#define RX_OK   		0x40  //接收到数据中断

/**********************************NRF24L01寄存器指令*********************************************/
#define RF_READ_REG     0x00  	// 读寄存器指令
#define RF_WRITE_REG    0x20 	// 写寄存器指令
#define R_RX_PAYLOAD    0x61  	// 读取接收数据指令
#define W_TX_PAYLOAD    0xA0  	// 写待发数据指令
#define FLUSH_TX        0xE1 	// 冲洗发送 FIFO指令
#define FLUSH_RX        0xE2  	// 冲洗接收 FIFO指令
#define REUSE_TX_PL     0xE3  	// 定义重复装载数据指令
#define NOP             0xFF  	// 保留

/**********************************NRF24L01--SPI寄存器地址*****************************************/
#define CONFIG          0x00  // 配置收发状态，CRC校验模式以及收发状态响应方式
#define EN_AA           0x01  // 自动应答功能设置
#define EN_RXADDR       0x02  // 可用信道设置
#define SETUP_AW        0x03  // 收发地址宽度设置
#define SETUP_RETR      0x04  // 自动重发功能设置
#define RF_CH           0x05  // 工作频率设置
#define RF_SETUP        0x06  // 发射速率、功耗功能设置
#define STATUS          0x07  // 状态寄存器
#define OBSERVE_TX      0x08  // 发送监测功能
#define CD              0x09  // 地址检测           
#define RX_ADDR_P0      0x0A  // 频道0接收数据地址
#define RX_ADDR_P1      0x0B  // 频道1接收数据地址
#define RX_ADDR_P2      0x0C  // 频道2接收数据地址
#define RX_ADDR_P3      0x0D  // 频道3接收数据地址
#define RX_ADDR_P4      0x0E  // 频道4接收数据地址
#define RX_ADDR_P5      0x0F  // 频道5接收数据地址
#define TX_ADDR         0x10  // 发送地址寄存器
#define RX_PW_P0        0x11  // 接收频道0接收数据长度
#define RX_PW_P1        0x12  // 接收频道0接收数据长度
#define RX_PW_P2        0x13  // 接收频道0接收数据长度
#define RX_PW_P3        0x14  // 接收频道0接收数据长度
#define RX_PW_P4        0x15  // 接收频道0接收数据长度
#define RX_PW_P5        0x16  // 接收频道0接收数据长度
#define FIFO_STATUS     0x17  // FIFO栈入栈出状态寄存器设置

//接收数据通道N
enum ERX_Pn
{
	ERX_P0 = 0x01,
	ERX_P1 = 0x02,
	ERX_P2 = 0x04,
	ERX_P3 = 0x08,
	ERX_P4 = 0x10,
	ERX_P5 = 0x20,
};
//数据通道N自动应答
enum ENAA_Pn
{
	ENAA_P0 = 0x01,
	ENAA_P1 = 0x02,
	ENAA_P2 = 0x04,
	ENAA_P3 = 0x08,
	ENAA_P4 = 0x10,
	ENAA_P5 = 0x20,
};
//发射功率
enum RF_PWRn
{
	RF_PWR18 = 0x00,
	RF_PWR12 = 0x02,
	RF_PWR06 = 0x04,
	RF_PWR00 = 0x06,
};
//发射速率
enum RF_DR
{
	_1Mbps = 0x00,
	_2Mbps = 0x08,
};
/**********************************RNF24L01状态标志位************************************************/
extern u8 Statu; 
//extern u8 Dev0_TX[TX_ADR_WIDTH];
//extern u8 Dev1_TX[TX_ADR_WIDTH];
//extern u8 Dev0_RX[RX_ADR_WIDTH];
//extern u8 Dev1_RX[RX_ADR_WIDTH];
extern u8 Dev_TX[TX_ADR_WIDTH];
extern u8 Dev_RX[RX_ADR_WIDTH];
#define MAX_RT ((Statu >> 4) & 0x01)
#define TX_DS  ((Statu >> 5) & 0x01) 
#define RX_DR  ((Statu >> 6) & 0x01) 

#define NRF24L01_CE    PBout(12) 	  //24L01片选信号
#define NRF24L01_CSN0  PAout(2) 	  //SPI片选信号0
#define NRF24L01_CSN1  PAout(3) 	  //SPI片选信号1
#define NRF24L01_IRQ0  PEin(5)  	  //IRQ主机数据输入0
#define NRF24L01_IRQ1  PEin(6)  	  //IRQ主机数据输入1

#define NRF_0          0
#define NRF_1          1


void NRF24L01_CSN_ON(u8 X);
void NRF24L01_CSN_OFF(u8 X);
u8 NRF24L01_Read_Reg(u8 CMD);
u8 NRF24L01_Write_Reg(u8 CMD, u8 value);
u8 NRF24L01_Read_Buf(u8 CMD, u8 *pBuf, u8 num);
u8 NRF24L01_Write_Buf(u8 CMD, u8 *pBuf, u8 num);

u8 NRF24L01_RxPacket(u8* rx_buf);                     //这个函数用于接收数据
u8 NRF24L01_TxPacket(unsigned char * tx_buf);	    //这个函数用于发送数据

void NRF24L01_RX_Mode(u8 *_RX_ADDRESS);
void NRF24L01_TX_Mode(u8 *_TX_ADDRESS,u8 *_RX_ADDRESS);
void NRF24L01_Init(void);
u8 NRF24L01_Check(void);


#endif  //____24L01_ENABLE
#endif  //__24L01_H_
