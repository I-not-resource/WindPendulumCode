#include "24L01.h"
#if __24L01_ENABLE

u8 Statu;   
u8 RX_DATA[6]="NO";

//u8 Dev0_TX[TX_ADR_WIDTH]={0x11,0x35,0x02,0xAA,0xCF};
//u8 Dev0_RX[RX_ADR_WIDTH]={0X11,0X35,0X02,0XBB,0XDE};
//u8 Dev1_TX[TX_ADR_WIDTH]={0x11,0x35,0x02,0xAA,0xCF};
//u8 Dev1_RX[RX_ADR_WIDTH]={0X11,0X35,0X02,0XBB,0XDE};
u8 Dev_TX[TX_ADR_WIDTH]={0x34,0x43,0x10,0x10,0x02};
u8 Dev_RX[RX_ADR_WIDTH]={0x34,0x43,0x10,0x10,0x02};
/***************************************************************************
函数名称：NRF24L01_CSN_ON
函数功能：NRF片选使能
函数备注：
***************************************************************************/
void NRF24L01_CSN_ON(u8 X)
{
  switch(X)
	{
    case 0:NRF24L01_CSN0 = 0;break;
		case 1:NRF24L01_CSN1 = 0;break;
		default:break;
	}
}

/***************************************************************************
函数名称：NRF24L01_CSN_OFF
函数功能：NRF片选失能
函数备注：
***************************************************************************/
void NRF24L01_CSN_OFF(u8 X)
{
  switch(X)
	{
    case 0:NRF24L01_CSN0 = 1;break;
		case 1:NRF24L01_CSN1 = 1;break;
		default:break;
	}
}

/***************************************************************************
函数名称：SPI读时序函数
函数功能：NRF24L01的SPI读时序
函数备注：
***************************************************************************/
u8 NRF24L01_Read_Reg(u8 CMD)
{
	u8 value;
	NRF24L01_CSN0=0;              
	SPI2_ReadWriteByte(CMD);            
	value = SPI2_ReadWriteByte(0xff);  
  NRF24L01_CSN0=1;	
	
	return(value);        
}

/***************************************************************************
函数名称：NRF24L01读写寄存器函数
函数功能：写入NRF24L01寄寄存器和从NRF24L01寄存器中读出
函数备注：返回状态
***************************************************************************/
u8 NRF24L01_Write_Reg(u8 CMD, u8 value)
{	         
  u8 status;	
	NRF24L01_CSN0=0;  
	status = SPI2_ReadWriteByte(CMD);     
	SPI2_ReadWriteByte(value);
  NRF24L01_CSN0=1;  
  
  return	status;
}

/***************************************************************************
函数名称：NRF24L01读寄存器数据函数
函数功能：从NRF24L01寄存器中读出数据
函数备注：CMD：为寄存器地址，pBuf：为待读出数据地址，u8s：读出数据的个数
***************************************************************************/
u8 NRF24L01_Read_Buf(u8 CMD, u8 *pBuf, u8 num)
{
	u8 status,i;
	
	
  NRF24L01_CSN0=0;							 
	status = SPI2_ReadWriteByte( CMD);       		
	for(i=0;i<num;i++)
		pBuf[i] = SPI2_ReadWriteByte(0xff);
  NRF24L01_CSN0=1;		
	
	return(status);                    
}

/***************************************************************************
函数名称：NRF24L01写数据到寄存器函数
函数功能：在NRF24L01寄存器中写入数据
函数备注：CMD：为寄存器地址，pBuf：为待写入数据地址，u8s：写入数据的个数
***************************************************************************/
u8 NRF24L01_Write_Buf(u8 CMD, u8 *pBuf, u8 num)
{
	u8 i,status;  
  NRF24L01_CSN0=0;	
	status = SPI2_ReadWriteByte(CMD);   
	for(i=0; i<num; i++)  
		SPI2_ReadWriteByte(*pBuf++);
	NRF24L01_CSN0=1;
	return status;
}

/***************************************************************************
函数名称：NRF24L01接收函数
函数功能：数据读取后放入rx_buf接收缓冲区中
函数备注：
***************************************************************************/
u8 NRF24L01_RxPacket(u8* rx_buf)
{
  u8 flag = 0;
	NRF24L01_CSN0=0;
	NRF24L01_CE=0;
	Delay_ms(1);
	Statu=NRF24L01_Read_Reg(RF_READ_REG + STATUS);                      // 读取状态寄存其来判断数据接收状况	
	NRF24L01_Write_Reg(RF_WRITE_REG+STATUS,Statu); //清除TX_DS或MAX_RT中断标志
	if(RX_DR)						                           // 判断是否接收到数据  如果置1则说明接到数据并且放置在接收缓存器
	{
		NRF24L01_Read_Buf(R_RX_PAYLOAD,rx_buf,RX_PAYLOAD_WIDTH); //去缓存器里读
		flag =1;			                               	           //读取数据完成标志
	}
	
	NRF24L01_Write_Reg(FLUSH_RX,0xff);                       //清除RX FIFO寄存器 
	NRF24L01_CSN0=1;
  NRF24L01_CE=1;
	return flag;
}

/***************************************************************************
函数名称：NRF24L01发射函数
函数功能：发送tx_buf中数据
函数备注：
***************************************************************************/
u8 NRF24L01_TxPacket(unsigned char * tx_buf)
{
	NRF24L01_CE=0;
	NRF24L01_CSN0=0;
	NRF24L01_Write_Reg(FLUSH_TX,0XFF);
	NRF24L01_Write_Buf(W_TX_PAYLOAD, tx_buf, TX_PAYLOAD_WIDTH);       //装载数据  将数据放入发送缓存器	
	NRF24L01_CSN0=0;
	NRF24L01_CE=1;
		while(NRF24L01_IRQ0 !=0);                                                    //等待发送完成
	Statu=NRF24L01_Read_Reg(STATUS);                                           //读取状态寄存器的值	   
    NRF24L01_Write_Reg(RF_WRITE_REG | STATUS,Statu);                     //清除TX_DS或MAX_RT中断标志
    if(Statu&0x10)//达到最大重发次数
	{
		NRF24L01_Write_Reg(FLUSH_TX,0xff);//清除TX FIFO寄存器 
		return 1; 
	}                                                      //使NRF24L01处于工作模式
	if(Statu&TX_OK)//发送完成
	{
		return TX_OK;
	}
	return 0;
} 

/***************************************************************************
函数名称：NRF24L01接收模式
函数功能：初始化NRF24L01
函数备注：在调用NRF24L01作为接收时，都要先调用该函数对NRF24L01进行初始化
***************************************************************************/
void NRF24L01_RX_Mode(u8 *_RX_ADDRESS)
{	
	SPI2_SetSpeed(SPI_BaudRatePrescaler_4); 
	NRF24L01_CE=0;
	NRF24L01_Write_Buf(RF_WRITE_REG + RX_ADDR_P0, _RX_ADDRESS, RX_ADR_WIDTH);    //写接收端地址
	NRF24L01_Write_Reg(RF_WRITE_REG + EN_AA, ENAA_P0);                          //频道0自动	ACK应答允许	  要应答说明还要配置发送才能应答！
	NRF24L01_Write_Reg(RF_WRITE_REG + EN_RXADDR, ERX_P0);                       //允许接收地址只有频道0，
	NRF24L01_Write_Reg(RF_WRITE_REG + RF_CH, 40);                               //设置信道工作为2.4GHZ + 40，收发必须一致，
	NRF24L01_Write_Reg(RF_WRITE_REG + RX_PW_P0, RX_PAYLOAD_WIDTH);              //设置接收数据长度，本次设置为32字节
	NRF24L01_Write_Reg(RF_WRITE_REG + RF_SETUP, 0X0F);   					     //设置发射速率为2MHZ，发射功率为最大值18dB
	NRF24L01_Write_Reg(RF_WRITE_REG + CONFIG, 0x0F);   		                 //IRQ收发完成中断响应，16位CRC	，主发送	
	NRF24L01_CE=1;
}

/***************************************************************************
函数名称：NRF24L01发送模式
函数功能：初始化NRF24L01
函数备注：在调用NRF24L01作为发送时，都要先调用该函数对NRF24L01进行初始化
***************************************************************************/
void NRF24L01_TX_Mode(u8 *_TX_ADDRESS,u8 *_RX_ADDRESS)
{
	SPI2_SetSpeed(SPI_BaudRatePrescaler_4); 
	NRF24L01_CE=0;
	NRF24L01_Write_Buf(RF_WRITE_REG + TX_ADDR, _TX_ADDRESS, TX_ADR_WIDTH);       //写本地地址	
	NRF24L01_Write_Buf(RF_WRITE_REG + RX_ADDR_P0, _RX_ADDRESS, RX_ADR_WIDTH);    //写接收端地址
	
	NRF24L01_Write_Reg(RF_WRITE_REG + EN_AA, ENAA_P0);                          //频道0自动	ACK应答允许	  要应答说明还要配置发送才能应答！
	NRF24L01_Write_Reg(RF_WRITE_REG + EN_RXADDR, ERX_P0);                       //允许接收地址只有频道0，
	NRF24L01_Write_Reg(RF_WRITE_REG + SETUP_RETR, 0x1a);				    	 // 500us + 86us, 10次重发
	NRF24L01_Write_Reg(RF_WRITE_REG + RF_CH, 40);                               //设置信道工作为2.4GHZ + 40，收发必须一致，
	NRF24L01_Write_Reg(RF_WRITE_REG + RX_PW_P0, RX_PAYLOAD_WIDTH);              //设置接收数据长度，本次设置为32字节
	NRF24L01_Write_Reg(RF_WRITE_REG + RF_SETUP, 0X0F);   					     //设置发射速率为2MHZ，发射功率为最大值18dB
	NRF24L01_Write_Reg(RF_WRITE_REG + CONFIG, 0x0E);   		                 //IRQ收发完成中断响应，16位CRC	，主接收	
	NRF24L01_CE=1;
}


/***************************************************************************
函数名称：NRF24L01初始化函数
函数功能：初始化NRF24L01
函数备注：在调用NRF24L01作为接收时，都要先调用该函数对NRF24L01进行初始化
***************************************************************************/
void NRF24L01_Init(void)
{  
	GPIO_InitTypeDef  GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB|RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_GPIOE, ENABLE);//使能GPIOB,G时钟
	
  //GPIOB14初始化设置:推挽输出
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
  GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化PB14
	
	//GPIOG6,7推挽输出
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
  GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化PG6,7
	
	//GPIOG.8上拉输入
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//输入
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
  GPIO_Init(GPIOE, &GPIO_InitStructure);//初始化PE5,6
	
	SPIx2_Init();
	SPI2_SetSpeed(SPI_BaudRatePrescaler_4);                                           //spi速度为10.5/2Mhz（24L01的最大SPI时钟为10Mhz）   
	
 	NRF24L01_CE   =0;   														                                  //使NRF24L01处于空闲模式
 	NRF24L01_CSN0 =1;                                                                 //失能NRF24L01
//	NRF24L01_TX_Mode(Dev_TX,Dev_RX);
//  NRF24L01_RX_Mode(Dev_RX);
	NRF24L01_CE   =1; 

}

/***************************************************************************
函数名称：NRF24测试函数
函数功能：用于测试NRF24是否正常
函数备注：返回 1 表示失败，返回 0 表示成功
***************************************************************************/
u8 NRF24L01_Check(void)
{
  u8 buf[5]={0xA5,0xA5,0xA5,0xA5,0xA5};
  u8 i;
	SPI2_SetSpeed(SPI_BaudRatePrescaler_4); 
  NRF24L01_Write_Buf(RF_WRITE_REG + TX_ADDR ,buf,5);
  NRF24L01_Read_Buf(TX_ADDR,buf,5);  
  for(i=0;i<5;i++)
  {
    if(buf[i]!=0xA5)
       break;        
  }                
  if(i!=5)
    return 1;        
  else
		return 0;   	
}




#endif  //____24L01_ENABLE
