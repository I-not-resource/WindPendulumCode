#include "Usart.h"
#if __USART_ENABLE

u8 USART_RX_BUF[USART_REC_LEN];     //接收缓冲,最大USART_REC_LEN个字节.
u16 USART_RX_STA=0;                 //接收状态标记

/****************************对printf进行重定义*****************************/
#pragma import(__use_no_semihosting)             //标准库需要的支持函数 
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式 
/***************************************************************************
函数名称：sys_exit
函数功能：定义_sys_exit()以避免使用半主机模式 
函数备注：
***************************************************************************/
sys_exit(int x) 
{ 
	
	x = x; 
	
}

/***************************************************************************
函数名称：fputc
函数功能：重定义fputc函数
函数备注：
***************************************************************************/
int fputc(int ch, FILE *f)
{ 	
	while((USART1->SR&0X40)==0);//循环发送,直到发送完毕
	USART1->DR = (u8) ch;      
	return ch;
}

/***************************************************************************
函数名称：usart_Init
函数功能：串口1初始化
函数备注：
***************************************************************************/
void usart_Init(void)
{
	
	GPIO_InitTypeDef  GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef  NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
	
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9, GPIO_AF_USART1);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1);
	
	GPIO_InitStructure.GPIO_Pin                          = GPIO_Pin_9 | GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode                         = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed                        = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType                        = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd                         = GPIO_PuPd_UP;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	USART_InitStructure.USART_BaudRate                   = 115200;
	USART_InitStructure.USART_WordLength                 = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits                   = USART_StopBits_1;
	USART_InitStructure.USART_Parity                     = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl        = USART_HardwareFlowControl_None;  //无硬件数据流
	USART_InitStructure.USART_Mode                       = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1,&USART_InitStructure);
	USART_Cmd(USART1,ENABLE);
	USART_ClearFlag(USART1,USART_FLAG_TC);
	
#if EN_USART1_RX	
	USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);
	
	NVIC_InitStructure.NVIC_IRQChannel                   = USART1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = USART1_PP;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority        = USART1_PS;
	NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
#endif   //EN_USART1_RX
	
}



/***************************************************************************
函数名称：usart_Init
函数功能：串口1初始化
函数备注：
***************************************************************************/
void USART6_Config(void)   //初始化 配置USART2
{
	GPIO_InitTypeDef    GPIO_InitStructure;	   //串口端口配置结构体变量
	USART_InitTypeDef   USART_InitStructure;   //串口参数配置结构体变量
	NVIC_InitTypeDef    NVIC_InitStructure; 

	//使能 USART2 时钟
	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);	//打开串口复用时钟
   RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);   //打开PA端口时钟

	//将USART1 Tx（发送脚）的GPIO配置为推挽复用模式   
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;    //选定哪个IO口 现选定PA2
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;           //复用推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;     //100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;           //上拉
	GPIO_Init(GPIOC, &GPIO_InitStructure);    				     //初始化GPIOA

	//将USART1 Rx（接收脚）的GPIO配置为浮空输入模式														  
	
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource6,GPIO_AF_USART6);
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource7,GPIO_AF_USART6);
	  
	//配置USART2参数
	USART_InitStructure.USART_BaudRate = 115200;	                  //波特率设置：115200
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;	    //数据位数设置：8位
	USART_InitStructure.USART_StopBits = USART_StopBits_1; 	        //停止位设置：1位
	USART_InitStructure.USART_Parity = USART_Parity_No ;            //是否奇偶校验：无
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;	//硬件流控制模式设置：没有使能
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; //接收与发送都使能
	USART_Init(USART6, &USART_InitStructure);                       //初始化USART2
	
	//打开发送中断和接收中断(如果需要中断)
  	USART_ITConfig(USART6, USART_IT_RXNE, ENABLE); // 使能指定的USART2接收中断

	//使能 USART2， 配置完毕
	USART_Cmd(USART6, ENABLE);                             // USART1使能

	//如下语句解决第1个字节无法正确发送出去的问题
    USART_ClearFlag(USART6, USART_FLAG_TC);                //清串口1发送标志

  NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;			   //指定中断源
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;	   //抢占优先级1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;             //指定响应优先级别1
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	               //打开中断
	NVIC_Init(&NVIC_InitStructure);			
}

/***************************************************************************
函数名称：USART1_IRQHandler
函数功能：串口1中断服务函数
函数备注：以0x0d,0x0a结尾
***************************************************************************/
void USART1_IRQHandler(void)
{
	u8 res;
	
	if(USART_GetITStatus(USART1,USART_IT_RXNE) != RESET)   //接收中断
	{
		res = USART_ReceiveData(USART1);
		
		
	if((USART_RX_STA & 0x8000) == 0)                       //接收未完成
		{
			if(USART_RX_STA & 0x4000)                          //接收到了0x0d
			{
				if(res != 0x0a)                                  //接收错误,重新开始
					USART_RX_STA = 0;
				else
					USART_RX_STA |= 0x8000;                        //接收完成了 
			}
			else                                               //还没收到0X0D
			{	
				if(res == 0x0d)
					USART_RX_STA |= 0x4000;
				else
				{
					USART_RX_BUF[USART_RX_STA&0X3FFF] = res ;
					USART_RX_STA++;
					if(USART_RX_STA>(USART_REC_LEN-1))
						USART_RX_STA = 0;                            //接收数据错误,重新开始接收	  
				}
			}
		}
	
	}
	PFout(9) = !PFin(9) ;
}












#endif   //__USART_ENABLE
