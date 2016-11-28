#include "Usart.h"
#if __USART_ENABLE

u8 USART_RX_BUF[USART_REC_LEN];     //���ջ���,���USART_REC_LEN���ֽ�.
u16 USART_RX_STA=0;                 //����״̬���

/****************************��printf�����ض���*****************************/
#pragma import(__use_no_semihosting)             //��׼����Ҫ��֧�ֺ��� 
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
//����_sys_exit()�Ա���ʹ�ð�����ģʽ 
/***************************************************************************
�������ƣ�sys_exit
�������ܣ�����_sys_exit()�Ա���ʹ�ð�����ģʽ 
������ע��
***************************************************************************/
sys_exit(int x) 
{ 
	
	x = x; 
	
}

/***************************************************************************
�������ƣ�fputc
�������ܣ��ض���fputc����
������ע��
***************************************************************************/
int fputc(int ch, FILE *f)
{ 	
	while((USART1->SR&0X40)==0);//ѭ������,ֱ���������
	USART1->DR = (u8) ch;      
	return ch;
}

/***************************************************************************
�������ƣ�usart_Init
�������ܣ�����1��ʼ��
������ע��
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
	USART_InitStructure.USART_HardwareFlowControl        = USART_HardwareFlowControl_None;  //��Ӳ��������
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
�������ƣ�usart_Init
�������ܣ�����1��ʼ��
������ע��
***************************************************************************/
void USART6_Config(void)   //��ʼ�� ����USART2
{
	GPIO_InitTypeDef    GPIO_InitStructure;	   //���ڶ˿����ýṹ�����
	USART_InitTypeDef   USART_InitStructure;   //���ڲ������ýṹ�����
	NVIC_InitTypeDef    NVIC_InitStructure; 

	//ʹ�� USART2 ʱ��
	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);	//�򿪴��ڸ���ʱ��
   RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);   //��PA�˿�ʱ��

	//��USART1 Tx�����ͽţ���GPIO����Ϊ���츴��ģʽ   
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;    //ѡ���ĸ�IO�� ��ѡ��PA2
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;           //�����������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;     //100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;           //����
	GPIO_Init(GPIOC, &GPIO_InitStructure);    				     //��ʼ��GPIOA

	//��USART1 Rx�����սţ���GPIO����Ϊ��������ģʽ														  
	
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource6,GPIO_AF_USART6);
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource7,GPIO_AF_USART6);
	  
	//����USART2����
	USART_InitStructure.USART_BaudRate = 115200;	                  //���������ã�115200
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;	    //����λ�����ã�8λ
	USART_InitStructure.USART_StopBits = USART_StopBits_1; 	        //ֹͣλ���ã�1λ
	USART_InitStructure.USART_Parity = USART_Parity_No ;            //�Ƿ���żУ�飺��
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;	//Ӳ��������ģʽ���ã�û��ʹ��
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; //�����뷢�Ͷ�ʹ��
	USART_Init(USART6, &USART_InitStructure);                       //��ʼ��USART2
	
	//�򿪷����жϺͽ����ж�(�����Ҫ�ж�)
  	USART_ITConfig(USART6, USART_IT_RXNE, ENABLE); // ʹ��ָ����USART2�����ж�

	//ʹ�� USART2�� �������
	USART_Cmd(USART6, ENABLE);                             // USART1ʹ��

	//�����������1���ֽ��޷���ȷ���ͳ�ȥ������
    USART_ClearFlag(USART6, USART_FLAG_TC);                //�崮��1���ͱ�־

  NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;			   //ָ���ж�Դ
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;	   //��ռ���ȼ�1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;             //ָ����Ӧ���ȼ���1
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	               //���ж�
	NVIC_Init(&NVIC_InitStructure);			
}

/***************************************************************************
�������ƣ�USART1_IRQHandler
�������ܣ�����1�жϷ�����
������ע����0x0d,0x0a��β
***************************************************************************/
void USART1_IRQHandler(void)
{
	u8 res;
	
	if(USART_GetITStatus(USART1,USART_IT_RXNE) != RESET)   //�����ж�
	{
		res = USART_ReceiveData(USART1);
		
		
	if((USART_RX_STA & 0x8000) == 0)                       //����δ���
		{
			if(USART_RX_STA & 0x4000)                          //���յ���0x0d
			{
				if(res != 0x0a)                                  //���մ���,���¿�ʼ
					USART_RX_STA = 0;
				else
					USART_RX_STA |= 0x8000;                        //��������� 
			}
			else                                               //��û�յ�0X0D
			{	
				if(res == 0x0d)
					USART_RX_STA |= 0x4000;
				else
				{
					USART_RX_BUF[USART_RX_STA&0X3FFF] = res ;
					USART_RX_STA++;
					if(USART_RX_STA>(USART_REC_LEN-1))
						USART_RX_STA = 0;                            //�������ݴ���,���¿�ʼ����	  
				}
			}
		}
	
	}
	PFout(9) = !PFin(9) ;
}












#endif   //__USART_ENABLE
