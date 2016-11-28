#ifndef __USART_H_
#define __USART_H_
#include "sys.h"
#include "stdio.h"
#include "stm32f4xx_conf.h"
#if __USART_ENABLE

#define USART_REC_LEN  	200  	          //�����������ֽ��� 200
#define EN_USART1_RX 			1	           	//ʹ�ܣ�1��/��ֹ��0������1����

extern u8  USART_RX_BUF[USART_REC_LEN]; //���ջ���,���USART_REC_LEN���ֽ�.ĩ�ֽ�Ϊ���з� 
extern u16 USART_RX_STA;                //����״̬���

void usart_Init(void);
void USART6_Config(void);   //��ʼ�� ����USART2

#endif    //__USART_ENABLE

#endif    //__USART_H_
