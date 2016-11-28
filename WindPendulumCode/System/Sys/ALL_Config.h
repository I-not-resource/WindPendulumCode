#ifndef __ALL_CONFIG_H_
#define __ALL_CONFIG_H_
/******************ʵʱʱ�ӿ���***********************/
#define __RTC_ENABLE       1u

/******************ͨ��    ����***********************/
#define __USART_ENABLE     1u
#define __SPI_ENABLE       1u
#define __IIC_ENABLE       1u
#define __24L01_ENABLE     1U

/******************DMA ���俪��***********************/
#define __DMA_ENABLE       1u

/******************��ʾӦ�ÿ���***********************/
#define __SPECIALLY_ENABLE 1u
#define __LCD_ENABLE       1u
#define __DISPLAY_ENABLE   1u

/******************�������뿪��***********************/
#define __ORT_ENABLE       0u

/******************PWM ���ܿ���***********************/
#define __PWM_ENABLE       1u

/******************PID �㷨����***********************/
#define __PID_ENABLE       1u

/******************ADC ���ܿ���***********************/
#define __ADC_ENABLE       1u

/******************��ʱ��  ����***********************/
#define __TIM_ENABLE       1U

/******************�������ܿ���***********************/
#define __KEY_ENABLE       1U

/******************MPU ���ܿ���***********************/
#define __MPU6050_ENABLE   1u

/******************RUN ���ܿ���***********************/
#define __RUN_ENABLE       1u


#define __IRQ_CELL_ENABLE  1u
/******************�жϼ�����***********************/
#define ORT_ENC_PP      2                           //�������붨ʱ����ռ���ȼ�
#define ORT_ENC_SP      2                           //�������붨ʱ����Ӧ���ȼ�

#define USART1_PP       3                           //����ͨ��1��ռ���ȼ�
#define USART1_PS       3                           //����ͨ��1��Ӧ���ȼ�

#define RTC_PP          1                           //RTC��ռ���ȼ�
#define RTC_PS          1                           //RTC��Ӧ���ȼ�

#define TIM5_PP         0                           //��ʱ��5��ռ���ȼ�
#define TIM5_PS         4                           //��ʱ��5��Ӧ���ȼ�

#define TIM7_PP         1                           //��ʱ��7��ռ���ȼ�
#define TIM7_PS         2                           //��ʱ��7��Ӧ���ȼ�

#define TIM9_PP         2                           //��ʱ��9��ռ���ȼ�
#define TIM9_PS         2                           //��ʱ��9��Ӧ���ȼ�

#define TIM11_PP        2                           //��ʱ��11��ռ���ȼ�
#define TIM11_PS        3                           //��ʱ��11��Ӧ���ȼ�

#define DMA2S0_PP       1
#define DMA2S0_PS       3



#endif
