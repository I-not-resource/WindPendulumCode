#ifndef __ALL_CONFIG_H_
#define __ALL_CONFIG_H_
/******************实时时钟开关***********************/
#define __RTC_ENABLE       1u

/******************通信    开关***********************/
#define __USART_ENABLE     1u
#define __SPI_ENABLE       1u
#define __IIC_ENABLE       1u
#define __24L01_ENABLE     1U

/******************DMA 传输开关***********************/
#define __DMA_ENABLE       1u

/******************显示应用开关***********************/
#define __SPECIALLY_ENABLE 1u
#define __LCD_ENABLE       1u
#define __DISPLAY_ENABLE   1u

/******************正交编码开关***********************/
#define __ORT_ENABLE       0u

/******************PWM 功能开关***********************/
#define __PWM_ENABLE       1u

/******************PID 算法开关***********************/
#define __PID_ENABLE       1u

/******************ADC 功能开关***********************/
#define __ADC_ENABLE       1u

/******************定时器  开关***********************/
#define __TIM_ENABLE       1U

/******************按键功能开关***********************/
#define __KEY_ENABLE       1U

/******************MPU 功能开关***********************/
#define __MPU6050_ENABLE   1u

/******************RUN 功能开关***********************/
#define __RUN_ENABLE       1u


#define __IRQ_CELL_ENABLE  1u
/******************中断级别定义***********************/
#define ORT_ENC_PP      2                           //正交编码定时器抢占优先级
#define ORT_ENC_SP      2                           //正交编码定时器响应优先级

#define USART1_PP       3                           //串口通信1抢占优先级
#define USART1_PS       3                           //串口通信1响应优先级

#define RTC_PP          1                           //RTC抢占优先级
#define RTC_PS          1                           //RTC响应优先级

#define TIM5_PP         0                           //定时器5抢占优先级
#define TIM5_PS         4                           //定时器5响应优先级

#define TIM7_PP         1                           //定时器7抢占优先级
#define TIM7_PS         2                           //定时器7响应优先级

#define TIM9_PP         2                           //定时器9抢占优先级
#define TIM9_PS         2                           //定时器9响应优先级

#define TIM11_PP        2                           //定时器11抢占优先级
#define TIM11_PS        3                           //定时器11响应优先级

#define DMA2S0_PP       1
#define DMA2S0_PS       3



#endif
