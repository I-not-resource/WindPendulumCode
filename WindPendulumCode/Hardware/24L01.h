#ifndef __24L01_H_
#define __24L01_H_
#include "sys.h"
#include "spi.h"
#if __24L01_ENABLE

#define TX_ADR_WIDTH      5   		
#define RX_ADR_WIDTH      5 
#define TX_PAYLOAD_WIDTH  6
#define RX_PAYLOAD_WIDTH  6


#define MAX_TX  		0x10  //�ﵽ����ʹ����ж�
#define TX_OK   		0x20  //TX��������ж�
#define RX_OK   		0x40  //���յ������ж�

/**********************************NRF24L01�Ĵ���ָ��*********************************************/
#define RF_READ_REG     0x00  	// ���Ĵ���ָ��
#define RF_WRITE_REG    0x20 	// д�Ĵ���ָ��
#define R_RX_PAYLOAD    0x61  	// ��ȡ��������ָ��
#define W_TX_PAYLOAD    0xA0  	// д��������ָ��
#define FLUSH_TX        0xE1 	// ��ϴ���� FIFOָ��
#define FLUSH_RX        0xE2  	// ��ϴ���� FIFOָ��
#define REUSE_TX_PL     0xE3  	// �����ظ�װ������ָ��
#define NOP             0xFF  	// ����

/**********************************NRF24L01--SPI�Ĵ�����ַ*****************************************/
#define CONFIG          0x00  // �����շ�״̬��CRCУ��ģʽ�Լ��շ�״̬��Ӧ��ʽ
#define EN_AA           0x01  // �Զ�Ӧ��������
#define EN_RXADDR       0x02  // �����ŵ�����
#define SETUP_AW        0x03  // �շ���ַ�������
#define SETUP_RETR      0x04  // �Զ��ط���������
#define RF_CH           0x05  // ����Ƶ������
#define RF_SETUP        0x06  // �������ʡ����Ĺ�������
#define STATUS          0x07  // ״̬�Ĵ���
#define OBSERVE_TX      0x08  // ���ͼ�⹦��
#define CD              0x09  // ��ַ���           
#define RX_ADDR_P0      0x0A  // Ƶ��0�������ݵ�ַ
#define RX_ADDR_P1      0x0B  // Ƶ��1�������ݵ�ַ
#define RX_ADDR_P2      0x0C  // Ƶ��2�������ݵ�ַ
#define RX_ADDR_P3      0x0D  // Ƶ��3�������ݵ�ַ
#define RX_ADDR_P4      0x0E  // Ƶ��4�������ݵ�ַ
#define RX_ADDR_P5      0x0F  // Ƶ��5�������ݵ�ַ
#define TX_ADDR         0x10  // ���͵�ַ�Ĵ���
#define RX_PW_P0        0x11  // ����Ƶ��0�������ݳ���
#define RX_PW_P1        0x12  // ����Ƶ��0�������ݳ���
#define RX_PW_P2        0x13  // ����Ƶ��0�������ݳ���
#define RX_PW_P3        0x14  // ����Ƶ��0�������ݳ���
#define RX_PW_P4        0x15  // ����Ƶ��0�������ݳ���
#define RX_PW_P5        0x16  // ����Ƶ��0�������ݳ���
#define FIFO_STATUS     0x17  // FIFOջ��ջ��״̬�Ĵ�������

//��������ͨ��N
enum ERX_Pn
{
	ERX_P0 = 0x01,
	ERX_P1 = 0x02,
	ERX_P2 = 0x04,
	ERX_P3 = 0x08,
	ERX_P4 = 0x10,
	ERX_P5 = 0x20,
};
//����ͨ��N�Զ�Ӧ��
enum ENAA_Pn
{
	ENAA_P0 = 0x01,
	ENAA_P1 = 0x02,
	ENAA_P2 = 0x04,
	ENAA_P3 = 0x08,
	ENAA_P4 = 0x10,
	ENAA_P5 = 0x20,
};
//���书��
enum RF_PWRn
{
	RF_PWR18 = 0x00,
	RF_PWR12 = 0x02,
	RF_PWR06 = 0x04,
	RF_PWR00 = 0x06,
};
//��������
enum RF_DR
{
	_1Mbps = 0x00,
	_2Mbps = 0x08,
};
/**********************************RNF24L01״̬��־λ************************************************/
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

#define NRF24L01_CE    PBout(12) 	  //24L01Ƭѡ�ź�
#define NRF24L01_CSN0  PAout(2) 	  //SPIƬѡ�ź�0
#define NRF24L01_CSN1  PAout(3) 	  //SPIƬѡ�ź�1
#define NRF24L01_IRQ0  PEin(5)  	  //IRQ������������0
#define NRF24L01_IRQ1  PEin(6)  	  //IRQ������������1

#define NRF_0          0
#define NRF_1          1


void NRF24L01_CSN_ON(u8 X);
void NRF24L01_CSN_OFF(u8 X);
u8 NRF24L01_Read_Reg(u8 CMD);
u8 NRF24L01_Write_Reg(u8 CMD, u8 value);
u8 NRF24L01_Read_Buf(u8 CMD, u8 *pBuf, u8 num);
u8 NRF24L01_Write_Buf(u8 CMD, u8 *pBuf, u8 num);

u8 NRF24L01_RxPacket(u8* rx_buf);                     //����������ڽ�������
u8 NRF24L01_TxPacket(unsigned char * tx_buf);	    //����������ڷ�������

void NRF24L01_RX_Mode(u8 *_RX_ADDRESS);
void NRF24L01_TX_Mode(u8 *_TX_ADDRESS,u8 *_RX_ADDRESS);
void NRF24L01_Init(void);
u8 NRF24L01_Check(void);


#endif  //____24L01_ENABLE
#endif  //__24L01_H_
