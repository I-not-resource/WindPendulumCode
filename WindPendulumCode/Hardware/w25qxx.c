#include "w25qxx.h"
#include "spi.h"
#include "DMA.h"
#if  __W25QXX_ENABLE

u16 W25QXX_TYPE=0;

													 
/***************************************************************************
�������ƣ�W25QXX_Init
�������ܣ���ʼ��SPI FLASH��IO��
������ע��W25Q128,����Ϊ16M�ֽ�,����128��Block,4096��Sector 
         4KbytesΪһ��Sector,16������Ϊ1��Block
***************************************************************************/
u8 W25QXX_Init(void)
{ 
  GPIO_InitTypeDef  GPIO_InitStructure;
 
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);//ʹ��GPIOEʱ��

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;            //PB2
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;        //���
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;       //�������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;   //100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;         //����
  GPIO_Init(GPIOE, &GPIO_InitStructure);               //��ʼ��

	SPIx1_Init();		   			                             //��ʼ��SPI
	SPI1_SetSpeed(SPI_BaudRatePrescaler_2);		           //����Ϊ42Mʱ��,����ģʽ 
	W25QXX_TYPE=W25QXX_ReadID();	                       //��ȡFLASH ID.
	if(W25QXX_TYPE == W25Q128)
	  return 0;
	else
		return 1;
}  

/***************************************************************************
�������ƣ�W25QXX_ReadSR
�������ܣ���ȡW25QXX��״̬�Ĵ���
������ע��
         
***************************************************************************/
u8 W25QXX_ReadSR(void)   
{  
	u8 byte=0;   
	W25QXX_CS=0;                               //ʹ������   
	SPI1_ReadWriteByte(W25X_ReadStatusReg);    //���Ͷ�ȡ״̬�Ĵ�������    
	byte=SPI1_ReadWriteByte(0Xff);             //��ȡһ���ֽ�  
	W25QXX_CS=1;                               //ȡ��Ƭѡ     
	return byte;   
} 

/***************************************************************************
�������ƣ�W25QXX_Write_SR
�������ܣ�дW25QXX״̬�Ĵ���
������ע��ֻ��SPR,TB,BP2,BP1,BP0(bit 7,5,4,3,2)����д
         
***************************************************************************/
void W25QXX_Write_SR(u8 sr)   
{   
	W25QXX_CS=0;                               //ʹ������   
	SPI1_ReadWriteByte(W25X_WriteStatusReg);   //����дȡ״̬�Ĵ�������    
	SPI1_ReadWriteByte(sr);                    //д��һ���ֽ�  
	W25QXX_CS=1;                               //ȡ��Ƭѡ     	      
}   
 
/***************************************************************************
�������ƣ�W25QXX_Write_Enable
�������ܣ�W25QXXдʹ��,��WEL��λ  
������ע��
         
***************************************************************************/
void W25QXX_Write_Enable(void)   
{
	W25QXX_CS=0;                                 //ʹ������   
    SPI1_ReadWriteByte(W25X_WriteEnable);      //����дʹ��  
	W25QXX_CS=1;                                 //ȡ��Ƭѡ     	      
} 

/***************************************************************************
�������ƣ�W25QXX_Write_Disable
�������ܣ�W25QXXд��ֹ,��WEL����  
������ע��
         
***************************************************************************/
void W25QXX_Write_Disable(void)   
{  
	W25QXX_CS=0;                                 //ʹ������   
    SPI1_ReadWriteByte(W25X_WriteDisable);     //����д��ָֹ��    
	W25QXX_CS=1;                                 //ȡ��Ƭѡ     	      
} 		

/***************************************************************************
�������ƣ�W25QXX_ReadID
�������ܣ���ȡоƬID
������ע��
         
***************************************************************************/
u16 W25QXX_ReadID(void)
{
	u16 Temp = 0;	  
	W25QXX_CS=0;				    
	SPI1_ReadWriteByte(0x90);         //���Ͷ�ȡID����	    
	SPI1_ReadWriteByte(0x00); 	    
	SPI1_ReadWriteByte(0x00); 	    
	SPI1_ReadWriteByte(0x00); 	 			   
	Temp|=SPI1_ReadWriteByte(0xFF)<<8;  
	Temp|=SPI1_ReadWriteByte(0xFF);	 
	W25QXX_CS=1;				    
	return Temp;
}   		    

/***************************************************************************
�������ƣ�W25QXX_Read
�������ܣ���ȡSPI FLASH 
������ע��pBuffer:���ݴ洢��
         ReadAddr:��ʼ��ȡ�ĵ�ַ(24bit)
         NumByteToRead:Ҫ��ȡ���ֽ���(���65535)      
***************************************************************************/
void W25QXX_Read(u8* pBuffer,u32 ReadAddr,u16 NumByteToRead)   
{ 
 	u16 i;   										    
	W25QXX_CS=0;                                 //ʹ������   
    SPI1_ReadWriteByte(W25X_ReadData);         //���Ͷ�ȡ����   
    SPI1_ReadWriteByte((u8)((ReadAddr)>>16));  //����24bit��ַ    
    SPI1_ReadWriteByte((u8)((ReadAddr)>>8));   
    SPI1_ReadWriteByte((u8)ReadAddr);		 
    for(i=0;i<NumByteToRead;i++)
	  { 
        pBuffer[i]=SPI1_ReadWriteByte(0XFF);   //ѭ������  
    }
	W25QXX_CS=1;  				    	      
}  

/***************************************************************************
�������ƣ�W25QXX_Write_Page
�������ܣ���ָ����ַ��ʼд�����256�ֽڵ�����
         SPI��һҳ(0~65535)��д������256���ֽڵ�����
������ע��pBuffer:���ݴ洢��
         WriteAddr:��ʼд��ĵ�ַ(24bit)
				 NumByteToWrite:Ҫд����ֽ���(���256),������Ӧ�ó�����ҳ��ʣ���ֽ���
***************************************************************************/
void W25QXX_Write_Page(u8* pBuffer,u32 WriteAddr,u16 NumByteToWrite)
{
 	u16 i;  
    W25QXX_Write_Enable();                     //SET WEL 
	W25QXX_CS=0;                                 //ʹ������   
    SPI1_ReadWriteByte(W25X_PageProgram);      //����дҳ����   
    SPI1_ReadWriteByte((u8)((WriteAddr)>>16)); //����24bit��ַ    
    SPI1_ReadWriteByte((u8)((WriteAddr)>>8));   
    SPI1_ReadWriteByte((u8)WriteAddr); 
    for(i=0;i<NumByteToWrite;i++)
		SPI1_ReadWriteByte(pBuffer[i]);          //ѭ��д��  
	W25QXX_CS=1;                                 //ȡ��Ƭѡ 
	W25QXX_Wait_Busy();					                 //�ȴ�д�����
} 

/***************************************************************************
�������ƣ�W25QXX_Write_NoCheck
�������ܣ��޼���дSPI FLASH,����ȷ����д�ĵ�ַ��Χ�ڵ�����ȫ��Ϊ0XFF
          �����ڷ�0XFF��д������ݽ�ʧ��,�����Զ���ҳ����,
					��ָ����ַ��ʼд��ָ�����ȵ�����
������ע��pBuffer:���ݴ洢��
         WriteAddr:��ʼд��ĵ�ַ(24bit)
				 NumByteToWrite:Ҫд����ֽ���(���65535)
***************************************************************************/
void W25QXX_Write_NoCheck(u8* pBuffer,u32 WriteAddr,u16 NumByteToWrite)   
{ 			 		 
	u16 pageremain;	   
	pageremain=256-WriteAddr%256; //��ҳʣ����ֽ���		 	    
	if(NumByteToWrite<=pageremain)pageremain=NumByteToWrite;//������256���ֽ�
	while(1)
	{	   
		W25QXX_Write_Page(pBuffer,WriteAddr,pageremain);
		if(NumByteToWrite==pageremain)break;//д�������
	 	else                                      //NumByteToWrite>pageremain
		{
			pBuffer+=pageremain;
			WriteAddr+=pageremain;	

			NumByteToWrite-=pageremain;			  //��ȥ�Ѿ�д���˵��ֽ���
			if(NumByteToWrite>256)pageremain=256; //һ�ο���д��256���ֽ�
			else pageremain=NumByteToWrite; 	  //����256���ֽ���
		}
	};	    
} 

/***************************************************************************
�������ƣ�W25QXX_Write
�������ܣ�дSPI FLASH,��ָ����ַ��ʼд��ָ�����ȵ�����,�ú�������������
������ע��pBuffer:���ݴ洢��
         WriteAddr:��ʼд��ĵ�ַ(24bit)
				 NumByteToWrite:Ҫд����ֽ���(���65535) 
***************************************************************************/
u8 W25QXX_BUFFER[4096];		 
void W25QXX_Write(u8* pBuffer,u32 WriteAddr,u16 NumByteToWrite)   
{ 
	u32 secpos;
	u16 secoff;
	u16 secremain;	   
 	u16 i;    
	u8 * W25QXX_BUF;	  
   	W25QXX_BUF=W25QXX_BUFFER;	     
 	secpos=WriteAddr/4096;                                    //������ַ  
	secoff=WriteAddr%4096;                                    //�������ڵ�ƫ��
	secremain=4096-secoff;                                    //����ʣ��ռ��С   
 	if(NumByteToWrite<=secremain)secremain=NumByteToWrite;    //������4096���ֽ�
	while(1) 
	{	
		W25QXX_Read(W25QXX_BUF,secpos*4096,4096);               //������������������
		for(i=0;i<secremain;i++)                                //У������
		{
			if(W25QXX_BUF[secoff+i]!=0XFF)break;                  //��Ҫ����  	  
		}
		if(i<secremain)                                         //��Ҫ����
		{
			W25QXX_Erase_Sector(secpos);                          //�����������
			for(i=0;i<secremain;i++)	                            //����
			{
				W25QXX_BUF[i+secoff]=pBuffer[i];	  
			}
			W25QXX_Write_NoCheck(W25QXX_BUF,secpos*4096,4096);    //д����������  

		}else W25QXX_Write_NoCheck(pBuffer,WriteAddr,secremain);//д�Ѿ������˵�,ֱ��д������ʣ������. 				   
		if(NumByteToWrite==secremain)break;       //д�������
		else                                      //д��δ����
		{
			secpos++;                               //������ַ��1
			secoff=0;                               //ƫ��λ��Ϊ0 	 

		   	pBuffer+=secremain;                   //ָ��ƫ��
			WriteAddr+=secremain;                   //д��ַƫ��	   
		   	NumByteToWrite-=secremain;			    	//�ֽ����ݼ�
			if(NumByteToWrite>4096)secremain=4096;	//��һ����������д����
			else secremain=NumByteToWrite;			    //��һ����������д����
		}	 
	};	 
}

/***************************************************************************
�������ƣ�W25QXX_Erase_Chip
�������ܣ���������оƬ	
������ע���ȴ�ʱ�䳬��
         
***************************************************************************/
void W25QXX_Erase_Chip(void)   
{                                   
    W25QXX_Write_Enable();                     //SET WEL 
    W25QXX_Wait_Busy();   
  	W25QXX_CS=0;                               //ʹ������   
    SPI1_ReadWriteByte(W25X_ChipErase);        //����Ƭ��������  
	W25QXX_CS=1;                                 //ȡ��Ƭѡ     	      
	W25QXX_Wait_Busy();   				               //�ȴ�оƬ��������
}   

/***************************************************************************
�������ƣ�W25QXX_Erase_Sector
�������ܣ�����һ������,����һ��ɽ��������ʱ��:150ms
������ע��Dst_Addr:������ַ ����ʵ����������
         
***************************************************************************/
void W25QXX_Erase_Sector(u32 Dst_Addr)   
{
 	Dst_Addr*=4096;
    W25QXX_Write_Enable();                     //SET WEL 	 
    W25QXX_Wait_Busy();   
  	W25QXX_CS=0;                               //ʹ������   
    SPI1_ReadWriteByte(W25X_SectorErase);      //������������ָ�� 
    SPI1_ReadWriteByte((u8)((Dst_Addr)>>16));  //����24bit��ַ    
    SPI1_ReadWriteByte((u8)((Dst_Addr)>>8));   
    SPI1_ReadWriteByte((u8)Dst_Addr);  
	W25QXX_CS=1;                                 //ȡ��Ƭѡ     	      
    W25QXX_Wait_Busy();   				             //�ȴ��������
}

/***************************************************************************
�������ƣ�W25QXX_Wait_Busy
�������ܣ��ȴ�����
������ע��
         
***************************************************************************/
void W25QXX_Wait_Busy(void)   
{   
	while((W25QXX_ReadSR()&0x01)==0x01);   // �ȴ�BUSYλ���
}

/***************************************************************************
�������ƣ�W25QXX_PowerDown
�������ܣ��������ģʽ
������ע��
         
***************************************************************************/
void W25QXX_PowerDown(void)   
{ 
  	W25QXX_CS=0;                               //ʹ������   
    SPI1_ReadWriteByte(W25X_PowerDown);        //���͵�������  
	W25QXX_CS=1;                                 //ȡ��Ƭѡ     	      
    Delay_us(3);                               //�ȴ�TPD  
}   

/***************************************************************************
�������ƣ�W25QXX_WAKEUP
�������ܣ�����
������ע��
         
***************************************************************************/
void W25QXX_WAKEUP(void)   
{  
  	W25QXX_CS=0;                                 //ʹ������   
    SPI1_ReadWriteByte(W25X_ReleasePowerDown);   //send W25X_PowerDown command 0xAB    
	W25QXX_CS=1;                                   //ȡ��Ƭѡ     	      
    Delay_us(3);                                 //�ȴ�TRES1
}   






#endif  //__W25QXX_ENABLE
