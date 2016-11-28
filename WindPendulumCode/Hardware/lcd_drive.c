#include "lcd_drive.h"
#if      __LCD_ENABLE
////////////////////////////////////////////////////////////////////////////
//���棡����
//��Ӧ�ó���רΪ5510�Ż����
//��˲��߱����õļ�����
////////////////////////////////////////////////////////////////////////////
u16 POINT_COLOR = GREEN;	  //������ɫ
u16 BACK_COLOR  = BLACK;  //����ɫ 

//����LCD��Ҫ����
//Ĭ��Ϊ����
_lcd_dev lcddev;

/***************************************************************************
�������ƣ�LCD_WR_REG
�������ܣ�д�Ĵ�������
������ע��
***************************************************************************/
void LCD_WR_REG(u16 regval)
{
	regval = regval;
	LCD -> LCD_REG = regval;
}

/***************************************************************************
�������ƣ�LCD_WR_DATA
�������ܣ�дLCD�Ĵ���ֵ
������ע��
***************************************************************************/
void LCD_WR_DATA(u16 regval)
{
	regval = regval;
	LCD -> LCD_RAM = regval;
}

/***************************************************************************
�������ƣ�LCD_RD_DATA
�������ܣ���LCD�Ĵ���ֵ
������ע��
***************************************************************************/
u16 LCD_RD_DATA(void)
{
	vu16 ram;
	ram = LCD -> LCD_RAM;
	return ram;
}

/***************************************************************************
�������ƣ�LCD_WriteReg
�������ܣ�д�Ĵ���
������ע��
***************************************************************************/
void LCD_WriteReg(vu16 LCD_Reg,vu16 LCD_RegValue)
{
	LCD -> LCD_REG = LCD_Reg;
	LCD -> LCD_RAM = LCD_RegValue;
}

/***************************************************************************
�������ƣ�LCD_ReadReg
�������ܣ����Ĵ���
������ע��
***************************************************************************/
u16 LCD_ReadReg(vu16 LCD_Reg)
{
	LCD_WR_REG(LCD_Reg);
	Delay_us(5);
	return LCD_RD_DATA();
}

/***************************************************************************
�������ƣ�LCD_WriteRAM_Perepare
�������ܣ���ʼдGRAM
������ע��д�������ɫֵ
***************************************************************************/
void LCD_WriteRAM_Prepare(void)
{
	LCD -> LCD_REG = lcddev.wramcmd;
}


/***************************************************************************
�������ƣ�LCD_WriteRAM
�������ܣ�LCDдGRAM
������ע��д�������ɫֵ
***************************************************************************/
void LCD_WriteRAM(u16 RGB_Code)
{							    
	LCD->LCD_RAM = RGB_Code;//дʮ��λGRAM
}

/***************************************************************************
�������ƣ�LCD_BGR2RGB
�������ܣ�LCDдGram
������ע����ILI93xx����������ΪGBR��ʽ��������д���ʱ��ΪRGB��ʽ��
����ֵ��RGB��ʽ����ɫֵ
***************************************************************************/
u16 LCD_BGR2RGB(u16 c)
{
	u16  r,g,b,rgb;   
	b=(c>>0)&0x1f;
	g=(c>>5)&0x3f;
	r=(c>>11)&0x1f;	 
	rgb=(b<<11)+(g<<5)+(r<<0);		 
	return(rgb);
}

/***************************************************************************
�������ƣ�opt_delay
�������ܣ���mdk -O1ʱ���Ż�ʱ��Ҫ����
������ע��
***************************************************************************/
void opt_delay(u8 i)
{
	while(i--);
}

/***************************************************************************
�������ƣ�LCD_ReadPoint
�������ܣ���ȡ��ĳ�����ɫֵ
������ע��
***************************************************************************/
u16 LCD_ReadPoint(u16 x,u16 y)
{
 	vu16 r=0,g=0,b=0;
	if(x>=lcddev.width||y>=lcddev.height)return 0; //�����˷�Χ,ֱ�ӷ���		   
	LCD_SetCursor(x,y);	    
	LCD_WR_REG(0X2E00);                            //5510 ���Ͷ�GRAMָ��    
 	LCD_RD_DATA();									               //dummy Read	   
	opt_delay(2);	  
 	r=LCD_RD_DATA();  		  						           //ʵ��������ɫ
	opt_delay(2);	  
	b=LCD_RD_DATA(); 
	g=r&0XFF;		                                 //��5510,��һ�ζ�ȡ����RG��ֵ,R��ǰ,G�ں�,��ռ8λ
	g<<=8; 
	return (((r>>11)<<11)|((g>>10)<<5)|(b>>11));   ///NT35510��Ҫ��ʽת��һ��
}			 

/***************************************************************************
�������ƣ�LCD_DisplayOn
�������ܣ�LCD������ʾ
������ע��
***************************************************************************/
void LCD_DisplayOn(void)
{
  LCD_WR_REG(0X2900);	//������ʾ
}	 

/***************************************************************************
�������ƣ�LCD_DisplayOff
�������ܣ�LCD�ر���ʾ
������ע��
***************************************************************************/
void LCD_DisplayOff(void)
{	   
  LCD_WR_REG(0X2800);	//�ر���ʾ
} 

/***************************************************************************
�������ƣ�LCD_SetCursor
�������ܣ����ù��λ��
������ע��
***************************************************************************/
void LCD_SetCursor(u16 Xpos, u16 Ypos)
{	 
		LCD_WR_REG(lcddev.setxcmd); 
		LCD_WR_DATA(Xpos>>8); 
		LCD_WR_REG(lcddev.setxcmd+1); 
		LCD_WR_DATA(Xpos&0XFF);	 
		LCD_WR_REG(lcddev.setycmd); 
		LCD_WR_DATA(Ypos>>8); 
		LCD_WR_REG(lcddev.setycmd+1); 
		LCD_WR_DATA(Ypos&0XFF);		
} 		 

/***************************************************************************
�������ƣ�LCD_Scan_Dir
�������ܣ�����LCD���Զ�ɨ�跽��
������ע�������������ܻ��ܵ��˺������õ�Ӱ��,
         ����,һ������ΪL2R_U2D����,�������Ϊ����ɨ�跽ʽ,���ܵ�����ʾ������.
				 L2R_U2D 
				 L2R_D2U 
				 R2L_U2D 
				 R2L_D2U 

				 U2D_L2R 
				 U2D_R2L
				 D2U_L2R 
				 D2U_R2L
***************************************************************************/
void LCD_Scan_Dir(u8 dir)
{
	u16 regval=0;
	u16 dirreg=0;
	u16 temp;  

  switch(dir)
  {
		case L2R_U2D:                   //������,���ϵ���
			regval|=(0<<7)|(0<<6)|(0<<5); 
			break;
		case L2R_D2U:                   //������,���µ���
			regval|=(1<<7)|(0<<6)|(0<<5); 
			break;
		case R2L_U2D:                   //���ҵ���,���ϵ���
			regval|=(0<<7)|(1<<6)|(0<<5); 
			break;
		case R2L_D2U:                   //���ҵ���,���µ���
			regval|=(1<<7)|(1<<6)|(0<<5); 
			break;	 
		case U2D_L2R:                   //���ϵ���,������
			regval|=(0<<7)|(0<<6)|(1<<5); 
			break;
		case U2D_R2L:                   //���ϵ���,���ҵ���
			regval|=(0<<7)|(1<<6)|(1<<5); 
			break;
		case D2U_L2R:                   //���µ���,������
			regval|=(1<<7)|(0<<6)|(1<<5); 
			break;
		case D2U_R2L:                   //���µ���,���ҵ���
      regval|=(1<<7)|(1<<6)|(1<<5); 
      break;	 
  }
		
  dirreg=0X3600;   
	LCD_WriteReg(dirreg,regval);
	
  if(regval&0X20)
  {
    if(lcddev.width<lcddev.height)                          //����X,Y
    {
      temp=lcddev.width;
      lcddev.width=lcddev.height;
      lcddev.height=temp;
 			}
  }
   else  
  {
    if(lcddev.width>lcddev.height)                          //����X,Y
    {
      temp=lcddev.width;
      lcddev.width=lcddev.height;
      lcddev.height=temp;
    }
  }  

  LCD_WR_REG(lcddev.setxcmd);  LCD_WR_DATA(0); 
  LCD_WR_REG(lcddev.setxcmd+1);LCD_WR_DATA(0); 
  LCD_WR_REG(lcddev.setxcmd+2);LCD_WR_DATA((lcddev.width-1)>>8); 
  LCD_WR_REG(lcddev.setxcmd+3);LCD_WR_DATA((lcddev.width-1)&0XFF); 
  LCD_WR_REG(lcddev.setycmd);  LCD_WR_DATA(0); 
  LCD_WR_REG(lcddev.setycmd+1);LCD_WR_DATA(0); 
  LCD_WR_REG(lcddev.setycmd+2);LCD_WR_DATA((lcddev.height-1)>>8); 
  LCD_WR_REG(lcddev.setycmd+3);LCD_WR_DATA((lcddev.height-1)&0XFF);

}

/***************************************************************************
�������ƣ�LCD_DrawPoint
�������ܣ�����
������ע��
***************************************************************************/
void LCD_DrawPoint(u16 x,u16 y)
{
	LCD_SetCursor(x,y);		//���ù��λ�� 
	LCD_WriteRAM_Prepare();	//��ʼд��GRAM
	LCD->LCD_RAM=POINT_COLOR; 
}

/***************************************************************************
�������ƣ�LCD_Fast_DrawPoint
�������ܣ����ٻ���
������ע��
***************************************************************************/
void LCD_Fast_DrawPoint(u16 x,u16 y,u16 color)
{	   
  LCD_WR_REG(lcddev.setxcmd);  LCD_WR_DATA(x>>8);  
  LCD_WR_REG(lcddev.setxcmd+1);LCD_WR_DATA(x&0XFF);	  
  LCD_WR_REG(lcddev.setycmd);  LCD_WR_DATA(y>>8);  
  LCD_WR_REG(lcddev.setycmd+1);LCD_WR_DATA(y&0XFF); 
		
  LCD->LCD_REG=lcddev.wramcmd; 
  LCD->LCD_RAM=color; 
}

/***************************************************************************
�������ƣ�LCD_Display_Dir
�������ܣ�����LCD��ʾ����
������ע��dir:0,������1,����
***************************************************************************/
void LCD_Display_Dir(u8 dir)
{
	if(dir==0)			                 //����
	{
		lcddev.dir=0;	                 //����

    lcddev.wramcmd=0X2C00;
    lcddev.setxcmd=0X2A00;
    lcddev.setycmd=0X2B00; 
    lcddev.width=480;
    lcddev.height=800;
	}
	else 				                   //����
	{	  				 
		lcddev.dir=1;	               //����

    lcddev.wramcmd=0X2C00;
    lcddev.setxcmd=0X2A00;
    lcddev.setycmd=0X2B00; 
    lcddev.width=800;
    lcddev.height=480;
	}

	LCD_Scan_Dir(DFT_SCAN_DIR);	      //Ĭ��ɨ�跽��
}	
 
/***************************************************************************
�������ƣ�LCD_Set_Window
�������ܣ����ô���,���Զ����û������굽�������Ͻ�(sx,sy).
������ע��
***************************************************************************/
void LCD_Set_Window(u16 sx,u16 sy,u16 width,u16 height)
{   
	width=sx+width-1;
	height=sy+height-1;
		
  LCD_WR_REG(lcddev.setxcmd);LCD_WR_DATA(sx>>8);  
  LCD_WR_REG(lcddev.setxcmd+1);LCD_WR_DATA(sx&0XFF);	  
  LCD_WR_REG(lcddev.setxcmd+2);LCD_WR_DATA(width>>8);   
  LCD_WR_REG(lcddev.setxcmd+3);LCD_WR_DATA(width&0XFF);   
  LCD_WR_REG(lcddev.setycmd);LCD_WR_DATA(sy>>8);   
  LCD_WR_REG(lcddev.setycmd+1);LCD_WR_DATA(sy&0XFF);  
  LCD_WR_REG(lcddev.setycmd+2);LCD_WR_DATA(height>>8);   
  LCD_WR_REG(lcddev.setycmd+3);LCD_WR_DATA(height&0XFF);  
}
/***************************************************************************
�������ƣ�LCD_Clear
�������ܣ���������
������ע��
***************************************************************************/
void LCD_Clear(void)
{
	u32 index=0;      
	u32 totalpoint=lcddev.width;
	totalpoint*=lcddev.height; 			             //�õ��ܵ���
  LCD_SetCursor(0x00,0x0000);	                 //���ù��λ�� 
	LCD_WriteRAM_Prepare();     		             //��ʼд��GRAM	 	  
	for(index=0;index<totalpoint;index++)
	{
		LCD->LCD_RAM=BACK_COLOR;	
	}
} 



/***************************************************************************
�������ƣ�LCD_Init
�������ܣ���ʼ��lcd
������ע���ó�ʼ���������Գ�ʼ������ILI93XXҺ��,�������������ǻ���ILI9320��!!!
         �������ͺŵ�����оƬ��û�в���! 
***************************************************************************/
void LCD_Init(void)
{ 	
	vu32 i=0;
	
  GPIO_InitTypeDef               GPIO_InitStructure;
	FSMC_NORSRAMInitTypeDef        FSMC_NORSRAMInitStructure;
  FSMC_NORSRAMTimingInitTypeDef  readWriteTiming; 
	FSMC_NORSRAMTimingInitTypeDef  writeTiming;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB|
	                       RCC_AHB1Periph_GPIOD|
												 RCC_AHB1Periph_GPIOE|
												 RCC_AHB1Periph_GPIOF|
												 RCC_AHB1Periph_GPIOG, ENABLE);        //ʹ��PD,PE,PF,PGʱ��  
  RCC_AHB3PeriphClockCmd(RCC_AHB3Periph_FSMC,ENABLE);          //ʹ��FSMCʱ��  
	
 
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_11;                 //PB15 �������,���Ʊ���
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;               //��ͨ���ģʽ
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;               //�������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;            //100MHz
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;                //����
  GPIO_Init(GPIOB, &GPIO_InitStructure);                       //��ʼ�� //PB15 �������,���Ʊ���
	
  GPIO_InitStructure.GPIO_Pin   = (3<<0)|(3<<4)|(7<<8)|(3<<14);//PD0,1,4,5,8,9,10,14,15 AF OUT
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;                //�������
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;               //�������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;           //100MHz
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;                //����
  GPIO_Init(GPIOD, &GPIO_InitStructure);                       //��ʼ��  
	
  GPIO_InitStructure.GPIO_Pin   = (0X1FF<<7);                  //PE7~15,AF OUT
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;                //�������
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;               //�������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;           //100MHz
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;                //����
  GPIO_Init(GPIOE, &GPIO_InitStructure);                       //��ʼ��  

	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_12;                 //PF12,FSMC_A6
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;                //�������
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;               //�������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;           //100MHz
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;                //����
  GPIO_Init(GPIOF, &GPIO_InitStructure);                       //��ʼ��  

	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_12;                 //PF12,FSMC_A6
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;                //�������
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;               //�������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;           //100MHz
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;                //����
  GPIO_Init(GPIOG, &GPIO_InitStructure);                       //��ʼ�� 

  GPIO_PinAFConfig(GPIOD,GPIO_PinSource0,GPIO_AF_FSMC);        //PD0,AF12
  GPIO_PinAFConfig(GPIOD,GPIO_PinSource1,GPIO_AF_FSMC);        //PD1,AF12
  GPIO_PinAFConfig(GPIOD,GPIO_PinSource4,GPIO_AF_FSMC);
  GPIO_PinAFConfig(GPIOD,GPIO_PinSource5,GPIO_AF_FSMC); 
  GPIO_PinAFConfig(GPIOD,GPIO_PinSource8,GPIO_AF_FSMC); 
  GPIO_PinAFConfig(GPIOD,GPIO_PinSource9,GPIO_AF_FSMC);
  GPIO_PinAFConfig(GPIOD,GPIO_PinSource10,GPIO_AF_FSMC);
  GPIO_PinAFConfig(GPIOD,GPIO_PinSource14,GPIO_AF_FSMC);
  GPIO_PinAFConfig(GPIOD,GPIO_PinSource15,GPIO_AF_FSMC);       //PD15,AF12
 
  GPIO_PinAFConfig(GPIOE,GPIO_PinSource7,GPIO_AF_FSMC);        //PE7,AF12
  GPIO_PinAFConfig(GPIOE,GPIO_PinSource8,GPIO_AF_FSMC);
  GPIO_PinAFConfig(GPIOE,GPIO_PinSource9,GPIO_AF_FSMC);
  GPIO_PinAFConfig(GPIOE,GPIO_PinSource10,GPIO_AF_FSMC);
  GPIO_PinAFConfig(GPIOE,GPIO_PinSource11,GPIO_AF_FSMC);
  GPIO_PinAFConfig(GPIOE,GPIO_PinSource12,GPIO_AF_FSMC);
  GPIO_PinAFConfig(GPIOE,GPIO_PinSource13,GPIO_AF_FSMC);
  GPIO_PinAFConfig(GPIOE,GPIO_PinSource14,GPIO_AF_FSMC);
  GPIO_PinAFConfig(GPIOE,GPIO_PinSource15,GPIO_AF_FSMC);       //PE15,AF12
 
  GPIO_PinAFConfig(GPIOF,GPIO_PinSource12,GPIO_AF_FSMC);       //PF12,AF12
  GPIO_PinAFConfig(GPIOG,GPIO_PinSource12,GPIO_AF_FSMC);


  readWriteTiming.FSMC_AddressSetupTime      = 0XF;	               //��ַ����ʱ�䣨ADDSET��Ϊ16��HCLK 1/168M=6ns*16=96ns	
  readWriteTiming.FSMC_AddressHoldTime       = 0x00;	             //��ַ����ʱ�䣨ADDHLD��ģʽAδ�õ�	
  readWriteTiming.FSMC_DataSetupTime         = 60;			           //���ݱ���ʱ��Ϊ60��HCLK	=6*60=360ns
  readWriteTiming.FSMC_BusTurnAroundDuration = 0x00;
  readWriteTiming.FSMC_CLKDivision           = 0x00;
  readWriteTiming.FSMC_DataLatency           = 0x00;
  readWriteTiming.FSMC_AccessMode            = FSMC_AccessMode_A;	 //ģʽA 
    

	writeTiming.FSMC_AddressSetupTime          = 9;                  //��ַ����ʱ�䣨ADDSET��Ϊ9��HCLK =54ns 
  writeTiming.FSMC_AddressHoldTime           = 0x00;               //��ַ����ʱ�䣨A		
  writeTiming.FSMC_DataSetupTime             = 8;                  //���ݱ���ʱ��Ϊ6ns*9��HCLK=54ns
  writeTiming.FSMC_BusTurnAroundDuration     = 0x00;
  writeTiming.FSMC_CLKDivision               = 0x00;
  writeTiming.FSMC_DataLatency               = 0x00;
  writeTiming.FSMC_AccessMode = FSMC_AccessMode_A;                 //ģʽA 

 
  FSMC_NORSRAMInitStructure.FSMC_Bank                  = FSMC_Bank1_NORSRAM4;                  //  ��������ʹ��NE4 ��Ҳ�Ͷ�ӦBTCR[6],[7]��
  FSMC_NORSRAMInitStructure.FSMC_DataAddressMux        = FSMC_DataAddressMux_Disable;          // ���������ݵ�ַ
  FSMC_NORSRAMInitStructure.FSMC_MemoryType            = FSMC_MemoryType_SRAM;                 // FSMC_MemoryType_SRAM;  //SRAM   
  FSMC_NORSRAMInitStructure.FSMC_MemoryDataWidth       = FSMC_MemoryDataWidth_16b;             //�洢�����ݿ��Ϊ16bit   
  FSMC_NORSRAMInitStructure.FSMC_BurstAccessMode       = FSMC_BurstAccessMode_Disable;         // FSMC_BurstAccessMode_Disable; 
  FSMC_NORSRAMInitStructure.FSMC_WaitSignalPolarity    = FSMC_WaitSignalPolarity_Low;
	FSMC_NORSRAMInitStructure.FSMC_AsynchronousWait      = FSMC_AsynchronousWait_Disable; 
  FSMC_NORSRAMInitStructure.FSMC_WrapMode              = FSMC_WrapMode_Disable;   
  FSMC_NORSRAMInitStructure.FSMC_WaitSignalActive      = FSMC_WaitSignalActive_BeforeWaitState;  
  FSMC_NORSRAMInitStructure.FSMC_WriteOperation        = FSMC_WriteOperation_Enable;	         //  �洢��дʹ��
  FSMC_NORSRAMInitStructure.FSMC_WaitSignal            = FSMC_WaitSignal_Disable;   
  FSMC_NORSRAMInitStructure.FSMC_ExtendedMode          = FSMC_ExtendedMode_Enable;             // ��дʹ�ò�ͬ��ʱ��
  FSMC_NORSRAMInitStructure.FSMC_WriteBurst            = FSMC_WriteBurst_Disable; 
  FSMC_NORSRAMInitStructure.FSMC_ReadWriteTimingStruct = &readWriteTiming;                     //��дʱ��
  FSMC_NORSRAMInitStructure.FSMC_WriteTimingStruct     = &writeTiming;                         //дʱ��

  FSMC_NORSRAMInit(&FSMC_NORSRAMInitStructure);                                                //��ʼ��FSMC����

  FSMC_NORSRAMCmd(FSMC_Bank1_NORSRAM4, ENABLE);                                                // ʹ��BANK1 
		
 	Delay_ms(50); // delay 50 ms 
 	LCD_WriteReg(0x0000,0x0001);
	Delay_ms(50); // delay 50 ms 
 	lcddev.id = LCD_ReadReg(0x0000); 
 	if(lcddev.id<0XFF||lcddev.id==0XFFFF)//����ID����ȷ
	{	

					LCD_WR_REG(0XDA00);	
					lcddev.id=LCD_RD_DATA();//����0X00	 
					LCD_WR_REG(0XDB00);	
					lcddev.id=LCD_RD_DATA();//����0X80
					lcddev.id<<=8;	
					LCD_WR_REG(0XDC00);	
					lcddev.id|=LCD_RD_DATA();//����0X00		
					if(lcddev.id==0x8000)lcddev.id=0x5510;//NT35510���ص�ID��8000H,Ϊ��������,����ǿ������Ϊ5510
	}   
	if(lcddev.id==0X5510)           //�����������IC,������WRʱ��Ϊ���
	{
		//��������дʱ����ƼĴ�����ʱ��   	 							    
		FSMC_Bank1E->BWTR[6]&=~(0XF<<0);//��ַ����ʱ�䣨ADDSET������ 	 
		FSMC_Bank1E->BWTR[6]&=~(0XF<<8);//���ݱ���ʱ������
		FSMC_Bank1E->BWTR[6]|=3<<0;		//��ַ����ʱ�䣨ADDSET��Ϊ3��HCLK =18ns  	 
		FSMC_Bank1E->BWTR[6]|=2<<8; 	//���ݱ���ʱ��Ϊ6ns*3��HCLK=18ns
	}
  if(lcddev.id==0x5510)
	{
		LCD_WriteReg(0xF000,0x55);
		LCD_WriteReg(0xF001,0xAA);
		LCD_WriteReg(0xF002,0x52);
		LCD_WriteReg(0xF003,0x08);
		LCD_WriteReg(0xF004,0x01);
		//AVDD Set AVDD 5.2V
		LCD_WriteReg(0xB000,0x0D);
		LCD_WriteReg(0xB001,0x0D);
		LCD_WriteReg(0xB002,0x0D);
		//AVDD ratio
		LCD_WriteReg(0xB600,0x34);
		LCD_WriteReg(0xB601,0x34);
		LCD_WriteReg(0xB602,0x34);
		//AVEE -5.2V
		LCD_WriteReg(0xB100,0x0D);
		LCD_WriteReg(0xB101,0x0D);
		LCD_WriteReg(0xB102,0x0D);
		//AVEE ratio
		LCD_WriteReg(0xB700,0x34);
		LCD_WriteReg(0xB701,0x34);
		LCD_WriteReg(0xB702,0x34);
		//VCL -2.5V
		LCD_WriteReg(0xB200,0x00);
		LCD_WriteReg(0xB201,0x00);
		LCD_WriteReg(0xB202,0x00);
		//VCL ratio
		LCD_WriteReg(0xB800,0x24);
		LCD_WriteReg(0xB801,0x24);
		LCD_WriteReg(0xB802,0x24);
		//VGH 15V (Free pump)
		LCD_WriteReg(0xBF00,0x01);
		LCD_WriteReg(0xB300,0x0F);
		LCD_WriteReg(0xB301,0x0F);
		LCD_WriteReg(0xB302,0x0F);
		//VGH ratio
		LCD_WriteReg(0xB900,0x34);
		LCD_WriteReg(0xB901,0x34);
		LCD_WriteReg(0xB902,0x34);
		//VGL_REG -10V
		LCD_WriteReg(0xB500,0x08);
		LCD_WriteReg(0xB501,0x08);
		LCD_WriteReg(0xB502,0x08);
		LCD_WriteReg(0xC200,0x03);
		//VGLX ratio
		LCD_WriteReg(0xBA00,0x24);
		LCD_WriteReg(0xBA01,0x24);
		LCD_WriteReg(0xBA02,0x24);
		//VGMP/VGSP 4.5V/0V
		LCD_WriteReg(0xBC00,0x00);
		LCD_WriteReg(0xBC01,0x78);
		LCD_WriteReg(0xBC02,0x00);
		//VGMN/VGSN -4.5V/0V
		LCD_WriteReg(0xBD00,0x00);
		LCD_WriteReg(0xBD01,0x78);
		LCD_WriteReg(0xBD02,0x00);
		//VCOM
		LCD_WriteReg(0xBE00,0x00);
		LCD_WriteReg(0xBE01,0x64);
		//Gamma Setting
		LCD_WriteReg(0xD100,0x00);
		LCD_WriteReg(0xD101,0x33);
		LCD_WriteReg(0xD102,0x00);
		LCD_WriteReg(0xD103,0x34);
		LCD_WriteReg(0xD104,0x00);
		LCD_WriteReg(0xD105,0x3A);
		LCD_WriteReg(0xD106,0x00);
		LCD_WriteReg(0xD107,0x4A);
		LCD_WriteReg(0xD108,0x00);
		LCD_WriteReg(0xD109,0x5C);
		LCD_WriteReg(0xD10A,0x00);
		LCD_WriteReg(0xD10B,0x81);
		LCD_WriteReg(0xD10C,0x00);
		LCD_WriteReg(0xD10D,0xA6);
		LCD_WriteReg(0xD10E,0x00);
		LCD_WriteReg(0xD10F,0xE5);
		LCD_WriteReg(0xD110,0x01);
		LCD_WriteReg(0xD111,0x13);
		LCD_WriteReg(0xD112,0x01);
		LCD_WriteReg(0xD113,0x54);
		LCD_WriteReg(0xD114,0x01);
		LCD_WriteReg(0xD115,0x82);
		LCD_WriteReg(0xD116,0x01);
		LCD_WriteReg(0xD117,0xCA);
		LCD_WriteReg(0xD118,0x02);
		LCD_WriteReg(0xD119,0x00);
		LCD_WriteReg(0xD11A,0x02);
		LCD_WriteReg(0xD11B,0x01);
		LCD_WriteReg(0xD11C,0x02);
		LCD_WriteReg(0xD11D,0x34);
		LCD_WriteReg(0xD11E,0x02);
		LCD_WriteReg(0xD11F,0x67);
		LCD_WriteReg(0xD120,0x02);
		LCD_WriteReg(0xD121,0x84);
		LCD_WriteReg(0xD122,0x02);
		LCD_WriteReg(0xD123,0xA4);
		LCD_WriteReg(0xD124,0x02);
		LCD_WriteReg(0xD125,0xB7);
		LCD_WriteReg(0xD126,0x02);
		LCD_WriteReg(0xD127,0xCF);
		LCD_WriteReg(0xD128,0x02);
		LCD_WriteReg(0xD129,0xDE);
		LCD_WriteReg(0xD12A,0x02);
		LCD_WriteReg(0xD12B,0xF2);
		LCD_WriteReg(0xD12C,0x02);
		LCD_WriteReg(0xD12D,0xFE);
		LCD_WriteReg(0xD12E,0x03);
		LCD_WriteReg(0xD12F,0x10);
		LCD_WriteReg(0xD130,0x03);
		LCD_WriteReg(0xD131,0x33);
		LCD_WriteReg(0xD132,0x03);
		LCD_WriteReg(0xD133,0x6D);
		LCD_WriteReg(0xD200,0x00);
		LCD_WriteReg(0xD201,0x33);
		LCD_WriteReg(0xD202,0x00);
		LCD_WriteReg(0xD203,0x34);
		LCD_WriteReg(0xD204,0x00);
		LCD_WriteReg(0xD205,0x3A);
		LCD_WriteReg(0xD206,0x00);
		LCD_WriteReg(0xD207,0x4A);
		LCD_WriteReg(0xD208,0x00);
		LCD_WriteReg(0xD209,0x5C);
		LCD_WriteReg(0xD20A,0x00);

		LCD_WriteReg(0xD20B,0x81);
		LCD_WriteReg(0xD20C,0x00);
		LCD_WriteReg(0xD20D,0xA6);
		LCD_WriteReg(0xD20E,0x00);
		LCD_WriteReg(0xD20F,0xE5);
		LCD_WriteReg(0xD210,0x01);
		LCD_WriteReg(0xD211,0x13);
		LCD_WriteReg(0xD212,0x01);
		LCD_WriteReg(0xD213,0x54);
		LCD_WriteReg(0xD214,0x01);
		LCD_WriteReg(0xD215,0x82);
		LCD_WriteReg(0xD216,0x01);
		LCD_WriteReg(0xD217,0xCA);
		LCD_WriteReg(0xD218,0x02);
		LCD_WriteReg(0xD219,0x00);
		LCD_WriteReg(0xD21A,0x02);
		LCD_WriteReg(0xD21B,0x01);
		LCD_WriteReg(0xD21C,0x02);
		LCD_WriteReg(0xD21D,0x34);
		LCD_WriteReg(0xD21E,0x02);
		LCD_WriteReg(0xD21F,0x67);
		LCD_WriteReg(0xD220,0x02);
		LCD_WriteReg(0xD221,0x84);
		LCD_WriteReg(0xD222,0x02);
		LCD_WriteReg(0xD223,0xA4);
		LCD_WriteReg(0xD224,0x02);
		LCD_WriteReg(0xD225,0xB7);
		LCD_WriteReg(0xD226,0x02);
		LCD_WriteReg(0xD227,0xCF);
		LCD_WriteReg(0xD228,0x02);
		LCD_WriteReg(0xD229,0xDE);
		LCD_WriteReg(0xD22A,0x02);
		LCD_WriteReg(0xD22B,0xF2);
		LCD_WriteReg(0xD22C,0x02);
		LCD_WriteReg(0xD22D,0xFE);
		LCD_WriteReg(0xD22E,0x03);
		LCD_WriteReg(0xD22F,0x10);
		LCD_WriteReg(0xD230,0x03);
		LCD_WriteReg(0xD231,0x33);
		LCD_WriteReg(0xD232,0x03);
		LCD_WriteReg(0xD233,0x6D);
		LCD_WriteReg(0xD300,0x00);
		LCD_WriteReg(0xD301,0x33);
		LCD_WriteReg(0xD302,0x00);
		LCD_WriteReg(0xD303,0x34);
		LCD_WriteReg(0xD304,0x00);
		LCD_WriteReg(0xD305,0x3A);
		LCD_WriteReg(0xD306,0x00);
		LCD_WriteReg(0xD307,0x4A);
		LCD_WriteReg(0xD308,0x00);
		LCD_WriteReg(0xD309,0x5C);
		LCD_WriteReg(0xD30A,0x00);

		LCD_WriteReg(0xD30B,0x81);
		LCD_WriteReg(0xD30C,0x00);
		LCD_WriteReg(0xD30D,0xA6);
		LCD_WriteReg(0xD30E,0x00);
		LCD_WriteReg(0xD30F,0xE5);
		LCD_WriteReg(0xD310,0x01);
		LCD_WriteReg(0xD311,0x13);
		LCD_WriteReg(0xD312,0x01);
		LCD_WriteReg(0xD313,0x54);
		LCD_WriteReg(0xD314,0x01);
		LCD_WriteReg(0xD315,0x82);
		LCD_WriteReg(0xD316,0x01);
		LCD_WriteReg(0xD317,0xCA);
		LCD_WriteReg(0xD318,0x02);
		LCD_WriteReg(0xD319,0x00);
		LCD_WriteReg(0xD31A,0x02);
		LCD_WriteReg(0xD31B,0x01);
		LCD_WriteReg(0xD31C,0x02);
		LCD_WriteReg(0xD31D,0x34);
		LCD_WriteReg(0xD31E,0x02);
		LCD_WriteReg(0xD31F,0x67);
		LCD_WriteReg(0xD320,0x02);
		LCD_WriteReg(0xD321,0x84);
		LCD_WriteReg(0xD322,0x02);
		LCD_WriteReg(0xD323,0xA4);
		LCD_WriteReg(0xD324,0x02);
		LCD_WriteReg(0xD325,0xB7);
		LCD_WriteReg(0xD326,0x02);
		LCD_WriteReg(0xD327,0xCF);
		LCD_WriteReg(0xD328,0x02);
		LCD_WriteReg(0xD329,0xDE);
		LCD_WriteReg(0xD32A,0x02);
		LCD_WriteReg(0xD32B,0xF2);
		LCD_WriteReg(0xD32C,0x02);
		LCD_WriteReg(0xD32D,0xFE);
		LCD_WriteReg(0xD32E,0x03);
		LCD_WriteReg(0xD32F,0x10);
		LCD_WriteReg(0xD330,0x03);
		LCD_WriteReg(0xD331,0x33);
		LCD_WriteReg(0xD332,0x03);
		LCD_WriteReg(0xD333,0x6D);
		LCD_WriteReg(0xD400,0x00);
		LCD_WriteReg(0xD401,0x33);
		LCD_WriteReg(0xD402,0x00);
		LCD_WriteReg(0xD403,0x34);
		LCD_WriteReg(0xD404,0x00);
		LCD_WriteReg(0xD405,0x3A);
		LCD_WriteReg(0xD406,0x00);
		LCD_WriteReg(0xD407,0x4A);
		LCD_WriteReg(0xD408,0x00);
		LCD_WriteReg(0xD409,0x5C);
		LCD_WriteReg(0xD40A,0x00);
		LCD_WriteReg(0xD40B,0x81);

		LCD_WriteReg(0xD40C,0x00);
		LCD_WriteReg(0xD40D,0xA6);
		LCD_WriteReg(0xD40E,0x00);
		LCD_WriteReg(0xD40F,0xE5);
		LCD_WriteReg(0xD410,0x01);
		LCD_WriteReg(0xD411,0x13);
		LCD_WriteReg(0xD412,0x01);
		LCD_WriteReg(0xD413,0x54);
		LCD_WriteReg(0xD414,0x01);
		LCD_WriteReg(0xD415,0x82);
		LCD_WriteReg(0xD416,0x01);
		LCD_WriteReg(0xD417,0xCA);
		LCD_WriteReg(0xD418,0x02);
		LCD_WriteReg(0xD419,0x00);
		LCD_WriteReg(0xD41A,0x02);
		LCD_WriteReg(0xD41B,0x01);
		LCD_WriteReg(0xD41C,0x02);
		LCD_WriteReg(0xD41D,0x34);
		LCD_WriteReg(0xD41E,0x02);
		LCD_WriteReg(0xD41F,0x67);
		LCD_WriteReg(0xD420,0x02);
		LCD_WriteReg(0xD421,0x84);
		LCD_WriteReg(0xD422,0x02);
		LCD_WriteReg(0xD423,0xA4);
		LCD_WriteReg(0xD424,0x02);
		LCD_WriteReg(0xD425,0xB7);
		LCD_WriteReg(0xD426,0x02);
		LCD_WriteReg(0xD427,0xCF);
		LCD_WriteReg(0xD428,0x02);
		LCD_WriteReg(0xD429,0xDE);
		LCD_WriteReg(0xD42A,0x02);
		LCD_WriteReg(0xD42B,0xF2);
		LCD_WriteReg(0xD42C,0x02);
		LCD_WriteReg(0xD42D,0xFE);
		LCD_WriteReg(0xD42E,0x03);
		LCD_WriteReg(0xD42F,0x10);
		LCD_WriteReg(0xD430,0x03);
		LCD_WriteReg(0xD431,0x33);
		LCD_WriteReg(0xD432,0x03);
		LCD_WriteReg(0xD433,0x6D);
		LCD_WriteReg(0xD500,0x00);
		LCD_WriteReg(0xD501,0x33);
		LCD_WriteReg(0xD502,0x00);
		LCD_WriteReg(0xD503,0x34);
		LCD_WriteReg(0xD504,0x00);
		LCD_WriteReg(0xD505,0x3A);
		LCD_WriteReg(0xD506,0x00);
		LCD_WriteReg(0xD507,0x4A);
		LCD_WriteReg(0xD508,0x00);
		LCD_WriteReg(0xD509,0x5C);
		LCD_WriteReg(0xD50A,0x00);
		LCD_WriteReg(0xD50B,0x81);

		LCD_WriteReg(0xD50C,0x00);
		LCD_WriteReg(0xD50D,0xA6);
		LCD_WriteReg(0xD50E,0x00);
		LCD_WriteReg(0xD50F,0xE5);
		LCD_WriteReg(0xD510,0x01);
		LCD_WriteReg(0xD511,0x13);
		LCD_WriteReg(0xD512,0x01);
		LCD_WriteReg(0xD513,0x54);
		LCD_WriteReg(0xD514,0x01);
		LCD_WriteReg(0xD515,0x82);
		LCD_WriteReg(0xD516,0x01);
		LCD_WriteReg(0xD517,0xCA);
		LCD_WriteReg(0xD518,0x02);
		LCD_WriteReg(0xD519,0x00);
		LCD_WriteReg(0xD51A,0x02);
		LCD_WriteReg(0xD51B,0x01);
		LCD_WriteReg(0xD51C,0x02);
		LCD_WriteReg(0xD51D,0x34);
		LCD_WriteReg(0xD51E,0x02);
		LCD_WriteReg(0xD51F,0x67);
		LCD_WriteReg(0xD520,0x02);
		LCD_WriteReg(0xD521,0x84);
		LCD_WriteReg(0xD522,0x02);
		LCD_WriteReg(0xD523,0xA4);
		LCD_WriteReg(0xD524,0x02);
		LCD_WriteReg(0xD525,0xB7);
		LCD_WriteReg(0xD526,0x02);
		LCD_WriteReg(0xD527,0xCF);
		LCD_WriteReg(0xD528,0x02);
		LCD_WriteReg(0xD529,0xDE);
		LCD_WriteReg(0xD52A,0x02);
		LCD_WriteReg(0xD52B,0xF2);
		LCD_WriteReg(0xD52C,0x02);
		LCD_WriteReg(0xD52D,0xFE);
		LCD_WriteReg(0xD52E,0x03);
		LCD_WriteReg(0xD52F,0x10);
		LCD_WriteReg(0xD530,0x03);
		LCD_WriteReg(0xD531,0x33);
		LCD_WriteReg(0xD532,0x03);
		LCD_WriteReg(0xD533,0x6D);
		LCD_WriteReg(0xD600,0x00);
		LCD_WriteReg(0xD601,0x33);
		LCD_WriteReg(0xD602,0x00);
		LCD_WriteReg(0xD603,0x34);
		LCD_WriteReg(0xD604,0x00);
		LCD_WriteReg(0xD605,0x3A);
		LCD_WriteReg(0xD606,0x00);
		LCD_WriteReg(0xD607,0x4A);
		LCD_WriteReg(0xD608,0x00);
		LCD_WriteReg(0xD609,0x5C);
		LCD_WriteReg(0xD60A,0x00);
		LCD_WriteReg(0xD60B,0x81);

		LCD_WriteReg(0xD60C,0x00);
		LCD_WriteReg(0xD60D,0xA6);
		LCD_WriteReg(0xD60E,0x00);
		LCD_WriteReg(0xD60F,0xE5);
		LCD_WriteReg(0xD610,0x01);
		LCD_WriteReg(0xD611,0x13);
		LCD_WriteReg(0xD612,0x01);
		LCD_WriteReg(0xD613,0x54);
		LCD_WriteReg(0xD614,0x01);
		LCD_WriteReg(0xD615,0x82);
		LCD_WriteReg(0xD616,0x01);
		LCD_WriteReg(0xD617,0xCA);
		LCD_WriteReg(0xD618,0x02);
		LCD_WriteReg(0xD619,0x00);
		LCD_WriteReg(0xD61A,0x02);
		LCD_WriteReg(0xD61B,0x01);
		LCD_WriteReg(0xD61C,0x02);
		LCD_WriteReg(0xD61D,0x34);
		LCD_WriteReg(0xD61E,0x02);
		LCD_WriteReg(0xD61F,0x67);
		LCD_WriteReg(0xD620,0x02);
		LCD_WriteReg(0xD621,0x84);
		LCD_WriteReg(0xD622,0x02);
		LCD_WriteReg(0xD623,0xA4);
		LCD_WriteReg(0xD624,0x02);
		LCD_WriteReg(0xD625,0xB7);
		LCD_WriteReg(0xD626,0x02);
		LCD_WriteReg(0xD627,0xCF);
		LCD_WriteReg(0xD628,0x02);
		LCD_WriteReg(0xD629,0xDE);
		LCD_WriteReg(0xD62A,0x02);
		LCD_WriteReg(0xD62B,0xF2);
		LCD_WriteReg(0xD62C,0x02);
		LCD_WriteReg(0xD62D,0xFE);
		LCD_WriteReg(0xD62E,0x03);
		LCD_WriteReg(0xD62F,0x10);
		LCD_WriteReg(0xD630,0x03);
		LCD_WriteReg(0xD631,0x33);
		LCD_WriteReg(0xD632,0x03);
		LCD_WriteReg(0xD633,0x6D);
		//LV2 Page 0 enable
		LCD_WriteReg(0xF000,0x55);
		LCD_WriteReg(0xF001,0xAA);
		LCD_WriteReg(0xF002,0x52);
		LCD_WriteReg(0xF003,0x08);
		LCD_WriteReg(0xF004,0x00);
		//Display control
		LCD_WriteReg(0xB100, 0xCC);
		LCD_WriteReg(0xB101, 0x00);
		//Source hold time
		LCD_WriteReg(0xB600,0x05);
		//Gate EQ control
		LCD_WriteReg(0xB700,0x70);
		LCD_WriteReg(0xB701,0x70);
		//Source EQ control (Mode 2)
		LCD_WriteReg(0xB800,0x01);
		LCD_WriteReg(0xB801,0x03);
		LCD_WriteReg(0xB802,0x03);
		LCD_WriteReg(0xB803,0x03);
		//Inversion mode (2-dot)
		LCD_WriteReg(0xBC00,0x02);
		LCD_WriteReg(0xBC01,0x00);
		LCD_WriteReg(0xBC02,0x00);
		//Timing control 4H w/ 4-delay
		LCD_WriteReg(0xC900,0xD0);
		LCD_WriteReg(0xC901,0x02);
		LCD_WriteReg(0xC902,0x50);
		LCD_WriteReg(0xC903,0x50);
		LCD_WriteReg(0xC904,0x50);
		LCD_WriteReg(0x3500,0x00);
		LCD_WriteReg(0x3A00,0x55);  //16-bit/pixel
		LCD_WR_REG(0x1100);
		Delay_us(120);
		LCD_WR_REG(0x2900);
	}	 

	LCD_Display_Dir(0);		 	//Ĭ��Ϊ����
	LCD_Clear();
	LCD_LED=1;					    //��������
} 


#endif

