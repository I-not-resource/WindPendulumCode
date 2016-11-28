#include "LCD_OWN.h"
#include "lcd_drive.h"
#include "Fonts.h"
#if __SPECIALLY_ENABLE
////////////////////////////////////////////////////////////////////////////
//���棡����
//��Ӧ�ó���רΪ5510�Ż����
//��˲��߱����õļ�����
////////////////////////////////////////////////////////////////////////////

/***************************************************************************
�������ƣ�LCD_Fill
�������ܣ���ָ����������䵥����ɫ
������ע��
***************************************************************************/
void LCD_Fill(u16 sx,u16 sy,u16 ex,u16 ey,u16 color)
{          
	u16 i,j;
	u16 xlen=0;
	xlen=ex-sx+1;		
	for(i=sy;i<=ey;i++)
	{
	 	LCD_SetCursor(sx,i);      				      //���ù��λ�� 
		LCD_WriteRAM_Prepare();     			      //��ʼд��GRAM	  
		for(j=0;j<xlen;j++)
		LCD->LCD_RAM = color;                   //��ʾ��ɫ 	    
	}	 
}

/***************************************************************************
�������ƣ�LCD_Color_Fill
�������ܣ���ָ�����������ָ����ɫ��
������ע��color:Ҫ������ɫ,
***************************************************************************/
void LCD_Color_Fill(u16 sx,u16 sy,u16 ex,u16 ey,u16 *color)
{  
	u16 height,width;
	u16 i,j;
	width=ex-sx+1; 			                                 //�õ����Ŀ��
	height=ey-sy+1;			                                 //�߶�
 	for(i=0;i<height;i++)
	{
 		LCD_SetCursor(sx,sy+i);                            //���ù��λ�� 
		LCD_WriteRAM_Prepare();                            //��ʼд��GRAM
		for(j=0;j<width;j++)
		LCD->LCD_RAM=color[i*width+j];                     //д������ 
	}		  
} 

/***************************************************************************
�������ƣ�LCD_DrawLine
�������ܣ�����
������ע��x1,y1:�������
         x2,y2:�յ�����  
***************************************************************************/
void LCD_DrawLine(u16 x1, u16 y1, u16 x2, u16 y2)
{
	u16 t; 
	int xerr=0,yerr=0,delta_x,delta_y,distance; 
	int incx,incy,uRow,uCol; 
	delta_x=x2-x1;                           //������������ 
	delta_y=y2-y1; 
	uRow=x1; 
	uCol=y1; 
	if(delta_x>0)incx=1;                     //���õ������� 
	else if(delta_x==0)incx=0;               //��ֱ�� 
	else {incx=-1;delta_x=-delta_x;} 
	if(delta_y>0)incy=1; 
	else if(delta_y==0)
		incy=0;                                //ˮƽ�� 
	else{incy=-1;delta_y=-delta_y;} 
	if( delta_x>delta_y)
		distance=delta_x;                      //ѡȡ�������������� 
	else distance=delta_y; 
	for(t=0;t<=distance+1;t++ )              //������� 
	{  
		LCD_DrawPoint(uRow,uCol);              //���� 
		xerr+=delta_x ; 
		yerr+=delta_y ; 
		if(xerr>distance) 
		{ 
			xerr-=distance; 
			uRow+=incx; 
		} 
		if(yerr>distance) 
		{ 
			yerr-=distance; 
			uCol+=incy; 
		} 
	}  
}    

/***************************************************************************
�������ƣ�LCD_DrawRectangle
�������ܣ�������	
������ע��x1,y1:�������
         x2,y2:�յ�����  
***************************************************************************/
void LCD_DrawRectangle(u16 x1, u16 y1, u16 x2, u16 y2)
{
	LCD_DrawLine(x1,y1,x2,y1);
	LCD_DrawLine(x1,y1,x1,y2);
	LCD_DrawLine(x1,y2,x2,y2);
	LCD_DrawLine(x2,y1,x2,y2);
}


/***************************************************************************
�������ƣ�LCD_Draw_Circle
�������ܣ���ָ��λ�û�һ��ָ����С��Բ
������ע��(x,y):���ĵ�
         r    :�뾶 
***************************************************************************/
void LCD_Draw_Circle(u16 x0,u16 y0,u8 r)
{
	int a,b;
	int di;
	a=0;b=r;	  
	di=3-(r<<1);                            //�ж��¸���λ�õı�־
	while(a<=b)
	{
		LCD_DrawPoint(x0+a,y0-b);             //5
 		LCD_DrawPoint(x0+b,y0-a);             //0           
		LCD_DrawPoint(x0+b,y0+a);             //4               
		LCD_DrawPoint(x0+a,y0+b);             //6 
		LCD_DrawPoint(x0-a,y0+b);             //1       
 		LCD_DrawPoint(x0-b,y0+a);             
		LCD_DrawPoint(x0-a,y0-b);             //2             
  	LCD_DrawPoint(x0-b,y0-a);             //7     	         
		a++;
		//ʹ��Bresenham�㷨��Բ     
		if(di<0)di +=4*a+6;	  
		else
		{
			di+=10+4*(a-b);   
			b--;
		} 						    
	}
} 									  

/***************************************************************************
�������ƣ�LCD_ShowChar
�������ܣ���ָ��λ����ʾһ���ַ�
������ע��x,y:��ʼ����
         num:Ҫ��ʾ���ַ�:" "--->"~"
				 size:�����С 12/16/24
				 mode:���ӷ�ʽ(1)���Ƿǵ��ӷ�ʽ(0)
***************************************************************************/
void LCD_ShowChar(u16 x,u16 y,u8 num,u8 size,u8 mode)
{  							  
    u8 temp,t1,t;
	u16 y0=y;
	u8 csize=(size/8+((size%8)?1:0))*(size/2);		//�õ�����һ���ַ���Ӧ������ռ���ֽ���	
	//���ô���		   
	num=num-' ';                                  //�õ�ƫ�ƺ��ֵ
	for(t=0;t<csize;t++)
	{   
		if(size==12)      temp=asc2_1206[num][t];   //����1206����
		else if(size==16) temp=asc2_1608[num][t];	  //����1608����
		else if(size==24) temp=asc2_2412[num][t];	  //����2412����
		else return;								                //û�е��ֿ�
		for(t1=0;t1<8;t1++)
		{			    
			if(temp&0x80)   LCD_Fast_DrawPoint(x,y,POINT_COLOR);
			else if(mode==0)LCD_Fast_DrawPoint(x,y,BACK_COLOR);
			temp<<=1;
			y++;
			if(y>=lcddev.height)return;		//��������
			if((y-y0)==size)
			{
				y=y0;
				x++;
				if(x>=lcddev.width)return;	//��������
				break;
			}
		}  	 
	}  	    	   	 	  
}   

/***************************************************************************
�������ƣ�LCD_Pow
�������ܣ�m^n����
������ע��
***************************************************************************/
u32 LCD_Pow(u8 m,u8 n)
{
	u32 result=1;	 
	while(n--)result*=m;    
	return result;
}			 

/***************************************************************************
�������ƣ�LCD_ShowNum
�������ܣ���ʾ����,��λΪ0,����ʾ
������ע��x,y :�������	
         xlen :���ֵ�λ�� 
				 size:�����С
				 color:��ɫ 
				 num:��ֵ(0~4294967295);	
***************************************************************************/
void LCD_ShowNum(u16 x,u16 y,u32 num,u8 len,u8 size)
{         	
	u8 t,temp;
	u8 enshow=0;						   
	for(t=0;t<len;t++)
	{
		temp=(num/LCD_Pow(10,len-t-1))%10;
		if(enshow==0&&t<(len-1))
		{
			if(temp==0)
			{
				LCD_ShowChar(x+(size/2)*t,y,' ',size,0);
				continue;
			}else enshow=1; 
		 	 
		}
	 	LCD_ShowChar(x+(size/2)*t,y,temp+'0',size,0); 
	}
} 

/***************************************************************************
�������ƣ�LCD_ShowxNum
�������ܣ���ʾ����,��λΪ0,������ʾ
������ע��x,y:�������
         num:��ֵ(0~999999999); 
				 len:����(��Ҫ��ʾ��λ��)
				 size:�����С
				 mode:[7]:0,�����;1,���0.
				      [6:1]:����
							[0]:0,�ǵ�����ʾ;1,������ʾ
***************************************************************************/
void LCD_ShowxNum(u16 x,u16 y,u32 num,u8 len,u8 size,u8 mode)
{  
	u8 t,temp;
	u8 enshow=0;						   
	for(t=0;t<len;t++)
	{
		temp=(num/LCD_Pow(10,len-t-1))%10;
		if(enshow==0&&t<(len-1))
		{
			if(temp==0)
			{
				if(mode&0X80)LCD_ShowChar(x+(size/2)*t,y,'0',size,mode&0X01);  
				else LCD_ShowChar(x+(size/2)*t,y,' ',size,mode&0X01);  
 				continue;
			}else enshow=1; 
		 	 
		}
	 	LCD_ShowChar(x+(size/2)*t,y,temp+'0',size,mode&0X01); 
	}
}


/***************************************************************************
�������ƣ�LCD_ShowWord
�������ܣ�ֱ���ڶ�����ʾ�֣�����һ���ԣ��������룬ֱ����Ļ�޷������ַ�
������ע��sizeΪ�����С
***************************************************************************/
void LCD_ShowWord(u16 x,u16 y,u8 size,u8 *font,u8 mode)
{

	u16 i = 0,c;                   //i:��ʾ��ǰ��i���ַ���c:��ȡ��ǰ���ַ���ASCIIֵ���൱�ڶ�һ���ַ������ַ����λ�ӣ�
	u16 y0 = y,x0 = x,line = 0;    //x0,y0:��¼��ʵ���ꣻline����¼����ڼ���
	while(font[i] != '\0')
	{
		c = font[i];
		y = y0;
		x = i * size / 2 + x0 - lcddev.width * line;
		if(x > (lcddev.width - size / 2))
		{
			x   = 0;
			y  += size;
			y0 += size;
			line++;
		}
		if(y >= lcddev.height)return;
		 LCD_ShowChar(x,y,c,size,mode);
		i++;
	}

}



/***************************************************************************
�������ƣ�LCD_Location_Med
�������ܣ�ʹ�ַ������м���ʾ��yΪ��0��3/0-7��font[]Ϊ�ַ�����
������ע��
***************************************************************************/
void LCD_Location_Med(u16 y,u8 size,u8 *font)
{
	
	u16 i=0,j,n,k=0,num=0,c,x,m=0,a,b,d,cnum;		   //iΪ���ַ�����jΪ����д�룬numĿǰ��д���ַ�������cΪ�ַ���Ӧ��ʮ�����ƣ�xΪ�����ַ�����λ�ã�mΪǰ����д���ַ�����
	while(font[i]!='\0')
	{
		i++;
	}
	cnum = lcddev.width / size * 2;
	k    = i / cnum + ((i % cnum) && 1);					//����	
	if(i % k)
		a  = k - i % k;                                 //�ٵ��ַ���
	else
		a = 0;
	b    = i/k+(a&&1);                            //ÿһ���ַ���
	n    = (lcddev.width - b * size / 2) / 2;	    //����λ��	
	for(j=0;j<k;j++)
	{
		x  = n + ((a - a % 2) * size / 4) * ((j + 1) / k);      //ʹ���һ������a�� ���ַ� ��λ��
		d  = b - a * ((j + 1) / k);                             //���һ���ַ��� 
		num=0;
		while (num<d)
		{    
			c =font[(num+m)];
	    LCD_ShowChar(x,y,c,size,0);  
      x += size / 2;
			num++;
		}
		y += size;
		m += b;
	}
	
}

/***************************************************************************
�������ƣ�LCD_ShowFonts
�������ܣ���ָ��λ����ʾһ���ַ�
������ע��x,y:��ʼ����
         font[][72]���ַ���������
				 size:�ڼ����ַ�
				 mode:���ӷ�ʽ(1)���Ƿǵ��ӷ�ʽ(0)
***************************************************************************/
void LCD_ShowFonts(u16 x,u16 y,u8 font[][72],u8 num,u8 mode)
{  							  
  u8 temp,t1,t;
	u16 y0=y;
	for(t=0;t<72;t++)
	{   
    temp=font[num][t]; 
		for(t1=0;t1<8;t1++)
		{			
			if(temp&0x80)
				LCD_Fast_DrawPoint(x,y,POINT_COLOR);
			else if(mode==0)
				LCD_Fast_DrawPoint(x,y,BACK_COLOR);
			temp<<=1;
			y++;
			if((y-y0)==24)
			{
				y=y0;
				x++;
				break;
			}
		}			
	}  	    	   	 	  
} 

/***************************************************************************
�������ƣ�LCD_Fonts_Med
�������ܣ�ʹ�ַ������м���ʾ��yΪ�У�font[][72]Ϊ�ַ����룬������ʾ����
������ע��iΪ���ַ�����jΪ����д�룬numĿǰ��д���ַ�������xΪ�����ַ�����λ�ã�mΪǰ����д���ַ�����
***************************************************************************/
void LCD_Fonts_Med(u16 y,u8 size,u8 font[][72])
{
	
	u16 i=0,j,n,k=0,num=0,x,m=0,a,b,d,cnum;
  i = size;
	cnum = lcddev.width / 24;                              //����һ�п����ɵ�����
	k    = i / cnum + ((i % cnum) && 1);                   //����	
	if(i % k)
		a  = k - i % k;                                      //�ٵ��ַ���
	else
		a = 0;
	b    = i/k+(a&&1);                                     //ÿһ���ַ���
	n    = (lcddev.width - b * 24) / 2;                    //����λ��	
	for(j=0;j<k;j++)
	{
		x  = n + ((a - a % 2) * 24 / 2) * ((j + 1) / k);     //ʹ���һ������a�� ���ַ� ��λ��0
		d  = b - a * ((j + 1) / k);                          //���һ���ַ��� 
		num=0;
		while (num<d)
		{    
	    LCD_ShowFonts(x,y,font,num,0);
      x += 24;                                           //ÿдһ���ַ���λ��ƫ��һ���ַ��Ĵ�С��24��
			num++;                                             //�ַ���ƫ��1
		}
		y += 24;
		m += b;
	}
	
}


#endif   //__SPECIALLY_ENABLE





///***************************************************************************
//�������ƣ�LCD_ShowWord_In
//�������ܣ���ʾ���ַ��ײ���б���ɫ
//������ע��
//***************************************************************************/
//void LCD_ShowWord_In(u16 x,u16 y,u8 size,u8 *font)
//{

//	u8 i = 0,j,k,c,word,s;        //i:��ʾ��ǰ��i���ַ���j:����������ȡ�ַ��ĵ�j���ֽڣ�k�������ַ��ĳ��ȣ�
//	                              //c:��ȡ��ǰ���ַ���ASCIIֵ����ȥһ��32���൱�ڶ�һ���ַ������ַ����λ�ӣ�
//																//word:��ȡ��ǰ�ַ����ֽڣ�s������ִ��һ���ֽڵ�����
//	u8 y0 = y,x0 = x,line = 0;    //x0,y0:��¼��ʵ���ꣻline����¼����ڼ���
//	k = (size * size) >> 4;
//	while(font[i] != '\0')
//	{
//		c = font[i] - 32;
//		y = y0;
//		x = i * size / 2 + x0 - lcddev.width * line;
//		if(x > (lcddev.width - size / 2))
//		{
//			x   = 0;
//			y  += size;
//			y0 += size;
//			line++;
//		}
//		if(y >= lcddev.height)return;
//		for(j = 0;j < k;j++)
//		{
//			if(size==12)     word=asc2_1206[c][j]; 	 	//����1206����
//			else if(size==16)word=asc2_1608[c][j];	  //����1608����
//			else if(size==24)word=asc2_2412[c][j];	  //����2412����
//			else
//			return;
//			
//			for(s = 0;s < 8;s++)
//			{
//				if(word & 0x80)
//					LCD_Fast_DrawPoint(x,y,POINT_COLOR);
//				else
//					LCD_Fast_DrawPoint(x,y,BACK_COLOR);
//				word <<= 1;
//				y++;
//				if((y - y0) == size)
//				{
//					y = y0;
//					x++;
//					break;
//				}
//			}
//		}
//		i++;
//	}

//}





