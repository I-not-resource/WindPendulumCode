#include "LCD_OWN.h"
#include "lcd_drive.h"
#include "Fonts.h"
#if __SPECIALLY_ENABLE
////////////////////////////////////////////////////////////////////////////
//警告！！！
//此应用程序，专为5510优化设计
//因此不具备良好的兼容性
////////////////////////////////////////////////////////////////////////////

/***************************************************************************
函数名称：LCD_Fill
函数功能：在指定区域内填充单个颜色
函数备注：
***************************************************************************/
void LCD_Fill(u16 sx,u16 sy,u16 ex,u16 ey,u16 color)
{          
	u16 i,j;
	u16 xlen=0;
	xlen=ex-sx+1;		
	for(i=sy;i<=ey;i++)
	{
	 	LCD_SetCursor(sx,i);      				      //设置光标位置 
		LCD_WriteRAM_Prepare();     			      //开始写入GRAM	  
		for(j=0;j<xlen;j++)
		LCD->LCD_RAM = color;                   //显示颜色 	    
	}	 
}

/***************************************************************************
函数名称：LCD_Color_Fill
函数功能：在指定区域内填充指定颜色块
函数备注：color:要填充的颜色,
***************************************************************************/
void LCD_Color_Fill(u16 sx,u16 sy,u16 ex,u16 ey,u16 *color)
{  
	u16 height,width;
	u16 i,j;
	width=ex-sx+1; 			                                 //得到填充的宽度
	height=ey-sy+1;			                                 //高度
 	for(i=0;i<height;i++)
	{
 		LCD_SetCursor(sx,sy+i);                            //设置光标位置 
		LCD_WriteRAM_Prepare();                            //开始写入GRAM
		for(j=0;j<width;j++)
		LCD->LCD_RAM=color[i*width+j];                     //写入数据 
	}		  
} 

/***************************************************************************
函数名称：LCD_DrawLine
函数功能：画线
函数备注：x1,y1:起点坐标
         x2,y2:终点坐标  
***************************************************************************/
void LCD_DrawLine(u16 x1, u16 y1, u16 x2, u16 y2)
{
	u16 t; 
	int xerr=0,yerr=0,delta_x,delta_y,distance; 
	int incx,incy,uRow,uCol; 
	delta_x=x2-x1;                           //计算坐标增量 
	delta_y=y2-y1; 
	uRow=x1; 
	uCol=y1; 
	if(delta_x>0)incx=1;                     //设置单步方向 
	else if(delta_x==0)incx=0;               //垂直线 
	else {incx=-1;delta_x=-delta_x;} 
	if(delta_y>0)incy=1; 
	else if(delta_y==0)
		incy=0;                                //水平线 
	else{incy=-1;delta_y=-delta_y;} 
	if( delta_x>delta_y)
		distance=delta_x;                      //选取基本增量坐标轴 
	else distance=delta_y; 
	for(t=0;t<=distance+1;t++ )              //画线输出 
	{  
		LCD_DrawPoint(uRow,uCol);              //画点 
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
函数名称：LCD_DrawRectangle
函数功能：画矩形	
函数备注：x1,y1:起点坐标
         x2,y2:终点坐标  
***************************************************************************/
void LCD_DrawRectangle(u16 x1, u16 y1, u16 x2, u16 y2)
{
	LCD_DrawLine(x1,y1,x2,y1);
	LCD_DrawLine(x1,y1,x1,y2);
	LCD_DrawLine(x1,y2,x2,y2);
	LCD_DrawLine(x2,y1,x2,y2);
}


/***************************************************************************
函数名称：LCD_Draw_Circle
函数功能：在指定位置画一个指定大小的圆
函数备注：(x,y):中心点
         r    :半径 
***************************************************************************/
void LCD_Draw_Circle(u16 x0,u16 y0,u8 r)
{
	int a,b;
	int di;
	a=0;b=r;	  
	di=3-(r<<1);                            //判断下个点位置的标志
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
		//使用Bresenham算法画圆     
		if(di<0)di +=4*a+6;	  
		else
		{
			di+=10+4*(a-b);   
			b--;
		} 						    
	}
} 									  

/***************************************************************************
函数名称：LCD_ShowChar
函数功能：在指定位置显示一个字符
函数备注：x,y:起始坐标
         num:要显示的字符:" "--->"~"
				 size:字体大小 12/16/24
				 mode:叠加方式(1)还是非叠加方式(0)
***************************************************************************/
void LCD_ShowChar(u16 x,u16 y,u8 num,u8 size,u8 mode)
{  							  
    u8 temp,t1,t;
	u16 y0=y;
	u8 csize=(size/8+((size%8)?1:0))*(size/2);		//得到字体一个字符对应点阵集所占的字节数	
	//设置窗口		   
	num=num-' ';                                  //得到偏移后的值
	for(t=0;t<csize;t++)
	{   
		if(size==12)      temp=asc2_1206[num][t];   //调用1206字体
		else if(size==16) temp=asc2_1608[num][t];	  //调用1608字体
		else if(size==24) temp=asc2_2412[num][t];	  //调用2412字体
		else return;								                //没有的字库
		for(t1=0;t1<8;t1++)
		{			    
			if(temp&0x80)   LCD_Fast_DrawPoint(x,y,POINT_COLOR);
			else if(mode==0)LCD_Fast_DrawPoint(x,y,BACK_COLOR);
			temp<<=1;
			y++;
			if(y>=lcddev.height)return;		//超区域了
			if((y-y0)==size)
			{
				y=y0;
				x++;
				if(x>=lcddev.width)return;	//超区域了
				break;
			}
		}  	 
	}  	    	   	 	  
}   

/***************************************************************************
函数名称：LCD_Pow
函数功能：m^n函数
函数备注：
***************************************************************************/
u32 LCD_Pow(u8 m,u8 n)
{
	u32 result=1;	 
	while(n--)result*=m;    
	return result;
}			 

/***************************************************************************
函数名称：LCD_ShowNum
函数功能：显示数字,高位为0,则不显示
函数备注：x,y :起点坐标	
         xlen :数字的位数 
				 size:字体大小
				 color:颜色 
				 num:数值(0~4294967295);	
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
函数名称：LCD_ShowxNum
函数功能：显示数字,高位为0,还是显示
函数备注：x,y:起点坐标
         num:数值(0~999999999); 
				 len:长度(即要显示的位数)
				 size:字体大小
				 mode:[7]:0,不填充;1,填充0.
				      [6:1]:保留
							[0]:0,非叠加显示;1,叠加显示
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
函数名称：LCD_ShowWord
函数功能：直接在顶层显示字，可以一次性，任意输入，直到屏幕无法容纳字符
函数备注：size为字体大小
***************************************************************************/
void LCD_ShowWord(u16 x,u16 y,u8 size,u8 *font,u8 mode)
{

	u16 i = 0,c;                   //i:显示当前第i个字符；c:读取当前的字符的ASCII值，相当于读一个字符所在字符库的位子；
	u16 y0 = y,x0 = x,line = 0;    //x0,y0:记录其实坐标；line：记录输出第几行
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
函数名称：LCD_Location_Med
函数功能：使字符在行中间显示，y为行0—3/0-7，font[]为字符输入
函数备注：
***************************************************************************/
void LCD_Location_Med(u16 y,u8 size,u8 *font)
{
	
	u16 i=0,j,n,k=0,num=0,c,x,m=0,a,b,d,cnum;		   //i为总字符数，j为逐行写入，num目前行写入字符个数，c为字符对应的十六进制，x为横轴字符坐标位置，m为前几行写入字符个数
	while(font[i]!='\0')
	{
		i++;
	}
	cnum = lcddev.width / size * 2;
	k    = i / cnum + ((i % cnum) && 1);					//行数	
	if(i % k)
		a  = k - i % k;                                 //少的字符数
	else
		a = 0;
	b    = i/k+(a&&1);                            //每一行字符数
	n    = (lcddev.width - b * size / 2) / 2;	    //缩进位置	
	for(j=0;j<k;j++)
	{
		x  = n + ((a - a % 2) * size / 4) * ((j + 1) / k);      //使最后一行缩进a个 半字符 的位置
		d  = b - a * ((j + 1) / k);                             //最后一行字符数 
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
函数名称：LCD_ShowFonts
函数功能：在指定位置显示一个字符
函数备注：x,y:起始坐标
         font[][72]：字符所在数组
				 size:第几个字符
				 mode:叠加方式(1)还是非叠加方式(0)
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
函数名称：LCD_Fonts_Med
函数功能：使字符在行中间显示，y为行，font[][72]为字符输入，用于显示汉字
函数备注：i为总字符数，j为逐行写入，num目前行写入字符个数，x为横轴字符坐标位置，m为前几行写入字符个数
***************************************************************************/
void LCD_Fonts_Med(u16 y,u8 size,u8 font[][72])
{
	
	u16 i=0,j,n,k=0,num=0,x,m=0,a,b,d,cnum;
  i = size;
	cnum = lcddev.width / 24;                              //计算一行可容纳的字数
	k    = i / cnum + ((i % cnum) && 1);                   //行数	
	if(i % k)
		a  = k - i % k;                                      //少的字符数
	else
		a = 0;
	b    = i/k+(a&&1);                                     //每一行字符数
	n    = (lcddev.width - b * 24) / 2;                    //缩进位置	
	for(j=0;j<k;j++)
	{
		x  = n + ((a - a % 2) * 24 / 2) * ((j + 1) / k);     //使最后一行缩进a个 半字符 的位置0
		d  = b - a * ((j + 1) / k);                          //最后一行字符数 
		num=0;
		while (num<d)
		{    
	    LCD_ShowFonts(x,y,font,num,0);
      x += 24;                                           //每写一个字符，位置偏移一个字符的大小（24）
			num++;                                             //字符数偏移1
		}
		y += 24;
		m += b;
	}
	
}


#endif   //__SPECIALLY_ENABLE





///***************************************************************************
//函数名称：LCD_ShowWord_In
//函数功能：显示的字符底层会有背景色
//函数备注：
//***************************************************************************/
//void LCD_ShowWord_In(u16 x,u16 y,u8 size,u8 *font)
//{

//	u8 i = 0,j,k,c,word,s;        //i:显示当前第i个字符；j:这个嘛，用来读取字符的第j个字节；k：计算字符的长度；
//	                              //c:读取当前的字符的ASCII值，减去一个32，相当于读一个字符所在字符库的位子；
//																//word:读取当前字符的字节；s：用来执行一个字节点的输出
//	u8 y0 = y,x0 = x,line = 0;    //x0,y0:记录其实坐标；line：记录输出第几行
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
//			if(size==12)     word=asc2_1206[c][j]; 	 	//调用1206字体
//			else if(size==16)word=asc2_1608[c][j];	  //调用1608字体
//			else if(size==24)word=asc2_2412[c][j];	  //调用2412字体
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





