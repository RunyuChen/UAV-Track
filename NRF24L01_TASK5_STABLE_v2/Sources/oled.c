#include "oled.h"
#include "oledfont.h"
#include "main.h"
//几个变量声明
//uint8_t **Hzk;
extern I2C_HandleTypeDef hi2c1;
//初始化命令
uint8_t CMD_Data[]={
0xAE, 0x00, 0x10, 0x40, 0xB0, 0x81, 0xFF, 0xA1, 0xA6, 0xA8, 0x3F,
0xC8, 0xD3, 0x00, 0xD5, 0x80, 0xD8, 0x05, 0xD9, 0xF1, 0xDA, 0x12,
0xD8, 0x30, 0x8D, 0x14, 0xAF};
void WriteCmd()
{
  uint8_t i = 0;
  for(i=0; i<27; i++)
  {
    HAL_I2C_Mem_Write(&hi2c1 ,0x78,0x00,I2C_MEMADD_SIZE_8BIT,CMD_Data+i,1,0x100);
  }
}
//向设备写控制命令
void OLED_WR_CMD(uint8_t cmd)
{
  HAL_I2C_Mem_Write(&hi2c1 ,0x78,0x00,I2C_MEMADD_SIZE_8BIT,&cmd,1,0x100);
}
//向设备写数据
void OLED_WR_DATA(uint8_t data)
{
  HAL_I2C_Mem_Write(&hi2c1 ,0x78,0x40,I2C_MEMADD_SIZE_8BIT,&data,1,0x100);
}
//初始化oled屏幕
void OLED_Init(void)
{   
  HAL_Delay(200);
  WriteCmd();
}
//清屏size12 size16要清两行，其他函数有类似情况
void OLED_Clear()
{
  uint8_t i,n;        
  for(i=0;i<8;i++)  
  {  
    OLED_WR_CMD(0xb0+i);
    OLED_WR_CMD (0x00); 
    OLED_WR_CMD (0x10); 
    for(n=0;n<128;n++)
      OLED_WR_DATA(0);
  } 
}
//清行
void OLED_Clearrow(uint8_t i)
{
  uint8_t n;
  OLED_WR_CMD(0xb0+i);
  OLED_WR_CMD(0x00); 
  OLED_WR_CMD(0x10); 
  for(n=0;n<128;n++)
    OLED_WR_DATA(0);
}
//开启OLED显示    
void OLED_Display_On(void)
{
  OLED_WR_CMD(0X8D);  //SET DCDC命令
  OLED_WR_CMD(0X14);  //DCDC ON
  OLED_WR_CMD(0XAF);  //DISPLAY ON
}
//关闭OLED显示     
void OLED_Display_Off(void)
{
  OLED_WR_CMD(0X8D);  //SET DCDC命令
  OLED_WR_CMD(0X10);  //DCDC OFF
  OLED_WR_CMD(0XAE);  //DISPLAY OFF
}              
void OLED_Set_Pos(uint8_t x, uint8_t y) 
{   
  OLED_WR_CMD(0xb0+y);
  OLED_WR_CMD(((x&0xf0)>>4)|0x10);
  OLED_WR_CMD(x&0x0f);
} 
 
void OLED_On(void)  
{  
  uint8_t i,n;        
  for(i=0;i<8;i++)  
  {  
    OLED_WR_CMD(0xb0+i);    //设置页地址（0~7）
    OLED_WR_CMD(0x00);      //设置显示位置—列低地址
    OLED_WR_CMD(0x10);      //设置显示位置—列高地址   
    for(n=0;n<128;n++)
      OLED_WR_DATA(1); 
  } //更新显示
}
unsigned int oled_pow(uint8_t m,uint8_t n)
{
  unsigned int result=1;   
  while(n--)result*=m;    
  return result;
}

//在指定位置显示一个字符,包括部分字符
//x:0~127
//y:0~63
//mode:0,反白显示;1,正常显示         
//size:选择字体 16/12 
void OLED_ShowChar(uint8_t x,uint8_t y,uint8_t chr,uint8_t Char_Size)
{        
  unsigned char c=0,i=0;  
  c=chr-' ';//得到偏移后的值      
  if(x>128-1){x=0;y=y+2;}
  if(Char_Size ==16)
  {
    OLED_Set_Pos(x,y);  
    for(i=0;i<8;i++)
    OLED_WR_DATA(F8x16[c*16+i]);
    OLED_Set_Pos(x,y+1);
    for(i=0;i<8;i++)
    OLED_WR_DATA(F8x16[c*16+i+8]);
  }
  else 
  {  
    OLED_Set_Pos(x,y);
    for(i=0;i<6;i++)
    OLED_WR_DATA(F6x8[c][i]);
  }
}
 //显示2个数字
//x,y :起点坐标   
//len :数字的位数
//size:字体大小
//mode:模式  0,填充模式;1,叠加模式
//num:数值(0~4294967295);
void OLED_ShowNum(uint8_t x,uint8_t y,int num,uint8_t len,uint8_t size2)
{         
  uint8_t str[7],i=0;
  str[6]=0;
  if(num<0)
  { num=-num;
    str[0]='-';
  }
  else
    str[0]=' ';
  
  do
  { str[5-i]=num%10+'0';
    num=num/10;
    i++;
  } while(num!=0);
  
  if(str[0]=='-')
    str[5-i++]='-';
  
  if(i<len)
  for(;i<len;i++)
  { str[5-i]=' ';
  }
  else if (i>len)  //超出len指定的长度时，低位不显示
  { str[9-i]=0;
  }    
  OLED_ShowString(x,y,str+6-i,size2);  
} 
//显示一个字符号串
void OLED_ShowString(uint8_t x,uint8_t y,uint8_t *chr,uint8_t Char_Size)
{
  unsigned char j=0;
  while (chr[j]!='\0')
  { OLED_ShowChar(x,y,chr[j],Char_Size);
    x+=8;
    if(x>120){x=0;y+=2;}
      j++;
  }
}
void OLED_ShowFloatNumber(uint8_t x,uint8_t y,float num,uint8_t Char_Size)
{
	unsigned char zheng_len=0,xiao_len=0,polarity_flag=0;
	int zheng=0,xiao=0;		//经读者“迷迷惘惘”提醒，为这两行局部变量赋值
	//附注：为防止程序运行过程中程序行为异常，需要对局部变量赋值
	if(num>0)
		polarity_flag=1;
	else
	{
		polarity_flag=0;
		num=-num;
	}
	zheng=(int)num;
	xiao=((num-zheng)*100)/1;	//显示小数点后两位
	while(zheng)	//计算整数部分位数
	{
		zheng_len++;
		zheng/=10;
	}
	xiao_len=2;
	zheng=(int)num;	//在计算整数部分数值时，原赋值被篡改，这里重新赋值
	if(polarity_flag)
	{	
		if(Char_Size==16)
		{
			OLED_ShowNum(x,y,zheng,zheng_len,Char_Size);
			OLED_ShowChar(x+zheng_len*8,y,'.',Char_Size);
			OLED_ShowNum(x+(zheng_len+1)*8,y,xiao,xiao_len,Char_Size);
		}
		else
		{
			OLED_ShowNum(x,y,zheng,zheng_len,Char_Size);
			OLED_ShowChar(x+zheng_len*6,y,'.',Char_Size);
			OLED_ShowNum(x+(zheng_len+1)*6,y,xiao,xiao_len,Char_Size);
		}
	}
	else
	{
		OLED_ShowChar(x,y,'-',Char_Size);
		if(Char_Size==16)
		{
			OLED_ShowNum(x+8,y,zheng,zheng_len,Char_Size);
			OLED_ShowChar(x+(zheng_len+1)*8,y,'.',Char_Size);
			OLED_ShowNum(x+(zheng_len+2)*8,y,xiao,xiao_len,Char_Size);
		}
		else
		{
			OLED_ShowNum(x+6,y,zheng,zheng_len,Char_Size);
			OLED_ShowChar(x+(zheng_len+1)*6,y,'.',Char_Size);
			OLED_ShowNum(x+(zheng_len+2)*6,y,xiao,xiao_len,Char_Size);
		}
	}
}



