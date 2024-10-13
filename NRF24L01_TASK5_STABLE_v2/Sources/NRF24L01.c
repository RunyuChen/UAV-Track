#include "NRF24L01.h"
#include "main.h"
//////////////////////////////////////////////////////////////////////////////////   
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//////////////////////////////////////////////////////////////////////////////////     
#define EN_DYNAMIC_DATA_LENGTH 1  //1:��̬���ݳ���;0:�̶�����(32�ֽ�)

extern uint8_t pipe;
extern uint8_t desNode;
extern uint8_t currentNode;
extern uint8_t MC_flag;

const uint8_t TX_ADDRESS1_2[TX_ADR_WIDTH]={0xC2,0xC2,0xC2,0xC2,0xC2};
const uint8_t RX_ADDRESS1_2[TX_ADR_WIDTH]={0xC2,0xC2,0xC2,0xC2,0xC2};
const uint8_t TX_ADDRESS1_3[TX_ADR_WIDTH]={0xC3,0xC2,0xC2,0xC2,0xC2};
const uint8_t RX_ADDRESS1_3[TX_ADR_WIDTH]={0xC3,0xC2,0xC2,0xC2,0xC2};

const uint8_t TX_ADDRESS2_1[TX_ADR_WIDTH]={0xC7,0xC2,0xC2,0xC2,0xC2};
const uint8_t RX_ADDRESS2_1[TX_ADR_WIDTH]={0xC7,0xC2,0xC2,0xC2,0xC2};
const uint8_t TX_ADDRESS2_3[TX_ADR_WIDTH]={0xC4,0xC2,0xC2,0xC2,0xC2};
const uint8_t RX_ADDRESS2_3[TX_ADR_WIDTH]={0xC4,0xC2,0xC2,0xC2,0xC2};

const uint8_t TX_ADDRESS3_1[TX_ADR_WIDTH]={0xC5,0xC2,0xC2,0xC2,0xC2};
const uint8_t RX_ADDRESS3_1[TX_ADR_WIDTH]={0xC5,0xC2,0xC2,0xC2,0xC2};
const uint8_t TX_ADDRESS3_2[TX_ADR_WIDTH]={0xC6,0xC2,0xC2,0xC2,0xC2};
const uint8_t RX_ADDRESS3_2[TX_ADR_WIDTH]={0xC6,0xC2,0xC2,0xC2,0xC2};

const uint8_t TX_ADDRESSB_C1[TX_ADR_WIDTH]={0xC8,0xC2,0xC2,0xC2,0xC2};
const uint8_t RX_ADDRESSB_C1[TX_ADR_WIDTH]={0xC8,0xC2,0xC2,0xC2,0xC2};
const uint8_t TX_ADDRESSB_C2[TX_ADR_WIDTH]={0xC9,0xC2,0xC2,0xC2,0xC2};
const uint8_t RX_ADDRESSB_C2[TX_ADR_WIDTH]={0xC9,0xC2,0xC2,0xC2,0xC2};
const uint8_t TX_ADDRESSB_C3[TX_ADR_WIDTH]={0xCA,0xC2,0xC2,0xC2,0xC2};
const uint8_t RX_ADDRESSB_C3[TX_ADR_WIDTH]={0xCA,0xC2,0xC2,0xC2,0xC2};

const uint8_t TX_ADDRESS_RateT[TX_ADR_WIDTH]={0xCB,0xC2,0xC2,0xC2,0xC2};
const uint8_t RX_ADDRESS_RateT[1]={0xCB};

const uint8_t TX_ADDRESS_RateR[TX_ADR_WIDTH]={0xC5,0xC2,0xC2,0xC2,0xC2};
const uint8_t RX_ADDRESS_RateR[1]={0xC5};

extern SPI_HandleTypeDef hspi1; 
extern UART_HandleTypeDef huart1;
uint8_t status_si2401=0;
extern uint8_t channel_flag;

uint8_t SPIx_ReadWriteByte(uint8_t TxData)
{ uint8_t RxData;
  HAL_SPI_TransmitReceive(&hspi1,&TxData,&RxData,1,10);
  return RxData;
}
//���24L01�Ƿ����
//����ֵ:0���ɹ�;1��ʧ��  
uint8_t NRF24L01_Check(void)
{
  uint8_t buf[5]={0XA5,0XA5,0XA5,0XA5,0XA5},buf2[5];
  uint8_t i;
  NRF24L01_Write_Buf(NRF24L01_WRITE_REG+TX_ADDR,buf,5);//д��5���ֽڵĵ�ַ.  
  NRF24L01_Read_Buf(TX_ADDR,buf2,5); //����д��ĵ�ַ  
  for(i=0;i<5;i++)if(buf2[i]!=0XA5)break;                    
  if(i!=5)return 1;//���24L01����  
  return 0;     //��⵽24L01
}      
//SPIд�Ĵ���
//reg:ָ���Ĵ�����ַ
//value:д���ֵ
//����״̬�Ĵ�����ֵ
uint8_t NRF24L01_Write_Reg(uint8_t reg,uint8_t value)
{
  uint8_t status;  
  HAL_GPIO_WritePin(NRF24L01_CSN_GPIO_Port, NRF24L01_CSN_Pin, GPIO_PIN_RESET);                 //ʹ��SPI����
  status =SPIx_ReadWriteByte(reg);//���ͼĴ����� 
  SPIx_ReadWriteByte(value);      //д��Ĵ�����ֵ
  HAL_GPIO_WritePin(NRF24L01_CSN_GPIO_Port, NRF24L01_CSN_Pin, GPIO_PIN_SET);                 //��ֹSPI����     
  return(status);             //����״ֵ̬
}
//��ȡSPI�Ĵ���ֵ
//reg:Ҫ���ļĴ���
uint8_t NRF24L01_Read_Reg(uint8_t reg)
{ //static uint8_t status;
  static uint8_t reg_val;      
  HAL_GPIO_WritePin(NRF24L01_CSN_GPIO_Port, NRF24L01_CSN_Pin, GPIO_PIN_RESET);          //ʹ��SPI����    
  SPIx_ReadWriteByte(reg);   //���ͼĴ�����
  reg_val=SPIx_ReadWriteByte(0XFF);//��ȡ�Ĵ�������
  HAL_GPIO_WritePin(NRF24L01_CSN_GPIO_Port, NRF24L01_CSN_Pin, GPIO_PIN_SET);          //��ֹSPI����        
  return(reg_val);           //����״ֵ̬
}  
//��ָ��λ�ö���ָ�����ȵ�����
//reg:�Ĵ���(λ��)
//*pBuf:����ָ��
//len:���ݳ���
//����ֵ,�˴ζ�����״̬�Ĵ���ֵ 
uint8_t NRF24L01_Read_Buf(uint8_t reg,uint8_t *pBuf,uint8_t len)
{
  uint8_t status,u8_ctr;         
  HAL_GPIO_WritePin(NRF24L01_CSN_GPIO_Port, NRF24L01_CSN_Pin, GPIO_PIN_RESET);           //ʹ��SPI����
  status=SPIx_ReadWriteByte(reg);//���ͼĴ���ֵ(λ��),����ȡ״ֵ̬        
  for(u8_ctr=0;u8_ctr<len;u8_ctr++)
    pBuf[u8_ctr]=SPIx_ReadWriteByte(0XFF);//��������
  HAL_GPIO_WritePin(NRF24L01_CSN_GPIO_Port, NRF24L01_CSN_Pin, GPIO_PIN_SET);       //�ر�SPI����
  return status;        //���ض�����״ֵ̬
}
//��ָ��λ��дָ�����ȵ�����
//reg:�Ĵ���(λ��)
//*pBuf:����ָ��
//len:���ݳ���
//����ֵ,�˴ζ�����״̬�Ĵ���ֵ
uint8_t NRF24L01_Write_Buf(uint8_t reg, uint8_t *pBuf, uint8_t len)
{
  uint8_t status,u8_ctr;      
  
  HAL_GPIO_WritePin(NRF24L01_CSN_GPIO_Port, NRF24L01_CSN_Pin, GPIO_PIN_RESET);//NRF24L01_CSN = 0;//ʹ��SPI����
  status = SPIx_ReadWriteByte(reg);//���ͼĴ���ֵ(λ��),����ȡ״ֵ̬
  for(u8_ctr=0; u8_ctr<len; u8_ctr++)
    SPIx_ReadWriteByte(*pBuf++); //д������   
    
  HAL_GPIO_WritePin(NRF24L01_CSN_GPIO_Port, NRF24L01_CSN_Pin, GPIO_PIN_SET); //NRF24L01_CSN = 1;//�ر�SPI����
  return status;          //���ض�����״ֵ̬
}           
//����NRF24L01����һ������
//txbuf:�����������׵�ַ
//����ֵ:�������״��
uint8_t NRF24L01_TxPacket(uint8_t *txbuf,uint8_t len)
{
  uint8_t sta;
  uint16_t cnt=0;  
  //SPIx_SetSpeed(SPI_SPEED_8);//spi�ٶ�Ϊ9Mhz��24L01�����SPIʱ��Ϊ10Mhz��   
  HAL_GPIO_WritePin(NRF24L01_CE_GPIO_Port, NRF24L01_CE_Pin, GPIO_PIN_RESET);
#if EN_DYNAMIC_DATA_LENGTH//ifʹ�ܶ�̬���ݳ���
  NRF24L01_Write_Buf(WR_TX_PLOAD,txbuf,len);//д���ݵ�TX BUF  ���32���ֽ�
#else
  NRF24L01_Write_Buf(WR_TX_PLOAD,txbuf,TX_PLOAD_WIDTH);//д���ݵ�TX BUF  ���32���ֽ�  
#endif
  HAL_GPIO_WritePin(NRF24L01_CE_GPIO_Port, NRF24L01_CE_Pin, GPIO_PIN_SET);//��������
  //while((NRF24L01_IRQ!=0)&&cnt<10000)cnt++;//�ȴ��������
  while(cnt<1000)cnt++; //�ȴ�������ɡ�  ��֣����������һ��䣬���ݾ������Ͳ��ɹ���
  sta=NRF24L01_Read_Reg(STATUS);  //��ȡ״̬�Ĵ�����ֵ
  while( ((sta&TX_OK)==0) && (cnt<20000) ) //����δ��ɣ���ȴ�
  { sta=NRF24L01_Read_Reg(STATUS);  //��ȡ״̬�Ĵ�����ֵ
    cnt++;
  }    
  NRF24L01_Write_Reg(NRF24L01_WRITE_REG+STATUS,sta); //���TX_DS��MAX_RT�жϱ�־
  
	if(sta&MAX_TX)//�ﵽ����ط�����
  {
    NRF24L01_Write_Reg(FLUSH_TX,0xff);//���TX FIFO�Ĵ��� 
    return MAX_TX; 
  }
  if(sta&TX_OK)//�������
  {
		
    return TX_OK;
		
  }
  return 0xff;//����ԭ����ʧ��
}
//����NRF24L01����һ������
//rxbuf:�洢���յ������ݣ��׵�ַ
//����ֵ:n���յ����ֽ�����0��û�յ�����
uint8_t NRF24L01_RxPacket(uint8_t *rxbuf)
{
  uint8_t sta,len,RX_P_NO;                         
  //SPIx_SetSpeed(SPI_SPEED_8); //spi�ٶ�Ϊ9Mhz��24L01�����SPIʱ��Ϊ10Mhz��   
  sta=NRF24L01_Read_Reg(STATUS);  //��ȡ״̬�Ĵ�����ֵ 
	//RX_P_NO=sta&0x0e;	
  NRF24L01_Write_Reg(NRF24L01_WRITE_REG+STATUS,sta); //���RX_DR���жϱ�־
  if(sta&RX_OK)//���յ�����
  {
		//send_str(&huart1,(uint8_t *)("�յ�����"));
		RX_P_NO = sta;
		//send_int(&huart1,sta);
		switch(currentNode)
		{
			case 1:
			{
				switch(RX_P_NO & 0x0E)
				{
					case 0x02:channel_flag = 2;pipe = 1;break;			//��ȡͨ��1����:2�Ż�
					case 0x04:channel_flag = 3;pipe = 2;break;			//��ȡͨ��2����:3�Ż�
					case 0x06:channel_flag = 2;pipe = 3;break;			//��ȡͨ��3����:2�Ż�
					case 0x08:channel_flag = 3;pipe = 4;break;			//��ȡͨ��4����:3�Ż�
				}
				break;
			}
			case 2:
			{
				switch(RX_P_NO & 0x0E)
				{
					case 0x02:channel_flag = 1;pipe = 1;break;			//��ȡͨ��1����:1�Ż�
					case 0x04:channel_flag = 3;pipe = 2;break;			//��ȡͨ��2����:3�Ż�
					case 0x06:channel_flag = 1;pipe = 3;break;			//��ȡͨ��3����:1�Ż�
					case 0x08:channel_flag = 3;pipe = 4;break;			//��ȡͨ��4����:3�Ż�
				}
				break;
			}
			case 3:
			{
				switch(RX_P_NO & 0x0E)
				{
					case 0x02:channel_flag = 1;pipe = 1;break;			//��ȡͨ��1����:1�Ż�
					case 0x04:channel_flag = 2;pipe = 2;break;			//��ȡͨ��2����:2�Ż�
					case 0x06:channel_flag = 1;pipe = 3;break;			//��ȡͨ��3����:1�Ż�
					case 0x08:channel_flag = 2;pipe = 4;break;			//��ȡͨ��4����:2�Ż�
				}
				break;
			}
				
		}
		
  #if EN_DYNAMIC_DATA_LENGTH//ifʹ�ܶ�̬���ݳ���
    NRF24L01_Read_Buf(R_RX_PL_WID,&len,1);//��ȡ��Ч���ݳ���,����len��
    NRF24L01_Read_Buf(RD_RX_PLOAD,rxbuf,len);//��ȡlen���ֽ�����
    NRF24L01_Write_Reg(FLUSH_RX,0xff);//���RX FIFO�Ĵ��� 
    return len; 
  #else
    NRF24L01_Read_Buf(RD_RX_PLOAD,rxbuf,RX_PLOAD_WIDTH);//��ȡ����
    NRF24L01_Write_Reg(FLUSH_RX,0xff);//���RX FIFO�Ĵ���
    return RX_PLOAD_WIDTH;     
  #endif
  }     
  return 0;//û�յ��κ�����
} 

//�ú�����ʼ��NRF24L01��RXģʽ
//����RX��ַ,дRX���ݿ��,ѡ��RFƵ��,�����ʺ�LNA HCURR
//��CE��ߺ�,������RXģʽ,�����Խ���������            
void RX_Mode(void)
{
	//NRF24L01_Write_Reg(FLUSH_RX,0xff);
	//NRF24L01_Write_Reg(FLUSH_TX,0xff);
	
  HAL_GPIO_WritePin(NRF24L01_CE_GPIO_Port, NRF24L01_CE_Pin, GPIO_PIN_RESET);    
  
	switch(currentNode)
	{
		case 1:
		{
			NRF24L01_Write_Buf(NRF24L01_WRITE_REG+RX_ADDR_P1,(uint8_t*)RX_ADDRESS2_1,1);//дRX�ڵ��ַ
			NRF24L01_Write_Buf(NRF24L01_WRITE_REG+RX_ADDR_P2,(uint8_t*)RX_ADDRESS3_1,1);//дRX�ڵ��ַ
			NRF24L01_Write_Buf(NRF24L01_WRITE_REG+RX_ADDR_P3,(uint8_t*)RX_ADDRESSB_C2,1);//дRX�ڵ��ַ
			NRF24L01_Write_Buf(NRF24L01_WRITE_REG+RX_ADDR_P4,(uint8_t*)RX_ADDRESSB_C3,1);//дRX�ڵ��ַ
			NRF24L01_Write_Reg(NRF24L01_WRITE_REG+EN_AA,0x1e);    //ʹ��ͨ��1,2,3,4���Զ�Ӧ��    
			NRF24L01_Write_Reg(NRF24L01_WRITE_REG+EN_RXADDR,0x1e);//ʹ��ͨ��1,2,3,4�Ľ��յ�ַ   
			break;
		}
		case 2:
		{
			NRF24L01_Write_Buf(NRF24L01_WRITE_REG+RX_ADDR_P1,(uint8_t*)RX_ADDRESS1_2,1);//дRX�ڵ��ַ
			NRF24L01_Write_Buf(NRF24L01_WRITE_REG+RX_ADDR_P2,(uint8_t*)RX_ADDRESS3_2,1);//дRX�ڵ��ַ
			NRF24L01_Write_Buf(NRF24L01_WRITE_REG+RX_ADDR_P3,(uint8_t*)RX_ADDRESSB_C1,1);//дRX�ڵ��ַ
			NRF24L01_Write_Buf(NRF24L01_WRITE_REG+RX_ADDR_P4,(uint8_t*)RX_ADDRESSB_C3,1);//дRX�ڵ��ַ
			NRF24L01_Write_Reg(NRF24L01_WRITE_REG+EN_AA,0x1e);    //ʹ��ͨ��1,2,3,4���Զ�Ӧ��    
			NRF24L01_Write_Reg(NRF24L01_WRITE_REG+EN_RXADDR,0x1e);//ʹ��ͨ��1,2.3,4�Ľ��յ�ַ   
			break;
		}
		case 3:
		{
			NRF24L01_Write_Buf(NRF24L01_WRITE_REG+RX_ADDR_P1,(uint8_t*)RX_ADDRESS1_3,1);//дRX�ڵ��ַ
			NRF24L01_Write_Buf(NRF24L01_WRITE_REG+RX_ADDR_P2,(uint8_t*)RX_ADDRESS2_3,1);//дRX�ڵ��ַ
			NRF24L01_Write_Buf(NRF24L01_WRITE_REG+RX_ADDR_P3,(uint8_t*)RX_ADDRESSB_C1,1);//дRX�ڵ��ַ
			NRF24L01_Write_Buf(NRF24L01_WRITE_REG+RX_ADDR_P4,(uint8_t*)RX_ADDRESSB_C2,1);//дRX�ڵ��ַ
			NRF24L01_Write_Reg(NRF24L01_WRITE_REG+EN_AA,0x1e);    //ʹ��ͨ��1,2,4���Զ�Ӧ��    
			NRF24L01_Write_Reg(NRF24L01_WRITE_REG+EN_RXADDR,0x1e);//ʹ��ͨ��1,2,4�Ľ��յ�ַ   
			break;
		}
	}
  //NRF24L01_Write_Reg(NRF24L01_WRITE_REG+EN_AA,0x0e);    //ʹ��ͨ��0,1,2,3���Զ�Ӧ��    
  //NRF24L01_Write_Reg(NRF24L01_WRITE_REG+EN_RXADDR,0x0e);//ʹ��ͨ��0,1,2,3�Ľ��յ�ַ     
  NRF24L01_Write_Reg(NRF24L01_WRITE_REG+RF_CH,72);       //����RFͨ��Ƶ��      
  //NRF24L01_Write_Reg(NRF24L01_WRITE_REG+RX_PW_P0,RX_PLOAD_WIDTH);//ѡ��ͨ��0����Ч���ݿ��
	NRF24L01_Write_Reg(NRF24L01_WRITE_REG+RX_PW_P1,RX_PLOAD_WIDTH);
	NRF24L01_Write_Reg(NRF24L01_WRITE_REG+RX_PW_P2,RX_PLOAD_WIDTH);
	NRF24L01_Write_Reg(NRF24L01_WRITE_REG+RX_PW_P3,RX_PLOAD_WIDTH);      
  NRF24L01_Write_Reg(NRF24L01_WRITE_REG+RF_SETUP,0x0f);//����TX�������,0db����,2Mbps,���������濪��   
  NRF24L01_Write_Reg(NRF24L01_WRITE_REG+STATUS,0x70); //���RX_DR��TX_DS��MAX_RT�жϱ�־

#if EN_DYNAMIC_DATA_LENGTH//ifʹ�ܶ�̬���ݳ���
  NRF24L01_Write_Reg(NRF24L01_WRITE_REG+DYNPD,0x1e);  //DPL_P0
  NRF24L01_Write_Reg(NRF24L01_WRITE_REG+FEATURE,0x0e);//EN_DPL
  //NRF24L01_Write_Reg(NRF24L01_WRITE_REG+FEATURE,0x04+0x02);//EN_DPL+ʹ��ACK�����ݷ���ȷ��֡
#endif  
  //NRF24L01_Write_Reg(NRF24L01_WRITE_REG+CONFIG, 0x0f);//���û�������ģʽ�Ĳ���;PWR_UP,EN_CRC,16BIT_CRC,����ģʽ 
  NRF24L01_Write_Reg(NRF24L01_WRITE_REG+CONFIG, 0x07);//���û�������ģʽ�Ĳ���;PWR_UP,EN_CRC,16BIT_CRC,����ģʽ,���η����жϺͷ����������ж� 

  HAL_GPIO_WritePin(NRF24L01_CE_GPIO_Port, NRF24L01_CE_Pin, GPIO_PIN_SET); //CEΪ��,�������ģʽ 
  //������Ӻ��������䣬��ɿذ�Ҫ�ȷ��ͳɹ����ݲ��ܽ��գ�����֡� ������������ϵ��������ܽ���
  //STM32F401���İ������ģ�飬���ü�����������ϵ缴�ɽ��ա�
  HAL_Delay(1);
  HAL_GPIO_WritePin(NRF24L01_CE_GPIO_Port, NRF24L01_CE_Pin, GPIO_PIN_RESET);  
  HAL_GPIO_WritePin(NRF24L01_CE_GPIO_Port, NRF24L01_CE_Pin, GPIO_PIN_SET); //CEΪ��,�������ģʽ 
}


//�ú�����ʼ��NRF24L01��TXģʽ
//����TX��ַ,дTX���ݿ��,����RX�Զ�Ӧ��ĵ�ַ,���TX��������,ѡ��RFƵ��,�����ʺ�LNA HCURR
//PWR_UP,CRCʹ��
//��CE��ߺ�,������RXģʽ,�����Խ���������       
//CEΪ�ߴ���10us,����������.    

//1�Ż�����ģʽ
void TX_Mode0(void)
{                             
  HAL_GPIO_WritePin(NRF24L01_CE_GPIO_Port, NRF24L01_CE_Pin, GPIO_PIN_RESET);
	if(!MC_flag)
	{
		switch(desNode)
		{
			case 2:
			{
				NRF24L01_Write_Buf(NRF24L01_WRITE_REG+TX_ADDR,(uint8_t*)TX_ADDRESS1_2,TX_ADR_WIDTH);//дTX�ڵ��ַ 
				NRF24L01_Write_Buf(NRF24L01_WRITE_REG+RX_ADDR_P0,(uint8_t*)RX_ADDRESS1_2,RX_ADR_WIDTH); //����TX�ڵ��ַ,��ҪΪ��ʹ��ACK 
				break;
			}
			case 3:
			{
				NRF24L01_Write_Buf(NRF24L01_WRITE_REG+TX_ADDR,(uint8_t*)TX_ADDRESS1_3,TX_ADR_WIDTH);//дTX�ڵ��ַ 
				NRF24L01_Write_Buf(NRF24L01_WRITE_REG+RX_ADDR_P0,(uint8_t*)RX_ADDRESS1_3,RX_ADR_WIDTH); //����TX�ڵ��ַ,��ҪΪ��ʹ��ACK 
				break;
			}
		}
	}
	else
	{
		NRF24L01_Write_Buf(NRF24L01_WRITE_REG+TX_ADDR,(uint8_t*)TX_ADDRESSB_C1,TX_ADR_WIDTH);//дTX�ڵ��ַ 
		NRF24L01_Write_Buf(NRF24L01_WRITE_REG+RX_ADDR_P0,(uint8_t*)RX_ADDRESSB_C1,RX_ADR_WIDTH); //����TX�ڵ��ַ,��ҪΪ��ʹ��ACK 
	}
	
 
  NRF24L01_Write_Reg(NRF24L01_WRITE_REG+EN_AA,0x1f);     //ʹ��ͨ��0��1��2��3,4���Զ�Ӧ��    
  NRF24L01_Write_Reg(NRF24L01_WRITE_REG+EN_RXADDR,0x1f); //ʹ��ͨ��0��1��2��3�Ľ��յ�ַ  
  NRF24L01_Write_Reg(NRF24L01_WRITE_REG+SETUP_RETR,0x12);//�����Զ��ط����ʱ��:500us + 86us;����Զ��ط�����:2��
  NRF24L01_Write_Reg(NRF24L01_WRITE_REG+RF_CH,72);       //����RFͨ��Ϊ40
  //NRF24L01_Write_Reg(NRF24L01_WRITE_REG+RF_SETUP,0x0f);  //����TX�������,7db����,2Mbps,���������濪��   
  NRF24L01_Write_Reg(NRF24L01_WRITE_REG+RF_SETUP,0x0b);  //����TX�������,0db����,2Mbps,���������濪��   
  //NRF24L01_Write_Reg(NRF24L01_WRITE_REG+CONFIG,0x7e);    //���û�������ģʽ�Ĳ���;PWR_UP,EN_CRC,16BIT_CRC,����ģʽ�����������ж�
  NRF24L01_Write_Reg(NRF24L01_WRITE_REG+CONFIG,0x3e);   //���û�������ģʽ�Ĳ���;PWR_UP,EN_CRC,16BIT_CRC,����ģʽ�����Ž����жϣ����η����жϺͷ����������ж�
  NRF24L01_Write_Reg(NRF24L01_WRITE_REG+STATUS,0x70); //���RX_DR��TX_DS��MAX_RT�жϱ�־
#if EN_DYNAMIC_DATA_LENGTH//ifʹ�ܶ�̬���ݳ���
  NRF24L01_Write_Reg(NRF24L01_WRITE_REG+DYNPD,0x1f);  //DPL_P0
  //NRF24L01_Write_Reg(NRF24L01_WRITE_REG+FEATURE,0x04);//EN_DPL
  NRF24L01_Write_Reg(NRF24L01_WRITE_REG+FEATURE,0x07);//EN_DPL+ʹ��ACK�����ݷ���ȷ��֡
#endif  
  
  HAL_GPIO_WritePin(NRF24L01_CE_GPIO_Port, NRF24L01_CE_Pin, GPIO_PIN_SET);//CEΪ��,10us����������
} 

//2�Ż�����ģʽ
void TX_Mode1(void)
{                             
  HAL_GPIO_WritePin(NRF24L01_CE_GPIO_Port, NRF24L01_CE_Pin, GPIO_PIN_RESET);
	if(!MC_flag)
	{
		switch(desNode)
		{
			case 1:
			{
				NRF24L01_Write_Buf(NRF24L01_WRITE_REG+TX_ADDR,(uint8_t*)TX_ADDRESS2_1,TX_ADR_WIDTH);//дTX�ڵ��ַ 
				NRF24L01_Write_Buf(NRF24L01_WRITE_REG+RX_ADDR_P0,(uint8_t*)RX_ADDRESS2_1,RX_ADR_WIDTH); //����TX�ڵ��ַ,��ҪΪ��ʹ��ACK 
				break;
			}
			case 3:
			{
				NRF24L01_Write_Buf(NRF24L01_WRITE_REG+TX_ADDR,(uint8_t*)TX_ADDRESS2_3,TX_ADR_WIDTH);//дTX�ڵ��ַ 
				NRF24L01_Write_Buf(NRF24L01_WRITE_REG+RX_ADDR_P0,(uint8_t*)RX_ADDRESS2_3,RX_ADR_WIDTH); //����TX�ڵ��ַ,��ҪΪ��ʹ��ACK 
				break;
			}
		}
	}
	else
	{
		NRF24L01_Write_Buf(NRF24L01_WRITE_REG+TX_ADDR,(uint8_t*)TX_ADDRESSB_C2,TX_ADR_WIDTH);//дTX�ڵ��ַ 
		NRF24L01_Write_Buf(NRF24L01_WRITE_REG+RX_ADDR_P0,(uint8_t*)RX_ADDRESSB_C2,RX_ADR_WIDTH); //����TX�ڵ��ַ,��ҪΪ��ʹ��ACK 
	}
	
  NRF24L01_Write_Reg(NRF24L01_WRITE_REG+EN_AA,0x1f);     //ʹ��ͨ��0��1��2��3,4���Զ�Ӧ��    
  NRF24L01_Write_Reg(NRF24L01_WRITE_REG+EN_RXADDR,0x1f); //ʹ��ͨ��0��1��2��3,4�Ľ��յ�ַ  
  NRF24L01_Write_Reg(NRF24L01_WRITE_REG+SETUP_RETR,0x12);//�����Զ��ط����ʱ��:500us + 86us;����Զ��ط�����:2��
  NRF24L01_Write_Reg(NRF24L01_WRITE_REG+RF_CH,72);       //����RFͨ��Ϊ40
  //NRF24L01_Write_Reg(NRF24L01_WRITE_REG+RF_SETUP,0x0f);  //����TX�������,7db����,2Mbps,���������濪��   
  NRF24L01_Write_Reg(NRF24L01_WRITE_REG+RF_SETUP,0x0b);  //����TX�������,0db����,2Mbps,���������濪��   
  //NRF24L01_Write_Reg(NRF24L01_WRITE_REG+CONFIG,0x7e);    //���û�������ģʽ�Ĳ���;PWR_UP,EN_CRC,16BIT_CRC,����ģʽ�����������ж�
  NRF24L01_Write_Reg(NRF24L01_WRITE_REG+CONFIG,0x3e);    //���û�������ģʽ�Ĳ���;PWR_UP,EN_CRC,16BIT_CRC,����ģʽ�����Ž����жϣ����η����жϺͷ����������ж�
  NRF24L01_Write_Reg(NRF24L01_WRITE_REG+STATUS,0x70); //���RX_DR��TX_DS��MAX_RT�жϱ�־
#if EN_DYNAMIC_DATA_LENGTH//ifʹ�ܶ�̬���ݳ���
  NRF24L01_Write_Reg(NRF24L01_WRITE_REG+DYNPD,0x0f);  //DPL_P0
  //NRF24L01_Write_Reg(NRF24L01_WRITE_REG+FEATURE,0x04);//EN_DPL
  NRF24L01_Write_Reg(NRF24L01_WRITE_REG+FEATURE,0x07);//EN_DPL+ʹ��ACK�����ݷ���ȷ��֡
#endif  
  
  HAL_GPIO_WritePin(NRF24L01_CE_GPIO_Port, NRF24L01_CE_Pin, GPIO_PIN_SET);//CEΪ��,10us����������
} 

//3�Ż�����ģʽ
void TX_Mode2(void)
{                             
  HAL_GPIO_WritePin(NRF24L01_CE_GPIO_Port, NRF24L01_CE_Pin, GPIO_PIN_RESET);
	if(!MC_flag)
	{
		switch(desNode)
		{
			case 1:
			{
				NRF24L01_Write_Buf(NRF24L01_WRITE_REG+TX_ADDR,(uint8_t*)TX_ADDRESS3_1,TX_ADR_WIDTH);//дTX�ڵ��ַ 
				NRF24L01_Write_Buf(NRF24L01_WRITE_REG+RX_ADDR_P0,(uint8_t*)RX_ADDRESS3_1,RX_ADR_WIDTH); //����TX�ڵ��ַ,��ҪΪ��ʹ��ACK 
				break;
			}
			case 2:
			{
				NRF24L01_Write_Buf(NRF24L01_WRITE_REG+TX_ADDR,(uint8_t*)TX_ADDRESS3_2,TX_ADR_WIDTH);//дTX�ڵ��ַ 
				NRF24L01_Write_Buf(NRF24L01_WRITE_REG+RX_ADDR_P0,(uint8_t*)RX_ADDRESS3_2,RX_ADR_WIDTH); //����TX�ڵ��ַ,��ҪΪ��ʹ��ACK 
				break;
			}
		}
	}
	else
	{
		NRF24L01_Write_Buf(NRF24L01_WRITE_REG+TX_ADDR,(uint8_t*)TX_ADDRESSB_C3,TX_ADR_WIDTH);//дTX�ڵ��ַ 
		NRF24L01_Write_Buf(NRF24L01_WRITE_REG+RX_ADDR_P0,(uint8_t*)RX_ADDRESSB_C3,RX_ADR_WIDTH); //����TX�ڵ��ַ,��ҪΪ��ʹ��ACK 
	}
	
  NRF24L01_Write_Reg(NRF24L01_WRITE_REG+EN_AA,0x1f);     //ʹ��ͨ��0��1��2��3��4���Զ�Ӧ��    
  NRF24L01_Write_Reg(NRF24L01_WRITE_REG+EN_RXADDR,0x1f); //ʹ��ͨ��0��1��2,4�Ľ��յ�ַ  
  NRF24L01_Write_Reg(NRF24L01_WRITE_REG+SETUP_RETR,0x12);//�����Զ��ط����ʱ��:500us + 86us;����Զ��ط�����:2��
  NRF24L01_Write_Reg(NRF24L01_WRITE_REG+RF_CH,72);       //����RFͨ��Ϊ40
  //NRF24L01_Write_Reg(NRF24L01_WRITE_REG+RF_SETUP,0x0f);  //����TX�������,7db����,2Mbps,���������濪��   
  NRF24L01_Write_Reg(NRF24L01_WRITE_REG+RF_SETUP,0x0b);  //����TX�������,0db����,2Mbps,���������濪��   
  //NRF24L01_Write_Reg(NRF24L01_WRITE_REG+CONFIG,0x7e);    //���û�������ģʽ�Ĳ���;PWR_UP,EN_CRC,16BIT_CRC,����ģʽ�����������ж�
  NRF24L01_Write_Reg(NRF24L01_WRITE_REG+CONFIG,0x3e);    //���û�������ģʽ�Ĳ���;PWR_UP,EN_CRC,16BIT_CRC,����ģʽ�����Ž����жϣ����η����жϺͷ����������ж�
  NRF24L01_Write_Reg(NRF24L01_WRITE_REG+STATUS,0x70); //���RX_DR��TX_DS��MAX_RT�жϱ�־
#if EN_DYNAMIC_DATA_LENGTH//ifʹ�ܶ�̬���ݳ���
  NRF24L01_Write_Reg(NRF24L01_WRITE_REG+DYNPD,0x1f);  //DPL_P0
  //NRF24L01_Write_Reg(NRF24L01_WRITE_REG+FEATURE,0x04);//EN_DPL
  NRF24L01_Write_Reg(NRF24L01_WRITE_REG+FEATURE,0x07);//EN_DPL+ʹ��ACK�����ݷ���ȷ��֡
#endif  
  
  HAL_GPIO_WritePin(NRF24L01_CE_GPIO_Port, NRF24L01_CE_Pin, GPIO_PIN_SET);//CEΪ��,10us����������
} 

void TX_ModeRate(void)
{                             
  HAL_GPIO_WritePin(NRF24L01_CE_GPIO_Port, NRF24L01_CE_Pin, GPIO_PIN_RESET);
	
	NRF24L01_Write_Buf(NRF24L01_WRITE_REG+TX_ADDR,(uint8_t*)TX_ADDRESS_RateT,TX_ADR_WIDTH);//дTX�ڵ��ַ 
	NRF24L01_Write_Buf(NRF24L01_WRITE_REG+RX_ADDR_P0,(uint8_t*)RX_ADDRESS_RateT,RX_ADR_WIDTH); //����TX�ڵ��ַ,��ҪΪ��ʹ��ACK  
 
  NRF24L01_Write_Reg(NRF24L01_WRITE_REG+EN_AA,0x3f);     //ʹ��ͨ��0��1��2��3���Զ�Ӧ��    
  NRF24L01_Write_Reg(NRF24L01_WRITE_REG+EN_RXADDR,0x3f); //ʹ��ͨ��0��1��2��3�Ľ��յ�ַ  
  NRF24L01_Write_Reg(NRF24L01_WRITE_REG+SETUP_RETR,0x12);//�����Զ��ط����ʱ��:500us + 86us;����Զ��ط�����:2��
  NRF24L01_Write_Reg(NRF24L01_WRITE_REG+RF_CH,72);       //����RFͨ��Ϊ40
  //NRF24L01_Write_Reg(NRF24L01_WRITE_REG+RF_SETUP,0x0f);  //����TX�������,7db����,2Mbps,���������濪��   
  NRF24L01_Write_Reg(NRF24L01_WRITE_REG+RF_SETUP,0x0b);  //����TX�������,0db����,2Mbps,���������濪��   
  //NRF24L01_Write_Reg(NRF24L01_WRITE_REG+CONFIG,0x7e);    //���û�������ģʽ�Ĳ���;PWR_UP,EN_CRC,16BIT_CRC,����ģʽ�����������ж�
  NRF24L01_Write_Reg(NRF24L01_WRITE_REG+CONFIG,0x3e);    //���û�������ģʽ�Ĳ���;PWR_UP,EN_CRC,16BIT_CRC,����ģʽ�����Ž����жϣ����η����жϺͷ����������ж�
  NRF24L01_Write_Reg(NRF24L01_WRITE_REG+STATUS,0x70); //���RX_DR��TX_DS��MAX_RT�жϱ�־
#if EN_DYNAMIC_DATA_LENGTH//ifʹ�ܶ�̬���ݳ���
  NRF24L01_Write_Reg(NRF24L01_WRITE_REG+DYNPD,0x3f);  //DPL_P0
  //NRF24L01_Write_Reg(NRF24L01_WRITE_REG+FEATURE,0x04);//EN_DPL
  NRF24L01_Write_Reg(NRF24L01_WRITE_REG+FEATURE,0x07);//EN_DPL+ʹ��ACK�����ݷ���ȷ��֡
#endif  
  
  HAL_GPIO_WritePin(NRF24L01_CE_GPIO_Port, NRF24L01_CE_Pin, GPIO_PIN_SET);//CEΪ��,10us����������
} 

void RX2TX(void)
{
  HAL_GPIO_WritePin(NRF24L01_CE_GPIO_Port, NRF24L01_CE_Pin, GPIO_PIN_RESET);
  //NRF24L01_Write_Reg(NRF24L01_WRITE_REG+CONFIG,0x0e); //���û�������ģʽ�Ĳ���;PWR_UP,EN_CRC,16BIT_CRC,����ģʽ,���������ж�
  NRF24L01_Write_Reg(NRF24L01_WRITE_REG+CONFIG,0x7e); //���û�������ģʽ�Ĳ���;PWR_UP,EN_CRC,16BIT_CRC,����ģʽ,���������ж�
  NRF24L01_Write_Reg(NRF24L01_WRITE_REG+STATUS,0x70); //���RX_DR��TX_DS��MAX_RT�жϱ�־
  HAL_GPIO_WritePin(NRF24L01_CE_GPIO_Port, NRF24L01_CE_Pin, GPIO_PIN_SET);//CEΪ��,10us����������  
}
void TX2RX(void)
{
  HAL_GPIO_WritePin(NRF24L01_CE_GPIO_Port, NRF24L01_CE_Pin, GPIO_PIN_RESET);
  NRF24L01_Write_Reg(NRF24L01_WRITE_REG+CONFIG, 0x3f);//���û�������ģʽ�Ĳ���;PWR_UP,EN_CRC,16BIT_CRC,����ģʽ,���η����жϺͷ����������ж� 
  NRF24L01_Write_Reg(NRF24L01_WRITE_REG+STATUS,0x70); //���RX_DR��TX_DS��MAX_RT�жϱ�־  
  HAL_GPIO_WritePin(NRF24L01_CE_GPIO_Port, NRF24L01_CE_Pin, GPIO_PIN_SET); //CEΪ��,�������ģʽ 

}

