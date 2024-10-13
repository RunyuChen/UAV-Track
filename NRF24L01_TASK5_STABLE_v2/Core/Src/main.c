/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "flash.h"
#include "oled.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint8_t nrf2401_tx_flag=0;
uint8_t nrf2401_txbuf[33];
uint8_t nrf_txbuf[33];
uint8_t txbuf_pos=0;
uint8_t currentNode = 0;
uint8_t desNode = 0;
uint8_t channel_flag=0;
uint8_t txflag = 2;
uint8_t MC_flag = 0;
uint8_t test_flag = 0;
uint32_t latency = 0;
uint32_t time = 0;
uint8_t pipe = 0;
float rate = 0;
uint8_t rx_cnt = 0;
uint32_t max_latency = 0;
uint32_t min_latency = 65536;
// �ڵ��ַ
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
void sendData(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)  //NRF2401_IRQ�����ж�
{ uint8_t tmp_buf[35],len;
  if(GPIO_Pin==NRF24L01_IRQ_Pin)
  { //*
    len=NRF24L01_RxPacket(tmp_buf); //�����յ����ֽ���,�յ���len���ֽ����ݴ���tmp_buf��
    if(len>0)//һ�����յ���Ϣ,�򴮿ڷ��ͳ�ȥ.
    {
			tmp_buf[len+1]=0;
			//if(channel_flag != 0)
			//{
				send_int(&huart1,channel_flag);
				send_str(&huart1,(uint8_t *)"�Ż�(ͨ��");
				send_int(&huart1,pipe);
				send_str(&huart1,(uint8_t *)"):");
				if((tmp_buf[0] == 'r' || tmp_buf[0] == 't') && tmp_buf[1] == 'x' && (tmp_buf[2] >= '0' && tmp_buf[2] <= '3') && tmp_buf[3] == ' ')
					send_char_array(&huart1,tmp_buf,len - 1);
				else
					send_char_array(&huart1,tmp_buf,len);
			//}
			HAL_GPIO_TogglePin(LED0_GPIO_Port,LED0_Pin);
			HAL_GPIO_TogglePin(LED1_GPIO_Port,LED2_Pin);
	//__HAL_GPIO_EXTI_CLEAR_IT(NRF24L01_IRQ_Pin);
    }  // */
  }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  HAL_NVIC_DisableIRQ(NRF24L01_IRQ_EXTI_IRQn); //��MX_GPIO_Init()������ĩβ���˴��жϣ��ȹرգ���NRL24L01��ʼ������ٿ��ж�
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE); //ʹ�ܴ���1�Ľ����жϣ��жϴ�����stm32f4xx_it.c�ļ�USART1_IRQHandler������
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE); //ʹ�ܴ���1�Ŀ����жϣ��жϴ�����stm32f4xx_it.c�ļ�USART1_IRQHandler������
	HAL_TIM_Base_Start_IT(&htim2);
  while(NRF24L01_Check())//��ⲻ��24L01
  {
    HAL_GPIO_TogglePin(LED0_GPIO_Port,LED0_Pin);
    HAL_GPIO_TogglePin(LED1_GPIO_Port,LED1_Pin);
    HAL_Delay(250);
  }
  RX_Mode();//NRF24L01����Ϊ����ģʽ
  
  // EXTI interrupt init
  HAL_NVIC_SetPriority(NRF24L01_IRQ_EXTI_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(NRF24L01_IRQ_EXTI_IRQn); //ʹ�ܴ��жϱ�����MX_GPIO_Init()������ĩβ���Ƶ��˵�NRL24L01��ʼ������ٿ��ж�
  //NRF24L01�������ݵĴ�������ڱ��ļ�HAL_GPIO_EXTI_Callback������

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  /*for(uint8_t i=0;i<32;i++)
    nrf2401_txbuf[i]=0x20+i;*/
  //send_str(&huart1,(uint8_t *)"STM32F401CCU6 SPI NRF24L01 TEST\r\n");
	
	//��������ϵ�ǰ�洢�������ţ��Ͱ�λ����Ŀ����ţ��߰�λ��
	uint16_t tem = *(uint16_t*)(FLASH_Start_Addr + 128);
	desNode = (uint8_t)(tem >> 8);
	currentNode = (uint8_t)(tem &0XFF);
	send_str(&huart1,(uint8_t *)"����");
	send_int(&huart1,currentNode);
	send_str(&huart1,(uint8_t *)"�Ż����͸�");
	send_int(&huart1,desNode);
	send_str(&huart1,(uint8_t *)"�Ż�\r\n");
		
  while (1)
  { 
     if(nrf2401_tx_flag) //����1����յ����ݣ��Ὣnrf2401_tx_flag������1 ,
    { //����1�յ������ݴ�������nrf2401_txbuf[]�У�����Ϊtxbuf_pos���ֽ�
			if(nrf2401_txbuf[0] == 't' && nrf2401_txbuf[1] == 'e' && nrf2401_txbuf[2] == 's' && nrf2401_txbuf[3] == 't' && txbuf_pos == 4) //tģʽΪ���������ӳ٣�����֣�ֻ�зֹ�����ң�����ܴ�ӡ����
			{
				test_flag++;
				test_flag = test_flag % 2;
				max_latency = 0;
				min_latency = 65535;
			}
			if((nrf2401_txbuf[0] == 'r' || nrf2401_txbuf[0] =='t') && txbuf_pos == 3 && nrf2401_txbuf[1] =='x' 
				&& (nrf2401_txbuf[2] >= '0' && nrf2401_txbuf[2] <= '3'))
			{
				switch(nrf2401_txbuf[0])
				{
					case 't'://nģʽ���ýڵ�ţ�n0���͹㲥
					{
						if(nrf2401_txbuf[2] - 48 != desNode)
						{
							switch(nrf2401_txbuf[2])
							{
								case '0':
									MC_flag = 1;break;
								case '1':
									currentNode = 1;MC_flag = 0;break;
								case '2':
									currentNode = 2;MC_flag = 0;break;
								case '3':
									currentNode = 3;MC_flag = 0;break;
							}
							if(MC_flag == 0 && (currentNode == 1 | currentNode == 2 | currentNode == 3))
							{
									send_str(&huart1,(uint8_t *)"����Ϊ");
									send_int(&huart1,currentNode);
									send_str(&huart1,(uint8_t *)"�Ż�\r\n");		
							}
							else
							{
								send_str(&huart1,(uint8_t *)"�㲥ģʽ\r\n");
							}
							break;
						}
						else
							send_str(&huart1,(uint8_t *)"Error:The transmitter and receiver cannot be the same.\r\n");
						break;
					}
					case 'r'://dģʽ���ý��ջ���d0�鿴���շ�
					{
						//if(currentNode != nrf2401_txbuf[1] - 48)
						//{
							switch(nrf2401_txbuf[2])
							{
								case '1':
									desNode = 1;break;
								case '2':
									desNode = 2;break;
								case '3':
									desNode = 3;break;
							}
							MC_flag = 0;
							send_str(&huart1,(uint8_t *)"���͸�");
							send_int(&huart1,desNode);
							send_str(&huart1,(uint8_t *)"�Ż�\r\n");
							//break;
						//}
						//else
							//send_str(&huart1,(uint8_t *)"Error:The transmitter and receiver cannot be the same.");
						break;
					}

				}
				
				txbuf_pos=0; //������0
				nrf2401_tx_flag=0; //��־λ��0
			}
      //TX_Mode0(); //NRF24L01�л�������ģʽ
			else
			{
				switch(currentNode)
				{
						case 1:
							TX_Mode0();break; //����ģʽ1����1�Ż�����2��3�Ż���
						case 2:
							TX_Mode1();break; //����ģʽ2����2�Ż�����1��3�Ż���
						case 3:
							TX_Mode2();break; //����ģʽ3����3�Ż�����1��2�Ż���
				}
				sendData(); //����sendData��������
				if(test_flag) //tģʽ���Զ��ط�
				{
					uint8_t cnt = 0;
					
					uint8_t nrf24l01_testbuf[32] = {'0','1','2','3','4','5','6','7',
																					 '8','9','a','b','c','d','e','f',
																					 '0','1','2','3','4','5','6','7',
																					 '8','9','a','b','c','d','e','f',
																					};//�ӳ����ʲ������ݰ�
					
					float avg_rate = 0.0f;
					float avg_latency = 0.0f;
					
					while(cnt < 200) //��200��
					{
						cnt++;
						time = 0;
						txflag = 1;
						if(NRF24L01_TxPacket(nrf24l01_testbuf,32)==TX_OK)  //if NRF24L01���ͳɹ�
						{
							txflag = 0;
							//HAL_Delay(10);
							rx_cnt++;
							max_latency = (max_latency > latency)?max_latency:latency;
							if(latency)
								min_latency = (min_latency < latency)?min_latency:latency;
							send_int(&huart1,rx_cnt);
							send_str(&huart1,(uint8_t *)"\r\n");
							send_str(&huart1,(uint8_t *)"����������ʱ��:");
							send_float(&huart1,latency/2.0f,2);
							send_str(&huart1,(uint8_t *)"us\r\n");
							send_str(&huart1,(uint8_t *)"��������������:");
							rate = 1000.0f*(32.0f)/(latency);
							send_float(&huart1,rate,3);
							send_str(&huart1,(uint8_t *)"KB/s\r\n");	
							avg_latency += latency;
							avg_rate += rate;
							HAL_GPIO_TogglePin(LED0_GPIO_Port,LED0_Pin); //�л��Ƶ�״̬
							HAL_GPIO_TogglePin(LED1_GPIO_Port,LED1_Pin); //�л��Ƶ�״̬
						}
						HAL_Delay(20);
					}
					send_str(&huart1,(uint8_t *)"tx:");
					send_int(&huart1,cnt);
					send_str(&huart1,(uint8_t *)"\r\n");
					send_str(&huart1,(uint8_t *)"rx:");
					send_int(&huart1,rx_cnt);
					send_str(&huart1,(uint8_t *)"\r\n");
					send_str(&huart1,(uint8_t *)"ƽ������ʱ��:");
					send_float(&huart1,1.0f*avg_latency/rx_cnt/2,3);
					send_str(&huart1,(uint8_t *)"us\r\n");
					send_str(&huart1,(uint8_t *)"���ƽ������ʱ��:");
					send_float(&huart1,max_latency/2,2);
					send_str(&huart1,(uint8_t *)"us\r\n");
					send_str(&huart1,(uint8_t *)"��Сƽ������ʱ��:");
					send_float(&huart1,min_latency/2,2);
					send_str(&huart1,(uint8_t *)"us\r\n");
					send_str(&huart1,(uint8_t *)"ƽ����������:");
					send_float(&huart1,1.0f*avg_rate/rx_cnt,3);
					send_str(&huart1,(uint8_t *)"KB/s\r\n:");	
					send_str(&huart1,(uint8_t *)"���ƽ����������:");
					send_float(&huart1,1.0f*1000.0f*32.0f/max_latency,3);
					send_str(&huart1,(uint8_t *)"KB/s\r\n");
					send_str(&huart1,(uint8_t *)"��Сƽ����������:");
					send_float(&huart1,1.0f*1000.0f*32.0f/min_latency,3);
					send_str(&huart1,(uint8_t *)"KB/s\r\n");	
					rx_cnt = 0;
				}
			}
      //else  //����ʧ��
        //HAL_Delay(10); //��ʱ10ms��������½��������if,���·��͡�
      txbuf_pos=0; //������0
			nrf2401_tx_flag=0; //��־λ��0
			RX_Mode(); //NRF24L01�л�������ģʽ
			uint16_t tem;
			if(currentNode != 0) //n0�ǹ㲥ģʽ����д��flash������n/d1,n/d2,n/d3д��flash
			{
				tem = (desNode<< 8)|currentNode;
				MEM_If_Erase_FS(FLASH_Start_Addr+128,FLASH_End_Addr);
				writedata_to_flash((int16_t *)&tem,2,FLASH_Start_Addr + 128);
			}
    
    /* USER CODE END WHILE */
		}
    /* USER CODE BEGIN 3 */
		  
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 84-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED3_Pin|LED4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(NRF24L01_CSN_GPIO_Port, NRF24L01_CSN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, NRF24L01_CE_Pin|LED1_Pin|LED2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED0_Pin */
  GPIO_InitStruct.Pin = LED0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED3_Pin NRF24L01_CSN_Pin LED4_Pin */
  GPIO_InitStruct.Pin = LED3_Pin|NRF24L01_CSN_Pin|LED4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : NRF24L01_CE_Pin LED1_Pin LED2_Pin */
  GPIO_InitStruct.Pin = NRF24L01_CE_Pin|LED1_Pin|LED2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : NRF24L01_IRQ_Pin */
  GPIO_InitStruct.Pin = NRF24L01_IRQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(NRF24L01_IRQ_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void sendData(void)
{
	if(NRF24L01_TxPacket(nrf2401_txbuf,txbuf_pos)==TX_OK)  //if NRF24L01���ͳɹ�
	{
		HAL_GPIO_TogglePin(LED0_GPIO_Port,LED0_Pin); //�л��Ƶ�״̬
		HAL_GPIO_TogglePin(LED1_GPIO_Port,LED1_Pin); //�л��Ƶ�״̬
	}
	
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
