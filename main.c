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
//#include "rc522.h"
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
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

uint16_t val_pls = 450;

//UART1
uint8_t dulieu_nhan;
uint8_t dulieu_gui[] = "N";

//UART2
uint8_t angleZ;   
uint8_t Z;  
//uint8_t str[MFRC522_MAX_LEN];
uint8_t dem=0;

// bien luu du lieu giao thuc SPI 
//uint8_t rxBuffer;

uint8_t status = 0;
uint8_t left = 0;
uint32_t time1 = 3100000;
uint32_t time2 = 3750000;

uint16_t Distance  = 0;  // cm
#define TRIG_PIN GPIO_PIN_10
#define TRIG_PORT GPIOB
#define ECHO_PIN GPIO_PIN_11
#define ECHO_PORT GPIOB

#define ENA_PIN GPIO_PIN_6
#define ENA_PORT GPIOA
#define ENB_PIN GPIO_PIN_7
#define ENA_PORT GPIOA

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int distance()
{
	uint32_t pMillis;
	uint32_t Value1 = 0;
	uint32_t Value2 = 0;

	HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_SET);  // pull the TRIG pin HIGH
	__HAL_TIM_SET_COUNTER(&htim1, 0);
	while (__HAL_TIM_GET_COUNTER (&htim1) < 10);  // wait for 10 us
	HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);  // pull the TRIG pin low

	pMillis = HAL_GetTick(); // used this to avoid infinite while loop  (for timeout)
	    // wait for the echo pin to go high
	while (!(HAL_GPIO_ReadPin (ECHO_PORT, ECHO_PIN)) && pMillis + 10 >  HAL_GetTick());
	Value1 = __HAL_TIM_GET_COUNTER (&htim1);

	pMillis = HAL_GetTick(); // used this to avoid infinite while loop (for timeout)
	    // wait for the echo pin to go low
	while ((HAL_GPIO_ReadPin (ECHO_PORT, ECHO_PIN)) && pMillis + 50 > HAL_GetTick());
	Value2 = __HAL_TIM_GET_COUNTER (&htim1);

	return (Value2-Value1)* 0.034/2;
	

}

void CHAY_THANG()
{
	Distance = distance();
	if(Distance > 20)
	{
	//HAL_GPIO_WritePin(GPIOB,IN3_Pin,GPIO_PIN_RESET); //PB13 va PB12 dung de xac dinh chieu dong co, PA8 thi bam xung
	//HAL_GPIO_WritePin(GPIOB,IN4_Pin,GPIO_PIN_SET);
		/*
	HAL_GPIO_WritePin(GPIOB,IN1_Pin|IN4_Pin,GPIO_PIN_SET); //PB15 va PB14 dung de xac dinh chieu dong co, PA2 thi bam xung
	HAL_GPIO_WritePin(GPIOB,IN2_Pin|IN3_Pin,GPIO_PIN_RESET);
	*/
  HAL_GPIO_WritePin(GPIOB,IN1_Pin,GPIO_PIN_SET); //PB15 va PB14 dung de xac dinh chieu dong co, PA2 thi bam xung
	HAL_GPIO_WritePin(GPIOB,IN2_Pin,GPIO_PIN_RESET);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, val_pls + 30 );   // banh trai
HAL_GPIO_WritePin(GPIOB,IN4_Pin,GPIO_PIN_SET); //PB15 va PB14 dung de xac dinh chieu dong co, PA2 thi bam xung
	HAL_GPIO_WritePin(GPIOB,IN3_Pin,GPIO_PIN_RESET);		
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, val_pls - 30); // banh phai	
	
	
	}
	else 
	{
	HAL_GPIO_WritePin(GPIOB,IN1_Pin|IN2_Pin|IN3_Pin|IN4_Pin,GPIO_PIN_RESET); //PB15 va PB14 dung de xac dinh chieu dong co, PA2 thi bam xung
	//HAL_GPIO_WritePin(GPIOB,IN2_Pin,GPIO_PIN_RESET);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
		
	//HAL_GPIO_WritePin(GPIOB,IN3_Pin,GPIO_PIN_RESET); //PB13 va PB12 dung de xac dinh chieu dong co, PA8 thi bam xung
	//HAL_GPIO_WritePin(GPIOB,IN4_Pin,GPIO_PIN_SET);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
	}
}

void DUNG()
{
	HAL_GPIO_WritePin(GPIOB,IN1_Pin|IN2_Pin|IN3_Pin|IN4_Pin,GPIO_PIN_RESET); //PB15 va PB14 dung de xac dinh chieu dong co, PA2 thi bam xung
	//HAL_GPIO_WritePin(GPIOB,IN2_Pin,GPIO_PIN_RESET);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
		
	//HAL_GPIO_WritePin(GPIOB,IN3_Pin|IN4_Pin,GPIO_PIN_RESET); //PB13 va PB12 dung de xac dinh chieu dong co, PA8 thi bam xung
	//HAL_GPIO_WritePin(GPIOB,IN4_Pin,GPIO_PIN_SET);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
}

void RE_TRAI()
{
	HAL_GPIO_WritePin(GPIOB,IN1_Pin,GPIO_PIN_RESET); //PB15 va PB14 dung de xac dinh chieu dong co, PA2 thi bam xung
	HAL_GPIO_WritePin(GPIOB,IN2_Pin,GPIO_PIN_RESET);
	
		
	HAL_GPIO_WritePin(GPIOB,IN3_Pin,GPIO_PIN_RESET); //PB13 va PB12 dung de xac dinh chieu dong co, PA8 thi bam xung
	HAL_GPIO_WritePin(GPIOB,IN4_Pin,GPIO_PIN_SET);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0); // banh xe ben trai
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, val_pls - 20); //banh xe ben phai
	
	//HAL_Delay(3000);
}

void RE_PHAI()
{
	HAL_GPIO_WritePin(GPIOB,IN1_Pin,GPIO_PIN_SET); //PB15 va PB14 dung de xac dinh chieu dong co, PA2 thi bam xung
	HAL_GPIO_WritePin(GPIOB,IN2_Pin,GPIO_PIN_RESET);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, val_pls - 20); //banh xe ben phai
		
	HAL_GPIO_WritePin(GPIOB,IN3_Pin,GPIO_PIN_RESET); //PB13 va PB12 dung de xac dinh chieu dong co, PA8 thi bam xung
	HAL_GPIO_WritePin(GPIOB,IN4_Pin,GPIO_PIN_RESET);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0); // banh xe ben trai
	
	//HAL_Delay(3000);
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
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start(&htim1);
	HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);  // pull the TRIG pin low
	
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	//khoi tao RFID
	//TM_MFRC522_Init();
	//ngat UART
	HAL_UART_Receive_IT(&huart1, &dulieu_nhan ,sizeof(dulieu_nhan));
	//HAL_UART_Receive_IT(&huart2, &angleZ ,sizeof(angleZ));
	//ngat SPI
	//HAL_SPI_Receive_IT(&hspi1,&rxBuffer,1);
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//		left = HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_4);
//		if(left == 1)
//		{
//			dem++;
//		}
		left = HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_5);
		if(status == 1)
			{
				CHAY_THANG();
				
								
			}
		else
			{
				DUNG();
			}
	
		
    /* USER CODE END WHILE */

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 7;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 799;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10|IN4_Pin|IN3_Pin|IN2_Pin
                          |IN1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 IN4_Pin IN3_Pin IN2_Pin
                           IN1_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_10|IN4_Pin|IN3_Pin|IN2_Pin
                          |IN1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB11 */
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	
	if(huart->Instance == huart1.Instance)
	{
			
		 if ((dulieu_nhan == 'A')|(dulieu_nhan == 'B')|(dulieu_nhan == 'C'))
			 { 						
					status = 1;						 
			 }
						 
				
		else if (dulieu_nhan == 'S')
				{
					 status = 0;
				}
			HAL_UART_Receive_IT(&huart1, &dulieu_nhan ,sizeof(dulieu_nhan));
	}
	/*
	if(huart->Instance == huart2.Instance)
		{
			Z = angleZ;
			if (Z > 20)
			{
				HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
			}
			HAL_UART_Receive_IT(&huart2, &angleZ ,sizeof(angleZ));
		}
		*/
}

// Cam bien quang tro khi co vat che anh sang lai se la muc 1, có anh sang la muc 0


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
		if (GPIO_Pin == GPIO_PIN_5)
			{
				dem++;
				switch (dem)
				{
						case 2:
							//for(int i1 = 0; i1 < time;i1++);
							RE_TRAI();
							HAL_UART_Transmit(&huart1, dulieu_gui, sizeof(dulieu_gui), 10);
							//HAL_Delay(3000);
							for(int i1 = 0; i1 < time1;i1++);
							break;
						case 3:
							//for(int i2 = 0; i2 < time;i2++);
							RE_TRAI();
							for(int i2 = 0; i2 < time1;i2++);
							break;
						/*
						case 4:
							if (dulieu_nhan == 'C')
							{
								dem = 8;
							}
							else break;
						*/
						case 5:
							if (dulieu_nhan == 'B')
							{
								dem = 8;
							}						
								else break;
						case 6:
							if (dulieu_nhan == 'A')
							{
								dem =8;
							}						
								else break;
						case 8:
							
							DUNG();
							
							for(int i3 = 0; i3 < time1;i3++);
							HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
							RE_TRAI();
							for(int i3 = 0; i3 < 2*time1; i3++);
							break;
						case 9:
							//for(int i4 = 0; i4 < time;i4++);
							HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
							RE_PHAI();
							for(int i4 = 0; i4 < time2;i4++);
							break; 
						case 10:
							//for(int i5 = 0; i5 < time;i5++);
							HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
							RE_PHAI();
							for(int i5 = 0; i5 < time2;i5++);
							break;
						case 11:
							DUNG();
							for(int i6 = 0; i6 < 2*time1;i6++);
							break;
						case 12:
							
							RE_TRAI();
							for(int i3 = 0; i3 < 2*time1; i3++);
							break;

							
					}
				//for(int i; i < 2000000;i++);
				//left = 1;
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
