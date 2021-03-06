/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "symbols.h"
#include "stdlib.h"
#include "string.h"
#include "ctype.h" //for toupper()
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RCLK_PIN	GPIOB, GPIO_PIN_6	//CS PIN
#define ROWS 5
#define COLS 2
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t buffer[3] = " \r\n";
uint8_t led_buffer[5][2];
uint8_t col_index = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void setString(uint8_t *str, uint16_t length);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	HAL_UART_Transmit_IT(huart, buffer, sizeof(buffer));
	/*
	 * If you want to receive the bytes from a transmitting board, select &huart1
	 * If you want to receive the bytes from putty, select &huart2
	 * Remember to change the parameter also in the main loop
	 */
	HAL_UART_Receive_IT(&huart2, buffer, sizeof(buffer[0]));

	setString(buffer, sizeof(buffer[0]));
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim == &htim2){
		HAL_SPI_Transmit_IT(&hspi1, led_buffer[col_index], 2);
	}
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi){
	if(hspi == &hspi1){
		HAL_GPIO_WritePin(RCLK_PIN, GPIO_PIN_SET);
		if (++col_index > 4) {
			col_index = 0;
		}
		HAL_GPIO_WritePin(RCLK_PIN, GPIO_PIN_RESET);
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
  HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_2);
  HAL_NVIC_SetPriority(SPI1_IRQn, 1, 1);
  HAL_NVIC_SetPriority(USART2_IRQn, 1, 1);
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim2);
	/*
	 * If you want to receive the bytes from a transmitting board, select &huart1
	 * If you want to receive the bytes from putty, select &huart2
	 * Remember to change the parameter also in HAL_UART_RxCpltCallback
	 */
  HAL_UART_Receive_IT(&huart2, buffer, sizeof(buffer[0]));
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
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
  htim2.Init.Prescaler = 840 - 1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 100 - 1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  huart1.Init.BaudRate = 2400;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_RX;
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}


/* USER CODE BEGIN 4 */
void setString(uint8_t *str, uint16_t length){
	for (uint16_t i = 0; i < length; i++){
		switch (toupper(str[i])){
		case 'A':
			memcpy(led_buffer, A, ROWS*COLS*sizeof(uint8_t));
			break;
		case 'B':
			memcpy(led_buffer, B, ROWS*COLS*sizeof(uint8_t));
			break;
		case 'C':
			memcpy(led_buffer, C, ROWS*COLS*sizeof(uint8_t));
			break;
		case 'D':
			memcpy(led_buffer, D, ROWS*COLS*sizeof(uint8_t));
			break;
		case 'E':
			memcpy(led_buffer, E, ROWS*COLS*sizeof(uint8_t));
			break;
		case 'F':
			memcpy(led_buffer, F, ROWS*COLS*sizeof(uint8_t));
			break;
		case 'G':
			memcpy(led_buffer, G, ROWS*COLS*sizeof(uint8_t));
			break;
		case 'H':
			memcpy(led_buffer, H, ROWS*COLS*sizeof(uint8_t));
			break;
		case 'I':
			memcpy(led_buffer, I, ROWS*COLS*sizeof(uint8_t));
			break;
		case 'J':
			memcpy(led_buffer, J, ROWS*COLS*sizeof(uint8_t));
			break;
		case 'K':
			memcpy(led_buffer, K, ROWS*COLS*sizeof(uint8_t));
			break;
		case 'L':
			memcpy(led_buffer, L, ROWS*COLS*sizeof(uint8_t));
			break;
		case 'M':
			memcpy(led_buffer, M, ROWS*COLS*sizeof(uint8_t));
			break;
		case 'N':
			memcpy(led_buffer, N, ROWS*COLS*sizeof(uint8_t));
			break;
		case 'O':
			memcpy(led_buffer, O, ROWS*COLS*sizeof(uint8_t));
			break;
		case 'P':
			memcpy(led_buffer, P, ROWS*COLS*sizeof(uint8_t));
			break;
		case 'Q':
			memcpy(led_buffer, Q, ROWS*COLS*sizeof(uint8_t));
			break;
		case 'R':
			memcpy(led_buffer, R, ROWS*COLS*sizeof(uint8_t));
			break;
		case 'S':
			memcpy(led_buffer, S, ROWS*COLS*sizeof(uint8_t));
			break;
		case 'T':
			memcpy(led_buffer, T, ROWS*COLS*sizeof(uint8_t));
			break;
		case 'U':
			memcpy(led_buffer, U, ROWS*COLS*sizeof(uint8_t));
			break;
		case 'V':
			memcpy(led_buffer, V, ROWS*COLS*sizeof(uint8_t));
			break;
		case 'W':
			memcpy(led_buffer, W, ROWS*COLS*sizeof(uint8_t));
			break;
		case 'X':
			memcpy(led_buffer, X, ROWS*COLS*sizeof(uint8_t));
			break;
		case 'Y':
			memcpy(led_buffer, Y, ROWS*COLS*sizeof(uint8_t));
			break;
		case 'Z':
			memcpy(led_buffer, Z, ROWS*COLS*sizeof(uint8_t));
			break;
		case '0':
			memcpy(led_buffer, ZERO, ROWS*COLS*sizeof(uint8_t));
			break;
		case '1':
			memcpy(led_buffer, ONE, ROWS*COLS*sizeof(uint8_t));
			break;
		case '2':
			memcpy(led_buffer, TWO, ROWS*COLS*sizeof(uint8_t));
			break;
		case '3':
			memcpy(led_buffer, THREE, ROWS*COLS*sizeof(uint8_t));
			break;
		case '4':
			memcpy(led_buffer, FOUR, ROWS*COLS*sizeof(uint8_t));
			break;
		case '5':
			memcpy(led_buffer, FIVE, ROWS*COLS*sizeof(uint8_t));
			break;
		case '6':
			memcpy(led_buffer, SIX, ROWS*COLS*sizeof(uint8_t));
			break;
		case '7':
			memcpy(led_buffer, SEVEN, ROWS*COLS*sizeof(uint8_t));
			break;
		case '8':
			memcpy(led_buffer, EIGHT, ROWS*COLS*sizeof(uint8_t));
			break;
		case '9':
			memcpy(led_buffer, NINE, ROWS*COLS*sizeof(uint8_t));
			break;
		case ' ':
			memcpy(led_buffer, BLANK, ROWS*COLS*sizeof(uint8_t));
			break;
		default:
			memcpy(led_buffer, BLANK, ROWS*COLS*sizeof(uint8_t));
			break;
		}
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
