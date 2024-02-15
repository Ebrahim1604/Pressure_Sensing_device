/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// Pin aliases
#define Zone_one_switch GPIO_PIN_0
#define Zone_one_RED_LED GPIO_PIN_3
#define Zone_one_GREEN_LED GPIO_PIN_6
#define Zone_one_sensor GPIO_PIN_7

#define Zone_two_switch GPIO_PIN_1
#define Zone_two_RED_LED GPIO_PIN_8
#define Zone_two_GREEN_LED GPIO_PIN_9
#define Zone_two_sensor GPIO_PIN_5

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef hlpuart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_LPUART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_LPUART1_UART_Init();
  /* USER CODE BEGIN 2 */

  char buff1[100];
  char buff2[100];
  char buff3[100];
  char buff4[100];

  sprintf(buff1,"Zone 1 Input Mode off!\n");
  sprintf(buff2,"Zone 1 Input Mode ON\n");
  sprintf(buff3,"Zone 1 signal detected....\n");
  sprintf(buff4,"No Zone 1 signal detected\n");

  char buff5[100];
  char buff6[100];
  char buff7[100];
  char buff8[100];

  sprintf(buff1,"Zone 2 Input Mode off!\n");
  sprintf(buff2,"Zone 2 Input Mode ON\n");
  sprintf(buff3,"Zone 2 signal detected....\n");
  sprintf(buff4,"No Zone 2 signal detected\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	  if (HAL_GPIO_ReadPin(GPIOA, Zone_one_switch) == GPIO_PIN_RESET) // If PA0 (switch) is low
	  	  {
	  		  HAL_UART_Transmit(&hlpuart1, (uint8_t *)buff2, strlen(buff1), HAL_MAX_DELAY);
	  		  HAL_Delay(10);

	  		  if(HAL_GPIO_ReadPin(GPIOC, Zone_one_sensor) == GPIO_PIN_SET) //PC7 == 1
	  		  {
	  			  HAL_UART_Transmit(&hlpuart1, (uint8_t *)buff3, strlen(buff1), HAL_MAX_DELAY);
	  			  HAL_GPIO_WritePin(GPIOC, Zone_one_GREEN_LED, GPIO_PIN_SET); // Turn on green LED
	  			  HAL_GPIO_WritePin(GPIOC, Zone_one_RED_LED, GPIO_PIN_RESET); // Turn off red LED
	  		  }

	  		  else
	  		  {
	  			  HAL_UART_Transmit(&hlpuart1, (uint8_t *)buff4, strlen(buff1), HAL_MAX_DELAY);
	  			  HAL_GPIO_WritePin(GPIOC, Zone_one_GREEN_LED, GPIO_PIN_RESET); // Turn off green LED
	  			  HAL_GPIO_WritePin(GPIOC, Zone_one_RED_LED, GPIO_PIN_SET); // Turn on red LED
	  		  }

	  	  }

	  	  else // If PA0 is high
	  	  {
	  		  	  HAL_UART_Transmit(&hlpuart1, (uint8_t *)buff1, strlen(buff1), HAL_MAX_DELAY);
	  			  HAL_GPIO_WritePin(GPIOC, Zone_one_GREEN_LED, GPIO_PIN_RESET); // Turn off red LED
	  			  HAL_GPIO_WritePin(GPIOC, Zone_one_RED_LED, GPIO_PIN_RESET); // Turn off green LED
	  	          }

		  HAL_Delay(10); // Adjust delay as needed

	  if (HAL_GPIO_ReadPin(GPIOA, Zone_two_switch) == GPIO_PIN_RESET) // If PA1 (switch) is low
		  {
			  HAL_UART_Transmit(&hlpuart1, (uint8_t *)buff6, strlen(buff1), HAL_MAX_DELAY);
			  HAL_Delay(10);

			  if(HAL_GPIO_ReadPin(GPIOC, Zone_two_sensor) == GPIO_PIN_SET) //PC7 == 1
			  {
				  HAL_UART_Transmit(&hlpuart1, (uint8_t *)buff7, strlen(buff1), HAL_MAX_DELAY);
				  HAL_GPIO_WritePin(GPIOC, Zone_two_GREEN_LED, GPIO_PIN_SET); // Turn on green LED
				  HAL_GPIO_WritePin(GPIOC, Zone_two_RED_LED, GPIO_PIN_RESET); // Turn off red LED
			  }

			  else
			  {
				  HAL_UART_Transmit(&hlpuart1, (uint8_t *)buff8, strlen(buff1), HAL_MAX_DELAY);
				  HAL_GPIO_WritePin(GPIOC, Zone_two_GREEN_LED, GPIO_PIN_RESET); // Turn off green LED
				  HAL_GPIO_WritePin(GPIOC, Zone_two_RED_LED, GPIO_PIN_SET); // Turn on red LED
			  }

		  }

		  else // If PA0 is high
		  {
				  HAL_UART_Transmit(&hlpuart1, (uint8_t *)buff5, strlen(buff1), HAL_MAX_DELAY);
				  HAL_GPIO_WritePin(GPIOC, Zone_two_GREEN_LED, GPIO_PIN_RESET); // Turn off red LED
				  HAL_GPIO_WritePin(GPIOC, Zone_two_RED_LED, GPIO_PIN_RESET); // Turn off green LED
				  }

		  HAL_Delay(10); // Adjust delay as needed



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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 115200;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3|GPIO_PIN_6|GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PC3 PC6 PC8 PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_6|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PC5 PC7 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
