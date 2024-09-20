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
#include <string.h>		//for strlen and string_copy
#include <stdio.h>		//for sprintf

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
I2C_HandleTypeDef hi2c3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
static const uint8_t TSL2561_ADDR_W = 0x72; //0x39 << 1 to write shift by 1 and add 0
static const uint8_t TSL2561_ADDR_R = 0x73; //(0x39 << 1) | 0x01 to read shift by 1 and add 1

uint8_t buffer[200]; //buffer for sending and receiving data

uint16_t read_value_TSL2561_ch0_both_A;	//raw value, both infrared and visible
uint16_t read_value_TSL2561_ch1_infraRed_A;	//raw value, just infrared
uint8_t buffer_get_ch0_A[2];	//buffer for  both infrared and visible reading
uint8_t buffer_get_ch1_A[2];	//buffer for just infrared reading
uint8_t id_A;

uint16_t read_value_TSL2561_ch0_both_B;	//raw value, both infrared and visible
uint16_t read_value_TSL2561_ch1_infraRed_B;	//raw value, just infrared
uint8_t buffer_get_ch0_B[2];	//buffer for  both infrared and visible reading
uint8_t buffer_get_ch1_B[2];	//buffer for just infrared reading
uint8_t id_B;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C3_Init(void);
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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_I2C3_Init();
  /* USER CODE BEGIN 2 */

  //to powered up --> manual 14 page " Control Register"

  	uint8_t resetControlRegister = 0x03;
//power up sensor A
  	HAL_I2C_Mem_Write(&hi2c1, TSL2561_ADDR_W, 0x80, 1, &resetControlRegister, 1,
  			HAL_MAX_DELAY);
  	HAL_Delay(500);
  	//read sensor ID A
  	HAL_I2C_Mem_Read(&hi2c1, TSL2561_ADDR_R, 0x8A, 1, &id_A, 1, HAL_MAX_DELAY);
  	sprintf((char*) buffer, "ID A%d  \r\n", (id_A));
  	HAL_UART_Transmit(&huart2, buffer, strlen((const char*) buffer),
  			HAL_MAX_DELAY);


  	//power up sensor B
  	  	HAL_I2C_Mem_Write(&hi2c3, TSL2561_ADDR_W, 0x80, 1, &resetControlRegister, 1,
  	  			HAL_MAX_DELAY);
  	  	HAL_Delay(500);
  	  	//read sensor ID B
  	  	HAL_I2C_Mem_Read(&hi2c3, TSL2561_ADDR_R, 0x8A, 1, &id_B, 1, HAL_MAX_DELAY);

  	  	sprintf((char*) buffer, "ID B%d  \r\n", (id_B));
  	    	  	HAL_UART_Transmit(&huart2, buffer, strlen((const char*) buffer),
  	  			HAL_MAX_DELAY);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	  /*After applying VDD, the device will initially be in the power-down state. To operate the device, issue a command
	  	  			 to access the CONTROL register followed by the data value 03h to power up the device. At this point, both ADC
	  	  			 channels will begin a conversion at the default integration time of 400 ms. After 400 ms, the conversion results
	  	  			 will be available in the DATA0 and DATA1 registers.*/
	  	  			//we do it before super loop
	  	  			/*One of the photodiodes 		(channel 0) is sensitive to both visible and infrared light, while the second
	  	  			 * photodiode (channel 1) is sensitive
	  	  			 primarily to infrared light.*/

	  	  			/* to calculate 'lux' there are formula in datasheet*/

	  	  			// Read 2 bytes from the channel 0 sensor A
	  	  			HAL_I2C_Mem_Read(&hi2c1, TSL2561_ADDR_R, 0xEC, 1, &buffer_get_ch0_A[0], 1,
	  	  					HAL_MAX_DELAY);
	  	  			HAL_Delay(70);
	  	  			HAL_I2C_Mem_Read(&hi2c1, TSL2561_ADDR_R, 0xED, 1, &buffer_get_ch0_A[1], 1,
	  	  					HAL_MAX_DELAY);
	  	  			HAL_Delay(70);
	  	  			// Read 2 bytes from the channel 1
	  	  			HAL_I2C_Mem_Read(&hi2c1, TSL2561_ADDR_R, 0xEE, 1, &buffer_get_ch1_A[0], 1,
	  	  					HAL_MAX_DELAY);
	  	  			HAL_Delay(70);
	  	  			HAL_I2C_Mem_Read(&hi2c1, TSL2561_ADDR_R, 0xEF, 1, &buffer_get_ch1_A[1], 1,
	  	  					HAL_MAX_DELAY);
	  	  			HAL_Delay(70);

	  	  			//Combine the bytes sensor A
	  	  			read_value_TSL2561_ch0_both_A = 256 * buffer_get_ch0_A[1]
	  	  					+ buffer_get_ch0_A[0];
	  	  			read_value_TSL2561_ch1_infraRed_A = 256 * buffer_get_ch1_A[1]
	  	  					+ buffer_get_ch1_A[0];

	  	  			//---------------------------------------
	  	  			// Read 2 bytes from the channel 0 sensor B
	  		  	  			HAL_I2C_Mem_Read(&hi2c3, TSL2561_ADDR_R, 0xEC, 1, &buffer_get_ch0_B[0], 1,
	  		  	  					HAL_MAX_DELAY);
	  		  	  			HAL_Delay(70);
	  		  	  			HAL_I2C_Mem_Read(&hi2c3, TSL2561_ADDR_R, 0xED, 1, &buffer_get_ch0_B[1], 1,
	  		  	  					HAL_MAX_DELAY);
	  		  	  			HAL_Delay(70);
	  		  	  			// Read 2 bytes from the channel 1
	  		  	  			HAL_I2C_Mem_Read(&hi2c3, TSL2561_ADDR_R, 0xEE, 1, &buffer_get_ch1_B[0], 1,
	  		  	  					HAL_MAX_DELAY);
	  		  	  			HAL_Delay(70);
	  		  	  			HAL_I2C_Mem_Read(&hi2c3, TSL2561_ADDR_R, 0xEF, 1, &buffer_get_ch1_B[1], 1,
	  		  	  					HAL_MAX_DELAY);
	  		  	  			HAL_Delay(70);

	  		  	  			//Combine the bytes sensor A
	  		  	  			read_value_TSL2561_ch0_both_B = 256 * buffer_get_ch0_B[1]
	  		  	  					+ buffer_get_ch0_B[0];
	  		  	  			read_value_TSL2561_ch1_infraRed_B = 256 * buffer_get_ch1_B[1]
	  		  	  					+ buffer_get_ch1_B[0];
	  		  	  		//Show sensors
	  		  	  			  		  	  			  	  			sprintf((char*) buffer, "Visible light A     %d  \t",
	  		  	  			  		  	  			  	  					(read_value_TSL2561_ch0_both_A));
	  		  	  			  		  	  			  	  			HAL_UART_Transmit(&huart2, buffer, strlen((const char*) buffer),
	  		  	  			  		  	  			  	  					HAL_MAX_DELAY);
	  		  	  			sprintf((char*) buffer, " %d  Visible light B\r\n",
	  		  	  					(read_value_TSL2561_ch0_both_B));
	  		  	  			HAL_UART_Transmit(&huart2, buffer, strlen((const char*) buffer),
	  		  	  					HAL_MAX_DELAY);

	  		  	  		sprintf((char*) buffer, "Infra red A         %d          ",
	  		  	  			  		  	  			  	  					(read_value_TSL2561_ch1_infraRed_A));
	  		  	  			  		  	  			  	  			HAL_UART_Transmit(&huart2, buffer, strlen((const char*) buffer),
	  		  	  			  		  	  			  	  					HAL_MAX_DELAY);


	  		  	  			sprintf((char*) buffer, " %d   Infra red B\r\n\n",
	  		  	  					(read_value_TSL2561_ch1_infraRed_B));
	  		  	  			HAL_UART_Transmit(&huart2, buffer, strlen((const char*) buffer),
	  		  	  					HAL_MAX_DELAY);





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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

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
  hi2c1.Init.ClockSpeed = 400000;
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
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 400000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

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
