/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include "stdio.h"
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
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;


/* USER CODE BEGIN PV */

#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

void __io_putchar(uint8_t ch) {
HAL_UART_Transmit(&huart2, &ch, 1, 1);
}


typedef struct{
	float omega_x;
	float omega_y;
	float omega_z;
	float accel_x;
	float accel_y;
	float accel_z;
}GYRO_DATA;

GYRO_DATA gyro_raw;
GYRO_DATA gyro_true;
GYRO_DATA gyro_offset;
uint8_t set_flag;
uint16_t gyro_calib_cnt;
uint8_t gyro_calib_flag;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */
void ICM20689_Init();

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
	setbuf(stdout, NULL);

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
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  ICM20689_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  ICM20689_DataUpdate();
	   printf("omegax=%f,y:%f,z=%f\r\n",gyro_raw.omega_x,gyro_raw.omega_y, gyro_raw.omega_z);
	   HAL_Delay(300);
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
  /** Initializes the CPU, AHB and APB busses clocks 
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
  /** Initializes the CPU, AHB and APB busses clocks 
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
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
uint8_t read_byte( uint8_t reg )
{
	uint8_t ret,val;
	HAL_GPIO_WritePin( GPIOB, GPIO_PIN_6, GPIO_PIN_RESET); //cs = 0;
	ret = reg | 0x80;
	HAL_SPI_Transmit(&hspi1, &ret,1,100);
	HAL_SPI_Receive(&hspi1,&val,1,100);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET); //cs = 1;
	return val;
}

//+++++++++++++++++++++++++++++++++++++++++++++++
//write_byte関数
/*SPI Operational Features
1.Data is delivered MSB first and LSB last
2.Data is latched on the rising edge of SCLK
3.Data should be transitioned on the falling edge of SPC
4.The maximum frequency of SPC is 10MHz
5.SPI read and write operations are completed in 16 or more clock cycles(two or more bytes.)
The first byte conains the SPI Adress
The following bytes contain the SPI data
The first bit of the first byte contains the Read/Write bit and indicates the Read(1) or Write(0) operation.
The following 7 bits is the Resister Address.
*/
//+++++++++++++++++++++++++++++++++++++++++++++++

void write_byte( uint8_t reg, uint8_t val )
{
	uint8_t ret;
	ret = reg & 0x7F ;
	HAL_GPIO_WritePin( GPIOB, GPIO_PIN_6, GPIO_PIN_RESET ); //cs = 0;
	HAL_SPI_Transmit( &hspi1, &ret,1,100 );
	HAL_SPI_Transmit( &hspi1, &val,1,100 );
	HAL_GPIO_WritePin( GPIOB, GPIO_PIN_6, GPIO_PIN_SET ); //cs = 1;
}

//+++++++++++++++++++++++++++++++++++++++++++++++
//ICM20602_Init
//+++++++++++++++++++++++++++++++++++++++++++++++

void ICM20689_Init( void )
{
	uint8_t who_am_i = 0;
	who_am_i = read_byte(0x75);	// check WHO_AM_I (0x75)
	printf( "who_am_i = 0x%x\r\n",who_am_i ); 	// Who am I = 0x98

	if (who_am_i != 0x98){	// recheck 0x98
		HAL_Delay(100);
		who_am_i = read_byte(0x98);

		if (who_am_i != 0x98){
			printf( "gyro_error\r\n\n");
			while(1){
				}

		}
	}

 //PWR_MIGHT_1 0x6B
	write_byte( 0x6B, 0x00 );	//Set pwr might
	HAL_Delay(50);
 //PWR_MIGHT_2 0x6C
	write_byte( 0x6C, 0x00 );
	HAL_Delay(50);
 //set gyro config
 //GYRO_CONFIG 0x1B
	write_byte( 0x1B, 0x18 ); // use 2000 dps
	HAL_Delay(50);
 //ACCEL_CONFIG 0x1C
	write_byte( 0x1B, 0x18 ); // use pm 16g
	HAL_Delay(50);

	 set_flag = 1;
}


float ICM20689_GYRO_READ( uint8_t H_reg )
{
	int16_t data = (int16_t)( ((uint8_t)read_byte(H_reg) << 8) | (uint8_t)read_byte(H_reg+1) );
	float omega = (float)(data / 16.4f); //[deg/s] FS_SEL=3-> Scale Factor=16.4[LSB/(dps)]
	return omega;
}


float ICM20689_ACCEL_READ( uint8_t H_reg )
{
	int16_t data = (int16_t)( ((uint8_t)read_byte(H_reg) << 8) | (uint8_t)read_byte(H_reg+1) );
	float accel = (float)(data / 2048.0f);
	return accel;
}

void ICM20689_DataUpdate(void)
{

	if ( set_flag == 1 ){
	// get yawrate
	gyro_raw.omega_x = -1 * ICM20689_GYRO_READ( 0x43 );
	gyro_raw.omega_y = ICM20689_GYRO_READ( 0x45 );
	gyro_raw.omega_z = ICM20689_GYRO_READ( 0x47 );

	// get accel

	gyro_raw.accel_x = -1 * ICM20689_ACCEL_READ( 0x3B );
	gyro_raw.accel_y = ICM20689_ACCEL_READ( 0x3D );
	gyro_raw.accel_z = ICM20689_ACCEL_READ( 0x3F );

	//True Value(Consider Offset)
	gyro_true.omega_x = gyro_raw.omega_x - gyro_offset.omega_x;
	gyro_true.omega_y = gyro_raw.omega_y - gyro_offset.omega_y;
	gyro_true.omega_z = gyro_raw.omega_z - gyro_offset.omega_z;
	gyro_true.accel_x = gyro_raw.accel_x - gyro_offset.accel_x;
	gyro_true.accel_y = gyro_raw.accel_y - gyro_offset.accel_y;
	gyro_true.accel_z = gyro_raw.accel_z - gyro_offset.accel_z;


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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
