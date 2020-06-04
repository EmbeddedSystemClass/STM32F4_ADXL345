/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "ADXL.h"
#include "arm_const_structs.h"
#include "arm_math.h"
#include "Calculate_statistic.h"
#include "BLE_USART.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define dataLength 4096
#define FFT_Frequency_Scale 3200/(dataLength/2)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
IWDG_HandleTypeDef hiwdg;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */

typedef struct FFTInstance
{
	float32_t bufferforFFT[dataLength];
	float32_t bufferforTimeSV[dataLength/2];
	uint16_t data0;
	uint16_t data1;
	int16_t acceleration;
	float32_t accelerationfloat;

}fftInstance;

fftInstance XfftInstance;
fftInstance YfftInstance;
fftInstance ZfftInstance;

uint8_t ADXLid = 0;

uint8_t data[6]={0,0,0,0,0,0};

GPIO_PinState pin3status;

int16_t sampleIndex = 0;
int16_t sampleCount = 0;
int16_t whilesampleCount = 0;

uint8_t dataReady = 0;

uint32_t ifftFlag = 0;
uint32_t doBitReverse = 1;
float32_t testOutput[dataLength/2];
int32_t fftSize = 2048;
float32_t maxValue = 0;
uint32_t testIndex = 0;


float32_t *ZstatisticDataSet = ZfftInstance.bufferforTimeSV;
float32_t *XstatisticDataSet = XfftInstance.bufferforTimeSV;
float32_t *YstatisticDataSet = YfftInstance.bufferforTimeSV;


// Bluetooth structure
USART_BLE USARTBLE;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_IWDG_Init(void);
/* USER CODE BEGIN PFP */

typedef enum {true,false} dataReadyFlag;
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
  MX_SPI1_Init();
  MX_USART6_UART_Init();
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */
  delay_init(168);


  ADXL_InitTypeDef adxl_config;
  adxl_config.Range = RANGE_16G;
  adxl_config.Resolution = RESOLUTION_FULL;
  adxl_config.AutoSleep = AUTOSLEEPOFF;
  adxl_config.LPMode = LPMODE_LOWPOWER;
  adxl_config.Rate = BWRATE_3200;
  adxl_config.SPIMode = SPIMODE_4WIRE;
  adxl_config.IntMode = INT_ACTIVEHIGH;

  adxlStatus InitStatus = ADXL_Init(&adxl_config, &ADXLid);
  if(ADXLid == 0xE5)InitStatus = ADXL_OK;
  if(InitStatus == ADXL_OK)
  {
	  __NOP();

  }

  ADXL_Measure(ON);
  writeRegister(INT_MAP, 0x10);
  writeRegister(INT_ENABLE, 0x80);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    ADXL_IntProto();
    readRegister(DATA0,data,6);
    if(dataReady == true)
    {

    	readRegister(DATA0,data,6);
    	XfftInstance.data0 = data[0];
    	XfftInstance.data1 = data[1];
    	YfftInstance.data0 = data[2];
    	YfftInstance.data1 = data[3];
    	ZfftInstance.data0 = data[4];
    	ZfftInstance.data1 = data[5];
    	XfftInstance.acceleration = ((uint8_t)XfftInstance.data1<<8)|(uint8_t)XfftInstance.data0;
    	YfftInstance.acceleration = ((uint8_t)YfftInstance.data1<<8)|(uint8_t)YfftInstance.data0;
    	ZfftInstance.acceleration = ((uint8_t)ZfftInstance.data1<<8)|(uint8_t)ZfftInstance.data0;

    	//3.9 is scale of LSB(one bit) mg, 1000 is scale to g
    	XfftInstance.accelerationfloat = (float)XfftInstance.acceleration * 3.9 / 1000;
    	YfftInstance.accelerationfloat = (float)YfftInstance.acceleration  * 3.9 / 1000;
    	ZfftInstance.accelerationfloat = (float)ZfftInstance.acceleration  * 3.9 / 1000;

    	// move X axis data to buffer
    	XfftInstance.bufferforFFT[sampleIndex * 2] = XfftInstance.accelerationfloat;
    	XfftInstance.bufferforFFT[sampleIndex * 2+1] = 0;
    	XfftInstance.bufferforTimeSV[sampleIndex] = XfftInstance.accelerationfloat;

    	// move Y axis data to buffer
    	YfftInstance.bufferforFFT[sampleIndex * 2] = YfftInstance.accelerationfloat;
    	YfftInstance.bufferforFFT[sampleIndex * 2+1] = 0;
    	YfftInstance.bufferforTimeSV[sampleIndex] = YfftInstance.accelerationfloat;

    	// move Z axis data to buffer
    	ZfftInstance.bufferforFFT[sampleIndex * 2] = ZfftInstance.accelerationfloat;
    	ZfftInstance.bufferforFFT[sampleIndex * 2+1] = 0;
    	ZfftInstance.bufferforTimeSV[sampleIndex] = ZfftInstance.accelerationfloat;


		sampleIndex++;
    	if(sampleIndex == dataLength/2)
		{
    		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
    		//Calculate Z axis statistic
    		Calculate_FFT_RMS(ZfftInstance.bufferforFFT, testOutput, fftSize, &Zstatistic_value);
			Calculate_All_statisitc(ZstatisticDataSet, dataLength/2, &Zstatistic_value);

			//Calculate X axis statistic
    		Calculate_FFT_RMS(XfftInstance.bufferforFFT, testOutput, fftSize, &Xstatistic_value);
			Calculate_All_statisitc(XstatisticDataSet, dataLength/2, &Xstatistic_value);

			//Calculate Y axis statistic
    		Calculate_FFT_RMS(YfftInstance.bufferforFFT , testOutput, fftSize, &Ystatistic_value);
			Calculate_All_statisitc(YstatisticDataSet, dataLength/2, &Ystatistic_value);

			//Data transmission by uart
    		sampleIndex = 0;
    		USARTBLE.sendflag =1;
    		BLE_USART(&huart6, &Zstatistic_value);
    		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
    		HAL_IWDG_Refresh(&hiwdg);
		}
    	dataReady = false;

    }

    //__NOP();
	  //ADXL_getAccel(data, OUTPUT_FLOAT);
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
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_128;
  hiwdg.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

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
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
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
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3|GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA3 PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PD12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{

	if(GPIO_Pin == GPIO_PIN_2)
	{

		dataReady = true;
		sampleCount++;
		if(sampleCount > 2048)
		{
			sampleCount = 0;
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
