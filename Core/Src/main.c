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
SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */
uint8_t ADXLid = 0;
uint16_t Zdata0 = 0;
uint16_t Zdata1 = 0;
uint16_t Xdata0 = 0;
uint16_t Xdata1 = 0;
uint16_t Ydata0 = 0;
uint16_t Ydata1 = 0;

int16_t  accX = 0;
float32_t accXfloat = 0;
int16_t  accY = 0;
float32_t accYfloat = 0;
int16_t  accZ = 0;
float32_t accZfloat = 0;

uint8_t data[6]={0,0,0,0,0,0};

float32_t ZbufferforFFT[dataLength];
float32_t ZbufferforTimeSV[dataLength/2];
float32_t XbufferforFFT[dataLength];
float32_t XbufferforTimeSV[dataLength/2];
float32_t YbufferforFFT[dataLength];
float32_t YbufferforTimeSV[dataLength/2];

float32_t ZScale = 1/31.0f;

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


float32_t *ZstatisticDataSet = ZbufferforTimeSV;
float32_t *XstatisticDataSet = XbufferforTimeSV;
float32_t *YstatisticDataSet = YbufferforTimeSV;




float32_t Statistic_max = 0;
uint32_t maxtestIndex = 0;
float32_t Statistic_min = 0;
uint32_t mintestIndex = 0;
float32_t Statistic_rms = 0;
float32_t Statistic_FreqOvall = 0;
float32_t Statistic_p2p = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
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
    	Xdata0 = data[0];
    	Xdata1 = data[1];
    	Ydata0 = data[2];
    	Ydata1 = data[3];
    	Zdata0 = data[4];
    	Zdata1 = data[5];
    	accX = ((uint8_t)Xdata1<<8)|(uint8_t)Xdata0;
    	accY = ((uint8_t)Ydata1<<8)|(uint8_t)Ydata0;
    	accZ = ((uint8_t)Zdata1<<8)|(uint8_t)Zdata0;

    	//3.9 is scale of LSB(one bit) mg, 1000 is scale to g
    	accXfloat = (float)accX  * 3.9 / 1000;
    	accYfloat = (float)accY  * 3.9 / 1000;
    	accZfloat = (float)accZ  * 3.9 / 1000;

    	// move X axis data to buffer
    	XbufferforFFT[sampleIndex * 2] = accXfloat;
    	XbufferforFFT[sampleIndex * 2+1] = 0;
    	XbufferforTimeSV[sampleIndex] = accXfloat;

    	// move Y axis data to buffer
    	YbufferforFFT[sampleIndex * 2] = accYfloat;
    	YbufferforFFT[sampleIndex * 2+1] = 0;
    	YbufferforTimeSV[sampleIndex] = accYfloat;

    	// move Z axis data to buffer
    	ZbufferforFFT[sampleIndex * 2] = accZfloat;
    	ZbufferforFFT[sampleIndex * 2+1] = 0;
    	ZbufferforTimeSV[sampleIndex] = accZfloat;




		sampleIndex++;
    	if(sampleIndex == dataLength/2)
		{

    		//Calculate Z axis statistic
    		Calculate_FFT_RMS(ZbufferforFFT, testOutput, fftSize, &Zstatistic_value);
			Calculate_All_statisitc(ZstatisticDataSet, dataLength/2, &Zstatistic_value);

			//Calculate X axis statistic
    		Calculate_FFT_RMS(XbufferforFFT, testOutput, fftSize, &Xstatistic_value);
			Calculate_All_statisitc(XstatisticDataSet, dataLength/2, &Xstatistic_value);

			//Calculate Y axis statistic
    		Calculate_FFT_RMS(YbufferforFFT, testOutput, fftSize, &Ystatistic_value);
			Calculate_All_statisitc(YstatisticDataSet, dataLength/2, &Ystatistic_value);





			//Data transmission by uart
    		sampleIndex = 0;
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3|GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA3 PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
