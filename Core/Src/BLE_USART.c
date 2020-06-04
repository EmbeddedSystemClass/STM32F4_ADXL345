/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "string.h"
#include "math.h"
#include "BLE_USART.h"
#include <stdint.h>
#include "printf.h"
#include "Calculate_statistic.h"

/* Private typedef -----------------------------------------------------------*/
extern USART_BLE USARTBLE;	//Wayne0905
extern Sv Xstatistic_value;
extern Sv Ystatistic_value;
extern Sv Zstatistic_value;
/* Private variables ---------------------------------------------------------*/


void BLE_USART(UART_HandleTypeDef *huart, Sv *sendpData )
{
	if(USARTBLE.sendflag ==1)
	{


		snprintf_(USARTBLE.buffer, 128 , "%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f",
		Xstatistic_value.Statistic_FreqOvall * 1000, Xstatistic_value.Statistic_SpeedOvall * 1000, Xstatistic_value.Statistic_p2p * 1000
		, Ystatistic_value.Statistic_FreqOvall * 1000, Ystatistic_value.Statistic_SpeedOvall * 1000, Ystatistic_value.Statistic_p2p * 1000
		, Zstatistic_value.Statistic_FreqOvall * 1000, Zstatistic_value.Statistic_SpeedOvall * 1000, Zstatistic_value.Statistic_p2p * 1000);

		USARTBLE.bufferSize = min_(APP_BUFFER_SIZE, strlen(USARTBLE.buffer));

		USARTBLE.sendTimeout = 100 ;
		HAL_StatusTypeDef BLE_transfer_status = HAL_UART_Transmit(huart, USARTBLE.buffer, USARTBLE.bufferSize,USARTBLE.sendTimeout);
		//HAL_StatusTypeDef BLE_transfer_status = HAL_UART_Transmit_DMA(huart, USARTBLE.buffer, USARTBLE.bufferSize);
		if(BLE_transfer_status == HAL_OK)
		{
			float a = 1;
		}
		/*
		 HAL_UART_Receive(huart , &USARTBLE.Rbuffer, 14, 1000);

		 char C[20];
		 strcpy(C,  USARTBLE.Rbuffer );
		 */
		 //0x1;
	}
}
