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

/* Private variables ---------------------------------------------------------*/


void BLE_USART(UART_HandleTypeDef *huart, Sv *sendpData )
{
	if(USARTBLE.sendflag ==1)
	{
		USARTBLE.bufferSize = min_(APP_BUFFER_SIZE, strlen(USARTBLE.buffer));


		snprintf_(USARTBLE.buffer, 128 , "%.4f,%.4f,%.4f", sendpData->Statistic_FreqOvall,
				sendpData->Statistic_SpeedOvall,sendpData->Statistic_p2p);
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
