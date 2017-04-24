/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "dma.h"
#include "fatfs.h"
#include "sdio.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "app.h"
#include <string.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
FATFS SDFatFs;
FIL MyFile;
char SDPath[4];
uint8_t gpsRxBuffer[51];  // 先看成是MSB的
uint8_t gpsStart[5] = {0xAA,0xEE,0x05,0x33,0xD0};
// file write variable

FRESULT res;                                          /* FatFs function common result code */
uint32_t byteswritten;// bytesread;                     /* File write/read counts */
uint8_t wtext[] = "hello world!\n"; /* File write buffer */

uint8_t motorRxbuffer[31];

uint8_t rxByte;

MotorRxDataUnion motorRxData;
uint8_t countReceived = 0;
uint8_t uart4RxState = 0;
gpsDataType gpsData;
gpsDataType gpsDataBuf[100];
gpsDataType *Ptr = (gpsDataType *)gpsDataBuf;
long countGps = 0;
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)   // 由DMA触发进入的串口中断
{
	if(huart->Instance == USART2)
	{
		if(gpsRxBuffer[0] == 0xEE && gpsRxBuffer[1] == 0xDD && gpsRxBuffer[50] == uint8Sum(gpsRxBuffer,50))
		{
			gpsData.starNumber = gpsRxBuffer[9];
			gpsData.year = ((uint16_t)gpsRxBuffer[11]<<8) + gpsRxBuffer[10];
			gpsData.month = gpsRxBuffer[12];
			gpsData.day = gpsRxBuffer[13];
			gpsData.hour = gpsRxBuffer[14];
			gpsData.minute = gpsRxBuffer[15];
			gpsData.second = gpsRxBuffer[16];
			gpsData.ms = ((uint16_t)gpsRxBuffer[18]<<8) + gpsRxBuffer[17];
			
			gpsData.longitude = ((uint32_t)gpsRxBuffer[22]<<24) + ((uint32_t)gpsRxBuffer[21]<<16) + ((uint32_t)gpsRxBuffer[20]<<8) + (uint32_t)gpsRxBuffer[19]; // 读取经度
			gpsData.latitude = ((uint32_t)gpsRxBuffer[26]<<24) + ((uint32_t)gpsRxBuffer[25]<<16) + ((uint32_t)gpsRxBuffer[24]<<8) + (uint32_t)gpsRxBuffer[23];  // 读任扯取
			gpsData.altitude = ((uint32_t)gpsRxBuffer[30]<<24) + ((uint32_t)gpsRxBuffer[29]<<16) + ((uint32_t)gpsRxBuffer[28]<<8) + (uint32_t)gpsRxBuffer[27];  // 读取海拔
					
			gpsData.northSpeed = ((uint32_t)gpsRxBuffer[32]<<8) + (uint32_t)gpsRxBuffer[31];																																	// 北向速度
			gpsData.upSpeed = ((uint32_t)gpsRxBuffer[34]<<8) + (uint32_t)gpsRxBuffer[33];																																	// 天向速度
			gpsData.eastSpeed = ((uint32_t)gpsRxBuffer[36]<<8) + (uint32_t)gpsRxBuffer[35];	

			gpsData.PDOP = ((uint16_t)gpsRxBuffer[38]<<8) + gpsRxBuffer[37];
			gpsData.GDOP = ((uint16_t)gpsRxBuffer[40]<<8) + gpsRxBuffer[39];
			*(Ptr+countGps) = gpsData;
			countGps ++;
			
			if(countGps >= 100)
			{
				writeGPS();
				countGps = 0;
			}
		}
		HAL_UART_Receive_IT(&huart2, (uint8_t *)gpsRxBuffer, sizeof(gpsRxBuffer));
	}
	
	if(huart->Instance == UART4)
	{			
			switch(uart4RxState)
			{
				case 0:if(rxByte == 0xAA) uart4RxState = 1;break;
				case 1:if(rxByte == 0x55) 
							{					
								uart4RxState = 2;
								motorRxData.y[0] = 0xAA;
								motorRxData.y[1] = 0x55;
								}
				        else if(rxByte == 0xAA)uart4RxState = 1; break;   // 返回1的状态
				case 2: 
					    if(countReceived < 28) //  除了帧头 校验位-3还有28 个字节要收
							{		
								motorRxData.y[countReceived+2] = rxByte; // 前三个单元放了 0xAA 0x55
								countReceived++;
							}
							else                                   //  校验位 -接受完成~                                      
							{
								int temp = uint8Sum(motorRxData.y,30);
							  motorRxData.y[30] = rxByte;
								if( temp == rxByte)
										writeSD();
								countReceived = 0;
								uart4RxState = 0;
							}
							break;
				
				default:break;
		}
			HAL_UART_Receive_IT(&huart4, &rxByte, sizeof(rxByte));
	}
	
	
}
/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */


//  uint8_t rtext[100];      
//  printf("hello \n");                           
	/* File read buffer */
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_SDIO_SD_Init();
  MX_UART4_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_FATFS_Init();

  /* USER CODE BEGIN 2 */
		    /*##-2- Register the file system object to the FatFs module ##############*/
    if(f_mount(&SDFatFs, (TCHAR const*)SDPath, 0) != FR_OK)
    {
      /* FatFs Initialization Error */
			
			//printf("FatFs Initialization Error");
      Error_Handler();
    }
		 // 建FAT 文件系统，不是每次都有必要。
      if(f_mkfs((TCHAR const*)SDPath, 0, 0) != FR_OK)
      {
        /* FatFs Format Error */
				//printf("FatFs Format Error ");
        Error_Handler();
      }
//		 if(f_open(&MyFile, "STM32.TXT", FA_CREATE_ALWAYS | FA_WRITE) != FR_OK)
//     {
//          /* 'STM32.TXT' file Open for write Error */
//					//printf("STM32.TXT' file Open for write Error");
//          Error_Handler();
//      }
//		 f_close(&MyFile);
//		 
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	HAL_UART_Transmit_IT(&huart2, (uint8_t *)gpsStart, sizeof(gpsStart));
	HAL_UART_Receive_IT(&huart2, (uint8_t *)gpsRxBuffer, sizeof(gpsRxBuffer));
	

	HAL_UART_Receive_IT(&huart4, &rxByte, sizeof(rxByte));
	//HAL_TIM_Base_Start_IT(&htim3);
	//HAL_TIM_Base_Start_IT(&htim4);
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
		//HAL_UART_Transmit_IT(&huart2, (uint8_t *)gpsStart, sizeof(gpsStart));
 		//HAL_UART_Transmit_IT(&huart4, (uint8_t *)motorTxbuffer, sizeof(motorTxbuffer));
    HAL_Delay(1000);
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks 
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

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
		HAL_UART_Receive_IT(&huart4, &rxByte, sizeof(&rxByte));
		HAL_UART_Receive_IT(&huart2, (uint8_t *)gpsRxBuffer, sizeof(gpsRxBuffer));
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
