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
  * Nucleo-F072
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "CO_app_STM32.h"
#include "301/CO_SDOclient.h"
#include "CANopen.h"
#include "OD.h"

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

/* USER CODE BEGIN PV */
uint8_t Tx_Array[16]={0x51,0x62,0x73,0x84,0x55,0x46,0x87,0x18,0x29,0x10,0x11,0x12,0x13,0x14,0x15,0x33};
uint8_t Rx_Array[16]={0};
uint32_t Array_32u[16]={0};
uint8_t Array_8u[16]={0x54,0x34,0x21,0xea,0xf3,0x7a,0xd4,0x46};
uint8_t Length_of_Ext_Var=0;

CAN_TxHeaderTypeDef Tx_Header;
uint32_t            TxMailbox;
uint32_t            tmp32u=0x0e0f0a0b;
uint32_t            Ticks;
char String_0[]={"String_for_Test_UART"};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* Timer interrupt function executes every 1 ms */
void
HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
    if (htim == canopenNodeSTM32->timerHandle) {
        canopen_app_interrupt();
    }
}

CO_SDO_abortCode_t
read_SDO (
		  CO_SDOclient_t* SDO_C,
		  uint8_t nodeId,
		  uint16_t index,
		  uint8_t subIndex,
		  uint8_t* buf,
		  size_t bufSize,
		  size_t* readSize
		  )
{
    CO_SDO_return_t SDO_ret;

    // setup client (this can be skipped, if remote device don't change)
    SDO_ret = CO_SDOclient_setup (
    								SDO_C, CO_CAN_ID_SDO_CLI + nodeId,
									CO_CAN_ID_SDO_SRV + nodeId,
									nodeId);

    if (SDO_ret != CO_SDO_RT_ok_communicationEnd) { return CO_SDO_AB_GENERAL; }



    // initiate upload
    SDO_ret = CO_SDOclientUploadInitiate ( SDO_C,
    										index,
											subIndex,
											1000,
											false);

    if (SDO_ret != CO_SDO_RT_ok_communicationEnd) { return CO_SDO_AB_GENERAL; }



    // upload data
    do 	{
        uint32_t timeDifference_us = 10000;
        CO_SDO_abortCode_t abortCode = CO_SDO_AB_NONE;

        SDO_ret = CO_SDOclientUpload(SDO_C, timeDifference_us, false, &abortCode, NULL, NULL, NULL);

        if (SDO_ret < 0) {  return abortCode;  }

        HAL_Delay(timeDifference_us/1000);// sleep_us(timeDifference_us);

    	} while (SDO_ret > 0);


    // copy data to the user buffer (for long data function must be called several times inside the loop)
    *readSize = CO_SDOclientUploadBufRead(SDO_C, buf, bufSize);

    return CO_SDO_AB_NONE;
}

CO_SDO_abortCode_t
write_SDO (
			CO_SDOclient_t* SDO_C,
			uint8_t nodeId,
			uint16_t index,
			uint8_t subIndex,
			uint8_t* data,
			size_t dataSize
			)
{
    CO_SDO_return_t SDO_ret;
    bool_t bufferPartial = false;

    // setup client (this can be skipped, if remote device is the same)
    SDO_ret = CO_SDOclient_setup (	SDO_C,
    								CO_CAN_ID_SDO_CLI + nodeId,
									CO_CAN_ID_SDO_SRV + nodeId,
									nodeId);

    if (SDO_ret != CO_SDO_RT_ok_communicationEnd) { return -1; }



    // initiate download
    SDO_ret = CO_SDOclientDownloadInitiate(SDO_C, index, subIndex, dataSize, 1000, false);

    if (SDO_ret != CO_SDO_RT_ok_communicationEnd) /**< Success, end of communication. SDO client: uploaded data must be read. */
    	{ return -1; }



    // fill data
    size_t nWritten = CO_SDOclientDownloadBufWrite(SDO_C, data, dataSize);

    if (nWritten < dataSize) { bufferPartial = true; } // If SDO Fifo buffer is too small, data can be refilled in the loop.




    // download data
    do {
        uint32_t timeDifference_us = 10000;
        CO_SDO_abortCode_t abortCode = CO_SDO_AB_NONE;

        SDO_ret = CO_SDOclientDownload (	SDO_C,
        									timeDifference_us,
											false, bufferPartial,
											&abortCode,
											NULL,
											NULL
										);

        if (SDO_ret < 0) {  return abortCode;}

        HAL_Delay(timeDifference_us/1000); //sleep_us(timeDifference_us);

       } while (SDO_ret > 0);

    return CO_SDO_AB_NONE;
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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_CAN_Init();
  MX_TIM17_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  /* CANHandle : Pass in the CAN Handle to this function and it wil be used for all CAN Communications. It can be FDCan or CAN
   * and CANOpenSTM32 Driver will take of care of handling that
   * HWInitFunction : Pass in the function that initialize the CAN peripheral, usually MX_CAN_Init
   * timerHandle : Pass in the timer that is going to be used for generating 1ms interrupt for tmrThread function,
   * please note that CANOpenSTM32 Library will override HAL_TIM_PeriodElapsedCallback function, if you also need this function
   * in your codes, please take required steps
   * desiredNodeID : This is the Node ID that you ask the CANOpen stack to assign to your device, although it might not always
   * be the final NodeID, after calling canopen_app_init() you should check ActiveNodeID of CANopenNodeSTM32 structure for assigned Node ID.
   * baudrate: This is the baudrate you've set in your CubeMX Configuration
   *
   */

//  HAL_Delay(500);
//  HAL_UART_Transmit_DMA( &huart2, (uint8_t*)(String_0), 10);
//  while (1){}
 #if 0
  Tx_Header.IDE    = CAN_ID_STD;
  Tx_Header.ExtId  = 0;
  Tx_Header.DLC    = 8;
  Tx_Header.StdId  = 0x7EC;
  Tx_Header.RTR    = CAN_RTR_DATA;
  HAL_CAN_Start(&hcan);  HAL_Delay(1500);

  if(HAL_CAN_AddTxMessage( &hcan,
    		               &Tx_Header,
							Tx_Array, &TxMailbox )==HAL_OK
  	 )
	  {  /* Wait transmission complete */
	  //while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan) != 3) {}
		  for(uint8_t cnt=0;cnt<22;cnt++)
		  {
		   HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
		   HAL_Delay(46);
		  }
	  }
#endif

  for(uint8_t cnt=0;cnt<50;cnt++)
  {
  	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5 );
  	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13 );
  HAL_Delay(33);
  }
 // HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET); HAL_Delay(2500);//off

  CANopenNodeSTM32 canOpenNodeSTM32;
  canOpenNodeSTM32.CANHandle = &hcan;
  canOpenNodeSTM32.HWInitFunction = MX_CAN_Init;
  canOpenNodeSTM32.timerHandle = &htim17;
  canOpenNodeSTM32.desiredNodeID = 72;
  canOpenNodeSTM32.baudrate = 125*4;
  canopen_app_init(&canOpenNodeSTM32);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

	  read_SDO (
			    canOpenNodeSTM32.canOpenStack->SDOclient,
			  	103,										//remote desiredNodeID
				0x6000,										//Index_of_OD_variable_at_remote_NodeID
				0,											//Sub_Index_of_OD_variable
				Rx_Array,									//Saved_Received_Data
				8,											//Number_of_Byte_to_read
				(size_t*)&Length_of_Ext_Var );

	  HAL_Delay(100);

		huart2.gState = HAL_UART_STATE_READY;
		HAL_UART_Transmit_DMA( &huart2, (uint8_t*)Rx_Array, 8);



	  write_SDO(
			    canOpenNodeSTM32.canOpenStack->SDOclient,
			  	103,										//remote desiredNodeID
				0x6000,										//Index_of_OD_variable_at_remote_NodeID
				0,											//Sub_Index_of_OD_variable
				Array_8u,									//
				8);

	  HAL_Delay(100);


	  read_SDO (
			    canOpenNodeSTM32.canOpenStack->SDOclient,
			  	103,										//remote desiredNodeID
				0x6000,										//Index_of_OD_variable_at_remote_NodeID
				0,											//Sub_Index_of_OD_variable
				Rx_Array,									//Saved_Received_Data
				8,											//Number_of_Byte_to_read
				(size_t*)&Length_of_Ext_Var );

	  HAL_Delay(100);

		huart2.gState = HAL_UART_STATE_READY;
		HAL_UART_Transmit_DMA( &huart2, (uint8_t*)Rx_Array, 8);

		HAL_Delay(100);


  while (1)
  {
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5  , canOpenNodeSTM32.outStatusLEDGreen);//включение
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13 ,!canOpenNodeSTM32.outStatusLEDRed  );//включение с инверсией


	canopen_app_process();
	if(tmp32u != OD_PERSIST_COMM.x6001_nucleo_RX_6001)
		{
		tmp32u = OD_PERSIST_COMM.x6001_nucleo_RX_6001;
		huart2.gState = HAL_UART_STATE_READY;
//		HAL_UART_Transmit_DMA( &huart2, (uint8_t*)(&tmp32u), 4);
		HAL_UART_Transmit_DMA( &huart2, (uint8_t*)( Ticks ), 4);
		}



	  if(HAL_GetTick() - Ticks>2000)
	  {
		OD_PERSIST_COMM.x6002_nucleo_TX_6002++;
		Ticks = HAL_GetTick();
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI48;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
