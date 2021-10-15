/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File           : main.c
  * Brief          : STM32 Development Board
  * Authors		   : Team 19
  * Version        : 1.0
  * Created		   : Sept 4, 2021
  * Last Modified  : Oct 15, 2021
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "NRF24L01.h"
#include "string.h"
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
CAN_HandleTypeDef hcan1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint8_t TxAddress[] = {0xEE,0xDD,0xCC,0xBB,0xAA}; //RF transmit address
uint8_t RxAddress[] = {0xEE,0xDD,0xCC,0xBB,0xAA}; //RF receive address

CAN_TxHeaderTypeDef TxHeader; //CAN transmit header initialisation
CAN_RxHeaderTypeDef RxHeader; //CAN receive header initialisation
uint32_t TxMailbox; //CAN transmit mailbox initialisation

	/* This program assumes the data being sent over RF is the same being sent over a CAN Bus.
	 * If the user wishes to send different data over RF and CAN, or wishes to send 32 bytes
	 * of data over RF, separate TxData and RxData arrays should be specified for RF and CAN.
	 * Eg. TxDataRF[]= "Hello World /n" and RxData[32] */

/* PROJECT CODE VARIABLE AND DATA ARRAY DEFINITION BEGIN */
int datacheck = 0;
uint8_t TxData[8]; //RF and CAN transmit data array
uint8_t RxData[8]; //RF and CAN receive data array
/* PROJECT CODE VARIABLE AND DATA ARRAY DEFINITION END */

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan1)
{
	HAL_CAN_GetRxMessage(hcan1, CAN_RX_FIFO0, &RxHeader, RxData); // Reads the data received in CAN FIFO

	/* PROJECT CODE CAN DATA PROCESSING BEGIN */
	if (RxHeader.DLC == 2) // EXAMPLE CODE: Change value if DLC also changes
	{
		datacheck = 1;
	}
	/* PROJECT CODE CAN DATA PROCESSING END */
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
  MX_CAN1_Init();
  MX_SPI1_Init();
  MX_USB_DEVICE_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  nrfInit(); // Initialise NRF24L01

  /* PROJECT CODE RF CHANNEL BEGIN */
  nrfTxMode(TxAddress,0x0A); // Edit second argument to change channel for RF transmit
  nrfRxMode(RxAddress, 0x0A); // Edit second argument to change channel for RF receive
  /* PROJECT CODE RF CHANNEL END */

  HAL_CAN_Start(&hcan1);

  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING); // Activate notification if data


  /* PROJECT CODE CAN HEADER CONFIGURATION BEGIN */
  TxHeader.DLC = 2; // Edit this value to send up to 8 bytes of data in a CAN message.
  TxHeader.ExtId = 0;
  TxHeader.IDE = CAN_ID_STD; // standard CAN frame
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.StdId = 0x103; // ID of the sender
  TxHeader.TransmitGlobalTime = DISABLE;
  /* PROJECT CODE CAN HEADER CONFIGURATION END */

  	  /* PROJECT CODE DATA FRAME CONTENT BEGIN */
  	  /* The user can define the data for each array position.
  	   * An example below uses 2 bytes to send LED animation data.
  	   */
  TxData[0] = 200;  // EXAMPLE CODE: Delay between LED blinks in milliseconds (ms)
  TxData[1] = 12; // EXAMPLE CODE: How many times LED blinks
  /* PROJECT CODE DATA FRAME CONTENT END */

  /* Delete comment (//) below if programming a CAN transmitter */
  // HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox); // Sends message using header and data input defined in fields above


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  HAL_GPIO_WritePin(GPIOB,LED_OP_Pin,GPIO_PIN_SET); // LED_OP pin is constant HIGH when device is operating as expected

	/* NOTE: The current configuration is for an RF device in RECEIVER mode. If the user wishes
	 * to use this device in TRANSMITTER mode, swap code by commenting each line within RF
	 * RECEIVE MODE and uncommenting RF TRANSMIT MODE.
	 */

	/* RF RECEIVER MODE START */
	  if (dataAvailability(1) == 1) // Check to see if RF Rx data is available to read
	  {
		  nrfReceive(RxData); // Read data to RxData array
		  HAL_GPIO_TogglePin(GPIOB, LED_Rx_Pin); // Flash Rx LED when data is being received

		  /* PROJECT CODE RF DATA PROCESSING BEGIN */
		  	  //EXAMPLE CODE: Blink external LED for RxData[1] number of times with RxData[0] delay.
		  	  for (int i=0; i<RxData[1]; i++)
		  	  {
		  		  HAL_GPIO_TogglePin(GPIOE, OUT1_Pin);
		  		  HAL_Delay(RxData[0]);
		  	  }
		  /* PROJECT CODE RF DATA PROCESSING END */
	  }
	  else
	  {
		  HAL_GPIO_WritePin(GPIOB,LED_OP_Pin,GPIO_PIN_RESET); // Blink LED_OP to indicate no connection/idle state
	  	  HAL_Delay(1000); // Wait 1 second before restarting loop
	  }

	/* RF RECEIVER MODE END */

	/* RF TRANSMITTER MODE START

	   if (nrfTransmIdit(TxData) == 1) // Check to see if Tx data is being transmitted
	  {
		  HAL_GPIO_TogglePin(GPIOD, LED_Tx_Pin); // Flash Rx LED when data is being transmitted
	  }
	  else
	  {
	  	  HAL_GPIO_WritePin(GPIOB,LED_OP_Pin,GPIO_PIN_RESET); // Blink LED_OP to indicate no connection/idle state
	  	  HAL_DELAY(1000); // Wait 1 second before restarting loop
	  }

	  HAL_Delay(1000); // EXAMPLE CODE: Wait 1 second before transmitting data again. If the user
	  	  	  	  	   // requires data to be sent after a trigger (button press), remove the delay.

	   RF TRANSMITTER MODE END */


	  if (datacheck) // Checks to see if CAN message has been sent with expected number of bytes
	  {
		  /* PROJECT CODE RF DATA PROCESSING BEGIN */
		  	  //EXAMPLE CODE: Blink external LED for RxData[1] number of times with RxData[0] delay.
		  	  for (int i=0; i<RxData[1]; i++)
		  	  {
		  		  HAL_GPIO_TogglePin(GPIOE, OUT1_Pin); //blink LED at output Pin
		  		  HAL_Delay(RxData[0]);
		  	  }
		  /* PROJECT CODE CAN DATA PROCESSING END */

		  /* EXAMPLE CODE: With the line of code below enabled, two CAN devices will send
		   * data continuously back and forth. Removing the line will ensure this device
		   * will not transmit a message following a data read.
		   */
		  HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox); // Sends message using header and data array input as defined in fields.
		  datacheck = 0;
	  }
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV8;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 4-1;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_3TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_3TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */
  CAN_FilterTypeDef canfilterconfig;

  /* PROJECT CODE CAN FILTER CONFIGURATION BEGIN */
  canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
  canfilterconfig.FilterBank = 10; // Edit to choose which filter bank to use
  canfilterconfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  canfilterconfig.FilterIdHigh = 0x446<<5; // Edit to specify the CAN identifier/s permitted
  canfilterconfig.FilterIdLow = 0;
  canfilterconfig.FilterMaskIdHigh = 0x446<<5; // Edit to specify the CAN identifier/s permitted
  canfilterconfig.FilterMaskIdLow = 0x0000;
  canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
  canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
  canfilterconfig.SlaveStartFilterBank = 13; // Edit to specify how many filters to assign to CAN1 (master)
  /* PROJECT CODE CAN FILTER CONFIGURATION END */

  HAL_CAN_ConfigFilter(&hcan1, &canfilterconfig);
  /* USER CODE END CAN1_Init 2 */

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
  htim2.Init.Prescaler = 646-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 255-1;
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, OUT1_Pin|OUT2_Pin|NRF_IRQ_Pin|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10|LED_OP_Pin|LED_Rx_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LED_FAULT_Pin|LED_Tx_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(S_GPIO_Port, S_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : IN1_Pin IN2_Pin */
  GPIO_InitStruct.Pin = IN1_Pin|IN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : OUT1_Pin OUT2_Pin NRF_IRQ_Pin PE15 */
  GPIO_InitStruct.Pin = OUT1_Pin|OUT2_Pin|NRF_IRQ_Pin|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI1_CS_Pin */
  GPIO_InitStruct.Pin = SPI1_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI1_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 LED_OP_Pin LED_Rx_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_10|LED_OP_Pin|LED_Rx_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_FAULT_Pin LED_Tx_Pin */
  GPIO_InitStruct.Pin = LED_FAULT_Pin|LED_Tx_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : S_Pin */
  GPIO_InitStruct.Pin = S_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(S_GPIO_Port, &GPIO_InitStruct);

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
  __disable_irq();
  while (1)
  {
	  HAL_GPIO_WritePin(GPIOB, LED_OP_Pin,GPIO_PIN_RESET); // Sets LED_OP to LOW if an error has occurred
	  HAL_GPIO_WritePin(GPIOD, LED_FAULT_Pin,GPIO_PIN_SET); // Sets LED_FAULT to HIGH if an error has occurred
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
