/**
  ******************************************************************************
  * File           : main.h
  * Brief          : STM32 Development Board
  * Authors		   : Team 19
  * Version        : 1.0
  * Created		   : Sept 4, 2021
  * Last Modified  : Sept 4, 2021
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define IN1_Pin GPIO_PIN_2
#define IN1_GPIO_Port GPIOE
#define IN2_Pin GPIO_PIN_3
#define IN2_GPIO_Port GPIOE
#define OUT1_Pin GPIO_PIN_4
#define OUT1_GPIO_Port GPIOE
#define OUT2_Pin GPIO_PIN_5
#define OUT2_GPIO_Port GPIOE
#define HSE_IN_Pin GPIO_PIN_0
#define HSE_IN_GPIO_Port GPIOH
#define HSE_OUT_Pin GPIO_PIN_1
#define HSE_OUT_GPIO_Port GPIOH
#define SPI1_CS_Pin GPIO_PIN_4
#define SPI1_CS_GPIO_Port GPIOA
#define NRF_IRQ_Pin GPIO_PIN_11
#define NRF_IRQ_GPIO_Port GPIOE
#define LED_OP_Pin GPIO_PIN_12
#define LED_OP_GPIO_Port GPIOB
#define LED_Rx_Pin GPIO_PIN_15
#define LED_Rx_GPIO_Port GPIOB
#define LED_FAULT_Pin GPIO_PIN_14
#define LED_FAULT_GPIO_Port GPIOD
#define LED_Tx_Pin GPIO_PIN_15
#define LED_Tx_GPIO_Port GPIOD
#define USB_D__Pin GPIO_PIN_11
#define USB_D__GPIO_Port GPIOA
#define USB_D_A12_Pin GPIO_PIN_12
#define USB_D_A12_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define S_Pin GPIO_PIN_12
#define S_GPIO_Port GPIOC
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
