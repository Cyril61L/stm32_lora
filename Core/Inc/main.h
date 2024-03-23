/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

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
#define EN_PIR_PWR_Pin GPIO_PIN_13
#define EN_PIR_PWR_GPIO_Port GPIOC
#define EN_LORA_PWR_Pin GPIO_PIN_1
#define EN_LORA_PWR_GPIO_Port GPIOD
#define PIR_IN_Pin GPIO_PIN_0
#define PIR_IN_GPIO_Port GPIOA
#define PIR_IN_EXTI_IRQn EXTI0_IRQn
#define LED_B_Pin GPIO_PIN_1
#define LED_B_GPIO_Port GPIOA
#define LED_R_Pin GPIO_PIN_2
#define LED_R_GPIO_Port GPIOA
#define LED_V_Pin GPIO_PIN_3
#define LED_V_GPIO_Port GPIOA
#define CONF_3_Pin GPIO_PIN_4
#define CONF_3_GPIO_Port GPIOA
#define CONF_2_Pin GPIO_PIN_5
#define CONF_2_GPIO_Port GPIOA
#define CONF_1_Pin GPIO_PIN_6
#define CONF_1_GPIO_Port GPIOA
#define TB1_Pin GPIO_PIN_7
#define TB1_GPIO_Port GPIOA
#define TB1_EXTI_IRQn EXTI9_5_IRQn
#define VBAT_MES_Pin GPIO_PIN_1
#define VBAT_MES_GPIO_Port GPIOB
#define EN_MES_Pin GPIO_PIN_2
#define EN_MES_GPIO_Port GPIOB
#define LORA_RST_Pin GPIO_PIN_10
#define LORA_RST_GPIO_Port GPIOA
#define LORA_BUSY_Pin GPIO_PIN_11
#define LORA_BUSY_GPIO_Port GPIOA
#define LORA_DIO1_Pin GPIO_PIN_12
#define LORA_DIO1_GPIO_Port GPIOA
#define LORA_DIO1_EXTI_IRQn EXTI15_10_IRQn
#define LORA_CS_Pin GPIO_PIN_15
#define LORA_CS_GPIO_Port GPIOA
#define LORA_DIO3_Pin GPIO_PIN_6
#define LORA_DIO3_GPIO_Port GPIOB
#define LORA_DIO3_EXTI_IRQn EXTI9_5_IRQn
#define TXEN_Pin GPIO_PIN_7
#define TXEN_GPIO_Port GPIOB
#define RXEN_Pin GPIO_PIN_8
#define RXEN_GPIO_Port GPIOB
#define LORA_DIO2_Pin GPIO_PIN_9
#define LORA_DIO2_GPIO_Port GPIOB
#define LORA_DIO2_EXTI_IRQn EXTI9_5_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
