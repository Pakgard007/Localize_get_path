/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "stm32g4xx_hal.h"

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
#define E_Stop_Pin GPIO_PIN_13
#define E_Stop_GPIO_Port GPIOC
#define USER_SW_Pin GPIO_PIN_1
#define USER_SW_GPIO_Port GPIOC
#define RELAY_Brake_Pin GPIO_PIN_2
#define RELAY_Brake_GPIO_Port GPIOC
#define MCOR_Speed_Control_IN_Pin GPIO_PIN_1
#define MCOR_Speed_Control_IN_GPIO_Port GPIOA
#define WDI_Pin GPIO_PIN_5
#define WDI_GPIO_Port GPIOA
#define Backward_IN_to_MCU_Pin GPIO_PIN_4
#define Backward_IN_to_MCU_GPIO_Port GPIOC
#define Forward_IN_to_MCU_Pin GPIO_PIN_5
#define Forward_IN_to_MCU_GPIO_Port GPIOC
#define Pedal_IN_to_MCU_Pin GPIO_PIN_0
#define Pedal_IN_to_MCU_GPIO_Port GPIOB
#define Forward_IN_from_MCU_Pin GPIO_PIN_6
#define Forward_IN_from_MCU_GPIO_Port GPIOC
#define Pedal_IN_from_MCU_Pin GPIO_PIN_7
#define Pedal_IN_from_MCU_GPIO_Port GPIOC
#define Backward_IN_from_MCU_Pin GPIO_PIN_8
#define Backward_IN_from_MCU_GPIO_Port GPIOC
#define MODE_Selector_Pin GPIO_PIN_9
#define MODE_Selector_GPIO_Port GPIOC
#define MCP4725_SDA_Pin GPIO_PIN_8
#define MCP4725_SDA_GPIO_Port GPIOA
#define MCP4725_SCL_Pin GPIO_PIN_9
#define MCP4725_SCL_GPIO_Port GPIOA
#define Relay_Mode_Pin GPIO_PIN_10
#define Relay_Mode_GPIO_Port GPIOA
#define Speed_Sensor_to_MCU_Pin GPIO_PIN_15
#define Speed_Sensor_to_MCU_GPIO_Port GPIOA
#define LED_1_Pin GPIO_PIN_5
#define LED_1_GPIO_Port GPIOB
#define LED2_Pin GPIO_PIN_6
#define LED2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
