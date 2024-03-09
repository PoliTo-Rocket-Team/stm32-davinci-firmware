/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "W25Q128.h"
#include "servo.h"
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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define E_Match_Parachute_1_Pin GPIO_PIN_13
#define E_Match_Parachute_1_GPIO_Port GPIOC
#define E_Match_Parachute_2_Pin GPIO_PIN_14
#define E_Match_Parachute_2_GPIO_Port GPIOC
#define PITOT_DIFF_PRESS_SENS_Pin GPIO_PIN_1
#define PITOT_DIFF_PRESS_SENS_GPIO_Port GPIOC
#define SERIAL_COM_PORT_PRINT_TX_Pin GPIO_PIN_2
#define SERIAL_COM_PORT_PRINT_TX_GPIO_Port GPIOA
#define SERIAL_COM_PORT_PRINT_RX_Pin GPIO_PIN_3
#define SERIAL_COM_PORT_PRINT_RX_GPIO_Port GPIOA
#define SPI1_SCK_SENSOR1_Pin GPIO_PIN_5
#define SPI1_SCK_SENSOR1_GPIO_Port GPIOA
#define SPI1_MISO_SENSOR1_Pin GPIO_PIN_6
#define SPI1_MISO_SENSOR1_GPIO_Port GPIOA
#define SPI1_MOSI_SENSOR1_Pin GPIO_PIN_7
#define SPI1_MOSI_SENSOR1_GPIO_Port GPIOA
#define IMU_1_nCS_Pin GPIO_PIN_4
#define IMU_1_nCS_GPIO_Port GPIOC
#define BARO_1_nCS_Pin GPIO_PIN_5
#define BARO_1_nCS_GPIO_Port GPIOC
#define Channel1_PYRO_Pin GPIO_PIN_1
#define Channel1_PYRO_GPIO_Port GPIOB
#define Channel2_PYRO_Pin GPIO_PIN_2
#define Channel2_PYRO_GPIO_Port GPIOB
#define Status_LED_Pin GPIO_PIN_10
#define Status_LED_GPIO_Port GPIOB
#define SPI2_SCK_SENSOR2_Pin GPIO_PIN_13
#define SPI2_SCK_SENSOR2_GPIO_Port GPIOB
#define SPI2_MISO_SENSOR2_Pin GPIO_PIN_14
#define SPI2_MISO_SENSOR2_GPIO_Port GPIOB
#define SPI2_MOSI_SENSOR2_Pin GPIO_PIN_15
#define SPI2_MOSI_SENSOR2_GPIO_Port GPIOB
#define BARO_2_nCS_Pin GPIO_PIN_6
#define BARO_2_nCS_GPIO_Port GPIOC
#define IMU_2_nCS_Pin GPIO_PIN_7
#define IMU_2_nCS_GPIO_Port GPIOC
#define MAIN_COM_CLK_Pin GPIO_PIN_8
#define MAIN_COM_CLK_GPIO_Port GPIOA
#define MAIN_COM_TX_Pin GPIO_PIN_9
#define MAIN_COM_TX_GPIO_Port GPIOA
#define MAIN_COM_RX_Pin GPIO_PIN_10
#define MAIN_COM_RX_GPIO_Port GPIOA
#define FLASH_nCS_Pin GPIO_PIN_15
#define FLASH_nCS_GPIO_Port GPIOA
#define SPI3_SCK_FLASH_Pin GPIO_PIN_10
#define SPI3_SCK_FLASH_GPIO_Port GPIOC
#define SPI3_MISO_FLASH_Pin GPIO_PIN_11
#define SPI3_MISO_FLASH_GPIO_Port GPIOC
#define SPI3_MOSI_FLASH_Pin GPIO_PIN_12
#define SPI3_MOSI_FLASH_GPIO_Port GPIOC
#define DEBUG_LED_FLASH_Pin GPIO_PIN_2
#define DEBUG_LED_FLASH_GPIO_Port GPIOD
#define AIRBRAKES_PWM_Pin GPIO_PIN_4
#define AIRBRAKES_PWM_GPIO_Port GPIOB
#define BUZZER_Pin GPIO_PIN_9
#define BUZZER_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
