/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.h
 * @brief          : Header for main.c file.
 *                   This file contains the common defines of the application.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
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
#define DISP_RS_Pin GPIO_PIN_2
#define DISP_RS_GPIO_Port GPIOC
#define V_ADC_Pin GPIO_PIN_1
#define V_ADC_GPIO_Port GPIOA
#define MA_ADC_Pin GPIO_PIN_2
#define MA_ADC_GPIO_Port GPIOA
#define IN_RANGE_Pin GPIO_PIN_3
#define IN_RANGE_GPIO_Port GPIOA
#define V_BUS_MON_Pin GPIO_PIN_4
#define V_BUS_MON_GPIO_Port GPIOA
#define DEBUG_Pin GPIO_PIN_6
#define DEBUG_GPIO_Port GPIOA
#define EMERGENCY_Pin GPIO_PIN_12
#define EMERGENCY_GPIO_Port GPIOB
#define OE_Pin GPIO_PIN_13
#define OE_GPIO_Port GPIOB
#define OC_BOOST_Pin GPIO_PIN_14
#define OC_BOOST_GPIO_Port GPIOB
#define RELAY_Pin GPIO_PIN_6
#define RELAY_GPIO_Port GPIOC
#define SPI_CS_Pin GPIO_PIN_15
#define SPI_CS_GPIO_Port GPIOA
#define INPUT_B_BUF_Pin GPIO_PIN_11
#define INPUT_B_BUF_GPIO_Port GPIOC
#define INPUT_C_BUF_Pin GPIO_PIN_0
#define INPUT_C_BUF_GPIO_Port GPIOD
#define INPUT_D_BUF_Pin GPIO_PIN_1
#define INPUT_D_BUF_GPIO_Port GPIOD
#define INPUT_E_BUF_Pin GPIO_PIN_2
#define INPUT_E_BUF_GPIO_Port GPIOD
#define INPUT_A_BUF_Pin GPIO_PIN_3
#define INPUT_A_BUF_GPIO_Port GPIOD
#define ENCODER_PB_Pin GPIO_PIN_8
#define ENCODER_PB_GPIO_Port GPIOB
#define INT_N_Pin GPIO_PIN_9
#define INT_N_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
