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
#define MotorR_ER_Pin GPIO_PIN_13
#define MotorR_ER_GPIO_Port GPIOC
#define MotorR_EL_Pin GPIO_PIN_14
#define MotorR_EL_GPIO_Port GPIOC
#define TIM1_CH2ERPWML_Pin GPIO_PIN_5
#define TIM1_CH2ERPWML_GPIO_Port GPIOA
#define TIM3_CH1_Encoder1B_Pin GPIO_PIN_6
#define TIM3_CH1_Encoder1B_GPIO_Port GPIOA
#define TIM3_CH2_Encoder1A_Pin GPIO_PIN_7
#define TIM3_CH2_Encoder1A_GPIO_Port GPIOA
#define MotorL_isL_Pin GPIO_PIN_12
#define MotorL_isL_GPIO_Port GPIOB
#define MotorL_isR_Pin GPIO_PIN_13
#define MotorL_isR_GPIO_Port GPIOB
#define MotorL_EL_Pin GPIO_PIN_14
#define MotorL_EL_GPIO_Port GPIOB
#define MotorL_ER_Pin GPIO_PIN_15
#define MotorL_ER_GPIO_Port GPIOB
#define GPIO_OUTPUT_for_UART_Pin GPIO_PIN_8
#define GPIO_OUTPUT_for_UART_GPIO_Port GPIOC
#define TIM1_CH1_ELPWMR_Pin GPIO_PIN_8
#define TIM1_CH1_ELPWMR_GPIO_Port GPIOA
#define TIM1_CH4ELPWML_Pin GPIO_PIN_11
#define TIM1_CH4ELPWML_GPIO_Port GPIOA
#define TIM2_CH2_ERPWMR_Pin GPIO_PIN_3
#define TIM2_CH2_ERPWMR_GPIO_Port GPIOB
#define MotorR_isL_Pin GPIO_PIN_4
#define MotorR_isL_GPIO_Port GPIOB
#define MotorR_isR_Pin GPIO_PIN_5
#define MotorR_isR_GPIO_Port GPIOB
#define TIM4_CH1_Encoder2A_Pin GPIO_PIN_6
#define TIM4_CH1_Encoder2A_GPIO_Port GPIOB
#define TIM4_CH1_Encoder2B_Pin GPIO_PIN_7
#define TIM4_CH1_Encoder2B_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
