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
#include "stm32l0xx_hal.h"

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
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define B1_EXTI_IRQn EXTI4_15_IRQn
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define GREEN_BUTTON_Pin GPIO_PIN_4
#define GREEN_BUTTON_GPIO_Port GPIOC
#define GREEN_BUTTON_EXTI_IRQn EXTI4_15_IRQn
#define WHITE_BUTTON_Pin GPIO_PIN_5
#define WHITE_BUTTON_GPIO_Port GPIOC
#define WHITE_BUTTON_EXTI_IRQn EXTI4_15_IRQn
#define LEFT_EN_Pin GPIO_PIN_2
#define LEFT_EN_GPIO_Port GPIOB
#define SERVO_PWM_Pin GPIO_PIN_13
#define SERVO_PWM_GPIO_Port GPIOB
#define BLUE_BUTTON_Pin GPIO_PIN_14
#define BLUE_BUTTON_GPIO_Port GPIOB
#define BLUE_BUTTON_EXTI_IRQn EXTI4_15_IRQn
#define RED_BUTTON_Pin GPIO_PIN_15
#define RED_BUTTON_GPIO_Port GPIOB
#define RED_BUTTON_EXTI_IRQn EXTI4_15_IRQn
#define FRONT_ECHO_Pin GPIO_PIN_6
#define FRONT_ECHO_GPIO_Port GPIOC
#define FRONT_ECHO_EXTI_IRQn EXTI4_15_IRQn
#define RIGHT_ECHO_Pin GPIO_PIN_7
#define RIGHT_ECHO_GPIO_Port GPIOC
#define RIGHT_ECHO_EXTI_IRQn EXTI4_15_IRQn
#define LEFT_ECHO_Pin GPIO_PIN_8
#define LEFT_ECHO_GPIO_Port GPIOC
#define LEFT_ECHO_EXTI_IRQn EXTI4_15_IRQn
#define REAR_ECHO_Pin GPIO_PIN_9
#define REAR_ECHO_GPIO_Port GPIOC
#define REAR_ECHO_EXTI_IRQn EXTI4_15_IRQn
#define RIGHT_EN_Pin GPIO_PIN_8
#define RIGHT_EN_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define REAR_TRIG_Pin GPIO_PIN_10
#define REAR_TRIG_GPIO_Port GPIOC
#define LEFT_TRIG_Pin GPIO_PIN_11
#define LEFT_TRIG_GPIO_Port GPIOC
#define RIGHT_TRIG_Pin GPIO_PIN_12
#define RIGHT_TRIG_GPIO_Port GPIOC
#define FRONT_TRIG_Pin GPIO_PIN_2
#define FRONT_TRIG_GPIO_Port GPIOD
#define RIGHT_PWM_Pin GPIO_PIN_4
#define RIGHT_PWM_GPIO_Port GPIOB
#define LEFT_PWM_Pin GPIO_PIN_5
#define LEFT_PWM_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
