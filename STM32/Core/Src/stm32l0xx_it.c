/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32l0xx_it.c
  * @brief   Interrupt Service Routines.
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32l0xx_it.h"
#include "encoder.h"
#include "ultrasonic.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
int counter;
extern US_Handle_t ultrasonic;
extern volatile uint32_t exti_count;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim21;
extern TIM_HandleTypeDef htim22;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart1;
/* USER CODE BEGIN EV */
extern Encoder_Handle_t encoder;
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M0+ Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVC_IRQn 0 */

  /* USER CODE END SVC_IRQn 0 */
  /* USER CODE BEGIN SVC_IRQn 1 */

  /* USER CODE END SVC_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */
	counter++;
  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32L0xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32l0xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI line 4 to 15 interrupts.
  */
void EXTI4_15_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI4_15_IRQn 0 */

  /* USER CODE END EXTI4_15_IRQn 0 */
	
	/*
  uint32_t pending = EXTI->PR;  // Pending register
  
  
  EXTI->PR = 0xFFFFFFFF;  // Write 1 to clear
  
 
  if (pending & FRONT_ECHO_Pin) US_EXTI_Callback(&ultrasonic, FRONT_ECHO_Pin);
  if (pending & RIGHT_ECHO_Pin) US_EXTI_Callback(&ultrasonic, RIGHT_ECHO_Pin);
  if (pending & LEFT_ECHO_Pin)  US_EXTI_Callback(&ultrasonic, LEFT_ECHO_Pin);
  if (pending & REAR_ECHO_Pin)  US_EXTI_Callback(&ultrasonic, REAR_ECHO_Pin);
	*/

  HAL_GPIO_EXTI_IRQHandler(FRONT_ECHO_Pin);
  HAL_GPIO_EXTI_IRQHandler(RIGHT_ECHO_Pin);
  HAL_GPIO_EXTI_IRQHandler(LEFT_ECHO_Pin);
  HAL_GPIO_EXTI_IRQHandler(REAR_ECHO_Pin);
	
	HAL_GPIO_EXTI_IRQHandler(GREEN_BUTTON_Pin);
  HAL_GPIO_EXTI_IRQHandler(WHITE_BUTTON_Pin);
  HAL_GPIO_EXTI_IRQHandler(B1_Pin);
  HAL_GPIO_EXTI_IRQHandler(BLUE_BUTTON_Pin);
  HAL_GPIO_EXTI_IRQHandler(RED_BUTTON_Pin);
  /* USER CODE BEGIN EXTI4_15_IRQn 1 */

  /* USER CODE END EXTI4_15_IRQn 1 */
}
/*
void EXTI4_15_IRQHandler(void)
{
    
    uint32_t pending = EXTI->PR;
    
    
    uint8_t front_state = HAL_GPIO_ReadPin(FRONT_ECHO_GPIO_Port, FRONT_ECHO_Pin);
    uint8_t right_state = HAL_GPIO_ReadPin(RIGHT_ECHO_GPIO_Port, RIGHT_ECHO_Pin);
    uint8_t left_state = HAL_GPIO_ReadPin(LEFT_ECHO_GPIO_Port, LEFT_ECHO_Pin);
    uint8_t rear_state = HAL_GPIO_ReadPin(REAR_ECHO_GPIO_Port, REAR_ECHO_Pin);
    
    
    EXTI->PR = pending;  // Only clear what was pending
    
    
    if (pending & FRONT_ECHO_Pin) US_EXTI_CallbackWithState(&ultrasonic, FRONT_ECHO_Pin, front_state);
    if (pending & RIGHT_ECHO_Pin) US_EXTI_CallbackWithState(&ultrasonic, RIGHT_ECHO_Pin, right_state);
    if (pending & LEFT_ECHO_Pin)  US_EXTI_CallbackWithState(&ultrasonic, LEFT_ECHO_Pin, left_state);
    if (pending & REAR_ECHO_Pin)  US_EXTI_CallbackWithState(&ultrasonic, REAR_ECHO_Pin, rear_state);
    
    
    if (pending & GREEN_BUTTON_Pin) HAL_GPIO_EXTI_IRQHandler(GREEN_BUTTON_Pin);
    if (pending & WHITE_BUTTON_Pin) HAL_GPIO_EXTI_IRQHandler(WHITE_BUTTON_Pin);
    if (pending & B1_Pin) HAL_GPIO_EXTI_IRQHandler(B1_Pin);
    if (pending & BLUE_BUTTON_Pin) HAL_GPIO_EXTI_IRQHandler(BLUE_BUTTON_Pin);
    if (pending & RED_BUTTON_Pin) HAL_GPIO_EXTI_IRQHandler(RED_BUTTON_Pin);
    
    exti_count++;
}
*/
/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}

/**
  * @brief This function handles TIM21 global interrupt.
  */
void TIM21_IRQHandler(void)
{
  /* USER CODE BEGIN TIM21_IRQn 0 */

  /* USER CODE END TIM21_IRQn 0 */
  HAL_TIM_IRQHandler(&htim21);
  /* USER CODE BEGIN TIM21_IRQn 1 */

  /* USER CODE END TIM21_IRQn 1 */
}

/**
  * @brief This function handles TIM22 global interrupt.
  */
void TIM22_IRQHandler(void)
{
  /* USER CODE BEGIN TIM22_IRQn 0 */

  /* USER CODE END TIM22_IRQn 0 */
  HAL_TIM_IRQHandler(&htim22);
  /* USER CODE BEGIN TIM22_IRQn 1 */

  /* USER CODE END TIM22_IRQn 1 */
}

/**
  * @brief This function handles USART2 global interrupt / USART2 wake-up interrupt through EXTI line 26.
  */
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */

  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART2_IRQn 1 */

  /* USER CODE END USART2_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */
   /* Check for errors first */
    uint32_t isrflags = huart1.Instance->ISR;
    uint32_t errorflags = (isrflags & (USART_ISR_PE | USART_ISR_FE | USART_ISR_ORE | USART_ISR_NE));
    
    if (errorflags != 0) {
        /* Clear all error flags */
        huart1.Instance->ICR = (USART_ICR_PECF | USART_ICR_FECF | USART_ICR_ORECF | USART_ICR_NCF);
        
        /* Clear HAL error code */
        huart1.ErrorCode = HAL_UART_ERROR_NONE;
        
        /* Re-enable RXNE interrupt (it gets disabled on error) */
        __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
    }
    
    /* Check if RXNE flag is set (data received) */
    if ((isrflags & USART_ISR_RXNE) != 0) {
        uint8_t data = (uint8_t)(huart1.Instance->RDR & 0xFF);
        Encoder_ProcessByte(&encoder, data);
    }
  /* USER CODE END USART1_IRQn 0 */
  
  /*HAL_UART_IRQHandler(&huart1);*/
  
  /* USER CODE BEGIN USART1_IRQn 1 */
  /* USER CODE END USART1_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
