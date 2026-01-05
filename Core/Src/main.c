/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "car_config.h"
#include "servo.h"
#include "motor.h"
#include "buttons.h"
#include "utils.h"
#include "ultrasonic.h"
#include "encoder.h"
#include "navigation.h"
#include "pid.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DEBUG_PRINT_INTERVAL_MS   500
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
LPTIM_HandleTypeDef hlptim1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim21;
TIM_HandleTypeDef htim22;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* Hardware variables */
Servo_Handle_t servo;
Motor_Handle_t motor;
Buttons_Handle_t buttons;
US_Handle_t ultrasonic;
Encoder_Handle_t encoder;
Navigation_Handle_t navigation;

/* Timing variables */

uint32_t last_control_update = 0;
uint32_t last_us_sequence = 0;
uint32_t last_debug_print = 0;

/* Sensor data cache */
uint16_t front_mm = 0xFFFF;  // Max range = "no reading yet"
uint16_t left_mm = 0xFFFF;
uint16_t right_mm = 0xFFFF;
uint16_t rear_mm = 0xFFFF;

/* Numerical variables */
volatile extern int counter;

/* Debug counters */
volatile uint32_t exti_count = 0;
volatile uint32_t uart_rx_count = 0;
int servo_state = 0;
int counter_now = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_LPTIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM21_Init(void);
static void MX_TIM22_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
static void App_Init(void);
static void App_OnStart(void);
static void App_OnStop(void);
static void App_OnModeChange(Car_Mode_t mode);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
 * @brief EXTI callback - handles buttons and ultrasonic echo
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    exti_count++;
    
    /* Handle button interrupts */
    switch (GPIO_Pin) {
        case GREEN_BUTTON_Pin:
        case RED_BUTTON_Pin:
        case BLUE_BUTTON_Pin:
        case WHITE_BUTTON_Pin:
            Buttons_EXTI_Callback(&buttons, GPIO_Pin);
            break;
        
        /* Handle ultrasonic echo interrupts */
        case FRONT_ECHO_Pin:
        case RIGHT_ECHO_Pin: 
        case LEFT_ECHO_Pin:  
        case REAR_ECHO_Pin:
            US_EXTI_Callback(&ultrasonic, GPIO_Pin);
            break;
        
        default:
            break;
    }
}

/**
 * @brief Application initialization
 */
static void App_Init(void)
{
    /* Initialize utility functions (microsecond timer, debug UART) */
    Utils_Init(&htim6);
    Debug_Init(&huart2);
    
    Debug_Print("\r\n\r\n");
    Debug_Print("========================================\r\n");
    Debug_Print("    CARMODE v1.0 - Self Driving RC Car\r\n");
    Debug_Print("========================================\r\n");
    Debug_Print("Initializing.. .\r\n\r\n");
    
    /* Initialize servo */
    Servo_Init(&servo, &htim21, TIM_CHANNEL_1);
    Servo_Center(&servo);
    Debug_Print("[OK] Servo initialized\r\n");
    
    /* Initialize motor */
    Motor_Init(&motor, &htim22, TIM_CHANNEL_2, TIM_CHANNEL_1, 
							 LEFT_EN_GPIO_Port, LEFT_EN_Pin,
               RIGHT_EN_GPIO_Port, RIGHT_EN_Pin);
    Debug_Print("[OK] Motor driver initialized\r\n");
    
    /* Initialize ultrasonic sensors */
    US_Init(&ultrasonic, &htim6);
    Debug_Print("[OK] Ultrasonic sensors initialized\r\n");
    
    /* Initialize encoder (ESP32 UART) */
    Encoder_Init(&encoder, &huart1);
    Encoder_StartReceive(&encoder);
    Debug_Print("[OK] Encoder UART initialized\r\n");
    
    /* Initialize buttons with callbacks */
    Buttons_Init(&buttons);
    Buttons_SetCallbacks(&buttons, App_OnStart, App_OnStop, App_OnModeChange);
    Debug_Print("[OK] Buttons initialized\r\n");
    
    /* Initialize navigation */
    Navigation_Init(&navigation, &motor, &servo, &ultrasonic, &encoder, &buttons);
    Debug_Print("[OK] Navigation initialized\r\n");
    
    /* Set initial LED pattern (slow blink = IDLE) */
    Utils_SetLEDPattern(LED_PATTERN_BLINK_SLOW);
    
    Debug_Print("\r\nInitialization complete!\r\n");
    Debug_Print("----------------------------------------\r\n");
    Debug_Print("Controls:\r\n");
    Debug_Print("  BLUE  = Mode 1 (Obstacle Avoidance)\r\n");
    Debug_Print("          Press again to toggle 3005/7515mm\r\n");
    Debug_Print("  WHITE = Mode 2 (Wall Following Race)\r\n");
    Debug_Print("  GREEN = Start\r\n");
    Debug_Print("  RED   = Stop\r\n");
    Debug_Print("----------------------------------------\r\n\r\n");
}

/**
 * @brief Called when START button pressed
 */
static void App_OnStart(void)
{
    Debug_Print("\r\n>>> START <<<\r\n");
    
    Car_Mode_t mode = Buttons_GetMode(&buttons);
    uint16_t target = Buttons_GetTargetDistance(&buttons);
    
    if (mode == CAR_MODE_1_OBSTACLE) {
        Debug_Print("Mode 1: Obstacle Avoidance\r\n");
        Debug_Print("Target distance: %d mm\r\n", target);
    } else if (mode == CAR_MODE_2_WALLFOLLOW) {
        Debug_Print("Mode 2: Wall Following Race\r\n");
    }
    Debug_Print("\r\n");
    
    Navigation_Start(&navigation, mode, target);
    Utils_SetLEDPattern(LED_PATTERN_BLINK_FAST);
}

/**
 * @brief Called when STOP button pressed
 */
static void App_OnStop(void)
{
    Debug_Print("\r\n>>> STOP <<<\r\n\r\n");
    Navigation_Stop(&navigation);
    Utils_SetLEDPattern(LED_PATTERN_OFF);
}

/**
 * @brief Called when mode changes
 */
static void App_OnModeChange(Car_Mode_t mode)
{
    if (mode == CAR_MODE_1_OBSTACLE) {
        uint16_t dist = Buttons_GetTargetDistance(&buttons);
        Debug_Print("Selected:  Mode 1 - Distance: %d mm\r\n", dist);
        
        if (dist == TASK1_DISTANCE_SHORT) {
            Utils_SetLEDPattern(LED_PATTERN_BLINK_1);
        } else {
            Utils_SetLEDPattern(LED_PATTERN_BLINK_2);
        }
    } else if (mode == CAR_MODE_2_WALLFOLLOW) {
        Debug_Print("Selected: Mode 2 - Wall Following\r\n");
        Utils_SetLEDPattern(LED_PATTERN_SOLID);
    }
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
  MX_LPTIM1_Init();
  MX_TIM2_Init();
  MX_TIM6_Init();
  MX_TIM21_Init();
  MX_TIM22_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
	
  /* USER CODE BEGIN 2 */
	
	/* Initialize application */
  App_Init();
	
	/* === ADD THIS DEBUG CODE === */
	Debug_Print("\r\n=== UART1 Debug ===\r\n");

	/* Check UART1 state */
	Debug_Print("UART1 State: %d\r\n", huart1.gState);
	Debug_Print("UART1 RxState: %d\r\n", huart1.RxState);
	Debug_Print("UART1 Error: %lu\r\n", huart1.ErrorCode);

	/* Check if RXNE interrupt is enabled */
	if (__HAL_UART_GET_IT_SOURCE(&huart1, UART_IT_RXNE)) {
			Debug_Print("RXNE interrupt:  ENABLED\r\n");
	} else {
			Debug_Print("RXNE interrupt: DISABLED <-- PROBLEM!\r\n");
	}

	/* Check NVIC */
	if (NVIC_GetEnableIRQ(USART1_IRQn)) {
			Debug_Print("USART1_IRQn:  ENABLED\r\n");
	} else {
			Debug_Print("USART1_IRQn: DISABLED <-- PROBLEM!\r\n");
	}

	/* Manual loopback test - send something to ESP32 */
	Debug_Print("Sending PING to ESP32...\r\n");
	Encoder_SendCommand(&encoder, "PING");

	Debug_Print("===================\r\n\r\n");
	/* === END DEBUG CODE === */
	
	/* Start ultrasonic sequence */
  US_StartSequence(&ultrasonic);
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		uint32_t now = HAL_GetTick();
    
    /* ===== BUTTON PROCESSING ===== */
    Buttons_Update(&buttons);
    
    /* ===== LED UPDATE ===== */
    Utils_UpdateLED();
    
    /* ===== ULTRASONIC SENSOR SEQUENCING ===== */
    US_Update(&ultrasonic);
   
		
    if (US_SequenceComplete(&ultrasonic)) {
        US_GetAllDistances(&ultrasonic, &front_mm, &left_mm, &right_mm, &rear_mm);
        
        /* Start new sequence every 100ms */
        if ((now - last_us_sequence) >= 300) {
            last_us_sequence = now;
            US_StartSequence(&ultrasonic);
						
        }
    }
    
    /* ===== CONTROL LOOP (50Hz) ===== */
    if ((now - last_control_update) >= CONTROL_LOOP_PERIOD_MS) {
        last_control_update = now;
        
        if (Buttons_IsRunning(&buttons)) {
            /* Update navigation (handles encoder + navigation logic) */
            Navigation_Update(&navigation);
						
            
            /* Check for task completion (Mode 1 only) */
            if (Buttons_GetMode(&buttons) == CAR_MODE_1_OBSTACLE) {
                if (Navigation_IsComplete(&navigation)) {
                    Debug_Print("\r\n*** TASK COMPLETE ***\r\n");
                    Debug_Print("Final distance: %. 1f mm\r\n", Encoder_GetDistance(&encoder));
                    Debug_Print("Target was: %d mm\r\n", Buttons_GetTargetDistance(&buttons));
                    Debug_Print("Error: %.1f mm\r\n\r\n",
                                Encoder_GetDistance(&encoder) - Buttons_GetTargetDistance(&buttons));
                    
                    Buttons_SetState(&buttons, CAR_STATE_STOPPED);
                    Utils_SetLEDPattern(LED_PATTERN_ON);  /* Solid = complete */
                }
            }
        }
    }
    
    /* ===== DEBUG OUTPUT ===== */
		/* In the debug print section, add this:  */

		
    /*
		 if (servo_state == 0 & counter - counter_now >= 500) {
				counter_now = counter;
				Servo_SetAngle(&servo, 5);
				servo_state =1;
		}else if (servo_state == 1 & counter - counter_now >=500){
					counter_now = counter;
					Servo_SetAngle(&servo, -5);
					servo_state =0;
		}
		*/
		
		if ((now - last_debug_print) >= DEBUG_PRINT_INTERVAL_MS) {
        last_debug_print = now;
				
        Car_State_t state = Buttons_GetState(&buttons);
        
			/* Add this diagnostic block */
    static uint32_t last_diag = 0;
    if ((now - last_diag) >= 2000) {  // Every 2 seconds
        last_diag = now;
        
        /* Check echo pin states */
        Debug_Print("DIAG: Echo pins F:%d R:%d L:%d B:%d\r\n",
            HAL_GPIO_ReadPin(FRONT_ECHO_GPIO_Port, FRONT_ECHO_Pin),
            HAL_GPIO_ReadPin(RIGHT_ECHO_GPIO_Port, RIGHT_ECHO_Pin),
            HAL_GPIO_ReadPin(LEFT_ECHO_GPIO_Port, LEFT_ECHO_Pin),
            HAL_GPIO_ReadPin(REAR_ECHO_GPIO_Port, REAR_ECHO_Pin));
        
        /* Check sequence state */
        Debug_Print("DIAG: US seq_running:%d current:%d\r\n",
            ultrasonic.sequence_running,
            ultrasonic.current_sensor);
        
        /* Check sensor states */
        Debug_Print("DIAG:  States F:%d R:%d L:%d B:%d\r\n",
            ultrasonic.sensors[0].state,
            ultrasonic.sensors[1].state,
            ultrasonic.sensors[2].state,
            ultrasonic.sensors[3].state);
            
        /* Check EXTI configuration */
        Debug_Print("DIAG: EXTI IMR: 0x%08lX PR:0x%08lX\r\n",
            EXTI->IMR, EXTI->PR);
    }
        
            /* Print sensor distances */
					
				if (state == CAR_STATE_RUNNING) {
						Debug_Print("D:%.0f S:%.0f H:%.1f | F:%u L:%u R:%u | Nav:%d\r\n",
												Encoder_GetDistance(&encoder),
												Encoder_GetSpeed(&encoder),
												Encoder_GetHeading(&encoder),
												(unsigned int)front_mm, 
												(unsigned int)left_mm, 
												(unsigned int)right_mm,
												(Buttons_GetMode(&buttons) == CAR_MODE_1_OBSTACLE) ? 
														(int)Navigation_GetNav1State(&navigation) : 
														(int)Navigation_GetNav2State(&navigation));
        } else if (state == CAR_STATE_IDLE) {
            
            Debug_Print("IDLE | F:%4d L:%4d R:%4d B:%4d | UART:%lu EXTI:%lu\r\n",
                        front_mm, left_mm, right_mm, rear_mm,
                        encoder.packet_count, exti_count);
					
					
        }
    
				
	}
				/* Simple encoder test - add in main loop */
	/*	
	static uint32_t last_enc_test = 0;
		if ((now - last_enc_test) >= 200) {
				last_enc_test = now;
				Debug_Print("ENC pkts:%lu d: %.0f s:%.0f h:%.1f\r\n",
										encoder. packet_count,
										encoder.total_distance_mm,
										encoder.speed_mm_s,
										encoder.heading_rad * 57.3f);  // Convert to degrees
		}
	*/
	
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_4;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART2
                              |RCC_PERIPHCLK_LPTIM1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.LptimClockSelection = RCC_LPTIM1CLKSOURCE_PCLK;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief LPTIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPTIM1_Init(void)
{

  /* USER CODE BEGIN LPTIM1_Init 0 */

  /* USER CODE END LPTIM1_Init 0 */

  /* USER CODE BEGIN LPTIM1_Init 1 */

  /* USER CODE END LPTIM1_Init 1 */
  hlptim1.Instance = LPTIM1;
  hlptim1.Init.Clock.Source = LPTIM_CLOCKSOURCE_APBCLOCK_LPOSC;
  hlptim1.Init.Clock.Prescaler = LPTIM_PRESCALER_DIV32;
  hlptim1.Init.Trigger.Source = LPTIM_TRIGSOURCE_SOFTWARE;
  hlptim1.Init.OutputPolarity = LPTIM_OUTPUTPOLARITY_HIGH;
  hlptim1.Init.UpdateMode = LPTIM_UPDATE_IMMEDIATE;
  hlptim1.Init.CounterSource = LPTIM_COUNTERSOURCE_INTERNAL;
  if (HAL_LPTIM_Init(&hlptim1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPTIM1_Init 2 */

  /* USER CODE END LPTIM1_Init 2 */

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
  htim2.Init.Prescaler = 31;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
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
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 31;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 65535;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM21 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM21_Init(void)
{

  /* USER CODE BEGIN TIM21_Init 0 */

  /* USER CODE END TIM21_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM21_Init 1 */

  /* USER CODE END TIM21_Init 1 */
  htim21.Instance = TIM21;
  htim21.Init.Prescaler = 79;
  htim21.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim21.Init.Period = 7999;
  htim21.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim21.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim21) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim21, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim21) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim21, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim21, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM21_Init 2 */

  /* USER CODE END TIM21_Init 2 */
  HAL_TIM_MspPostInit(&htim21);

}

/**
  * @brief TIM22 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM22_Init(void)
{

  /* USER CODE BEGIN TIM22_Init 0 */

  /* USER CODE END TIM22_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM22_Init 1 */

  /* USER CODE END TIM22_Init 1 */
  htim22.Instance = TIM22;
  htim22.Init.Prescaler = 0;
  htim22.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim22.Init.Period = 1279;
  htim22.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim22.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim22) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim22, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim22) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim22, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim22, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim22, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM22_Init 2 */

  /* USER CODE END TIM22_Init 2 */
  HAL_TIM_MspPostInit(&htim22);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|RIGHT_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LEFT_EN_GPIO_Port, LEFT_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, REAR_TRIG_Pin|LEFT_TRIG_Pin|RIGHT_TRIG_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(FRONT_TRIG_GPIO_Port, FRONT_TRIG_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin RIGHT_EN_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|RIGHT_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	/*Configure GPIO pins : GREEN_BUTTON_Pin WHITE_BUTTON_Pin */
  GPIO_InitStruct.Pin = GREEN_BUTTON_Pin|WHITE_BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : FRONT_ECHO_Pin RIGHT_ECHO_Pin
                           LEFT_ECHO_Pin REAR_ECHO_Pin */
  GPIO_InitStruct.Pin = FRONT_ECHO_Pin|RIGHT_ECHO_Pin
                          |LEFT_ECHO_Pin|REAR_ECHO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LEFT_EN_Pin */
  GPIO_InitStruct.Pin = LEFT_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LEFT_EN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BLUE_BUTTON_Pin RED_BUTTON_Pin */
  GPIO_InitStruct.Pin = BLUE_BUTTON_Pin|RED_BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : REAR_TRIG_Pin LEFT_TRIG_Pin RIGHT_TRIG_Pin */
  GPIO_InitStruct.Pin = REAR_TRIG_Pin|LEFT_TRIG_Pin|RIGHT_TRIG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : FRONT_TRIG_Pin */
  GPIO_InitStruct.Pin = FRONT_TRIG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(FRONT_TRIG_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
	 HAL_NVIC_SetPriority(USART1_IRQn, 1, 0);
	 HAL_NVIC_EnableIRQ(USART1_IRQn);
   HAL_NVIC_SetPriority(USART2_IRQn, 2, 0);
  /* USER CODE END MX_GPIO_Init_2 */
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
