/**
 * @file    motor. h
 * @brief   BTS7960 motor driver control
 */

#ifndef MOTOR_H
#define MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32l0xx_hal.h"
#include "car_config.h"
#include <stdint.h>
#include <stdbool.h>

/* ============================================================================
 * TYPES
 * ============================================================================ */
typedef enum {
    MOTOR_DIR_STOP = 0,
    MOTOR_DIR_FORWARD,
    MOTOR_DIR_REVERSE
} Motor_Direction_t;

typedef struct {
    TIM_HandleTypeDef *htim;        // Timer handle (TIM22)
    uint32_t channel_forward;       // Forward PWM channel (RIGHT_PWM = CH1 = PB4)
    uint32_t channel_reverse;       // Reverse PWM channel (LEFT_PWM = CH2 = PB5)
    GPIO_TypeDef *en_r_port;        // Right enable port (PA8)
    uint16_t en_r_pin;              // Right enable pin
    GPIO_TypeDef *en_l_port;        // Left enable port (PB2)
    uint16_t en_l_pin;              // Left enable pin
    
    /* State */
    Motor_Direction_t direction;
    uint16_t pwm_value;
    bool is_running;
} Motor_Handle_t;

/* ============================================================================
 * FUNCTIONS
 * ============================================================================ */

/**
 * @brief Initialize motor driver
 * @param motor Pointer to motor handle
 * @param htim Timer handle for PWM (TIM22)
 * @param ch_fwd Forward channel (TIM_CHANNEL_1 = PB4 = RPWM on BTS7960)
 * @param ch_rev Reverse channel (TIM_CHANNEL_2 = PB5 = LPWM on BTS7960)
 * @param en_r_port Right enable GPIO port
 * @param en_r_pin Right enable GPIO pin
 * @param en_l_port Left enable GPIO port
 * @param en_l_pin Left enable GPIO pin
 */
void Motor_Init(Motor_Handle_t *motor, 
                TIM_HandleTypeDef *htim,
                uint32_t ch_fwd, 
                uint32_t ch_rev,
                GPIO_TypeDef *en_r_port, 
                uint16_t en_r_pin,
                GPIO_TypeDef *en_l_port, 
                uint16_t en_l_pin);

/**
 * @brief Set motor speed and direction
 * @param motor Pointer to motor handle
 * @param direction Direction (STOP, FORWARD, REVERSE)
 * @param pwm PWM value (0 to MOTOR_PWM_MAX)
 */
void Motor_Set(Motor_Handle_t *motor, Motor_Direction_t direction, uint16_t pwm);

/**
 * @brief Set motor speed (forward positive, reverse negative)
 * @param motor Pointer to motor handle
 * @param speed Speed value (-MOTOR_PWM_MAX to +MOTOR_PWM_MAX)
 */
void Motor_SetSpeed(Motor_Handle_t *motor, int16_t speed);

/**
 * @brief Stop motor (coast - disables outputs)
 * @param motor Pointer to motor handle
 */
void Motor_Stop(Motor_Handle_t *motor);

/**
 * @brief Brake motor (shorts motor windings)
 * @param motor Pointer to motor handle
 */
void Motor_Brake(Motor_Handle_t *motor);

/**
 * @brief Get current motor direction
 * @param motor Pointer to motor handle
 * @return Current direction
 */
Motor_Direction_t Motor_GetDirection(Motor_Handle_t *motor);

/**
 * @brief Get current PWM value
 * @param motor Pointer to motor handle
 * @return Current PWM value
 */
uint16_t Motor_GetPWM(Motor_Handle_t *motor);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_H */