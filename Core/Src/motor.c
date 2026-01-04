/**
 * @file    motor.c
 * @brief   BTS7960 motor driver implementation
 */

#include "motor.h"

/* ============================================================================
 * PUBLIC FUNCTIONS
 * ============================================================================ */

void Motor_Init(Motor_Handle_t *motor, 
                TIM_HandleTypeDef *htim,
                uint32_t ch_fwd, 
                uint32_t ch_rev,
                GPIO_TypeDef *en_r_port, 
                uint16_t en_r_pin,
                GPIO_TypeDef *en_l_port, 
                uint16_t en_l_pin)
{
    motor->htim = htim;
    motor->channel_forward = ch_fwd;
    motor->channel_reverse = ch_rev;
    motor->en_r_port = en_r_port;
    motor->en_r_pin = en_r_pin;
    motor->en_l_port = en_l_port;
    motor->en_l_pin = en_l_pin;
    
    motor->direction = MOTOR_DIR_STOP;
    motor->pwm_value = 0;
    motor->is_running = false;
    
    /* Ensure motor is stopped initially */
    HAL_GPIO_WritePin(motor->en_r_port, motor->en_r_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(motor->en_l_port, motor->en_l_pin, GPIO_PIN_RESET);
    
    /* Set PWM to 0 */
    __HAL_TIM_SET_COMPARE(motor->htim, motor->channel_forward, 0);
    __HAL_TIM_SET_COMPARE(motor->htim, motor->channel_reverse, 0);
    
    /* Start PWM channels */
    HAL_TIM_PWM_Start(motor->htim, motor->channel_forward);
    HAL_TIM_PWM_Start(motor->htim, motor->channel_reverse);
}

void Motor_Set(Motor_Handle_t *motor, Motor_Direction_t direction, uint16_t pwm)
{
    /* Clamp PWM value */
    if (pwm > MOTOR_PWM_LIMIT) {
        pwm = MOTOR_PWM_LIMIT;
    }
    
    motor->direction = direction;
    motor->pwm_value = pwm;
    
    switch (direction) {
        case MOTOR_DIR_FORWARD:
            /* Enable both drivers */
            HAL_GPIO_WritePin(motor->en_r_port, motor->en_r_pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(motor->en_l_port, motor->en_l_pin, GPIO_PIN_SET);
            
            /* Forward:  RIGHT_PWM active, LEFT_PWM = 0 */
            __HAL_TIM_SET_COMPARE(motor->htim, motor->channel_reverse, 0);
            __HAL_TIM_SET_COMPARE(motor->htim, motor->channel_forward, pwm);
            motor->is_running = true;
            break;
            
        case MOTOR_DIR_REVERSE: 
            /* Enable both drivers */
            HAL_GPIO_WritePin(motor->en_r_port, motor->en_r_pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(motor->en_l_port, motor->en_l_pin, GPIO_PIN_SET);
            
            /* Reverse: LEFT_PWM active, RIGHT_PWM = 0 */
            __HAL_TIM_SET_COMPARE(motor->htim, motor->channel_forward, 0);
            __HAL_TIM_SET_COMPARE(motor->htim, motor->channel_reverse, pwm);
            motor->is_running = true;
            break;
            
        case MOTOR_DIR_STOP:
        default:
            Motor_Stop(motor);
            break;
    }
}

void Motor_SetSpeed(Motor_Handle_t *motor, int16_t speed)
{
    if (speed > 0) {
        Motor_Set(motor, MOTOR_DIR_FORWARD, (uint16_t)speed);
    } else if (speed < 0) {
        Motor_Set(motor, MOTOR_DIR_REVERSE, (uint16_t)(-speed));
    } else {
        Motor_Stop(motor);
    }
}

void Motor_Stop(Motor_Handle_t *motor)
{
    /* Disable both drivers */
    HAL_GPIO_WritePin(motor->en_r_port, motor->en_r_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(motor->en_l_port, motor->en_l_pin, GPIO_PIN_RESET);
    
    /* Set PWM to 0 */
    __HAL_TIM_SET_COMPARE(motor->htim, motor->channel_forward, 0);
    __HAL_TIM_SET_COMPARE(motor->htim, motor->channel_reverse, 0);
    
    motor->direction = MOTOR_DIR_STOP;
    motor->pwm_value = 0;
    motor->is_running = false;
}

void Motor_Brake(Motor_Handle_t *motor)
{
    /* Enable both drivers */
    HAL_GPIO_WritePin(motor->en_r_port, motor->en_r_pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(motor->en_l_port, motor->en_l_pin, GPIO_PIN_SET);
    
    /* Both PWM LOW = brake (motor shorts through low-side FETs) */
    __HAL_TIM_SET_COMPARE(motor->htim, motor->channel_forward, 0);
    __HAL_TIM_SET_COMPARE(motor->htim, motor->channel_reverse, 0);
    
    motor->direction = MOTOR_DIR_STOP;
    motor->pwm_value = 0;
    motor->is_running = false;
}

Motor_Direction_t Motor_GetDirection(Motor_Handle_t *motor)
{
    return motor->direction;
}

uint16_t Motor_GetPWM(Motor_Handle_t *motor)
{
    return motor->pwm_value;
}